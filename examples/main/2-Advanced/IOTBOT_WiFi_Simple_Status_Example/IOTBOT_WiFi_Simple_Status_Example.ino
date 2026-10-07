/*
 * TR: KABLOSUZ İLETİŞİME İLK ADIM - En basit WiFi örneği
 *  - IOTBOT'u evinizin WiFi ağına bağlar; bağlantı başarılı olursa aldığı IP adresini
 *    ve sinyal gücünü (RSSI) LCD'de ve Seri Port'ta gösterir. Sinyal gücü canlı
 *    güncellenir: kartı modeme yaklaştırıp uzaklaştırarak değişimi izleyin!
 *  - Bağlantı koparsa LCD'de ve Seri Port'ta görünür, WiFi kendiliğinden yeniden bağlanır.
 *  - Sunucu YOK, web sayfası YOK - sadece "ağa katılmak" ne demek onu öğretir. Önce
 *    bunu deneyin, sonra IOTBOT_WiFi_Web_Control_Example.ino'ya geçin.
 *  - B3 butonu: LCD'de IP sayfası <-> MAC/kanal sayfası.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help      -> komut listesi
 *      durum   / status    -> bağlantı, IP, MAC, sinyal gücü, kanal
 *      tara    / scan      -> çevredeki WiFi ağlarını listele
 *      yeniden / reconnect -> ağa yeniden bağlan
 *      dil     / lang      -> dili değiştir (Türkçe <-> English)
 *
 * EN: FIRST STEP INTO WIRELESS COMMUNICATION - the simplest WiFi example
 *  - Connects the IOTBOT to your home WiFi network; once connected it shows the IP
 *    address and the signal strength (RSSI) on the LCD and Serial. The signal strength
 *    updates live: move the board closer to / away from the router and watch it change!
 *  - If the connection drops it is shown on the LCD and Serial, and WiFi reconnects by itself.
 *  - NO server, NO web page - it just teaches what "joining a network" means. Try this
 *    first, then move on to IOTBOT_WiFi_Web_Control_Example.ino.
 *  - B3 button: LCD IP page <-> MAC/channel page.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help      / yardim  -> command list
 *      status    / durum   -> connection, IP, MAC, signal strength, channel
 *      scan      / tara    -> list the WiFi networks around
 *      reconnect / yeniden -> reconnect to the network
 *      lang      / dil     -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. / NO extra module needed.
 */

#define USE_WIFI
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// ÖNEMLİ: Kendi WiFi ağınızın adını ve şifresini yazın.
// IMPORTANT: Fill in your own WiFi network's name and password.
#define WIFI_SSID "WIFI_SSID"
#define WIFI_PASS "WIFI_PASSWORD"

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

bool showMacPage = false;  // false = IP sayfası, true = MAC/kanal sayfası / false = IP page, true = MAC/channel page
bool wasConnected = false;
uint32_t lastDrawMs = 0;
bool lastB3 = false;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "DURUM" -> "durum"
// Lower-cases and simplifies Turkish letters: "DURUM" -> "durum"
String normalizeCommand(String s) {
  s.trim();
  s.replace("İ", "i"); s.replace("I", "i"); s.replace("ı", "i");
  s.replace("Ş", "s"); s.replace("ş", "s");
  s.replace("Ğ", "g"); s.replace("ğ", "g");
  s.replace("Ü", "u"); s.replace("ü", "u");
  s.replace("Ö", "o"); s.replace("ö", "o");
  s.replace("Ç", "c"); s.replace("ç", "c");
  s.toLowerCase();
  return s;
}

bool readCommand(String &cmd) {
  while (iotbot.serialAvailable() > 0) {
    char c = Serial.read();
    lastCharMs = millis();
    if (c == '\n' || c == '\r') {
      if (cmdBuffer.length() == 0) continue;
      cmd = normalizeCommand(cmdBuffer);
      cmdBuffer = "";
      return true;
    }
    if (cmdBuffer.length() < 40) cmdBuffer += c;
  }
  // "Satır sonu yok" seçiliyse: 150 ms sessizlikten sonra komutu kabul et.
  // "No line ending" selected: accept the command after 150 ms of silence.
  if (cmdBuffer.length() > 0 && millis() - lastCharMs > 150) {
    cmd = normalizeCommand(cmdBuffer);
    cmdBuffer = "";
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

bool wifiOk() { return WiFi.status() == WL_CONNECTED; }

// Sinyal gücünü kelimeyle anlatır / describes the signal strength in words
const char *signalWord(int rssi) {
  if (rssi >= -55) return L("çok iyi", "excellent");
  if (rssi >= -67) return L("iyi", "good");
  if (rssi >= -75) return L("orta", "fair");
  return L("zayıf", "weak");
}

void drawScreen() {
  char line[41];
  if (!wifiOk()) {
    lcdRow(0, L("BAĞLANTI YOK", "NOT CONNECTED"));
    snprintf(line, sizeof(line), L("Ağ: %s", "Net: %s"), WIFI_SSID);
    lcdRow(1, line);
    lcdRow(2, L("SSID/şifreyi kontrol", "Check SSID/password"));
    lcdRow(3, L("Yeniden deneniyor..", "Retrying..."));
    return;
  }
  int rssi = WiFi.RSSI();
  lcdRow(0, L("WIFI BAĞLANDI", "WIFI CONNECTED"));
  if (!showMacPage) {
    snprintf(line, sizeof(line), L("Ağ: %s", "Net: %s"), WIFI_SSID);
    lcdRow(1, line);
    snprintf(line, sizeof(line), "IP: %s", iotbot.wifiGetIPAddress().c_str());
    lcdRow(2, line);
    snprintf(line, sizeof(line), L("Sinyal: %d %s", "Signal: %d %s"), rssi, signalWord(rssi));
    lcdRow(3, line);
  } else {
    lcdRow(1, "MAC:");
    lcdRow(2, iotbot.wifiGetMACAddress().c_str());
    snprintf(line, sizeof(line), L("Kanal: %d  %d dBm", "Channel: %d  %d dBm"), (int)WiFi.channel(), rssi);
    lcdRow(3, line);
  }
}

void printHelp() {
  iotbot.serialWrite(L("---- WiFi DURUMU - Komutlar ----", "---- WiFi STATUS - Commands ----"));
  iotbot.serialWrite(L("  yardim  : bu liste", "  help      : this list"));
  iotbot.serialWrite(L("  durum   : bağlantı, IP, MAC, sinyal", "  status    : connection, IP, MAC, signal"));
  iotbot.serialWrite(L("  tara    : çevredeki ağları listele", "  scan      : list the networks around"));
  iotbot.serialWrite(L("  yeniden : ağa yeniden bağlan", "  reconnect : reconnect to the network"));
  iotbot.serialWrite(L("  dil     : English'e geç", "  lang      : switch to Turkish"));
  iotbot.serialWrite(L("  B3      : LCD sayfası (IP <-> MAC)", "  B3        : LCD page (IP <-> MAC)"));
}

void printStatus() {
  if (!wifiOk()) {
    iotbot.serialWrite(String(L("Bağlı değil. Ağ: ", "Not connected. Network: ")) + WIFI_SSID);
    return;
  }
  int rssi = WiFi.RSSI();
  iotbot.serialWrite(String(L("Bağlı! Ağ: ", "Connected! Network: ")) + WIFI_SSID + "   IP: " + iotbot.wifiGetIPAddress());
  iotbot.serialWrite(String("MAC: ") + iotbot.wifiGetMACAddress() + L("   Kanal: ", "   Channel: ") + WiFi.channel());
  iotbot.serialWrite(String(L("Sinyal gücü: ", "Signal strength: ")) + rssi + " dBm (" + signalWord(rssi) + ")");
}

void scanNetworks() {
  iotbot.serialWrite(L("Ağlar taranıyor (birkaç saniye)...", "Scanning networks (a few seconds)..."));
  int n = WiFi.scanNetworks();
  if (n <= 0) {
    iotbot.serialWrite(L("Hiç ağ bulunamadı.", "No networks found."));
    return;
  }
  iotbot.serialWrite(String(L("Bulunan ağ: ", "Networks found: ")) + n);
  for (int i = 0; i < n && i < 15; i++) {
    iotbot.serialWrite(String("  ") + (i + 1) + ") " + WiFi.SSID(i) + "  " + WiFi.RSSI(i) + " dBm" +
                       (WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? L("  (şifresiz)", "  (open)") : ""));
  }
  WiFi.scanDelete();
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "durum" || cmd == "status") {
    printStatus();
  } else if (cmd == "tara" || cmd == "scan") {
    scanNetworks();
  } else if (cmd == "yeniden" || cmd == "reconnect") {
    iotbot.serialWrite(L("Yeniden bağlanılıyor...", "Reconnecting..."));
    WiFi.disconnect();
    iotbot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS);
    printStatus();
  } else if (cmd == "dil" || cmd == "lang" || cmd == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    drawScreen();
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.lcdShowLoading(L("WiFi'ye bağlanıyor", "Connecting to WiFi"));

  iotbot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS);

  wasConnected = wifiOk();
  if (wasConnected) {
    iotbot.serialWrite(String(L("Bağlandı! IP adresi: ", "Connected! IP address: ")) + iotbot.wifiGetIPAddress());
    iotbot.buzzerPlayTone(2000, 300);
  } else {
    iotbot.serialWrite(L("Bağlantı başarısız! SSID/şifreyi kontrol edin.", "Connection failed! Check SSID/password."));
    iotbot.buzzerPlayTone(400, 500);
  }
  iotbot.lcdClear();
  drawScreen();
  printHelp();
}

void loop() {
  // B3 -> LCD sayfası değiştir (sadece basıldığı an) / B3 -> change the LCD page (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) {
    showMacPage = !showMacPage;
    drawScreen();
  }
  lastB3 = b3;

  // Bağlantı değişti mi? / did the connection change?
  bool connected = wifiOk();
  if (connected != wasConnected) {
    wasConnected = connected;
    iotbot.serialWrite(connected ? String(L("Yeniden bağlandı! IP: ", "Reconnected! IP: ")) + iotbot.wifiGetIPAddress()
                                 : String(L("Bağlantı KOPTU!", "Connection LOST!")));
    iotbot.buzzerPlayTone(connected ? 2000 : 400, 150);
    drawScreen();
  }

  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // Sinyal gücünü saniyede bir güncelle / update the signal strength once a second
  if (millis() - lastDrawMs >= 1000) {
    lastDrawMs = millis();
    drawScreen();
  }
  delay(10);
}
