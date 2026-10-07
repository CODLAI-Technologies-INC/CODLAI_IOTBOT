/*
 * TR: OTA (KABLOSUZ KOD YÜKLEME) - Temel örnek
 *  - IOTBOT WiFi'ye bağlanır ve OTA'yı başlatır. Bundan sonra yeni kodu USB kablosu
 *    OLMADAN, aynı ağdaki bilgisayardan yükleyebilirsiniz:
 *      Arduino IDE: Araçlar > Port > "IOTBOT-OTA at <IP>" (ağ portu), şifre: 1234
 *      PlatformIO : upload_protocol = espota, upload_port = <IP>, upload_flags = --auth=1234
 *  - ÖNEMLİ: Yeni yüklediğiniz kodda da OTA olmalı (otaBegin + loop'ta otaHandle),
 *    yoksa bir sonraki yükleme yine kabloyla yapılmak zorunda kalır.
 *  - OTA için WiFi bağlantısı kurun ve loop() içinde otaHandle()'ı sürekli çağırın.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help    -> komut listesi
 *      durum  / status  -> IP adresi, cihaz adı, port, sinyal gücü
 *      dil    / lang    -> dili değiştir (Türkçe <-> English)
 *
 * EN: OTA (WIRELESS CODE UPLOAD) - Basic example
 *  - The IOTBOT joins WiFi and starts OTA. From then on you can upload new code
 *    WITHOUT a USB cable, from a computer on the same network:
 *      Arduino IDE: Tools > Port > "IOTBOT-OTA at <IP>" (network port), password: 1234
 *      PlatformIO : upload_protocol = espota, upload_port = <IP>, upload_flags = --auth=1234
 *  - IMPORTANT: the new code must contain OTA too (otaBegin + otaHandle in loop),
 *    otherwise the next upload must be done with a cable again.
 *  - For OTA, connect to WiFi and call otaHandle() continuously in loop().
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim  -> command list
 *      status / durum   -> IP address, device name, port, signal strength
 *      lang   / dil     -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. / NO extra module needed.
 * Not / Note: USE_OTA ve USE_WIFI tanımları kütüphaneden ÖNCE yazılmalıdır.
 *             USE_OTA and USE_WIFI must be defined BEFORE including the library.
 */

#define USE_WIFI
#define USE_OTA
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

const char *WIFI_SSID = "YOUR_WIFI_SSID";
const char *WIFI_PASS = "YOUR_WIFI_PASSWORD";

const char *OTA_HOST = "IOTBOT-OTA"; // Cihaz adı (ağda görünen) / device name (shown on the network)
const char *OTA_PASS = "1234";       // OTA şifresi / OTA password
const uint16_t OTA_PORT = 3232;

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

uint32_t lastDrawMs = 0;

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

void drawScreen() {
  char line[41];
  lcdRow(0, L("  OTA GÜNCELLEME", "   OTA UPDATE"));
  if (wifiOk()) {
    snprintf(line, sizeof(line), "IP: %s", iotbot.wifiGetIPAddress().c_str());
    lcdRow(1, line);
    snprintf(line, sizeof(line), L("Ad: %s", "Name: %s"), OTA_HOST);
    lcdRow(2, line);
    snprintf(line, sizeof(line), L("OTA hazır  %d dBm", "OTA ready  %d dBm"), (int)WiFi.RSSI());
    lcdRow(3, line);
  } else {
    lcdRow(1, L("WiFi YOK", "NO WiFi"));
    lcdRow(2, L("Ad/şifreyi kontrol", "Check name/password"));
    lcdRow(3, L("edin", "and upload again"));
  }
}

void printHelp() {
  iotbot.serialWrite(L("---- OTA GÜNCELLEME - Komutlar ----", "---- OTA UPDATE - Commands ----"));
  iotbot.serialWrite(L("  yardim : bu liste", "  help   : this list"));
  iotbot.serialWrite(L("  durum  : IP, cihaz adı, port, sinyal", "  status : IP, device name, port, signal"));
  iotbot.serialWrite(L("  dil    : English'e geç", "  lang   : switch to Turkish"));
}

void printStatus() {
  if (!wifiOk()) {
    iotbot.serialWrite(L("WiFi bağlı değil - OTA çalışmaz.", "WiFi not connected - OTA does not work."));
    return;
  }
  iotbot.serialWrite(String("IP: ") + iotbot.wifiGetIPAddress() + L("   Cihaz adı: ", "   Device name: ") + OTA_HOST +
                     "   Port: " + OTA_PORT + L("   Şifre: ", "   Password: ") + OTA_PASS);
  iotbot.serialWrite(String(L("Sinyal gücü: ", "Signal strength: ")) + WiFi.RSSI() + " dBm");
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "durum" || cmd == "status") {
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
  iotbot.lcdClear();
  lcdRow(0, L("  OTA GÜNCELLEME", "   OTA UPDATE"));
  lcdRow(1, L("WiFi'ye bağlanıyor", "Connecting WiFi"));

  iotbot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS);
  iotbot.otaBegin(OTA_HOST, OTA_PASS, OTA_PORT); // OTA'yı başlat / start OTA

  drawScreen();
  if (wifiOk()) {
    iotbot.serialWrite(String(L("OTA hazır! Arduino IDE'de ağ portunu seçin: ", "OTA ready! Pick the network port in the Arduino IDE: ")) +
                       OTA_HOST + " (" + iotbot.wifiGetIPAddress() + ")");
  } else {
    iotbot.serialWrite(L("WiFi bağlantısı yok! SSID/şifreyi kontrol edin.", "No WiFi connection! Check SSID/password."));
  }
  printHelp();
}

void loop() {
  iotbot.otaHandle(); // OTA isteklerini dinle - SÜREKLİ çağrılmalı / listen for OTA requests - must be called ALL the time

  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // LCD 5 saniyede bir (sinyal gücü değişir) / LCD every 5 seconds (the signal strength changes)
  if (millis() - lastDrawMs >= 5000) {
    lastDrawMs = millis();
    drawScreen();
  }
  delay(10);
}
