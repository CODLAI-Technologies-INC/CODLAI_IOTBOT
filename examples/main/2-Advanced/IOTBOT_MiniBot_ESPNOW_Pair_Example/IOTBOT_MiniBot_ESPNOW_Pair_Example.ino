/*
 * TR: IOTBOT <-> MINIBOT EŞLEŞMESİ (ESP-NOW, MAC adresiyle)
 *  - IOTBOT, aynı odadaki bir MINIBOT ile ROUTER/WIFI AĞI OLMADAN doğrudan haberleşir.
 *  - IOTBOT her 0,5 saniyede potansiyometre değerini ve B3 butonunun durumunu
 *    MINIBOT'a gönderir; MINIBOT'un butonuna basılıp basılmadığını ve gönderdiği
 *    sayacı geri alıp LCD'de gösterir. MINIBOT butonuna basılınca kısa bir bip duyulur.
 *  - Eşlenecek MINIBOT'a CODLAI_MINIBOT kütüphanesindeki
 *    MINIBOT_IoTBot_ESPNOW_Pair_Example.ino dosyasını yükleyin.
 *  - ÖNEMLİ: Aşağıdaki kPeerMac dizisini GERÇEK MINIBOT'unuzun MAC adresiyle
 *    değiştirin (MINIBOT tarafındaki kod Seri Port'a kendi MAC'ini yazar).
 *  - Paketteki deviceType: IOTBOT 40 gönderir, MINIBOT 41 gönderir (kütüphane
 *    örneklerinin kart kimlikleri, bkz. COMMANDS_README "deviceType haritası").
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help    -> komut listesi
 *      durum  / status  -> MAC adresleri, gönderilen/alınan paket, son veri
 *      dil    / lang    -> dili değiştir (Türkçe <-> English)
 *
 * EN: IOTBOT <-> MINIBOT PAIRING (ESP-NOW, by MAC address)
 *  - The IOTBOT talks directly (peer-to-peer, no router/WiFi network needed) with a
 *    MINIBOT in the same room.
 *  - Every 0.5 s the IOTBOT sends its potentiometer value and B3 button state to the
 *    MINIBOT; it receives whether the MINIBOT's button is pressed and its counter, and
 *    shows them on the LCD. A short beep sounds when the MINIBOT button is pressed.
 *  - Upload MINIBOT_IoTBot_ESPNOW_Pair_Example.ino (in the CODLAI_MINIBOT library's
 *    examples) to the MINIBOT you want to pair with.
 *  - IMPORTANT: replace kPeerMac below with your actual MINIBOT's MAC address (the
 *    MINIBOT-side sketch prints its own MAC to Serial).
 *  - deviceType in the packet: the IOTBOT sends 40, the MINIBOT sends 41 (library
 *    example board IDs, see the COMMANDS_README "deviceType map").
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim  -> command list
 *      status / durum   -> MAC addresses, packets sent/received, last data
 *      lang   / dil     -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. Her iki kart da kanal 1'de olmalı.
 * NO extra module needed. Both boards must be on channel 1.
 */

#define USE_ESPNOW
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Test için kullanılan gerçek bir MINIBOT'un MAC adresi - kendi kartınıza göre değiştirin.
// A real MINIBOT's MAC address used for testing - change this to match your own board.
uint8_t kPeerMac[] = {0x8C, 0x4F, 0x00, 0x5C, 0x84, 0x9E};

const uint32_t kSendIntervalMs = 500;
uint32_t lastSendMs = 0;
uint32_t sentCount = 0, receivedCount = 0;
int lastPot = 0;
bool lastB3Sent = false;
int minibotCounter = -1;          // -1 = henüz veri yok / no data yet
bool minibotButtonPressed = false;
uint32_t lastReceiveMs = 0;

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

String peerMacText() {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", kPeerMac[0], kPeerMac[1], kPeerMac[2], kPeerMac[3], kPeerMac[4], kPeerMac[5]);
  return String(buf);
}

void drawScreen() {
  char line[41];
  lcdRow(0, L("MINIBOT EŞLEŞMESİ", "MINIBOT PAIRING"));
  if (minibotCounter < 0) {
    lcdRow(1, L("MiniBot bekleniyor", "Waiting for MiniBot"));
    lcdRow(2, "");
  } else {
    snprintf(line, sizeof(line), L("MiniBot sayaç: %d", "MiniBot count: %d"), minibotCounter);
    lcdRow(1, line);
    lcdRow(2, minibotButtonPressed ? L("Buton: BASILI", "Button: PRESSED") : L("Buton: serbest", "Button: released"));
  }
  snprintf(line, sizeof(line), L("Pot:%4d B3:%s", "Pot:%4d B3:%s"), lastPot, lastB3Sent ? L("basılı", "pressed") : L("serbest", "free"));
  lcdRow(3, line);
}

void printHelp() {
  iotbot.serialWrite(L("---- MINIBOT EŞLEŞMESİ - Komutlar ----", "---- MINIBOT PAIRING - Commands ----"));
  iotbot.serialWrite(L("  yardim : bu liste", "  help   : this list"));
  iotbot.serialWrite(L("  durum  : MAC adresleri, paket sayıları, son veri", "  status : MAC addresses, packet counts, last data"));
  iotbot.serialWrite(L("  dil    : English'e geç", "  lang   : switch to Turkish"));
  iotbot.serialWrite(L("  Pot ve B3 durumu MiniBot'a otomatik gönderilir.", "  The pot and B3 state are sent to the MiniBot automatically."));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "durum" || cmd == "status") {
    iotbot.serialWrite(String(L("Benim MAC adresim: ", "My MAC address: ")) + WiFi.macAddress());
    iotbot.serialWrite(String(L("MiniBot MAC (kPeerMac): ", "MiniBot MAC (kPeerMac): ")) + peerMacText());
    iotbot.serialWrite(String(L("Gönderilen: ", "Sent: ")) + sentCount + L("   Alınan: ", "   Received: ") + receivedCount);
    if (minibotCounter < 0) {
      iotbot.serialWrite(L("MiniBot'tan henüz veri gelmedi - MAC adresini ve MiniBot kodunu kontrol edin.",
                           "No data from the MiniBot yet - check the MAC address and the MiniBot sketch."));
    } else {
      iotbot.serialWrite(String(L("Son veri: sayaç ", "Last data: counter ")) + minibotCounter + L(", buton ", ", button ") +
                         (minibotButtonPressed ? L("BASILI", "PRESSED") : L("serbest", "released")) +
                         L(", ", ", ") + ((millis() - lastReceiveMs) / 1000) + L(" sn önce", " s ago"));
    }
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
  iotbot.lcdShowLoading(L("ESP-NOW başlıyor", "Starting ESP-NOW"));

  iotbot.initESPNow();
  iotbot.setWiFiChannel(1); // İki taraf da AYNI kanalda olmalı / both sides must use the SAME channel
  iotbot.startListening();  // Gelen mesajları iotbot.receivedData'ya yazar / fills iotbot.receivedData on arrival

  iotbot.serialWrite(String(L("Benim MAC adresim: ", "My MAC address: ")) + WiFi.macAddress());
  iotbot.serialWrite(String(L("Eşlenen MiniBot: ", "Paired MiniBot: ")) + peerMacText());
  iotbot.lcdClear();
  drawScreen();
  printHelp();
}

void loop() {
  // 1) Gönderim: potansiyometre + B3 durumu (paket yapısı MINIBOT ile AYNI kalmalı)
  // 1) Sending: pot + B3 state (the packet layout must stay the SAME as on the MINIBOT)
  if (millis() - lastSendMs >= kSendIntervalMs) {
    lastSendMs = millis();
    lastPot = iotbot.potentiometerRead();
    lastB3Sent = iotbot.button3Read();
    CodlaiESPNowMessage outgoing = {}; // Tüm alanlar sıfır / all fields zero
    outgoing.deviceType = 40;          // 40 = IOTBOT (örnek kart kimliği / example board id; MINIBOT = 41)
    outgoing.axis1 = lastPot;          // Potansiyometre (0-4095) / potentiometer (0-4095)
    outgoing.axis2 = 0;
    outgoing.axis3 = 0;
    outgoing.gripper = 0;
    outgoing.action = lastB3Sent ? 1 : 0; // 1 = B3 basılı / 1 = B3 pressed
    iotbot.sendESPNow(kPeerMac, (const uint8_t *)&outgoing, sizeof(outgoing));
    sentCount++;
    drawScreen();
  }

  // 2) Alış: MINIBOT'tan gelen veri / receiving: data from the MINIBOT
  if (iotbot.newData) {
    iotbot.newData = false;
    receivedCount++;
    bool pressed = iotbot.receivedData.action == 1;
    minibotCounter = iotbot.receivedData.axis1;
    lastReceiveMs = millis();

    iotbot.serialWrite(String(L("MiniBot'tan alındı -> sayaç: ", "Received from MiniBot -> counter: ")) + minibotCounter +
                       L(", buton: ", ", button: ") + (pressed ? L("BASILI", "PRESSED") : L("serbest", "released")));

    // MiniBot'un butonuna basıldığı an kısa bir bip / short beep when the MiniBot's button gets pressed
    if (pressed && !minibotButtonPressed) iotbot.buzzerPlayTone(1200, 60);
    minibotButtonPressed = pressed;
    drawScreen();
  }

  // 3) Seri komutlar / serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
}
