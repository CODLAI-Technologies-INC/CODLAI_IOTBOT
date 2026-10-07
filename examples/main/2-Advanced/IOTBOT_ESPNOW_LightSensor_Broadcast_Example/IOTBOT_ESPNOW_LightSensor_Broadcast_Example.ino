/*
 * TR: KABLOSUZ AKILLI EV FİKRİ - Işık sensörü yayını
 *  - IOTBOT'un üzerindeki ışık sensörünün (LDR) değerini sürekli olarak ESP-NOW ile
 *    yayınlar (broadcast). Bu tek başına bir şey yapmaz ama aynı odadaki BAŞKA kartlar
 *    bu veriyi dinleyip kendi kararlarını verebilir - ör. "hava kararınca lambayı aç".
 *  - Bkz. MINIBOT_ESPNOW_NightLight_Reactive_Example.ino ve ROLEBOT_ESPNOW_NightLight_
 *    Reactive_Example.ino: onları çalıştırın, IOTBOT'un ışık sensörünü elinizle
 *    kapatın, uzaktaki LED/lamba kendiliğinden yanacak!
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help    -> komut listesi
 *      oku    / read    -> ışık değerini şimdi yazdır
 *      hizli  / fast    -> hızlı yayın (100 ms) <-> normal (500 ms)
 *      dur    / pause   -> yayını durdur       devam / resume -> yayına devam et
 *      durum  / status  -> MAC adresi, yayın aralığı, gönderilen paket
 *      dil    / lang    -> dili değiştir (Türkçe <-> English)
 *
 * EN: A WIRELESS SMART HOME IDEA - Light sensor broadcast
 *  - Continuously broadcasts the reading of the IOTBOT's light sensor (LDR) over
 *    ESP-NOW. By itself this does nothing, but OTHER boards in the room can listen to
 *    it and make their own decisions - e.g. "turn on the lamp when it gets dark".
 *  - See MINIBOT_ESPNOW_NightLight_Reactive_Example.ino and ROLEBOT_ESPNOW_NightLight_
 *    Reactive_Example.ino: run one of them, cover the IOTBOT's light sensor with your
 *    hand and watch the remote LED/lamp turn on by itself!
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim  -> command list
 *      read   / oku     -> print the light value now
 *      fast   / hizli   -> fast broadcast (100 ms) <-> normal (500 ms)
 *      pause  / dur     -> stop broadcasting     resume / devam -> continue
 *      status / durum   -> MAC address, interval, packets sent
 *      lang   / dil     -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ - LDR kartın üzerindedir. / NO extra module
 * needed - the LDR is on the board.
 */

#define USE_ESPNOW
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Herkese / to everyone

uint32_t sendIntervalMs = 500; // Yayın aralığı / broadcast interval
uint32_t lastSendMs = 0;
uint32_t packetCount = 0;
bool sending = true;
int lightValue = 0;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "HIZLI" -> "hizli"
// Lower-cases and simplifies Turkish letters: "HIZLI" -> "hizli"
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

void drawStaticScreen() {
  lcdRow(0, L("IŞIK SENSÖRÜ YAYINI", "LIGHT SENSOR BCAST"));
  lcdRow(2, L("Uzaktaki kartlar", "Remote boards can"));
  lcdRow(3, L("bunu dinleyebilir", "listen to this"));
}

void printHelp() {
  iotbot.serialWrite(L("---- IŞIK SENSÖRÜ YAYINI - Komutlar ----", "---- LIGHT SENSOR BROADCAST - Commands ----"));
  iotbot.serialWrite(L("  yardim : bu liste", "  help   : this list"));
  iotbot.serialWrite(L("  oku    : ışık değerini yazdır", "  read   : print the light value"));
  iotbot.serialWrite(L("  hizli  : hızlı (100 ms) <-> normal (500 ms)", "  fast   : fast (100 ms) <-> normal (500 ms)"));
  iotbot.serialWrite(L("  dur / devam : yayını durdur / sürdür", "  pause / resume : stop / continue broadcasting"));
  iotbot.serialWrite(L("  durum  : MAC, aralık, paket sayısı", "  status : MAC, interval, packet count"));
  iotbot.serialWrite(L("  dil    : English'e geç", "  lang   : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "oku" || cmd == "read") {
    iotbot.serialWrite(String(L("Işık (LDR): ", "Light (LDR): ")) + iotbot.ldrRead() + L("  (0-4095, aydınlık = yüksek)", "  (0-4095, bright = high)"));
  } else if (cmd == "hizli" || cmd == "fast") {
    sendIntervalMs = (sendIntervalMs == 500) ? 100 : 500;
    iotbot.serialWrite(String(L("Yayın aralığı: ", "Broadcast interval: ")) + sendIntervalMs + " ms");
  } else if (cmd == "dur" || cmd == "pause" || cmd == "stop") {
    sending = false;
    lcdRow(1, L("Yayın DURDU", "Broadcast PAUSED"));
    iotbot.serialWrite(L("Yayın durduruldu.", "Broadcast paused."));
  } else if (cmd == "devam" || cmd == "resume") {
    sending = true;
    iotbot.serialWrite(L("Yayın devam ediyor.", "Broadcast resumed."));
  } else if (cmd == "durum" || cmd == "status") {
    iotbot.serialWrite(String(L("Benim MAC adresim: ", "My MAC address: ")) + WiFi.macAddress());
    iotbot.serialWrite(String(L("Yayın: ", "Broadcast: ")) + (sending ? L("açık", "on") : L("kapalı", "off")) +
                       L("   Aralık: ", "   Interval: ") + sendIntervalMs + L(" ms   Gönderilen paket: ", " ms   Packets sent: ") + packetCount);
  } else if (cmd == "dil" || cmd == "lang" || cmd == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    drawStaticScreen();
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.initESPNow();
  iotbot.lcdClear();
  drawStaticScreen();
  iotbot.serialWrite(L("Işık sensörü yayını başladı.", "Light sensor broadcast started."));
  printHelp();
}

void loop() {
  if (sending && millis() - lastSendMs >= sendIntervalMs) {
    lastSendMs = millis();
    lightValue = iotbot.ldrRead();

    // Paket yapısı alıcılarla AYNI kalmalı: deviceType 10, axis1 = ışık
    // The packet layout must stay the SAME as on the receivers: deviceType 10, axis1 = light
    CodlaiESPNowMessage outgoing = {}; // Tüm alanlar sıfır / all fields zero
    outgoing.deviceType = 10;          // 10 = IOTBOT (bu örnekte kullanılan kimlik / id used in this example)
    outgoing.axis1 = lightValue;       // Işık değeri / light value
    outgoing.axis2 = 0;
    outgoing.axis3 = 0;
    outgoing.gripper = 0;
    outgoing.action = 0;
    iotbot.sendESPNow(broadcastAddress, (const uint8_t *)&outgoing, sizeof(outgoing));
    packetCount++;

    char line[41];
    snprintf(line, sizeof(line), L("Işık: %d", "Light: %d"), lightValue);
    lcdRow(1, line);
  }

  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
}
