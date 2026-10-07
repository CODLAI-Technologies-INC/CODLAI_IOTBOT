/*
 * TR: ESP-NOW'A İLK ADIM - en basit kablosuz haberleşme örneği (yayın / broadcast).
 *  - MAC adresi bilmenize GEREK YOK: bu kod her saniye bir sayacı "yayın" olarak
 *    havaya gönderir; ESP-NOW ile dinleyen HERHANGİ bir CODLAI kartı (başka bir IOTBOT,
 *    MINIBOT ya da ROLEBOT - hepsi aynı veri yapısını kullanır) bunu duyabilir.
 *  - Aynı anda hem gönderiyor hem dinliyoruz: başka bir karttan gelen yayın Seri
 *    Monitör'e ve LCD'ye yazılır, kısa bir bip duyulur.
 *  - B3 butonu: sayacı HEMEN gönder (beklemeden).
 *  - Gönderen kart, paketteki deviceType numarasından anlaşılır. Kütüphane örnekleri
 *    40-49 aralığını kullanır: 40 = IOTBOT, 41 = MINIBOT, 42 = ROLEBOT (10, 11, 20...
 *    başka işlere ayrılmıştır - bkz. COMMANDS_README "deviceType haritası").
 *  - Sonra IOTBOT_MiniBot_ESPNOW_Pair_Example.ino ile İKİ BELİRLİ kart arasında
 *    (MAC adresine dayalı) eşleşmeyi öğrenebilirsiniz.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help          -> komut listesi
 *      gonder / send          -> sayacı hemen gönder
 *      sayi 42 / number 42    -> 42 değerini bir kez yayınla
 *      dur    / pause         -> otomatik gönderimi durdur
 *      devam  / resume        -> otomatik gönderime devam et
 *      durum  / status        -> MAC adresi, gönderilen/alınan sayısı
 *      dil    / lang          -> dili değiştir (Türkçe <-> English)
 *
 * EN: FIRST STEP INTO ESP-NOW - the simplest wireless example (broadcast).
 *  - You do NOT need any MAC address: this code broadcasts a counter into the air
 *    every second; ANY CODLAI board listening over ESP-NOW (another IOTBOT, a MINIBOT
 *    or a ROLEBOT - they all share the same data structure) can hear it.
 *  - We send AND listen at the same time: a broadcast from another board is printed
 *    on the Serial Monitor and the LCD, with a short beep.
 *  - B3 button: send the counter RIGHT NOW (without waiting).
 *  - The sending board is recognised by the deviceType number in the packet. Library
 *    examples use the 40-49 range: 40 = IOTBOT, 41 = MINIBOT, 42 = ROLEBOT (10, 11,
 *    20... are reserved for other uses - see the COMMANDS_README "deviceType map").
 *  - Later, see IOTBOT_MiniBot_ESPNOW_Pair_Example.ino to pair TWO SPECIFIC boards
 *    (using their MAC addresses).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim        -> command list
 *      send   / gonder        -> send the counter now
 *      number 42 / sayi 42    -> broadcast the value 42 once
 *      pause  / dur           -> stop the automatic sending
 *      resume / devam         -> continue the automatic sending
 *      status / durum         -> MAC address, sent/received counts
 *      lang   / dil           -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. / NO extra module needed.
 * Not / Note: ESP-NOW açıkken B1/B2 ve joystick X kullanılamaz; bu örnek B3 kullanır.
 *             With ESP-NOW on, B1/B2 and joystick X cannot be used; this example uses B3.
 */

#define USE_ESPNOW
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Herkese / to everyone

const uint32_t kSendIntervalMs = 1000; // Her saniye gönder / send every second
uint32_t counter = 0;
uint32_t lastSendMs = 0;
bool autoSend = true;   // Otomatik gönderim açık mı? / is automatic sending on?
int receivedCount = 0;
bool lastB3 = false;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "GÖNDER" -> "gonder"
// Lower-cases and simplifies Turkish letters: "GÖNDER" -> "gonder"
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
  lcdRow(0, L("ESP-NOW YAYIN", "ESP-NOW BROADCAST"));
  lcdRow(3, L("B3: hemen gönder", "B3: send now"));
}

void printHelp() {
  iotbot.serialWrite(L("---- ESP-NOW YAYIN - Komutlar ----", "---- ESP-NOW BROADCAST - Commands ----"));
  iotbot.serialWrite(L("  yardim     : bu liste", "  help       : this list"));
  iotbot.serialWrite(L("  gonder     : sayacı hemen gönder", "  send       : send the counter now"));
  iotbot.serialWrite(L("  sayi 42    : 42 değerini bir kez yayınla", "  number 42  : broadcast the value 42 once"));
  iotbot.serialWrite(L("  dur / devam: otomatik gönderimi durdur / sürdür", "  pause / resume : stop / continue auto sending"));
  iotbot.serialWrite(L("  durum      : MAC, gönderilen/alınan", "  status     : MAC, sent/received"));
  iotbot.serialWrite(L("  dil        : English'e geç", "  lang       : switch to Turkish"));
}

// Bir değeri yayınlar. Paket yapısı diğer kartlarla AYNI kalmalı (deviceType 40 = IOTBOT).
// Broadcasts a value. The packet layout must stay the SAME as on the other boards (deviceType 40 = IOTBOT).
void broadcastValue(int value) {
  CodlaiESPNowMessage outgoing = {}; // Tüm alanlar sıfır / all fields zero
  outgoing.deviceType = 40;          // 40 = IOTBOT (örnek kart kimliği / example board id; 41 MINIBOT, 42 ROLEBOT)
  outgoing.axis1 = value;
  outgoing.axis2 = 0;
  outgoing.axis3 = 0;
  outgoing.gripper = 0;
  outgoing.action = 0;
  iotbot.sendESPNow(broadcastAddress, (const uint8_t *)&outgoing, sizeof(outgoing));

  char line[41];
  snprintf(line, sizeof(line), L("Gönderilen: %d", "Sent: %d"), value);
  lcdRow(1, line);
}

void sendCounter() {
  counter++;
  lastSendMs = millis();
  broadcastValue((int)counter);
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "gonder" || word == "send") {
    sendCounter();
    iotbot.serialWrite(String(L("Sayaç gönderildi: ", "Counter sent: ")) + counter);
  } else if ((word == "sayi" || word == "number") && hasValue) {
    broadcastValue(value);
    iotbot.serialWrite(String(L("Yayınlandı: ", "Broadcast: ")) + value);
  } else if (word == "dur" || word == "pause" || word == "stop") {
    autoSend = false;
    iotbot.serialWrite(L("Otomatik gönderim DURDU (dinleme sürüyor).", "Automatic sending PAUSED (still listening)."));
  } else if (word == "devam" || word == "resume") {
    autoSend = true;
    iotbot.serialWrite(L("Otomatik gönderim devam ediyor.", "Automatic sending resumed."));
  } else if (word == "durum" || word == "status") {
    iotbot.serialWrite(String(L("Benim MAC adresim: ", "My MAC address: ")) + WiFi.macAddress());
    iotbot.serialWrite(String(L("Gönderilen sayaç: ", "Counter sent: ")) + counter + L("   Alınan yayın: ", "   Broadcasts received: ") + receivedCount +
                       L("   Otomatik: ", "   Auto: ") + (autoSend ? L("açık", "on") : L("kapalı", "off")));
  } else if (word == "dil" || word == "lang" || word == "language") {
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
  iotbot.lcdClear();
  drawStaticScreen();
  lcdRow(1, L("Başlatılıyor...", "Starting..."));

  iotbot.initESPNow();
  iotbot.startListening(); // Gelen HERHANGİ bir yayını iotbot.receivedData'ya yazar / stores ANY broadcast in iotbot.receivedData

  iotbot.serialWrite(L("Yayın modu hazır - herkese açığız!", "Broadcast mode ready - open to everyone!"));
  iotbot.serialWrite(String(L("Benim MAC adresim: ", "My MAC address: ")) + WiFi.macAddress());
  printHelp();
}

void loop() {
  // 1) Gönderim: her saniye sayacı yayınla / sending: broadcast the counter every second
  if (autoSend && millis() - lastSendMs >= kSendIntervalMs) sendCounter();

  // 2) B3 = hemen gönder (sadece basıldığı an) / B3 = send now (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) {
    sendCounter();
    iotbot.serialWrite(String(L("B3: sayaç gönderildi: ", "B3: counter sent: ")) + counter);
  }
  lastB3 = b3;

  // 3) Alış: başka bir karttan gelen HERHANGİ bir yayın / receiving: ANY broadcast from another board
  if (iotbot.newData) {
    iotbot.newData = false;
    receivedCount++;
    const char *senderName = "?";
    switch (iotbot.receivedData.deviceType) {
      case 40: senderName = "IOTBOT"; break;  // 40-42: örnek kart kimlikleri / example board ids
      case 41: senderName = "MINIBOT"; break;
      case 42: senderName = "ROLEBOT"; break;
      default: break;
    }
    char line[41];
    snprintf(line, sizeof(line), L("Alınan: %s %d", "Got: %s %d"), senderName, iotbot.receivedData.axis1);
    lcdRow(2, line);
    iotbot.serialWrite(String(L("Yayın alındı -> gönderen: ", "Broadcast received -> from: ")) + senderName +
                       L(", değer: ", ", value: ") + iotbot.receivedData.axis1);
    iotbot.buzzerPlayTone(1000, 40);
  }

  // 4) Seri komutlar / serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
}
