/*
 * TR: MANYETİK ALAN (HALL) SENSÖR MODÜLÜ
 *  - Sensöre bir mıknatıs yaklaşınca "Tespit edildi!" yazar ve kısa bir bip
 *    çalar; mıknatıs uzaklaşınca "Yok" yazar. Kaç kez algılandığını da sayar.
 *  - Durum değiştiğinde seri porta yazar (sürekli yazı akıtmaz).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help  -> komut listesi
 *      oku     / read  -> durumu ve sayacı yaz
 *      sifirla / reset -> sayacı sıfırla
 *      dil     / lang  -> dili değiştir (Türkçe <-> English)
 *
 * EN: MAGNETIC FIELD (HALL) SENSOR MODULE
 *  - When a magnet comes near the sensor it shows "Detected!" and beeps; when
 *    the magnet goes away it shows "None". It also counts the detections.
 *  - Prints to the serial port only when the state changes (no flooding).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help  / yardim  -> command list
 *      read  / oku     -> print the state and the counter
 *      reset / sifirla -> reset the counter
 *      lang  / dil     -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Manyetik sensör modülünü IO27'ye bağlı sokete takın.
 * Desteklenen pinler / Supported pins: IO25 - IO26 - IO27 - IO32 - IO33
 */

#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

#define SENSOR_PIN IO27 // Manyetik sensörün bağlı olduğu pin / Pin the magnetic sensor is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

bool magnet = false;      // Şu an mıknatıs var mı? / is a magnet there now?
int detectCount = 0;      // Kaç kez algılandı / how many detections
uint32_t lastReadMs = 0;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "AÇI" -> "aci"
// Lower-cases and simplifies Turkish letters: "AÇI" -> "aci"
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
// lcdWriteFixedTxt Türkçe harfleri (ç, ğ, ı, ö, ş, ü) LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- MANYETİK SENSÖR - Komutlar ----", "---- MAGNETIC SENSOR - Commands ----"));
  iotbot.serialWrite(L("  yardim  : bu liste", "  help    : this list"));
  iotbot.serialWrite(L("  oku     : durumu ve sayacı yaz", "  read    : print the state and the counter"));
  iotbot.serialWrite(L("  sifirla : sayacı sıfırla", "  reset   : reset the counter"));
  iotbot.serialWrite(L("  dil     : English'e geç", "  lang    : switch to Turkish"));
}

void drawScreen() {
  char line[41];
  lcdRow(0, L(" MANYETİK SENSÖR", " MAGNETIC SENSOR"));
  lcdRow(1, L("Manyetik alan:", "Magnetic field:"));
  lcdRow(2, magnet ? L("  >> TESPİT EDİLDİ!", "  >> DETECTED!") : L("  yok", "  none"));
  snprintf(line, sizeof(line), L("Algılama: %d", "Detections: %d"), detectCount);
  lcdRow(3, line);
}

void printState() {
  char msg[80];
  snprintf(msg, sizeof(msg), L("Manyetik alan: %s | Algılama sayısı: %d", "Magnetic field: %s | Detections: %d"),
           magnet ? L("TESPİT EDİLDİ", "DETECTED") : L("yok", "none"), detectCount);
  iotbot.serialWrite(msg);
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "oku" || cmd == "read") {
    printState();
  } else if (cmd == "sifirla" || cmd == "reset") {
    detectCount = 0;
    drawScreen();
    iotbot.serialWrite(L("Sayaç sıfırlandı.", "Counter reset."));
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
  iotbot.begin();             // IoTBot başlatılıyor / Initialize IoTBot
  iotbot.serialStart(115200); // Seri haberleşme / Serial communication
  iotbot.lcdClear();
  magnet = iotbot.moduleMagneticRead(SENSOR_PIN);
  drawScreen();
  iotbot.serialWrite(L("Manyetik sensör testi başladı. Bir mıknatıs yaklaştırın!", "Magnetic sensor test started. Bring a magnet close!"));
  printHelp();
  printState();
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) Sensörü 50 ms'de bir oku, sadece durum değişince yaz / read every 50 ms, report only on change
  if (millis() - lastReadMs < 50) return;
  lastReadMs = millis();

  bool now = iotbot.moduleMagneticRead(SENSOR_PIN); // true = mıknatıs var / magnet present
  if (now == magnet) return;
  magnet = now;
  if (magnet) {
    detectCount++;
    iotbot.buzzerPlayTone(1500, 40);
  }
  printState();
  drawScreen();
}
