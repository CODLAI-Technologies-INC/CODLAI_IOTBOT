/*
 * TR: PIR HAREKET SENSÖRÜ MODÜLÜ
 *  - Sensörün önünde bir insan/hayvan hareket edince "HAREKET VAR!" yazar ve
 *    kısa bir bip çalar. Kaç kez hareket algılandığını ve son hareketten beri
 *    geçen süreyi gösterir.
 *  - Durum değiştiğinde seri porta yazar (sürekli yazı akıtmaz).
 *  - Not: PIR sensörü açıldıktan sonra ~30-60 sn ısınır; bu sürede yanlış
 *    algılama yapabilir. Sensör hareketi bir süre (modüldeki ayara göre) "var"
 *    olarak tutar.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help  -> komut listesi
 *      oku     / read  -> durumu yaz
 *      sifirla / reset -> sayacı sıfırla
 *      dil     / lang  -> dili değiştir (Türkçe <-> English)
 *
 * EN: PIR MOTION SENSOR MODULE
 *  - When a person/animal moves in front of the sensor it shows "MOTION!" and
 *    beeps. It shows how many times motion was detected and the time since
 *    the last motion.
 *  - Prints to the serial port only when the state changes (no flooding).
 *  - Note: the PIR sensor warms up for ~30-60 s after power-on and may give
 *    false detections meanwhile. The sensor keeps "motion" on for a while
 *    (depending on the module's setting).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help  / yardim  -> command list
 *      read  / oku     -> print the state
 *      reset / sifirla -> reset the counter
 *      lang  / dil     -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: PIR modülünü IO27'ye bağlı sokete takın.
 * Desteklenen pinler / Supported pins: IO25 - IO26 - IO27 - IO32 - IO33
 */

#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

#define SENSOR_PIN IO27 // PIR sensörünün bağlı olduğu pin / Pin the PIR sensor is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

bool motion = false;        // Şu an hareket var mı? / is there motion now?
int motionCount = 0;        // Kaç kez hareket algılandı / how many motions detected
uint32_t lastMotionMs = 0;  // Son hareketin zamanı / time of the last motion
uint32_t lastReadMs = 0;
uint32_t lastScreenMs = 0;

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
// LCD'ye yazmak yavaştır (~1 ms/harf): sadece DEĞİŞEN satırları yazarız, loop hızlı kalır.
// Writing to the LCD is slow (~1 ms per letter): we write only rows that CHANGED, so loop stays fast.
String shownRows[4];
void lcdRow(int row, const char *text) {
  if (shownRows[row] == text) return;
  shownRows[row] = text;
  iotbot.lcdWriteFixedTxt(0, row, text, 20);
}

void printHelp() {
  iotbot.serialWrite(L("---- PIR HAREKET SENSÖRÜ - Komutlar ----", "---- PIR MOTION SENSOR - Commands ----"));
  iotbot.serialWrite(L("  yardim  : bu liste", "  help    : this list"));
  iotbot.serialWrite(L("  oku     : durumu yaz", "  read    : print the state"));
  iotbot.serialWrite(L("  sifirla : sayacı sıfırla", "  reset   : reset the counter"));
  iotbot.serialWrite(L("  dil     : English'e geç", "  lang    : switch to Turkish"));
}

void drawScreen() {
  char line[41];
  lcdRow(0, L(" PIR HAREKET SENSÖRÜ", " PIR MOTION SENSOR"));
  lcdRow(1, motion ? L("  >> HAREKET VAR! <<", "   >> MOTION! <<") : L("  Hareket yok", "  No motion"));
  snprintf(line, sizeof(line), L("Algılama: %d", "Detections: %d"), motionCount);
  lcdRow(2, line);
  if (motionCount == 0) snprintf(line, sizeof(line), "%s", L("Son hareket: -", "Last motion: -"));
  else {
    // Geçen süre: saniye, dakika veya saat (satır 20 harfi aşmasın) / elapsed: seconds, minutes or hours (fits 20 letters)
    unsigned long s = (millis() - lastMotionMs) / 1000;
    if (s < 60) snprintf(line, sizeof(line), L("Son hareket: %lu sn", "Last motion: %lu s"), s);
    else if (s < 3600) snprintf(line, sizeof(line), L("Son hareket: %lu dk", "Last motion: %lu min"), s / 60);
    else snprintf(line, sizeof(line), L("Son hareket: %lu sa", "Last motion: %lu h"), s / 3600);
  }
  lcdRow(3, line);
}

void printState() {
  char msg[96];
  snprintf(msg, sizeof(msg), L("PIR: %s | Algılama sayısı: %d", "PIR: %s | Detections: %d"),
           motion ? L("HAREKET VAR", "MOTION") : L("hareket yok", "no motion"), motionCount);
  iotbot.serialWrite(msg);
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "oku" || cmd == "read") {
    printState();
  } else if (cmd == "sifirla" || cmd == "reset") {
    motionCount = 0;
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
  drawScreen();
  iotbot.serialWrite(L("PIR testi başladı. Sensör ~30-60 sn ısınabilir.", "PIR test started. The sensor may need ~30-60 s to warm up."));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) Sensörü 50 ms'de bir oku, durum değişince yaz / read every 50 ms, report on change
  if (now - lastReadMs >= 50) {
    lastReadMs = now;
    bool m = iotbot.moduleMotionRead(SENSOR_PIN); // true = hareket / motion
    if (m != motion) {
      motion = m;
      if (motion) {
        motionCount++;
        lastMotionMs = now;
        iotbot.buzzerPlayTone(1500, 60);
      }
      printState();
      lastScreenMs = 0; // Ekranı hemen yenile / refresh the screen now
    }
    if (motion) lastMotionMs = now; // Hareket sürdükçe süre sıfırda kalır / timer stays at 0 while motion lasts
  }

  // 3) LCD (500 ms'de bir, "son hareket" süresi için) / LCD (every 500 ms, for the "last motion" timer)
  if (now - lastScreenMs >= 500) {
    lastScreenMs = now;
    drawScreen();
  }
}
