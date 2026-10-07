/*
 * TR: TİTREŞİM SENSÖRÜ MODÜLÜ
 *  - Titreşim seviyesini (0-4095) ölçer. Darbe çok kısa sürebilir; program bu
 *    yüzden her 300 ms içindeki EN YÜKSEK değeri alır, böylece kısa bir
 *    vuruşu kaçırmaz.
 *  - Seviye eşiği geçince LCD'de "TİTREŞİM VAR!" yazar, bip çalar ve sayar.
 *    Seviye saniyede bir seri porta yazılır.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim     / help           -> komut listesi
 *      oku        / read           -> seviyeyi şimdi yaz
 *      esik 1000  / threshold 1000 -> titreşim eşiğini ayarla
 *      sifirla    / reset          -> sayacı sıfırla
 *      dil        / lang           -> dili değiştir (Türkçe <-> English)
 *
 * EN: VIBRATION SENSOR MODULE
 *  - Measures the vibration level (0-4095). A knock can be very short, so the
 *    program takes the HIGHEST value in every 300 ms and never misses a short tap.
 *  - When the level passes the threshold the LCD shows "VIBRATION!", it beeps
 *    and counts. The level is printed to the serial port every second.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help           / yardim    -> command list
 *      read           / oku       -> print the level now
 *      threshold 1000 / esik 1000 -> set the vibration threshold
 *      reset          / sifirla   -> reset the counter
 *      lang           / dil       -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Titreşim sensörü modülünü IO27'ye bağlı sokete takın.
 * Desteklenen pinler / Supported pins: IO25 - IO26 - IO27 - IO32 - IO33
 */

#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

#define SENSOR_PIN IO27 // Titreşim sensörünün bağlı olduğu pin / Pin the vibration sensor is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

int threshold = 1000;     // Titreşim eşiği / vibration threshold
int level = 0;            // Son 300 ms'nin en yüksek değeri / highest value of the last 300 ms
int windowMax = 0;        // Şu anki penceredeki en yüksek / highest in the current window
int hitCount = 0;         // Kaç titreşim algılandı / how many vibrations detected
bool shaking = false;
uint32_t windowStartMs = 0;
uint32_t lastPrintMs = 0;

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
  iotbot.serialWrite(L("---- TİTREŞİM SENSÖRÜ - Komutlar ----", "---- VIBRATION SENSOR - Commands ----"));
  iotbot.serialWrite(L("  yardim    : bu liste", "  help           : this list"));
  iotbot.serialWrite(L("  oku       : seviyeyi şimdi yaz", "  read           : print the level now"));
  iotbot.serialWrite(L("  esik 1000 : titreşim eşiği (0-4095)", "  threshold 1000 : vibration threshold (0-4095)"));
  iotbot.serialWrite(L("  sifirla   : sayacı sıfırla", "  reset          : reset the counter"));
  iotbot.serialWrite(L("  dil       : English'e geç", "  lang           : switch to Turkish"));
}

void drawScreen() {
  char line[41];
  lcdRow(0, L(" TİTREŞİM SENSÖRÜ", " VIBRATION SENSOR"));
  snprintf(line, sizeof(line), L("Seviye: %d", "Level: %d"), level);
  lcdRow(1, line);
  snprintf(line, sizeof(line), L("Eşik:%d  Sayı:%d", "Thr:%d  Count:%d"), threshold, hitCount);
  lcdRow(2, line);
  lcdRow(3, shaking ? L(" >> TİTREŞİM VAR! <<", "  >> VIBRATION! <<") : L("  Sakin", "  Calm"));
}

void printLevel() {
  char msg[80];
  snprintf(msg, sizeof(msg), L("Titreşim seviyesi: %d (eşik %d) | algılama: %d", "Vibration level: %d (threshold %d) | detections: %d"),
           level, threshold, hitCount);
  iotbot.serialWrite(msg);
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oku" || word == "read") {
    printLevel();
  } else if ((word == "esik" || word == "threshold") && hasValue) {
    threshold = constrain(value, 1, 4095);
    iotbot.serialWrite(String(L("Yeni eşik: ", "New threshold: ")) + threshold);
  } else if (word == "sifirla" || word == "reset") {
    hitCount = 0;
    iotbot.serialWrite(L("Sayaç sıfırlandı.", "Counter reset."));
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
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
  iotbot.serialWrite(L("Titreşim sensörü testi başladı. Sensöre hafifçe vurun!", "Vibration sensor test started. Tap the sensor gently!"));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) Her loop'ta örnek al, pencerenin en yükseğini sakla / sample every loop, keep the window's highest
  int raw = iotbot.moduleVibrationAnalogRead(SENSOR_PIN);
  if (raw > windowMax) windowMax = raw;

  // 3) 300 ms'lik pencere bitti / the 300 ms window is over
  if (now - windowStartMs < 300) return;
  windowStartMs = now;
  level = windowMax;
  windowMax = 0;

  bool isShaking = level > threshold;
  if (isShaking && !shaking) { // Titreşim yeni başladı / vibration just started
    hitCount++;
    iotbot.serialWrite(String(L("UYARI: Titreşim algılandı! Seviye: ", "WARNING: Vibration detected! Level: ")) + level);
    iotbot.buzzerPlayTone(1800, 50);
  }
  shaking = isShaking;
  drawScreen();

  // 4) Seri port: saniyede bir / Serial: once a second
  if (now - lastPrintMs >= 1000) {
    lastPrintMs = now;
    printLevel();
  }
}
