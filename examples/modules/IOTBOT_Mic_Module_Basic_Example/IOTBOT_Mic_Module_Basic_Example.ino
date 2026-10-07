/*
 * TR: MİKROFON (SES SENSÖRÜ) MODÜLÜ
 *  - Mikrofon sesi, bir orta değer etrafında salınan bir dalga olarak verir.
 *    Program 300 ms boyunca en büyük ve en küçük değeri ölçer; aradaki fark o
 *    anki "ses seviyesi"dir. Sessizken küçük, alkışta büyük olur.
 *  - Ses seviyesi eşiği geçince LCD'de "YÜKSEK SES!" yazar ve seri porta uyarı
 *    gönderir. Seviye her saniye seri porta yazılır.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim    / help          -> komut listesi
 *      oku       / read          -> seviyeyi ve ham değeri şimdi yaz
 *      esik 500  / threshold 500 -> yüksek ses eşiğini ayarla
 *      dil       / lang          -> dili değiştir (Türkçe <-> English)
 *
 * EN: MICROPHONE (SOUND SENSOR) MODULE
 *  - The microphone gives sound as a wave swinging around a middle value. For
 *    300 ms the program measures the largest and smallest value; their
 *    difference is the current "sound level". Small in silence, big for a clap.
 *  - When the level passes the threshold the LCD shows "LOUD SOUND!" and a
 *    warning is printed. The level is printed to the serial port every second.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help          / yardim   -> command list
 *      read          / oku      -> print the level and the raw value now
 *      threshold 500 / esik 500 -> set the loud sound threshold
 *      lang          / dil      -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Mikrofon modülünü IO27'ye bağlı sokete takın.
 * Desteklenen pinler / Supported pins: IO25 - IO26 - IO27 - IO32 - IO33
 */

#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

#define SENSOR_PIN IO27 // Mikrofonun bağlı olduğu pin / Pin the microphone is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

int threshold = 500;      // Yüksek ses eşiği (seviye) / loud sound threshold (level)
int level = 0;            // Son ölçülen ses seviyesi / last measured sound level
int winMin = 4095, winMax = 0; // Ölçüm penceresindeki en küçük/büyük / min/max in the window
uint32_t windowStartMs = 0;
uint32_t lastPrintMs = 0;
bool loud = false;

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
  iotbot.serialWrite(L("---- MİKROFON - Komutlar ----", "---- MICROPHONE - Commands ----"));
  iotbot.serialWrite(L("  yardim   : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oku      : seviyeyi ve ham değeri yaz", "  read          : print the level and the raw value"));
  iotbot.serialWrite(L("  esik 500 : yüksek ses eşiği", "  threshold 500 : loud sound threshold"));
  iotbot.serialWrite(L("  dil      : English'e geç", "  lang          : switch to Turkish"));
}

void drawScreen() {
  char line[41];
  lcdRow(0, L("  MİKROFON MODÜLÜ", " MICROPHONE MODULE"));
  snprintf(line, sizeof(line), L("Ses seviyesi: %d", "Sound level: %d"), level);
  lcdRow(1, line);
  snprintf(line, sizeof(line), L("Eşik: %d", "Threshold: %d"), threshold);
  lcdRow(2, line);
  lcdRow(3, loud ? L("  >> YÜKSEK SES! <<", "  >> LOUD SOUND! <<") : L("  Ses normal", "  Sound normal"));
}

void printLevel() {
  char msg[80];
  snprintf(msg, sizeof(msg), L("Ses seviyesi: %d (eşik %d) | ham: %d", "Sound level: %d (threshold %d) | raw: %d"),
           level, threshold, iotbot.moduleMicRead(SENSOR_PIN));
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
    threshold = constrain(value, 10, 4095);
    iotbot.serialWrite(String(L("Yeni eşik: ", "New threshold: ")) + threshold);
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
  drawScreen();
  iotbot.serialWrite(L("Mikrofon testi başladı. Alkışlayın veya konuşun!", "Microphone test started. Clap or talk!"));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) Her loop'ta örnek al, pencerenin en küçük/büyük değerini sakla (kısa alkışı kaçırmaz)
  // 2) Sample every loop, keep the window's min/max (does not miss a short clap)
  int raw = iotbot.moduleMicRead(SENSOR_PIN);
  if (raw < winMin) winMin = raw;
  if (raw > winMax) winMax = raw;

  // 3) 300 ms'lik pencere bitti: seviye = en büyük - en küçük
  // 3) The 300 ms window is over: level = largest - smallest
  if (now - windowStartMs < 300) return;
  windowStartMs = now;
  level = winMax - winMin;
  winMin = 4095;
  winMax = 0;

  bool isLoud = level >= threshold;
  if (isLoud && !loud) iotbot.serialWrite(String(L("UYARI: Yüksek ses! Seviye: ", "WARNING: Loud sound! Level: ")) + level);
  loud = isLoud;
  drawScreen();

  if (now - lastPrintMs >= 1000) {
    lastPrintMs = now;
    printLevel();
  }
}
