/*
 * TR: DUMAN / GAZ SENSÖRÜ MODÜLÜ
 *  - Duman/gaz seviyesini (0-4095) okur; LCD'de ve her saniye seri portta gösterir.
 *  - Seviye eşiği geçince LCD'de "DUMAN VAR!" yazar, seri porta uyarı gönderir
 *    ve bip çalar. Seviye eşiğin altına inince "Duman yok"a döner.
 *  - Not: Bu tür (MQ) sensörler açıldıktan sonra 1-2 dakika ısınır; ilk
 *    değerler yüksek olabilir. Temiz havadaki değere bakıp eşiği ayarlayın.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim     / help           -> komut listesi
 *      oku        / read           -> seviyeyi şimdi yaz
 *      esik 1400  / threshold 1400 -> alarm eşiğini ayarla
 *      dil        / lang           -> dili değiştir (Türkçe <-> English)
 *
 * EN: SMOKE / GAS SENSOR MODULE
 *  - Reads the smoke/gas level (0-4095); shows it on the LCD and prints it to
 *    the serial port every second.
 *  - When the level passes the threshold the LCD shows "SMOKE!", a warning is
 *    printed and it beeps. When the level drops below, it returns to "No smoke".
 *  - Note: these (MQ) sensors warm up for 1-2 minutes after power-on; the
 *    first values may be high. Check the clean-air value and set the threshold.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help           / yardim    -> command list
 *      read           / oku       -> print the level now
 *      threshold 1400 / esik 1400 -> set the alarm threshold
 *      lang           / dil       -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Duman sensörü modülünü IO27'ye bağlı sokete takın.
 * Desteklenen pinler / Supported pins: IO25 - IO26 - IO27 - IO32 - IO33
 */

#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

#define SENSOR_PIN IO27 // Duman sensörünün bağlı olduğu pin / Pin the smoke sensor is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

int threshold = 1400;     // Alarm eşiği / alarm threshold
int smokeLevel = 0;       // Son okunan seviye / last level read
bool alarmOn = false;
uint32_t lastReadMs = 0;
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
  iotbot.serialWrite(L("---- DUMAN SENSÖRÜ - Komutlar ----", "---- SMOKE SENSOR - Commands ----"));
  iotbot.serialWrite(L("  yardim    : bu liste", "  help           : this list"));
  iotbot.serialWrite(L("  oku       : seviyeyi şimdi yaz", "  read           : print the level now"));
  iotbot.serialWrite(L("  esik 1400 : alarm eşiği (0-4095)", "  threshold 1400 : alarm threshold (0-4095)"));
  iotbot.serialWrite(L("  dil       : English'e geç", "  lang           : switch to Turkish"));
}

void drawScreen() {
  char line[41];
  lcdRow(0, L("   DUMAN SENSÖRÜ", "    SMOKE SENSOR"));
  snprintf(line, sizeof(line), L("Seviye: %d", "Level: %d"), smokeLevel);
  lcdRow(1, line);
  snprintf(line, sizeof(line), L("Eşik  : %d", "Threshold: %d"), threshold);
  lcdRow(2, line);
  lcdRow(3, alarmOn ? L("  >> DUMAN VAR! <<", "   >> SMOKE! <<") : L("  Duman yok", "  No smoke"));
}

void printLevel() {
  char msg[80];
  snprintf(msg, sizeof(msg), L("Duman seviyesi: %d (eşik %d) - %s", "Smoke level: %d (threshold %d) - %s"),
           smokeLevel, threshold, alarmOn ? L("DUMAN VAR", "SMOKE") : L("duman yok", "no smoke"));
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
    threshold = constrain(value, 0, 4095);
    iotbot.serialWrite(String(L("Yeni eşik: ", "New threshold: ")) + threshold);
    drawScreen();
  } else if (word == "dil" || word == "lang" || word == "language") {
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
  iotbot.serialWrite(L("Duman sensörü testi başladı. Sensör 1-2 dk ısınabilir.", "Smoke sensor test started. The sensor may need 1-2 min to warm up."));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) Sensörü 250 ms'de bir oku ve LCD'yi yenile / read the sensor every 250 ms and refresh the LCD
  if (now - lastReadMs < 250) return;
  lastReadMs = now;
  smokeLevel = iotbot.moduleSmokeRead(SENSOR_PIN);

  // Eşiği geçince bir kez uyar (sürekli bip yok) / warn once when it passes the threshold (no endless beeping)
  bool isSmoke = smokeLevel > threshold;
  if (isSmoke != alarmOn) {
    alarmOn = isSmoke;
    if (alarmOn) {
      iotbot.serialWrite(String(L("UYARI: Duman algılandı! Seviye: ", "WARNING: Smoke detected! Level: ")) + smokeLevel);
      iotbot.buzzerPlayTone(2000, 150);
    } else {
      iotbot.serialWrite(L("Duman seviyesi normale döndü.", "Smoke level back to normal."));
    }
  }
  drawScreen();

  // 3) Seri port: saniyede bir / Serial: once a second
  if (now - lastPrintMs >= 1000) {
    lastPrintMs = now;
    printLevel();
  }
}
