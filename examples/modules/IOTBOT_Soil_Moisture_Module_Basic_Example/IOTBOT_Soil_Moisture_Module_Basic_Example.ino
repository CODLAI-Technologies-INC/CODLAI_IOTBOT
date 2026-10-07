/*
 * TR: TOPRAK NEM SENSÖRÜ MODÜLÜ
 *  - Toprağın nem değerini (0-4095) okur; LCD'de ve her saniye seri portta gösterir.
 *  - Değer eşiğin ALTINA inince toprak kurudu demektir: LCD'de "TOPRAK KURU!"
 *    yazar, seri porta uyarı gönderir ve bip çalar.
 *  - İpucu: sensörü önce kuru, sonra ıslak toprağa batırıp değerlere bakın;
 *    eşiği ikisinin arasına ayarlayın.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim    / help          -> komut listesi
 *      oku       / read          -> değeri şimdi yaz
 *      esik 500  / threshold 500 -> kuruluk eşiğini ayarla
 *      dil       / lang          -> dili değiştir (Türkçe <-> English)
 *
 * EN: SOIL MOISTURE SENSOR MODULE
 *  - Reads the soil's moisture value (0-4095); shows it on the LCD and prints
 *    it to the serial port every second.
 *  - When the value drops BELOW the threshold the soil is dry: the LCD shows
 *    "SOIL IS DRY!", a warning is printed and it beeps.
 *  - Tip: put the sensor first in dry, then in wet soil and look at the values;
 *    set the threshold between them.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help          / yardim   -> command list
 *      read          / oku      -> print the value now
 *      threshold 500 / esik 500 -> set the dryness threshold
 *      lang          / dil      -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Toprak nem sensörünü IO27'ye bağlı sokete takın.
 * Desteklenen pinler / Supported pins: IO25 - IO26 - IO27 - IO32 - IO33
 */

#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

#define SENSOR_PIN IO27 // Toprak nem sensörünün bağlı olduğu pin / Pin the soil moisture sensor is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

int threshold = 500;      // Bunun altı = kuru / below this = dry
int moisture = 0;         // Son okunan değer / last value read
bool dry = false;
bool firstRead = true;
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
  iotbot.serialWrite(L("---- TOPRAK NEM SENSÖRÜ - Komutlar ----", "---- SOIL MOISTURE SENSOR - Commands ----"));
  iotbot.serialWrite(L("  yardim   : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oku      : değeri şimdi yaz", "  read          : print the value now"));
  iotbot.serialWrite(L("  esik 500 : kuruluk eşiği (0-4095)", "  threshold 500 : dryness threshold (0-4095)"));
  iotbot.serialWrite(L("  dil      : English'e geç", "  lang          : switch to Turkish"));
}

void drawScreen() {
  char line[41];
  lcdRow(0, L("  TOPRAK NEM SENSÖRÜ", " SOIL MOISTURE"));
  snprintf(line, sizeof(line), L("Nem değeri: %d", "Moisture: %d"), moisture);
  lcdRow(1, line);
  snprintf(line, sizeof(line), L("Eşik (kuru): %d", "Dry below: %d"), threshold);
  lcdRow(2, line);
  lcdRow(3, dry ? L(" >> TOPRAK KURU! <<", " >> SOIL IS DRY! <<") : L("  Toprak nemli", "  Soil is moist"));
}

void printValue() {
  char msg[80];
  snprintf(msg, sizeof(msg), L("Toprak nemi: %d (eşik %d) - %s", "Soil moisture: %d (threshold %d) - %s"),
           moisture, threshold, dry ? L("KURU", "DRY") : L("nemli", "moist"));
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
    printValue();
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
  iotbot.serialWrite(L("Toprak nem testi başladı. Sensörü toprağa batırın.", "Soil moisture test started. Put the sensor into the soil."));
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
  moisture = iotbot.moduleSoilMoistureRead(SENSOR_PIN);

  // Kuru/nemli durumu değişince bir kez uyar / warn once when dry/moist changes
  bool isDry = moisture < threshold;
  if (isDry != dry || firstRead) {
    dry = isDry;
    firstRead = false;
    if (dry) {
      iotbot.serialWrite(String(L("UYARI: Toprak kuru! Değer: ", "WARNING: Soil is dry! Value: ")) + moisture);
      iotbot.buzzerPlayTone(1000, 150);
    } else {
      iotbot.serialWrite(L("Toprak yeterince nemli.", "Soil is moist enough."));
    }
  }
  drawScreen();

  // 3) Seri port: saniyede bir / Serial: once a second
  if (now - lastPrintMs >= 1000) {
    lastPrintMs = now;
    printValue();
  }
}
