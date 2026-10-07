/*
 * TR: ULTRASONİK MESAFE SENSÖRÜ (HC-SR04) MODÜLÜ
 *  - Önündeki cismin uzaklığını (cm) ölçer; LCD'de bir çubukla gösterir ve
 *    seri porta yazar.
 *  - Cisim eşikten (varsayılan 10 cm) yakınsa "ÇOK YAKIN!" yazar ve bip çalar.
 *  - Önünde cisim yoksa veya 4 metreden uzaksa "Menzil dışı" yazar.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help         -> komut listesi
 *      oku     / read         -> mesafeyi şimdi yaz
 *      esik 15 / threshold 15 -> "çok yakın" mesafesini (cm) ayarla
 *      dil     / lang         -> dili değiştir (Türkçe <-> English)
 *
 * EN: ULTRASONIC DISTANCE SENSOR (HC-SR04) MODULE
 *  - Measures the distance (cm) to the object in front of it; shows it on the
 *    LCD with a bar and prints it to the serial port.
 *  - If the object is closer than the threshold (default 10 cm) it shows
 *    "TOO CLOSE!" and beeps.
 *  - If there is no object or it is farther than 4 metres it shows "Out of range".
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help         / yardim  -> command list
 *      read         / oku     -> print the distance now
 *      threshold 15 / esik 15 -> set the "too close" distance (cm)
 *      lang         / dil     -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Sensör sabit pinleri kullanır: TRIG = IO27, ECHO = IO32.
 *                    The sensor uses fixed pins: TRIG = IO27, ECHO = IO32.
 */

#define USE_HCSR04 // Şu an zorunlu değil, uyumluluk için duruyor / not required now, kept for compatibility
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

int closeCm = 10;         // Bundan yakın = "çok yakın" / closer than this = "too close"
int distance = 0;         // Son ölçüm (cm), 0 = menzil dışı / last measurement (cm), 0 = out of range
bool tooClose = false;
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
  iotbot.serialWrite(L("---- ULTRASONİK MESAFE - Komutlar ----", "---- ULTRASONIC DISTANCE - Commands ----"));
  iotbot.serialWrite(L("  yardim  : bu liste", "  help         : this list"));
  iotbot.serialWrite(L("  oku     : mesafeyi şimdi yaz", "  read         : print the distance now"));
  iotbot.serialWrite(L("  esik 15 : \"çok yakın\" mesafesi (cm)", "  threshold 15 : \"too close\" distance (cm)"));
  iotbot.serialWrite(L("  dil     : English'e geç", "  lang         : switch to Turkish"));
}

void drawScreen() {
  char line[41];
  lcdRow(0, L(" ULTRASONİK MESAFE", " ULTRASONIC DISTANCE"));
  if (distance == 0) {
    lcdRow(1, L("Mesafe: menzil dışı", "Distance: no echo"));
    lcdRow(2, "");
  } else {
    snprintf(line, sizeof(line), L("Mesafe: %d cm", "Distance: %d cm"), distance);
    lcdRow(1, line);
    // Çubuk: 0-100 cm, her kutu 5 cm (0xFF = dolu kutu) / Bar: 0-100 cm, each block 5 cm (0xFF = full block)
    int blocks = constrain(distance / 5, 0, 20);
    for (int i = 0; i < 20; i++) line[i] = (i < blocks) ? '\xFF' : ' ';
    line[20] = '\0';
    lcdRow(2, line);
  }
  lcdRow(3, tooClose ? L("  >> ÇOK YAKIN! <<", "  >> TOO CLOSE! <<") : L("  Güvenli mesafe", "  Safe distance"));
}

void printDistance() {
  if (distance == 0) iotbot.serialWrite(L("Mesafe: menzil dışı (cisim yok veya 4 m'den uzak)", "Distance: out of range (no object or farther than 4 m)"));
  else {
    char msg[64];
    snprintf(msg, sizeof(msg), L("Mesafe: %d cm%s", "Distance: %d cm%s"), distance, tooClose ? L("  - ÇOK YAKIN!", "  - TOO CLOSE!") : "");
    iotbot.serialWrite(msg);
  }
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oku" || word == "read") {
    printDistance();
  } else if ((word == "esik" || word == "threshold") && hasValue) {
    closeCm = constrain(value, 2, 400);
    iotbot.serialWrite(String(L("\"Çok yakın\" mesafesi: ", "\"Too close\" distance: ")) + closeCm + " cm");
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
  iotbot.serialWrite(L("Ultrasonik mesafe testi başladı.", "Ultrasonic distance test started."));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) 200 ms'de bir ölç (sensör yankıların sönmesi için ölçümler arası ~60 ms ister)
  // 2) Measure every 200 ms (the sensor needs ~60 ms between measurements for echoes to fade)
  if (now - lastReadMs < 200) return;
  lastReadMs = now;
  distance = iotbot.moduleUltrasonicDistanceRead(); // 0 = yankı yok / no echo

  // Kütüphane yankı gelmezse 0 döndürür: 0 "çok yakın" DEĞİL, "menzil dışı" demektir.
  // The library returns 0 when no echo comes back: 0 is NOT "too close", it means "out of range".
  bool isClose = distance > 0 && distance < closeCm;
  if (isClose && !tooClose) iotbot.buzzerPlayTone(2000, 80); // Yakına girince bir bip / one beep on getting close
  bool changed = isClose != tooClose;
  tooClose = isClose;
  drawScreen();

  // 3) Seri port: yarım saniyede bir veya "çok yakın" durumu değişince
  // 3) Serial: every half second or when "too close" changes
  if (changed || now - lastPrintMs >= 500) {
    lastPrintMs = now;
    printDistance();
  }
}
