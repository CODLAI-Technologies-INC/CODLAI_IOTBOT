/*
 * TR: LCD EKRAN TESTİ (20 sütun x 4 satır)
 *  - Açılışta DEMO çalışır: 4 satır yazısı, Türkçe harfler, kayan yazı, sayaç
 *    ve "BAŞARILI" ekranı sırayla tekrar eder.
 *  - Seri porttan "yaz Merhaba" yazınca o metin LCD'de görünür ve demo durur.
 *    B3 butonu veya "demo" komutu demoyu yeniden başlatır.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim       / help        -> komut listesi
 *      yaz <metin>  / write <text> -> metni LCD'ye yaz (en fazla 20 harf)
 *      temizle      / clear       -> LCD'yi temizle (demo durur)
 *      demo                       -> demoyu yeniden başlat
 *      dil          / lang        -> dili değiştir (Türkçe <-> English)
 *
 * EN: LCD SCREEN TEST (20 columns x 4 rows)
 *  - At startup a DEMO runs: 4-row text, Turkish letters, scrolling text, a
 *    counter and a "SUCCESSFUL" screen repeat in turn.
 *  - Typing "write Hello" on the serial port shows that text on the LCD and
 *    stops the demo. Button B3 or the "demo" command restarts the demo.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help         / yardim      -> command list
 *      write <text> / yaz <metin> -> write the text on the LCD (max 20 letters)
 *      clear        / temizle     -> clear the LCD (stops the demo)
 *      demo                       -> restart the demo
 *      lang         / dil         -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül gerekmez, LCD kartın üzerindedir.
 *                    No extra module needed, the LCD is on the board.
 */

#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

bool demoRunning = true;  // true = demo, false = kullanıcının metni / user's text
int page = 0;             // Demo sayfası (0-4) / demo page (0-4)
int pageStep = 0;         // Sayfa içindeki adım / step inside the page
uint32_t nextStepMs = 0;  // Bir sonraki adımın zamanı / time of the next step
bool lastB3 = false;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
String rawCommand; // Komutun değiştirilmemiş hali ("yaz" metni için) / the untouched command (for the "write" text)
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
      rawCommand = cmdBuffer;
      rawCommand.trim();
      cmd = normalizeCommand(cmdBuffer);
      cmdBuffer = "";
      return true;
    }
    if (cmdBuffer.length() < 40) cmdBuffer += c;
  }
  // "Satır sonu yok" seçiliyse: 150 ms sessizlikten sonra komutu kabul et.
  // "No line ending" selected: accept the command after 150 ms of silence.
  if (cmdBuffer.length() > 0 && millis() - lastCharMs > 150) {
    rawCommand = cmdBuffer;
    rawCommand.trim();
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

// Metni en fazla 20 harfe kısaltır (Türkçe harf UTF-8'de 2 bayttır, 1 harf sayılır).
// 20'yi aşan satır LCD'de başka bir satıra taşar. / Cuts the text to at most 20 letters
// (a Turkish letter is 2 bytes in UTF-8 but counts as 1). A longer row spills onto another row.
void fit20(const String &in, char *out, size_t outSize) {
  size_t o = 0;
  int letters = 0;
  for (size_t i = 0; i < in.length() && o + 1 < outSize; i++) {
    uint8_t b = in[i];
    if ((b & 0xC0) != 0x80 && ++letters > 20) break; // Yeni harf başlıyor / a new letter starts
    out[o++] = b;
  }
  out[o] = '\0';
}

void printHelp() {
  iotbot.serialWrite(L("---- LCD TESTİ - Komutlar ----", "---- LCD TEST - Commands ----"));
  iotbot.serialWrite(L("  yardim       : bu liste", "  help         : this list"));
  iotbot.serialWrite(L("  yaz <metin>  : metni LCD'ye yaz (demo durur)", "  write <text> : write the text on the LCD (demo stops)"));
  iotbot.serialWrite(L("  temizle      : LCD'yi temizle", "  clear        : clear the LCD"));
  iotbot.serialWrite(L("  demo         : demoyu yeniden başlat", "  demo         : restart the demo"));
  iotbot.serialWrite(L("  dil          : English'e geç", "  lang         : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu    : demoyu yeniden başlat", "  B3 button    : restart the demo"));
}

void startDemo() {
  demoRunning = true;
  page = 0;
  pageStep = 0;
  nextStepMs = 0; // Hemen başla / start right away
  iotbot.serialWrite(L(">> Demo başladı.", ">> Demo started."));
}

// Demonun bir adımını çizer ve bir sonraki adıma kadar ne kadar bekleneceğini döndürür.
// Draws one demo step and returns how long to wait until the next step.
uint32_t demoStep() {
  char line[41];
  switch (page) {
    case 0: // 4 satır / 4 rows
      lcdRow(0, L("IoTBot LCD Testi", "IoTBot LCD Test"));
      lcdRow(1, L("Satır 1", "Row 1"));
      lcdRow(2, L("Satır 2", "Row 2"));
      lcdRow(3, L("Satır 3", "Row 3"));
      iotbot.serialWrite(L("LCD: 4 satır yazıldı.", "LCD: 4 rows written."));
      page++;
      return 2500;
    case 1: // Türkçe harfler / Turkish letters
      lcdRow(0, L("Türkçe harfler:", "Turkish letters:"));
      lcdRow(1, "  ç ğ ı ö ş ü İ");
      lcdRow(2, "  Çiçek  Güneş");
      lcdRow(3, "  Işık   Öğrenci");
      iotbot.serialWrite(L("LCD: Türkçe harfler gösterildi.", "LCD: Turkish letters shown."));
      page++;
      return 3000;
    case 2: // Kayan yazı: 12 harflik yazı 0..8. sütunlar arasında kayar / scrolling text
      if (pageStep == 0) {
        lcdRow(0, L("Kayan yazı:", "Scrolling text:"));
        lcdRow(2, "");
        lcdRow(3, "");
      }
      snprintf(line, sizeof(line), "%*s-> IoTBot <-", pageStep, "");
      lcdRow(1, line);
      if (++pageStep > 8) { pageStep = 0; page++; }
      return 200;
    case 3: // Sayaç 0..10 / counter 0..10
      if (pageStep == 0) {
        lcdRow(0, L("Sayı testi:", "Number test:"));
        lcdRow(1, "");
        lcdRow(3, "");
      }
      snprintf(line, sizeof(line), L("      Sayı: %d", "    Number: %d"), pageStep);
      lcdRow(2, line);
      if (++pageStep > 10) { pageStep = 0; page++; }
      return 500;
    default: // Başarı ekranı / success screen
      iotbot.lcdWriteMid("IoTBot", L("LCD Testi", "LCD Test"), L("BAŞARILI!", "SUCCESSFUL!"), "");
      iotbot.serialWrite(L("LCD testi tamamlandı, baştan başlıyor.", "LCD test finished, starting over."));
      page = 0;
      return 3000;
  }
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if ((word == "yaz" || word == "write") && space > 0) {
    // Orijinal (büyük/küçük harf ve Türkçe harfleri korunmuş) metni al / take the original text
    String text = rawCommand.substring(rawCommand.indexOf(' ') + 1);
    text.trim();
    char line[41];
    fit20(text, line, sizeof(line));
    demoRunning = false;
    lcdRow(0, L("Sizin metniniz:", "Your text:"));
    lcdRow(1, "");
    lcdRow(2, line);
    lcdRow(3, L("B3: demoya dön", "B3: back to demo"));
    iotbot.serialWrite(String(L("LCD'ye yazıldı: ", "Written on the LCD: ")) + line);
  } else if (word == "temizle" || word == "clear") {
    demoRunning = false;
    iotbot.lcdClear();
    iotbot.serialWrite(L("LCD temizlendi. (demo için: demo)", "LCD cleared. (for the demo: demo)"));
  } else if (word == "demo") {
    startDemo();
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
  iotbot.serialWrite(L("LCD testi başladı.", "LCD test started."));
  printHelp();
  startDemo();
}

void loop() {
  // 1) B3 -> demoyu yeniden başlat (sadece basıldığı an) / B3 -> restart the demo (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) {
    iotbot.buzzerPlayTone(1200, 40);
    startDemo();
  }
  lastB3 = b3;

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Demo: delay() yok, sırası gelen adımı çiz / Demo: no delay(), draw the step that is due
  if (demoRunning && millis() >= nextStepMs) {
    nextStepMs = millis() + demoStep();
  }
}
