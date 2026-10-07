/*
 * TR: IR ALICI MODÜLÜ - Ondalık (decimal) kod
 *  - Bir IR kumandanın tuşuna basınca gelen kodu ondalık sayı olarak LCD'de ve
 *    seri portta gösterir, kısa bir bip çalar. Her tuşun kodu farklıdır: bu
 *    kodları kendi projelerinizde "hangi tuşa basıldı?" sorusu için kullanın.
 *  - Basılı tutulan tuşta bazı kumandalar "tekrar" kodu (4294967295) gönderir.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help  -> komut listesi
 *      oku     / read  -> son kodu yaz
 *      sifirla / reset -> son kodu ve sayacı sıfırla
 *      dil     / lang  -> dili değiştir (Türkçe <-> English)
 *  - IR özelliklerini açmak için "#define USE_IR" satırı #include'dan ÖNCE olmalıdır.
 *
 * EN: IR RECEIVER MODULE - Decimal code
 *  - Press a key on an IR remote: its code is shown as a decimal number on the
 *    LCD and the serial port, with a short beep. Every key has its own code:
 *    use these codes in your projects to know "which key was pressed?".
 *  - While a key is held, some remotes send a "repeat" code (4294967295).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help  / yardim  -> command list
 *      read  / oku     -> print the last code
 *      reset / sifirla -> clear the last code and the counter
 *      lang  / dil     -> switch language (Turkish <-> English)
 *  - To enable the IR features, "#define USE_IR" must come BEFORE #include.
 *
 * Bağlantı / Wiring: IR alıcı modülünü IO27'ye bağlı sokete takın.
 * Desteklenen pinler / Supported pins: IO25 - IO26 - IO27 - IO32 - IO33
 */

#define USE_IR
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

#define SENSOR_PIN IO27 // IR alıcının bağlı olduğu pin / Pin the IR receiver is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

uint32_t lastCode = 0;   // Son okunan kod (0 = henüz yok) / last code read (0 = none yet)
int codeCount = 0;       // Kaç kod okundu / how many codes were read

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
  iotbot.serialWrite(L("---- IR ALICI (DECIMAL) - Komutlar ----", "---- IR RECEIVER (DECIMAL) - Commands ----"));
  iotbot.serialWrite(L("  yardim  : bu liste", "  help    : this list"));
  iotbot.serialWrite(L("  oku     : son kodu yaz", "  read    : print the last code"));
  iotbot.serialWrite(L("  sifirla : son kodu ve sayacı sıfırla", "  reset   : clear the last code and the counter"));
  iotbot.serialWrite(L("  dil     : English'e geç", "  lang    : switch to Turkish"));
}

void drawScreen() {
  char line[41];
  lcdRow(0, L("  IR ALICI TESTİ", "  IR RECEIVER TEST"));
  lcdRow(1, L("Ondalık kod:", "Decimal code:"));
  if (codeCount == 0) lcdRow(2, L("Kumandaya basın...", "Press the remote..."));
  else {
    snprintf(line, sizeof(line), "%lu", (unsigned long)lastCode);
    lcdRow(2, line);
  }
  snprintf(line, sizeof(line), L("Okunan: %d", "Received: %d"), codeCount);
  lcdRow(3, line);
}

void printLast() {
  if (codeCount == 0) {
    iotbot.serialWrite(L("Henüz kod okunmadı - kumandanın bir tuşuna basın.", "No code yet - press a key on the remote."));
    return;
  }
  char msg[64];
  snprintf(msg, sizeof(msg), L("Son kod (decimal): %lu", "Last code (decimal): %lu"), (unsigned long)lastCode);
  iotbot.serialWrite(msg);
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "oku" || cmd == "read") {
    printLast();
  } else if (cmd == "sifirla" || cmd == "reset") {
    lastCode = 0;
    codeCount = 0;
    drawScreen();
    iotbot.serialWrite(L("Sıfırlandı.", "Cleared."));
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
  iotbot.serialWrite(L("IR alıcı testi başladı. Kumandanın bir tuşuna basın!", "IR receiver test started. Press a key on the remote!"));
  printHelp();
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) IR kodu geldiyse göster. Kütüphane int döndürür; büyük kodlar negatif görünmesin
  //    diye işaretsiz (uint32_t) sayıya çeviriyoruz. 0 = kod yok.
  //    Sadece son 8 biti (0-255) istiyorsanız: iotbot.moduleIRReadDecimalx8(SENSOR_PIN)
  // 2) Show the IR code if one arrived. The library returns an int; we turn it into an
  //    unsigned (uint32_t) number so big codes do not look negative. 0 = no code.
  //    If you only want the last 8 bits (0-255): iotbot.moduleIRReadDecimalx8(SENSOR_PIN)
  uint32_t code = (uint32_t)iotbot.moduleIRReadDecimalx32(SENSOR_PIN);
  if (code != 0) {
    lastCode = code;
    codeCount++;
    printLast();
    drawScreen();
    iotbot.buzzerPlayTone(1500, 30); // Kısa bip / short beep
  }
}
