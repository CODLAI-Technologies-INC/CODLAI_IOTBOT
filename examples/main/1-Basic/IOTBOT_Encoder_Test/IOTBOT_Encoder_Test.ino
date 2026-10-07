/*
 * TR: ENCODER TESTİ
 *  - Kart üzerindeki döner encoder'ı çevirdikçe sayaç artar/azalır; değer LCD'de
 *    ve seri portta görünür.
 *  - Encoder'ın düğmesine basınca sayaç sıfırlanır.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help   -> komut listesi
 *      oku     / read   -> sayacı ve düğme durumunu yaz
 *      sifirla / reset  -> sayacı sıfırla
 *      dil     / lang   -> dili değiştir (Türkçe <-> English)
 *
 * EN: ENCODER TEST
 *  - Turning the onboard rotary encoder counts up/down; the value is shown on
 *    the LCD and the serial port.
 *  - Pressing the encoder's knob resets the counter.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help  / yardim   -> command list
 *      read  / oku      -> print the counter and the knob state
 *      reset / sifirla  -> reset the counter
 *      lang  / dil      -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül gerekmez, encoder kartın üzerindedir.
 *                    No extra module needed, the encoder is on the board.
 */

#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

int zeroOffset = 0;      // Sıfırlama anındaki ham sayaç / raw counter at the last reset
int value = 0;           // Ekranda gösterilen değer / value shown on screen
int lastPrinted = 0;     // Seri porta en son yazılan değer / value last printed to serial
bool pressed = false;    // Düğme şu an basılı mı? / is the knob pressed now?
bool lastPressed = false;
uint32_t lastScreenMs = 0;
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
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- ENCODER TESTİ - Komutlar ----", "---- ENCODER TEST - Commands ----"));
  iotbot.serialWrite(L("  yardim  : bu liste", "  help    : this list"));
  iotbot.serialWrite(L("  oku     : sayacı ve düğmeyi yaz", "  read    : print the counter and the knob"));
  iotbot.serialWrite(L("  sifirla : sayacı sıfırla", "  reset   : reset the counter"));
  iotbot.serialWrite(L("  dil     : English'e geç", "  lang    : switch to Turkish"));
  iotbot.serialWrite(L("  Encoder düğmesi: sayacı sıfırla", "  Encoder knob press: reset the counter"));
}

void drawStaticScreen() {
  lcdRow(0, L("   ENCODER TESTİ", "    ENCODER TEST"));
  lcdRow(3, L("Düğmeye bas: sıfırla", "Press knob: reset"));
  lastScreenMs = 0; // Değerleri hemen çiz / draw the values right away
}

void printValue() {
  char msg[64];
  snprintf(msg, sizeof(msg), L("Encoder: %d | Düğme: %s", "Encoder: %d | Knob: %s"), value,
           pressed ? L("BASILI", "PRESSED") : L("serbest", "released"));
  iotbot.serialWrite(msg);
}

void resetCounter() {
  zeroOffset += value; // Ham sayaç değişmez, sadece sıfır noktasını kaydırırız / shift the zero point
  value = 0;
  lastPrinted = 0;
  iotbot.buzzerPlayTone(1200, 40);
  iotbot.serialWrite(L("Sayaç sıfırlandı.", "Counter reset."));
  lastScreenMs = 0;
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "oku" || cmd == "read") {
    printValue();
  } else if (cmd == "sifirla" || cmd == "reset") {
    resetCounter();
  } else if (cmd == "dil" || cmd == "lang" || cmd == "language") {
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
  iotbot.begin();             // IoTBot başlatılıyor / Initialize IoTBot
  iotbot.serialStart(115200); // Seri haberleşme / Serial communication
  iotbot.lcdClear();
  drawStaticScreen();
  iotbot.serialWrite(L("Encoder testi başladı. Encoder'ı çevirin!", "Encoder test started. Turn the encoder!"));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) encoderRead() her loop'ta çağrılmalı: adımları ancak böyle kaçırmaz.
  // 1) encoderRead() must be called every loop: only then it misses no steps.
  value = iotbot.encoderRead() - zeroOffset;

  // 2) Düğme: pin pull-up olduğu için encoderButtonRead() BASILIYKEN false (LOW) döner.
  // 2) Knob: the pin is pulled up, so encoderButtonRead() is false (LOW) while PRESSED.
  pressed = !iotbot.encoderButtonRead();
  if (pressed && !lastPressed) resetCounter();
  lastPressed = pressed;

  // 3) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 4) Seri port: değer değiştiyse en fazla 300 ms'de bir yaz / print on change, at most every 300 ms
  if (value != lastPrinted && now - lastPrintMs >= 300) {
    lastPrintMs = now;
    lastPrinted = value;
    printValue();
  }

  // 5) LCD (200 ms'de bir, titremesiz) / LCD (every 200 ms, no flicker)
  if (now - lastScreenMs >= 200) {
    lastScreenMs = now;
    char line[41];
    snprintf(line, sizeof(line), L("Değer: %d", "Value: %d"), value);
    lcdRow(1, line);
    snprintf(line, sizeof(line), L("Düğme: %s", "Knob: %s"), pressed ? L("BASILI", "PRESSED") : L("serbest", "released"));
    lcdRow(2, line);
  }
}
