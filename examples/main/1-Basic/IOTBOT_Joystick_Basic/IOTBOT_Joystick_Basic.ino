/*
 * TR: JOYSTICK TESTİ
 *  - Kart üzerindeki joystick'in X ve Y eksenlerini (0-4095 ham değer) ve
 *    orta noktaya göre yüzdesini (-100 ... +100) LCD'de ve seri portta gösterir.
 *  - Joystick'in düğmesine basınca LCD'de "BASILI" yazar ve kısa bir bip çalar.
 *  - Açılışta joystick'e dokunmayın: orta nokta ölçülür.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help   -> komut listesi
 *      oku    / read   -> değerleri şimdi yaz
 *      orta   / center -> orta noktayı yeniden ölç (joystick'e dokunmayın)
 *      dil    / lang   -> dili değiştir (Türkçe <-> English)
 *
 * EN: JOYSTICK TEST
 *  - Shows the onboard joystick's X and Y axes (0-4095 raw value) and their
 *    percent from the center (-100 ... +100) on the LCD and the serial port.
 *  - Pressing the joystick's button shows "PRESSED" on the LCD and beeps.
 *  - Do not touch the joystick at startup: its center point is measured.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim -> command list
 *      read   / oku    -> print the values now
 *      center / orta   -> measure the center again (do not touch the joystick)
 *      lang   / dil    -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül gerekmez, joystick kartın üzerindedir.
 *   Not: X ekseni GPIO15 (ADC2) - WiFi/ESP-NOW açıkken okunamaz; Y ekseni GPIO34.
 *   No extra module needed, the joystick is on the board.
 *   Note: the X axis is GPIO15 (ADC2) - it cannot be read while WiFi/ESP-NOW is on; Y is GPIO34.
 */

#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

int xCenter = 2048, yCenter = 2048; // Orta nokta (açılışta ölçülür) / center point (measured at startup)
int xRaw = 0, yRaw = 0;             // Ham değerler / raw values
int xPct = 0, yPct = 0;             // Ortaya göre yüzde / percent from the center
bool pressed = false, lastPressed = false;
uint32_t lastReadMs = 0;
uint32_t lastPrintMs = 0;
int lastPrintedX = 999, lastPrintedY = 999;

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

// Ham değeri ortaya göre -100..+100 yüzdeye çevirir. / Converts a raw value to -100..+100 percent from the center.
int toPercent(int raw, int center) {
  if (raw >= center) return constrain(map(raw, center, 4095, 0, 100), 0, 100);
  return -constrain(map(raw, center, 0, 0, 100), 0, 100);
}

void printHelp() {
  iotbot.serialWrite(L("---- JOYSTICK TESTİ - Komutlar ----", "---- JOYSTICK TEST - Commands ----"));
  iotbot.serialWrite(L("  yardim : bu liste", "  help   : this list"));
  iotbot.serialWrite(L("  oku    : değerleri şimdi yaz", "  read   : print the values now"));
  iotbot.serialWrite(L("  orta   : orta noktayı yeniden ölç", "  center : measure the center again"));
  iotbot.serialWrite(L("  dil    : English'e geç", "  lang   : switch to Turkish"));
}

void printValues() {
  char msg[96];
  snprintf(msg, sizeof(msg), L("X: %4d (%+4d%%) | Y: %4d (%+4d%%) | Düğme: %s", "X: %4d (%+4d%%) | Y: %4d (%+4d%%) | Button: %s"),
           xRaw, xPct, yRaw, yPct, pressed ? L("BASILI", "PRESSED") : L("serbest", "released"));
  iotbot.serialWrite(msg);
}

void drawScreen() {
  char line[41];
  lcdRow(0, L("   JOYSTICK TESTİ", "   JOYSTICK TEST"));
  snprintf(line, sizeof(line), "X: %4d  %+4d%%", xRaw, xPct);
  lcdRow(1, line);
  snprintf(line, sizeof(line), "Y: %4d  %+4d%%", yRaw, yPct);
  lcdRow(2, line);
  snprintf(line, sizeof(line), L("Düğme: %s", "Button: %s"), pressed ? L("BASILI", "PRESSED") : L("serbest", "released"));
  lcdRow(3, line);
}

void measureCenter() {
  iotbot.calibrateJoystick(xCenter, yCenter); // 20 okumanın ortalaması / average of 20 readings
  // Ölçüm sırasında kol itildiyse değer saçma olur: varsayılana dön.
  // If the stick was pushed during the measurement the value is nonsense: use the default.
  if (xCenter < 1000 || xCenter > 3100 || yCenter < 1000 || yCenter > 3100) {
    xCenter = 2048;
    yCenter = 2048;
  }
  char msg[64];
  snprintf(msg, sizeof(msg), L("Orta nokta: X=%d Y=%d", "Center: X=%d Y=%d"), xCenter, yCenter);
  iotbot.serialWrite(msg);
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "oku" || cmd == "read") {
    printValues();
  } else if (cmd == "orta" || cmd == "center") {
    measureCenter();
  } else if (cmd == "dil" || cmd == "lang" || cmd == "language") {
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
  measureCenter();
  iotbot.serialWrite(L("Joystick testi başladı. Joystick'i oynatın!", "Joystick test started. Move the joystick!"));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) Joystick'i 100 ms'de bir oku ve LCD'yi yenile (titremesiz)
  // 2) Read the joystick every 100 ms and refresh the LCD (no flicker)
  if (now - lastReadMs < 100) return;
  lastReadMs = now;

  xRaw = iotbot.joystickXRead();
  yRaw = iotbot.joystickYRead();
  xPct = toPercent(xRaw, xCenter);
  yPct = toPercent(yRaw, yCenter);
  // Düğme pull-up: BASILIYKEN false (LOW) döner. / Button is pulled up: false (LOW) while PRESSED.
  pressed = !iotbot.joystickButtonRead();
  if (pressed && !lastPressed) iotbot.buzzerPlayTone(1200, 30);
  bool buttonChanged = pressed != lastPressed;
  lastPressed = pressed;
  drawScreen();

  // 3) Seri port: kol en az %5 oynadıysa veya düğme değiştiyse yaz (en fazla 300 ms'de bir)
  // 3) Serial: print when the stick moved at least 5% or the button changed (at most every 300 ms)
  bool moved = abs(xPct - lastPrintedX) >= 5 || abs(yPct - lastPrintedY) >= 5;
  if (buttonChanged || (moved && now - lastPrintMs >= 300)) {
    lastPrintMs = now;
    lastPrintedX = xPct;
    lastPrintedY = yPct;
    printValues();
  }
}
