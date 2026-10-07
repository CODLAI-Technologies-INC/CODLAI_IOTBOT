/*
 * TR: BUTON TESTİ
 *  - Kart üzerindeki B1, B2 ve B3 butonlarının durumunu LCD'de gösterir.
 *  - Bir butona basınca veya bırakınca seri porta yazar ve kısa bir bip çalar.
 *  - B1 ve B2 aynı pini (GPIO4) paylaşır: hangisine basıldığını pinin analog
 *    değeri söyler (B1 > 3500, B2 1500-3000). B3 ayrı bir dijital pindir (GPIO0).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help  -> komut listesi
 *      oku    / read  -> butonların durumunu ve GPIO4'ün analog değerini yaz
 *      dil    / lang  -> dili değiştir (Türkçe <-> English)
 *
 * EN: BUTTON TEST
 *  - Shows the state of the onboard buttons B1, B2 and B3 on the LCD.
 *  - Pressing or releasing a button prints it to the serial port and beeps.
 *  - B1 and B2 share one pin (GPIO4): its analog value tells which one is
 *    pressed (B1 > 3500, B2 1500-3000). B3 is a separate digital pin (GPIO0).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help  / yardim -> command list
 *      read  / oku    -> print the button states and GPIO4's analog value
 *      lang  / dil    -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül gerekmez, butonlar kartın üzerindedir.
 *                    No extra module needed, the buttons are on the board.
 */

#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

bool lastState[3] = {false, false, false}; // B1, B2, B3'ün son durumu / last state of B1, B2, B3
uint32_t lastPollMs = 0;

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

const char *stateText(bool pressed) { return pressed ? L("BASILI", "PRESSED") : L("serbest", "released"); }

void printHelp() {
  iotbot.serialWrite(L("---- BUTON TESTİ - Komutlar ----", "---- BUTTON TEST - Commands ----"));
  iotbot.serialWrite(L("  yardim : bu liste", "  help   : this list"));
  iotbot.serialWrite(L("  oku    : buton durumları + GPIO4 analog değeri", "  read   : button states + GPIO4 analog value"));
  iotbot.serialWrite(L("  dil    : English'e geç", "  lang   : switch to Turkish"));
}

void drawScreen() {
  char line[41];
  lcdRow(0, L("    BUTON TESTİ", "    BUTTON TEST"));
  for (int i = 0; i < 3; i++) {
    snprintf(line, sizeof(line), "B%d: %s", i + 1, stateText(lastState[i]));
    lcdRow(i + 1, line);
  }
}

void printStates() {
  char msg[96];
  snprintf(msg, sizeof(msg), "B1: %s | B2: %s | B3: %s | GPIO4 analog: %d",
           stateText(lastState[0]), stateText(lastState[1]), stateText(lastState[2]),
           analogRead(B1_AND_B2_BUTTON_PIN));
  iotbot.serialWrite(msg);
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "oku" || cmd == "read") {
    printStates();
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
  iotbot.serialWrite(L("Buton testi başladı. Butonlara basın!", "Button test started. Press the buttons!"));
  printHelp();
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) Butonları 20 ms'de bir oku (kontak sıçramasını da süzer)
  // 2) Read the buttons every 20 ms (also filters contact bounce)
  if (millis() - lastPollMs < 20) return;
  lastPollMs = millis();

  bool now[3] = {iotbot.button1Read(), iotbot.button2Read(), iotbot.button3Read()};
  bool changed = false;
  for (int i = 0; i < 3; i++) {
    if (now[i] == lastState[i]) continue; // Değişmedi / no change
    lastState[i] = now[i];
    changed = true;
    char msg[48];
    snprintf(msg, sizeof(msg), "B%d: %s", i + 1, stateText(now[i]));
    iotbot.serialWrite(msg);
    if (now[i]) iotbot.buzzerPlayTone(1000 + i * 300, 30); // Her butona farklı bip / a different beep per button
  }
  if (changed) drawScreen(); // LCD'yi sadece değişince yaz / update the LCD only on change
}
