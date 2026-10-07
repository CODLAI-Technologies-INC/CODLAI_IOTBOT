/*
 * TR: POTANSİYOMETRE TESTİ
 *  - Kart üzerindeki potansiyometreyi (döner ayar düğmesi) okur: ham değer
 *    (0-4095), yüzde, pindeki gerilim ve bir çubuk LCD'de görünür.
 *  - Düğmeyi çevirdikçe yeni değer seri porta yazılır.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help -> komut listesi
 *      oku    / read -> değeri şimdi yaz
 *      dil    / lang -> dili değiştir (Türkçe <-> English)
 *
 * EN: POTENTIOMETER TEST
 *  - Reads the onboard potentiometer (rotary knob): the raw value (0-4095),
 *    percent, the voltage on the pin and a bar are shown on the LCD.
 *  - Turning the knob prints the new value to the serial port.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help / yardim -> command list
 *      read / oku    -> print the value now
 *      lang / dil    -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül gerekmez, potansiyometre kartın üzerindedir (GPIO36).
 *                    No extra module needed, the potentiometer is on the board (GPIO36).
 */

#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

int potValue = 0;          // Ham değer / raw value
int potPct = 0;            // Yüzde / percent
int lastPrintedPct = -100; // Seri porta en son yazılan yüzde / percent last printed to serial
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
// LCD'ye yazmak yavaştır (~1 ms/harf): sadece DEĞİŞEN satırları yazarız, loop hızlı kalır.
// Writing to the LCD is slow (~1 ms per letter): we write only rows that CHANGED, so loop stays fast.
String shownRows[4];
void lcdRow(int row, const char *text) {
  if (shownRows[row] == text) return;
  shownRows[row] = text;
  iotbot.lcdWriteFixedTxt(0, row, text, 20);
}

// Yaklaşık pin gerilimi (ESP32 ADC'si 0-4095 <-> ~0-3.3 V; tam doğrusal değildir)
// Approximate pin voltage (ESP32 ADC 0-4095 <-> ~0-3.3 V; not perfectly linear)
float voltage() { return potValue * 3.3f / 4095.0f; }

void printHelp() {
  iotbot.serialWrite(L("---- POTANSİYOMETRE TESTİ - Komutlar ----", "---- POTENTIOMETER TEST - Commands ----"));
  iotbot.serialWrite(L("  yardim : bu liste", "  help   : this list"));
  iotbot.serialWrite(L("  oku    : değeri şimdi yaz", "  read   : print the value now"));
  iotbot.serialWrite(L("  dil    : English'e geç", "  lang   : switch to Turkish"));
}

void printValue() {
  char msg[80];
  snprintf(msg, sizeof(msg), L("Potansiyometre: %4d / 4095  (%%%d, ~%.2f V)", "Potentiometer: %4d / 4095  (%d%%, ~%.2f V)"),
           potValue, potPct, voltage());
  iotbot.serialWrite(msg);
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "oku" || cmd == "read") {
    printValue();
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
  iotbot.serialWrite(L("Potansiyometre testi başladı. Düğmeyi çevirin!", "Potentiometer test started. Turn the knob!"));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) Potansiyometreyi oku ve LCD'yi yenile (200 ms'de bir, titremesiz)
  // 2) Read the potentiometer and refresh the LCD (every 200 ms, no flicker)
  if (now - lastScreenMs < 200) return;
  lastScreenMs = now;
  potValue = iotbot.potentiometerRead();
  potPct = map(potValue, 0, 4095, 0, 100);

  char line[41];
  lcdRow(0, L("  POTANSİYOMETRE", "   POTENTIOMETER"));
  snprintf(line, sizeof(line), L("Değer: %4d  %%%d", "Value: %4d  %d%%"), potValue, potPct);
  lcdRow(1, line);
  snprintf(line, sizeof(line), L("Gerilim: ~%.2f V", "Voltage: ~%.2f V"), voltage());
  lcdRow(2, line);
  // Çubuk: her dolu kutu %5 (0xFF = LCD'nin dolu kutu karakteri) / Bar: each full block is 5%
  int blocks = potPct / 5;
  for (int i = 0; i < 20; i++) line[i] = (i < blocks) ? '\xFF' : ' ';
  line[20] = '\0';
  lcdRow(3, line);

  // 3) Seri port: en az %2 değişince yaz (titreşimi yok sayar), en fazla 300 ms'de bir
  // 3) Serial: print when it changed by at least 2% (ignores jitter), at most every 300 ms
  if (abs(potPct - lastPrintedPct) >= 2 && now - lastPrintMs >= 300) {
    lastPrintMs = now;
    lastPrintedPct = potPct;
    printValue();
  }
}
