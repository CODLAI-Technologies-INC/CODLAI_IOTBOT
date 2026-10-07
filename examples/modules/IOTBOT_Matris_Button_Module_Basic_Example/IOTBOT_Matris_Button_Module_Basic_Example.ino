/*
 * TR: MATRİS BUTON MODÜLÜ (5 tuş, tek pin)
 *  - Modülde hangi tuşa (1-5) basıldığını LCD'de gösterir; her tuşa basışta
 *    seri porta yazar ve tuşa göre farklı bir bip çalar.
 *  - 5 tuş tek bir analog pini paylaşır: her tuş pinde farklı bir gerilim
 *    oluşturur, kütüphane bu değerden tuş numarasını bulur.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help -> komut listesi
 *      oku    / read -> şu anki tuşu yaz
 *      ham    / raw  -> pinin ham analog değerini yaz (tuş tanınmıyorsa yardımcı olur)
 *      dil    / lang -> dili değiştir (Türkçe <-> English)
 *
 * EN: MATRIX BUTTON MODULE (5 keys, one pin)
 *  - Shows which key (1-5) of the module is pressed on the LCD; every press is
 *    printed to the serial port with a different beep per key.
 *  - The 5 keys share one analog pin: each key makes a different voltage on
 *    the pin and the library finds the key number from that value.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help / yardim -> command list
 *      read / oku    -> print the key pressed now
 *      raw  / ham    -> print the pin's raw analog value (helps if a key is not recognised)
 *      lang / dil    -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Matris buton modülünü IO27'ye bağlı sokete takın.
 * Desteklenen pinler / Supported pins: IO25 - IO26 - IO27 - IO32 - IO33
 */

#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

#define SENSOR_PIN IO27 // Matris butonun bağlı olduğu pin / Pin the matrix button is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

int currentKey = 0;      // Şu an basılı tuş (0 = yok) / key pressed now (0 = none)
int lastKey = 0;         // Son basılan tuş / last key pressed
int pressCount = 0;      // Toplam basış / total presses
uint32_t lastReadMs = 0;

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
  iotbot.serialWrite(L("---- MATRİS BUTON - Komutlar ----", "---- MATRIX BUTTON - Commands ----"));
  iotbot.serialWrite(L("  yardim : bu liste", "  help   : this list"));
  iotbot.serialWrite(L("  oku    : şu anki tuşu yaz", "  read   : print the key pressed now"));
  iotbot.serialWrite(L("  ham    : pinin ham analog değerini yaz", "  raw    : print the pin's raw analog value"));
  iotbot.serialWrite(L("  dil    : English'e geç", "  lang   : switch to Turkish"));
}

void drawScreen() {
  char line[41];
  lcdRow(0, L("    MATRİS BUTON", "   MATRIX BUTTON"));
  if (currentKey > 0) snprintf(line, sizeof(line), L("Basılı tuş: %d", "Key pressed: %d"), currentKey);
  else snprintf(line, sizeof(line), "%s", L("Basılı tuş: yok", "Key pressed: none"));
  lcdRow(1, line);
  if (lastKey > 0) snprintf(line, sizeof(line), L("Son basılan: %d", "Last key: %d"), lastKey);
  else snprintf(line, sizeof(line), "%s", L("Bir tuşa basın...", "Press a key..."));
  lcdRow(2, line);
  snprintf(line, sizeof(line), L("Toplam basış: %d", "Total presses: %d"), pressCount);
  lcdRow(3, line);
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "oku" || cmd == "read") {
    if (currentKey > 0) iotbot.serialWrite(String(L("Basılı tuş: ", "Key pressed: ")) + currentKey);
    else iotbot.serialWrite(L("Şu an hiçbir tuşa basılmıyor.", "No key is pressed right now."));
  } else if (cmd == "ham" || cmd == "raw") {
    // Tuşlar yaklaşık / keys roughly: 1 > 4000, 2 ~2400, 3 ~2280, 4 ~2180, 5 ~2075
    iotbot.serialWrite(String(L("Ham analog değer (0-4095): ", "Raw analog value (0-4095): ")) + iotbot.moduleMatrisButtonAnalogRead(SENSOR_PIN));
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
  iotbot.serialWrite(L("Matris buton testi başladı. Bir tuşa basın!", "Matrix button test started. Press a key!"));
  printHelp();
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) Tuşu 30 ms'de bir oku; sadece değişince ekrana/seri porta yaz
  // 2) Read the key every 30 ms; update the screen/serial only on change
  if (millis() - lastReadMs < 30) return;
  lastReadMs = millis();

  int key = iotbot.moduleMatrisButtonNumberRead(SENSOR_PIN); // 0 = basılmıyor / not pressed
  // Tuşa basılırken gerilim bir an komşu tuşun değerinden geçebilir: aynı tuş
  // art arda 2 kez okununca kabul et. / While pressing, the voltage may pass a
  // neighbouring key's value for a moment: accept a key after 2 equal reads.
  static int candidate = 0;
  if (key != candidate) {
    candidate = key;
    return;
  }
  if (key == currentKey) return;
  currentKey = key;
  if (key > 0) {
    lastKey = key;
    pressCount++;
    iotbot.serialWrite(String(L("Basılan tuş: ", "Key pressed: ")) + key);
    iotbot.buzzerPlayTone(600 + key * 200, 40); // Her tuşa farklı ses / a different tone per key
  }
  drawScreen();
}
