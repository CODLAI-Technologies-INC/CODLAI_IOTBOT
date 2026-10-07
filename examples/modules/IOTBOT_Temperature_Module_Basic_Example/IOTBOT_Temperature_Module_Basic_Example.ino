/*
 * TR: NTC SICAKLIK SENSÖRÜ MODÜLÜ
 *  - Sıcaklığı (°C, bir ondalık basamakla) okur; LCD'de gösterir ve her saniye
 *    seri porta yazar. Ayrıca açılıştan beri ölçülen en düşük / en yüksek
 *    sıcaklığı tutar.
 *  - Sensör takılı değilse saçma bir değer çıkar: bu durumda LCD'de
 *    "Sensör bağlı mı?" uyarısı görünür.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help  -> komut listesi
 *      oku     / read  -> sıcaklığı şimdi yaz
 *      sifirla / reset -> en düşük / en yüksek değerleri sıfırla
 *      dil     / lang  -> dili değiştir (Türkçe <-> English)
 *
 * EN: NTC TEMPERATURE SENSOR MODULE
 *  - Reads the temperature (°C, one decimal place); shows it on the LCD and
 *    prints it to the serial port every second. It also keeps the lowest /
 *    highest temperature measured since startup.
 *  - If the sensor is not plugged in the value is nonsense: the LCD then
 *    shows "Sensor connected?".
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help  / yardim  -> command list
 *      read  / oku     -> print the temperature now
 *      reset / sifirla -> reset the lowest / highest values
 *      lang  / dil     -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: NTC sıcaklık modülünü IO27'ye bağlı sokete takın.
 * Desteklenen pinler / Supported pins: IO25 - IO26 - IO27 - IO32 - IO33
 */

#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

#define SENSOR_PIN IO27 // NTC sensörünün bağlı olduğu pin / Pin the NTC sensor is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

float temp = 0;            // Son ölçülen sıcaklık / last temperature
float minTemp = 1000;      // En düşük / lowest
float maxTemp = -1000;     // En yüksek / highest
bool valid = false;        // Ölçüm mantıklı mı? / is the reading sensible?
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
  iotbot.serialWrite(L("---- NTC SICAKLIK - Komutlar ----", "---- NTC TEMPERATURE - Commands ----"));
  iotbot.serialWrite(L("  yardim  : bu liste", "  help    : this list"));
  iotbot.serialWrite(L("  oku     : sıcaklığı şimdi yaz", "  read    : print the temperature now"));
  iotbot.serialWrite(L("  sifirla : en düşük / en yüksek sıfırla", "  reset   : reset the lowest / highest"));
  iotbot.serialWrite(L("  dil     : English'e geç", "  lang    : switch to Turkish"));
}

void drawScreen() {
  char line[41];
  lcdRow(0, L("  NTC SICAKLIK", "  NTC TEMPERATURE"));
  if (!valid) {
    lcdRow(1, L("Sensör bağlı mı?", "Sensor connected?"));
    snprintf(line, sizeof(line), L("Pin: IO%d", "Pin: IO%d"), SENSOR_PIN);
    lcdRow(2, line);
    lcdRow(3, "");
    return;
  }
  // 0xDF = LCD'nin derece işareti / the LCD's degree sign
  snprintf(line, sizeof(line), L("Sıcaklık: %.1f\xDF" "C", "Temperature: %.1f\xDF" "C"), temp);
  lcdRow(1, line);
  snprintf(line, sizeof(line), L("En düşük : %.1f\xDF" "C", "Lowest : %.1f\xDF" "C"), minTemp);
  lcdRow(2, line);
  snprintf(line, sizeof(line), L("En yüksek: %.1f\xDF" "C", "Highest: %.1f\xDF" "C"), maxTemp);
  lcdRow(3, line);
}

void printTemp() {
  char msg[96];
  if (!valid) snprintf(msg, sizeof(msg), L("Sıcaklık okunamadı - sensör IO%d'ye bağlı mı?", "Temperature not readable - is the sensor on IO%d?"), SENSOR_PIN);
  else snprintf(msg, sizeof(msg), L("Sıcaklık: %.1f °C | en düşük %.1f °C | en yüksek %.1f °C", "Temperature: %.1f °C | lowest %.1f °C | highest %.1f °C"),
                temp, minTemp, maxTemp);
  iotbot.serialWrite(msg);
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "oku" || cmd == "read") {
    printTemp();
  } else if (cmd == "sifirla" || cmd == "reset") {
    if (valid) minTemp = maxTemp = temp;
    else { minTemp = 1000; maxTemp = -1000; }
    iotbot.serialWrite(L("En düşük / en yüksek sıfırlandı.", "Lowest / highest reset."));
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
  iotbot.serialWrite(L("NTC sıcaklık testi başladı.", "NTC temperature test started."));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) 500 ms'de bir ölç; 4 okumanın ortalaması titremeyi azaltır
  // 2) Measure every 500 ms; the average of 4 readings reduces jitter
  if (now - lastReadMs < 500) return;
  lastReadMs = now;
  float sum = 0;
  for (int i = 0; i < 4; i++) sum += iotbot.moduleNtcTempRead(SENSOR_PIN);
  temp = sum / 4;

  // Takılı olmayan sensör -273 °C gibi saçma değerler verir / an unplugged sensor gives nonsense like -273 °C
  valid = !isnan(temp) && temp > -40 && temp < 125;
  if (valid) {
    if (temp < minTemp) minTemp = temp;
    if (temp > maxTemp) maxTemp = temp;
  }
  drawScreen();

  // 3) Seri port: saniyede bir / Serial: once a second
  if (now - lastPrintMs >= 1000) {
    lastPrintMs = now;
    printTemp();
  }
}
