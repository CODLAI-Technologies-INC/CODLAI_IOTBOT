/*
 * TR: DHT11 SICAKLIK VE NEM MODÜLÜ
 *  - Sıcaklık (°C veya °F), bağıl nem (%) ve hissedilen sıcaklığı 2 saniyede bir
 *    okur; LCD'de ve seri portta gösterir.
 *  - Sensör okunamazsa (kablo takılı değil / yanlış pin) LCD'de uyarı çıkar.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help -> komut listesi
 *      oku    / read -> değerleri şimdi oku ve yaz
 *      birim  / unit -> °C <-> °F
 *      dil    / lang -> dili değiştir (Türkçe <-> English)
 *  - DHT özelliklerini açmak için "#define USE_DHT" satırı #include'dan ÖNCE olmalıdır.
 *
 * EN: DHT11 TEMPERATURE AND HUMIDITY MODULE
 *  - Reads the temperature (°C or °F), relative humidity (%) and the "feels
 *    like" temperature every 2 seconds; shows them on the LCD and the serial port.
 *  - If the sensor cannot be read (not plugged in / wrong pin) the LCD warns you.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help / yardim -> command list
 *      read / oku    -> read and print the values now
 *      unit / birim  -> °C <-> °F
 *      lang / dil    -> switch language (Turkish <-> English)
 *  - To enable the DHT features, "#define USE_DHT" must come BEFORE #include.
 *
 * Bağlantı / Wiring: DHT11 modülünü IO27'ye bağlı sokete takın.
 * Desteklenen pinler / Supported pins: IO25 - IO26 - IO27 - IO32 - IO33
 */

#define USE_DHT
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

#define DHT_PIN IO27 // DHT sensörünün bağlı olduğu pin / Pin the DHT sensor is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

bool fahrenheit = false;  // false = °C, true = °F
int temp = 0, hum = 0, feel = 0;
bool sensorOk = false;
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
  iotbot.serialWrite(L("---- DHT11 SICAKLIK-NEM - Komutlar ----", "---- DHT11 TEMPERATURE-HUMIDITY - Commands ----"));
  iotbot.serialWrite(L("  yardim : bu liste", "  help   : this list"));
  iotbot.serialWrite(L("  oku    : şimdi oku ve yaz", "  read   : read and print now"));
  iotbot.serialWrite(L("  birim  : °C <-> °F", "  unit   : °C <-> °F"));
  iotbot.serialWrite(L("  dil    : English'e geç", "  lang   : switch to Turkish"));
}

// Sensörü okur, LCD'yi ve seri portu günceller. Kütüphane hata olunca -999 döndürür.
// Reads the sensor, updates the LCD and the serial port. The library returns -999 on error.
void readAndShow() {
  temp = fahrenheit ? iotbot.moduleDhtTempReadF(DHT_PIN) : iotbot.moduleDhtTempReadC(DHT_PIN);
  hum = iotbot.moduleDhtHumRead(DHT_PIN);
  feel = fahrenheit ? iotbot.moduleDthFeelingTempF(DHT_PIN) : iotbot.moduleDthFeelingTempC(DHT_PIN);
  sensorOk = (temp != -999 && hum != -999);

  const char *unit = fahrenheit ? "F" : "C";
  char line[41];
  lcdRow(0, L("DHT11 SICAKLIK - NEM", "DHT11 TEMP-HUMIDITY"));
  if (!sensorOk) {
    lcdRow(1, L("Sensör okunamadı!", "Sensor read failed!"));
    snprintf(line, sizeof(line), L("Kabloyu kontrol et", "Check the cable"));
    lcdRow(2, line);
    snprintf(line, sizeof(line), L("Pin: IO%d", "Pin: IO%d"), DHT_PIN);
    lcdRow(3, line);
    iotbot.serialWrite(L("HATA: DHT11 okunamadı - kabloyu ve pini kontrol edin.", "ERROR: DHT11 read failed - check the cable and the pin."));
    return;
  }
  // LCD'de derece işareti için 0xDF karakteri kullanılır / 0xDF is the degree sign on the LCD
  snprintf(line, sizeof(line), L("Sıcaklık : %d\xDF%s", "Temperature: %d\xDF%s"), temp, unit);
  lcdRow(1, line);
  snprintf(line, sizeof(line), L("Nem      : %%%d", "Humidity   : %d%%"), hum);
  lcdRow(2, line);
  snprintf(line, sizeof(line), L("Hissedilen: %d\xDF%s", "Feels like : %d\xDF%s"), feel, unit);
  lcdRow(3, line);

  char msg[96];
  snprintf(msg, sizeof(msg), L("Sıcaklık: %d °%s | Nem: %%%d | Hissedilen: %d °%s", "Temperature: %d °%s | Humidity: %d%% | Feels like: %d °%s"),
           temp, unit, hum, feel, unit);
  iotbot.serialWrite(msg);
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "oku" || cmd == "read") {
    readAndShow();
  } else if (cmd == "birim" || cmd == "unit") {
    fahrenheit = !fahrenheit;
    iotbot.serialWrite(fahrenheit ? L("Birim: °F", "Unit: °F") : L("Birim: °C", "Unit: °C"));
    readAndShow();
  } else if (cmd == "dil" || cmd == "lang" || cmd == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    printHelp();
    readAndShow();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();             // IoTBot başlatılıyor / Initialize IoTBot
  iotbot.serialStart(115200); // Seri haberleşme / Serial communication
  iotbot.lcdClear();
  iotbot.serialWrite(L("DHT11 testi başladı.", "DHT11 test started."));
  printHelp();
  lastReadMs = millis(); // DHT11 açıldıktan sonra ~1 sn ister / DHT11 needs ~1 s after power-up
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) DHT11 en fazla 1-2 saniyede bir okunabilir: 2 sn'de bir oku
  // 2) DHT11 can be read at most every 1-2 seconds: read every 2 s
  if (millis() - lastReadMs >= 2000) {
    lastReadMs = millis();
    readAndShow();
  }
}
