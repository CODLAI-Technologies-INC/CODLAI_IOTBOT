/*
 * TR: EEPROM (KALICI HAFIZA) - Gelişmiş örnek
 *  - EEPROM'a yazılan bilgiler kartın elektriği kesilse bile SİLİNMEZ.
 *  - Açılışta: kayıtlı "açılış sayacı" okunur, 1 artırılıp geri yazılır (CRC korumalı
 *    kayıt: eepromWriteRecord / eepromReadRecord). Kartı birkaç kez kapatıp açın,
 *    sayının arttığını görün!
 *  - B3 butonuna her basış "B3 sayacını" 1 artırır ve EEPROM'a kaydeder.
 *  - Ayrıca int32, ondalık sayı (float) ve metin (String) yazıp okumayı gösterir.
 *  - Adres haritası: 0 = B3 sayacı (int16), 10 = sayı (int32), 20 = ondalık (float),
 *    40 = metin (en fazla 64), 200 = açılış kaydı (CRC'li).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help                -> komut listesi
 *      oku    / read                -> EEPROM'daki tüm değerleri yazdır
 *      yaz 123 / write 123          -> int32 sayı kaydet (adres 10)
 *      ondalik 3.14 / float 3.14    -> ondalık sayı kaydet (adres 20)
 *      metin <yazı> / text <words>  -> metin kaydet (adres 40)
 *      sifirla / reset              -> sayaçları sıfırla
 *      sil    / clear               -> tüm EEPROM'u sil (0xFF)
 *      dil    / lang                -> dili değiştir (Türkçe <-> English)
 *  - Not: Flash bellek sınırlı sayıda yazılabilir (~10.000-100.000 kez); döngü
 *    içinde sürekli yazmayın, sadece değer değişince yazın.
 *
 * EN: EEPROM (PERMANENT MEMORY) - Advanced example
 *  - Data written to the EEPROM is NOT lost even when the board loses power.
 *  - At startup: the stored "boot counter" is read, increased by 1 and written back
 *    (CRC-protected record: eepromWriteRecord / eepromReadRecord). Power the board
 *    off and on a few times and watch the number grow!
 *  - Every press of B3 increases the "B3 counter" by 1 and saves it to the EEPROM.
 *  - It also shows writing and reading an int32, a decimal number (float) and a text.
 *  - Address map: 0 = B3 counter (int16), 10 = number (int32), 20 = decimal (float),
 *    40 = text (max 64), 200 = boot record (with CRC).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim              -> command list
 *      read   / oku                 -> print every value stored in the EEPROM
 *      write 123 / yaz 123          -> save an int32 number (address 10)
 *      float 3.14 / ondalik 3.14    -> save a decimal number (address 20)
 *      text <words> / metin <yazı>  -> save a text (address 40)
 *      reset  / sifirla             -> reset the counters
 *      clear  / sil                 -> erase the whole EEPROM (0xFF)
 *      lang   / dil                 -> switch language (Turkish <-> English)
 *  - Note: flash memory can only be written a limited number of times (~10,000-
 *    100,000); do not write in a loop all the time, write only when a value changes.
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. / NO extra module needed.
 */

#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// EEPROM adresleri (çakışmasın diye planlayın) / EEPROM addresses (plan them so they don't overlap)
const int ADDR_B3_COUNT = 0;   // int16: 2 bayt / 2 bytes
const int ADDR_NUMBER = 10;    // int32: 4 bayt / 4 bytes
const int ADDR_FLOAT = 20;     // float: 4 bayt / 4 bytes
const int ADDR_TEXT = 40;      // metin: 2 bayt uzunluk + en fazla 64 / text: 2-byte length + max 64
const int ADDR_RECORD = 200;   // CRC'li kayıt: 10 bayt başlık + veri / CRC record: 10-byte header + data
const int TEXT_MAX = 64;

// CRC korumalı kayıt (önerilen yöntem) / CRC-protected record (recommended way)
struct BootRecord {
  uint32_t bootCount;  // Kaç kez açıldı / how many times the board started
  float lastValue;     // Son kaydedilen ondalık sayı / last saved decimal number
};

BootRecord record;
int b3Count = 0;
bool lastB3 = false;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
String rawLine; // Komutun orijinal hali (metin için) / original command text (for the text)
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "ONDALIK" -> "ondalik"
// Lower-cases and simplifies Turkish letters: "ONDALIK" -> "ondalik"
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
      rawLine = cmdBuffer; rawLine.trim();
      cmd = normalizeCommand(cmdBuffer);
      cmdBuffer = "";
      return true;
    }
    if (cmdBuffer.length() < 80) cmdBuffer += c;
  }
  // "Satır sonu yok" seçiliyse: 150 ms sessizlikten sonra komutu kabul et.
  // "No line ending" selected: accept the command after 150 ms of silence.
  if (cmdBuffer.length() > 0 && millis() - lastCharMs > 150) {
    rawLine = cmdBuffer; rawLine.trim();
    cmd = normalizeCommand(cmdBuffer);
    cmdBuffer = "";
    return true;
  }
  return false;
}

// Komuttan sonraki metin (orijinal harflerle) / the text after the command word (original letters)
String argText() {
  int space = rawLine.indexOf(' ');
  if (space < 0) return "";
  String t = rawLine.substring(space + 1);
  t.trim();
  return t;
}

// Metni okur. Hiç yazılmamış (0xFF dolu) EEPROM'da eepromReadString boş metin ("") döndürür.
// Reads the text. On a never-written (0xFF) EEPROM eepromReadString returns an empty text ("").
String readText() {
  return iotbot.eepromReadString(ADDR_TEXT, TEXT_MAX);
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void drawScreen() {
  char line[41];
  lcdRow(0, L("  EEPROM ÖRNEĞİ", "  EEPROM EXAMPLE"));
  snprintf(line, sizeof(line), L("Açılış sayısı: %lu", "Boot count: %lu"), (unsigned long)record.bootCount);
  lcdRow(1, line);
  snprintf(line, sizeof(line), L("B3 sayacı: %d", "B3 counter: %d"), b3Count);
  lcdRow(2, line);
  String text = readText();
  snprintf(line, sizeof(line), L("Metin: %s", "Text: %s"), text.substring(0, 13).c_str());
  lcdRow(3, line);
}

void printHelp() {
  iotbot.serialWrite(L("---- EEPROM - Komutlar ----", "---- EEPROM - Commands ----"));
  iotbot.serialWrite(L("  yardim          : bu liste", "  help            : this list"));
  iotbot.serialWrite(L("  oku             : tüm değerleri yazdır", "  read            : print every value"));
  iotbot.serialWrite(L("  yaz 123         : int32 sayı kaydet", "  write 123       : save an int32 number"));
  iotbot.serialWrite(L("  ondalik 3.14    : ondalık sayı kaydet", "  float 3.14      : save a decimal number"));
  iotbot.serialWrite(L("  metin <yazı>    : metin kaydet", "  text <words>    : save a text"));
  iotbot.serialWrite(L("  sifirla         : sayaçları sıfırla", "  reset           : reset the counters"));
  iotbot.serialWrite(L("  sil             : tüm EEPROM'u sil", "  clear           : erase the whole EEPROM"));
  iotbot.serialWrite(L("  dil             : English'e geç", "  lang            : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu       : B3 sayacını artır ve kaydet", "  B3 button       : increase and save the B3 counter"));
}

void printAll() {
  iotbot.serialWrite(L("---- EEPROM içeriği ----", "---- EEPROM contents ----"));
  iotbot.serialWrite(String(L("  [0]   B3 sayacı (int16) : ", "  [0]   B3 counter (int16): ")) + iotbot.eepromReadInt(ADDR_B3_COUNT));
  iotbot.serialWrite(String(L("  [10]  sayı (int32)      : ", "  [10]  number (int32)    : ")) + (long)iotbot.eepromReadInt32(ADDR_NUMBER, -1));
  iotbot.serialWrite(String(L("  [20]  ondalık (float)   : ", "  [20]  decimal (float)   : ")) + String(iotbot.eepromReadFloat(ADDR_FLOAT, -1.0f), 4));
  iotbot.serialWrite(String(L("  [40]  metin             : ", "  [40]  text              : ")) + readText());

  BootRecord r;
  uint16_t len = 0, ver = 0;
  if (iotbot.eepromReadRecord(ADDR_RECORD, (uint8_t *)&r, sizeof(r), &len, &ver)) {
    iotbot.serialWrite(String(L("  [200] kayıt: sürüm=", "  [200] record: version=")) + ver + L(" uzunluk=", " length=") + len +
                       L(" açılış=", " boots=") + (unsigned long)r.bootCount + L(" son ondalık=", " last decimal=") + String(r.lastValue, 4));
  } else {
    iotbot.serialWrite(L("  [200] kayıt: GEÇERSİZ (boş ya da CRC hatası)", "  [200] record: INVALID (empty or CRC mismatch)"));
  }
}

bool saveRecord() {
  return iotbot.eepromWriteRecord(ADDR_RECORD, (const uint8_t *)&record, (uint16_t)sizeof(record), 1);
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  String arg = hasValue ? cmd.substring(space + 1) : "";

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oku" || word == "read") {
    printAll();
  } else if ((word == "yaz" || word == "write") && hasValue) {
    int32_t v = arg.toInt();
    bool ok = iotbot.eepromWriteInt32(ADDR_NUMBER, v);
    iotbot.serialWrite(String(ok ? L("Kaydedildi: ", "Saved: ") : L("YAZILAMADI: ", "WRITE FAILED: ")) + (long)v +
                       L("  -> geri okunan: ", "  -> read back: ") + (long)iotbot.eepromReadInt32(ADDR_NUMBER, -1));
  } else if ((word == "ondalik" || word == "float") && hasValue) {
    float v = arg.toFloat();
    iotbot.eepromWriteFloat(ADDR_FLOAT, v);
    record.lastValue = v;
    saveRecord();
    iotbot.serialWrite(String(L("Ondalık kaydedildi: ", "Decimal saved: ")) + String(v, 4) +
                       L("  -> geri okunan: ", "  -> read back: ") + String(iotbot.eepromReadFloat(ADDR_FLOAT, -1.0f), 4));
  } else if ((word == "metin" || word == "text") && hasValue) {
    String text = argText();
    if (text.length() > TEXT_MAX) text = text.substring(0, TEXT_MAX);
    iotbot.eepromWriteString(ADDR_TEXT, text, TEXT_MAX);
    iotbot.serialWrite(String(L("Metin kaydedildi: ", "Text saved: ")) + readText());
    drawScreen();
  } else if (word == "sifirla" || word == "reset") {
    b3Count = 0;
    iotbot.eepromWriteInt(ADDR_B3_COUNT, 0);
    record.bootCount = 0;
    saveRecord();
    iotbot.serialWrite(L("Sayaçlar sıfırlandı.", "Counters reset."));
    drawScreen();
  } else if (word == "sil" || word == "clear") {
    bool ok = iotbot.eepromClear(0, 0, 0xFF); // 0, 0 = tüm alan / 0, 0 = whole area
    iotbot.serialWrite(ok ? L("EEPROM silindi (hepsi 0xFF). Kartı yeniden başlatınca açılış sayacı 1'den başlar.",
                              "EEPROM erased (all 0xFF). After a restart the boot counter starts from 1.")
                          : L("Silme başarısız.", "Erase failed."));
    b3Count = 0;
    drawScreen();
  } else if (word == "dil" || word == "lang" || word == "language") {
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

  // EEPROM'u başlat (ESP32'de flash üzerinde taklit edilir) / start the EEPROM (emulated on flash on the ESP32)
  bool ok = iotbot.eepromBegin(1024);
  iotbot.serialWrite(ok ? L("[EEPROM] Hazır", "[EEPROM] Ready") : L("[EEPROM] Başlatılamadı!", "[EEPROM] Begin failed!"));

  // 1) CRC'li açılış kaydı: oku, 1 artır, geri yaz / CRC boot record: read, add 1, write back
  uint16_t len = 0, ver = 0;
  if (iotbot.eepromReadRecord(ADDR_RECORD, (uint8_t *)&record, sizeof(record), &len, &ver) && len == sizeof(record)) {
    record.bootCount++;
  } else {
    iotbot.serialWrite(L("Geçerli kayıt yok (ilk açılış) - yeni kayıt oluşturuluyor.", "No valid record (first start) - creating a new one."));
    record.bootCount = 1;
    record.lastValue = 3.14f;
  }
  bool wrec = saveRecord();
  iotbot.serialWrite(String(L("Açılış sayısı: ", "Boot count: ")) + (unsigned long)record.bootCount +
                     (wrec ? L("  (kaydedildi)", "  (saved)") : L("  (KAYDEDİLEMEDİ)", "  (SAVE FAILED)")));

  // 2) B3 sayacı (int16) - açılışta sadece OKUNUR / B3 counter (int16) - only READ at startup
  b3Count = iotbot.eepromReadInt(ADDR_B3_COUNT);
  // Boş EEPROM 0xFF doludur; eepromReadInt bunu -1 olarak okur (sayaç eksi olamaz)
  // An empty EEPROM is full of 0xFF; eepromReadInt reads it as -1 (the counter can't be negative)
  if (b3Count < 0) b3Count = 0;

  // 3) Metin hiç yazılmamışsa örnek bir metin yaz / write a sample text if none was written yet
  if (readText().length() == 0) {
    iotbot.eepromWriteString(ADDR_TEXT, String("Merhaba IOTBOT"), TEXT_MAX);
  }

  drawScreen();
  printAll();
  printHelp();
}

void loop() {
  // B3 -> sayacı artır ve kaydet (sadece basıldığı an) / B3 -> increase and save the counter (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) {
    b3Count++;
    iotbot.eepromWriteInt(ADDR_B3_COUNT, b3Count);
    iotbot.buzzerPlayTone(1500, 40);
    iotbot.serialWrite(String(L("B3 sayacı kaydedildi: ", "B3 counter saved: ")) + b3Count);
    drawScreen();
  }
  lastB3 = b3;

  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
  delay(10);
}
