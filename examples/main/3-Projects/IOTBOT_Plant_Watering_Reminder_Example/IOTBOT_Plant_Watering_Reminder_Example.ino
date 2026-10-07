/*
 * TR: GERÇEK PROJE - Bitki Sulama Hatırlatıcısı. Toprak nemi sensörünü
 * saksınızın toprağına yerleştirin. Açılışta sensörün o anki değeri "nemli
 * toprak" referansı olarak kaydedilir (bu yüzden toprak sulanmışken açın).
 * Toprak kuruyunca (değer referanstan yeterince uzaklaşınca) LCD "SULAMA
 * ZAMANI!" yazar ve birkaç saniyede bir kısa bir hatırlatma sesi çalar -
 * toprak nemlenene kadar devam eder.
 *  - B3 butonu hatırlatma sesini SUSTURUR (toprak nemlenene kadar).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim    / help           -> komut listesi
 *      oku       / read           -> sensör değerini şimdi yaz
 *      kalibre   / calibrate      -> şu anki toprağı "nemli" referansı yap
 *      esik 300  / threshold 300  -> kuruluk eşiği (referanstan fark)
 *      sustur    / mute           -> hatırlatma sesini sustur
 *      dil       / lang           -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Plant Watering Reminder. Place the soil moisture
 * sensor's probe in your plant pot's soil. At startup the sensor's current
 * value is saved as the "moist soil" reference (so power it on when the soil
 * is watered). When the soil dries out (the value drifts far enough from the
 * reference), the LCD shows "TIME TO WATER!" and it plays a short reminder
 * tone every few seconds - it keeps going until the soil is moist again.
 *  - Button B3 SILENCES the reminder tone (until the soil is moist again).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help          / yardim     -> command list
 *      read          / oku        -> print the sensor value now
 *      calibrate     / kalibre    -> make the current soil the "moist" reference
 *      threshold 300 / esik 300   -> dryness threshold (difference from reference)
 *      mute          / sustur     -> silence the reminder tone
 *      lang          / dil        -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Toprak nemi sensörünü P1-P5 soketlerinden BİRİNE takın
 * ve aşağıdaki SOIL_PIN değerini o soketin sinyaline göre ayarlayın. B3 kart
 * üzerindedir. / Plug the soil moisture sensor into ONE of the P1-P5 sockets
 * and set SOIL_PIN below to match that socket's signal. B3 is on the board.
 */

#include <IOTBOT.h>

IOTBOT iotbot;

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

#define SOIL_PIN IO27 // Toprak nemi sensörünün bağlı olduğu pin / Pin the soil moisture sensor is connected to
// Desteklenen pinler: IO25 - IO26 - IO27 - IO32 - IO33
// Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

namespace {
  int wetBaseline = -1; // Islak/normal toprak değeri (açılışta ölçülür) / wet/normal baseline (measured at startup)
  // Bu değerden ("wetBaseline") ne kadar UZAKLAŞIRSA toprak o kadar kurumuş demektir -
  // gerçek toprakla test edip ayarlayın. / The FARTHER the reading drifts from this
  // baseline, the drier the soil is - tune this by testing with real soil.
  int dryDifference = 300;
  constexpr uint32_t kReadIntervalMs = 500;     // Okuma aralığı / reading interval
  constexpr uint32_t kReminderEveryMs = 3000;   // Hatırlatma sesi aralığı / reminder tone interval

  bool isDry = false;
  bool muted = false;          // Bu kuru dönem için ses kapalı mı? / sound off for this dry period?
  int value = 0;
  uint32_t lastReadMs = 0;
  uint32_t lastReminderMs = 0;
  bool lastB3 = false;
  uint32_t lastB3Ms = 0;
}

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "EŞİK" -> "esik"
// Lower-cases and simplifies Turkish letters: "EŞİK" -> "esik"
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
// Sensör / Sensor
// ---------------------------------------------------------------------------
// 10 okumanın ortalaması: tek bir gürültülü okuma kararı bozmasın.
// Average of 10 readings: one noisy reading must not spoil the decision.
int readSoil() {
  long sum = 0;
  for (int i = 0; i < 10; i++) sum += iotbot.moduleSoilMoistureRead(SOIL_PIN);
  return sum / 10;
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
// lcdWriteFixedTxt Türkçe harfleri (ç, ğ, ı, ö, ş, ü) LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- BİTKİ SULAMA HATIRLATICISI - Komutlar ----", "---- PLANT WATERING REMINDER - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oku           : sensör değerini yaz", "  read          : print the sensor value"));
  iotbot.serialWrite(L("  kalibre       : şu anki toprak = nemli referans", "  calibrate     : current soil = moist reference"));
  iotbot.serialWrite(L("  esik 50-2000  : kuruluk eşiği (referanstan fark)", "  threshold 50-2000: dryness threshold (difference)"));
  iotbot.serialWrite(L("  sustur        : hatırlatma sesini sustur", "  mute          : silence the reminder tone"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : hatırlatma sesini sustur", "  B3 button     : silence the reminder tone"));
}

void drawScreen() {
  char line[41];
  lcdRow(0, isDry ? L("   SULAMA ZAMANI!", "   TIME TO WATER!") : L("    BİTKİ SULAMA", "   PLANT WATERING"));
  snprintf(line, sizeof(line), L("Değer:%4d Fark:%4d", "Value:%4d Diff:%4d"), value, abs(value - wetBaseline));
  lcdRow(1, line);
  lcdRow(2, isDry ? L("Toprak kurumuş", "Soil is dry") : L("Toprak nemli, iyi!", "Soil is moist, good!"));
  if (isDry) lcdRow(3, muted ? L("Ses susturuldu", "Sound muted") : L("B3: sustur", "B3: mute"));
  else lcdRow(3, "");
}

void printReading() {
  char msg[120];
  snprintf(msg, sizeof(msg), L("Toprak değeri: %d  (referans %d, fark %d, eşik %d) -> %s", "Soil value: %d  (reference %d, diff %d, threshold %d) -> %s"),
           value, wetBaseline, abs(value - wetBaseline), dryDifference, isDry ? L("KURU", "DRY") : L("nemli", "moist"));
  iotbot.serialWrite(msg);
}

void mute() {
  if (!isDry) {
    iotbot.serialWrite(L("Toprak nemli, susturulacak bir ses yok.", "The soil is moist, there is no sound to silence."));
    return;
  }
  muted = true;
  iotbot.serialWrite(L("Hatırlatma sesi susturuldu (toprak nemlenene kadar).", "Reminder tone silenced (until the soil is moist)."));
  drawScreen();
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int number = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oku" || word == "read") {
    printReading();
  } else if (word == "kalibre" || word == "calibrate") {
    wetBaseline = readSoil();
    value = wetBaseline;
    isDry = false;
    muted = false;
    iotbot.serialWrite(String(L("Yeni nemli toprak referansı: ", "New moist soil reference: ")) + wetBaseline);
    drawScreen();
  } else if ((word == "esik" || word == "threshold") && hasValue) {
    dryDifference = constrain(number, 50, 2000);
    iotbot.serialWrite(String(L("Kuruluk eşiği: ", "Dryness threshold: ")) + dryDifference);
  } else if (word == "sustur" || word == "mute") {
    mute();
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
  iotbot.begin();
  iotbot.serialStart(115200);

  wetBaseline = readSoil();
  value = wetBaseline;
  iotbot.lcdWriteMid(L("BİTKİ SULAMA", "PLANT WATERING"), L("HATIRLATICISI", "REMINDER"), L("Sensörü toprak", "Insert the sensor"),
                     L("içine yerleştirin", "into the soil"));
  iotbot.serialWrite(L("Bitki sulama hatırlatıcısı hazır.", "Plant watering reminder ready."));
  iotbot.serialWrite(String(L("Nemli toprak referansı: ", "Moist soil reference: ")) + wetBaseline);
  printHelp();
  delay(2000); // Karşılama ekranı görünsün / show the welcome screen
  drawScreen();
}

void loop() {
  uint32_t now = millis();

  // 1) B3 -> sesi sustur (sadece basıldığı an) / B3 -> mute (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3 && now - lastB3Ms > 200) {
    lastB3Ms = now;
    mute();
  }
  lastB3 = b3;

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Okuma ve karar (yarım saniyede bir) / Reading and decision (every half second)
  if (now - lastReadMs >= kReadIntervalMs) {
    lastReadMs = now;
    value = readSoil();
    int diff = abs(value - wetBaseline);
    // Histerezis: %80'in altına inmeden "nemli" sayma, eşiğin etrafında gidip gelmesin.
    // Hysteresis: not "moist" until below 80%, so it does not flip around the threshold.
    bool wasDry = isDry;
    if (!isDry && diff >= dryDifference) isDry = true;
    else if (isDry && diff < dryDifference * 8 / 10) isDry = false;
    if (isDry != wasDry) {
      muted = false; // Yeni dönem: ses yeniden açık / new period: sound is on again
      iotbot.serialWrite(isDry ? L("SULAMA ZAMANI! Toprak kurumuş.", "TIME TO WATER! The soil is dry.")
                               : L("Toprak yeniden nemli, teşekkürler!", "The soil is moist again, thank you!"));
      lastReminderMs = 0;
    }
    drawScreen();
  }

  // 4) Hatırlatma sesi / Reminder tone
  if (isDry && !muted && (lastReminderMs == 0 || now - lastReminderMs >= kReminderEveryMs)) {
    lastReminderMs = now;
    iotbot.buzzerPlayTone(1000, 150);
    iotbot.buzzerPlayTone(1300, 150);
  }
  delay(10);
}
