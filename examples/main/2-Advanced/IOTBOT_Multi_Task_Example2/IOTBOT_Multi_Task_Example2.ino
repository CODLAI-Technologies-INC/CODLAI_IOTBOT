/*
 * TR: ÇİFT ÇEKİRDEK ÇOKLU GÖREV - LED'lerle görsel kanıt
 *  - ESP32'nin iki çekirdeğinde çalışan görevler, farklı LED gruplarını birbirinden
 *    bağımsız hızlarda kontrol eder:
 *      Çekirdek 0 (Görev 1): IO25, IO26, IO27 LED'lerini çok hızlı "Kara Şimşek" gibi kaydırır.
 *      Çekirdek 1 (Görev 2): IO32 ve IO33 LED'lerini yavaşça (1 sn) sırayla yakıp söndürür.
 *      Çekirdek 1 (Görev 3): LCD'yi günceller.
 *    Sonuç: biri çok hızlı, diğeri çok yavaş çalışmasına rağmen birbirlerini HİÇ etkilemez!
 *  - B3 butonu: hızlı görevi DURAKLAT / SÜRDÜR. Yavaş görev hiç durmadan devam eder -
 *    görevlerin bağımsız olduğunu kendi gözünüzle görün.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help              -> komut listesi
 *      hizli 50 / fast 50         -> hızlı görevin adım süresi (ms, 20-1000)
 *      yavas 1000 / slow 1000     -> yavaş görevin adım süresi (ms, 100-5000)
 *      dur    / pause             -> hızlı görevi duraklat (B3 ile aynı)
 *      devam  / resume            -> hızlı göreve devam et
 *      durum  / status            -> görev hızları ve çalışma sayıları
 *      dil    / lang              -> dili değiştir (Türkçe <-> English)
 *
 * EN: DUAL-CORE MULTI-TASKING - visual proof with LEDs
 *  - Tasks running on the ESP32's two cores control different LED groups at
 *    independent speeds:
 *      Core 0 (Task 1): sweeps the IO25, IO26, IO27 LEDs very fast ("Knight Rider").
 *      Core 1 (Task 2): blinks the IO32 and IO33 LEDs slowly (1 s) in turn.
 *      Core 1 (Task 3): updates the LCD.
 *    Result: one is very fast, the other very slow, yet they do NOT affect each other!
 *  - B3 button: PAUSE / RESUME the fast task. The slow task keeps going without a
 *    break - see with your own eyes that the tasks are independent.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim            -> command list
 *      fast 50 / hizli 50         -> step time of the fast task (ms, 20-1000)
 *      slow 1000 / yavas 1000     -> step time of the slow task (ms, 100-5000)
 *      pause  / dur               -> pause the fast task (same as B3)
 *      resume / devam             -> resume the fast task
 *      status / durum             -> task speeds and run counts
 *      lang   / dil               -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: LED'ler P1-P5 hatlarındadır (IO25, IO26, IO27, IO32, IO33); ek modül
 * GEREKMEZ. Soketlere başka modül takılıysa çıkarın. / The LEDs are on the P1-P5 lines
 * (IO25, IO26, IO27, IO32, IO33); NO extra module needed. Remove other modules from the sockets.
 */

#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

// Sadece ESP32 için geçerlidir / Valid only for ESP32
#if !defined(ESP32)
  #error "This example is designed for ESP32 only! / Bu örnek sadece ESP32 içindir!"
#endif

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// LED pinleri / LED pins
const int ledPinsFast[] = {IO25, IO26, IO27}; // Çekirdek 0 kontrol eder (hızlı) / controlled by core 0 (fast)
const int ledPinsSlow[] = {IO32, IO33};       // Çekirdek 1 kontrol eder (yavaş) / controlled by core 1 (slow)

// Görevler arasında paylaşılan değişkenler / variables shared between tasks
volatile int fastDelayMs = 100;
volatile int slowDelayMs = 1000;
volatile bool fastPaused = false;
volatile uint32_t fastRuns = 0, slowRuns = 0;

void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

// ---------------------------------------------------------------------------
// Görev 1: Hızlı LED efekti (Çekirdek 0) / Task 1: fast LED effect (Core 0)
// ---------------------------------------------------------------------------
void TaskFastLEDs() {
  static int direction = 1;
  static int currentLed = 0;

  if (!fastPaused) {
    fastRuns++;
    // Tüm hızlı LED'leri söndür, şu ankini yak / turn all fast LEDs off, light the current one
    for (int i = 0; i < 3; i++) digitalWrite(ledPinsFast[i], LOW);
    digitalWrite(ledPinsFast[currentLed], HIGH);

    // Bir sonraki LED (uçlarda yön değiştir) / next LED (reverse at the ends)
    currentLed += direction;
    if (currentLed >= 2 || currentLed <= 0) direction *= -1;
  }
  iotbot.taskDelay(fastDelayMs); // Çok kısa bekleme = çok hızlı / very short wait = very fast
}

// ---------------------------------------------------------------------------
// Görev 2: Yavaş yanıp sönme (Çekirdek 1) / Task 2: slow blink (Core 1)
// ---------------------------------------------------------------------------
void TaskSlowLEDs() {
  static bool state = false;
  state = !state;
  slowRuns++;
  digitalWrite(ledPinsSlow[0], state);
  digitalWrite(ledPinsSlow[1], !state); // Ters çalışsın / the other one opposite

  // Seri Port dolmasın diye 5 adımda bir yaz / print every 5 steps so the Serial Monitor does not overflow
  if (slowRuns % 5 == 0) {
    iotbot.serialWrite(String(L("Yavaş görev çalışıyor, çekirdek: ", "Slow task running on core: ")) + xPortGetCoreID());
  }
  iotbot.taskDelay(slowDelayMs);
}

// ---------------------------------------------------------------------------
// Görev 3: Bilgi ekranı (Çekirdek 1) - LCD'yi SADECE bu görev kullanır
// Task 3: info screen (Core 1) - ONLY this task uses the LCD
// ---------------------------------------------------------------------------
void TaskInfo() {
  char line[41];
  snprintf(line, sizeof(line), L("Ç0 hızlı: %s", "C0 fast: %s"), fastPaused ? L("DURAKLADI", "PAUSED") : L("çalışıyor", "running"));
  lcdRow(0, line);
  snprintf(line, sizeof(line), L("Ç1 yavaş: %d ms", "C1 slow: %d ms"), slowDelayMs);
  lcdRow(1, line);
  snprintf(line, sizeof(line), L("Süre: %lu sn", "Uptime: %lu s"), (unsigned long)(millis() / 1000));
  lcdRow(2, line);
  lcdRow(3, fastPaused ? L("B3: hızlıyı sürdür", "B3: resume fast") : L("B3: hızlıyı durdur", "B3: pause fast"));
  iotbot.taskDelay(300);
}

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "HIZLI" -> "hizli"
// Lower-cases and simplifies Turkish letters: "HIZLI" -> "hizli"
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

void printHelp() {
  iotbot.serialWrite(L("---- ÇİFT ÇEKİRDEK LED - Komutlar ----", "---- DUAL-CORE LEDS - Commands ----"));
  iotbot.serialWrite(L("  yardim      : bu liste", "  help        : this list"));
  iotbot.serialWrite(L("  hizli 50    : hızlı görev adımı (ms)", "  fast 50     : fast task step (ms)"));
  iotbot.serialWrite(L("  yavas 1000  : yavaş görev adımı (ms)", "  slow 1000   : slow task step (ms)"));
  iotbot.serialWrite(L("  dur / devam : hızlı görevi duraklat / sürdür", "  pause / resume : pause / resume the fast task"));
  iotbot.serialWrite(L("  durum       : hızlar ve çalışma sayıları", "  status      : speeds and run counts"));
  iotbot.serialWrite(L("  dil         : English'e geç", "  lang        : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu   : hızlı görevi duraklat / sürdür", "  B3 button   : pause / resume the fast task"));
}

void setFastPaused(bool paused) {
  fastPaused = paused;
  iotbot.buzzerPlayTone(paused ? 800 : 1500, 50);
  iotbot.serialWrite(paused ? L(">> Hızlı görev DURAKLADI - yavaş görev devam ediyor!", ">> Fast task PAUSED - the slow task keeps going!")
                            : L(">> Hızlı görev devam ediyor.", ">> Fast task resumed."));
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if ((word == "hizli" || word == "fast") && hasValue) {
    fastDelayMs = constrain(value, 20, 1000);
    iotbot.serialWrite(String(L("Hızlı görev adımı: ", "Fast task step: ")) + fastDelayMs + " ms");
  } else if ((word == "yavas" || word == "slow") && hasValue) {
    slowDelayMs = constrain(value, 100, 5000);
    iotbot.serialWrite(String(L("Yavaş görev adımı: ", "Slow task step: ")) + slowDelayMs + " ms");
  } else if (word == "dur" || word == "pause" || word == "stop") {
    setFastPaused(true);
  } else if (word == "devam" || word == "resume") {
    setFastPaused(false);
  } else if (word == "durum" || word == "status") {
    iotbot.serialWrite(String(L("Hızlı görev (Ç0): ", "Fast task (C0): ")) + fastDelayMs + L(" ms, ", " ms, ") + (unsigned long)fastRuns +
                       L(" adım", " steps") + (fastPaused ? L(" (DURAKLADI)", " (PAUSED)") : ""));
    iotbot.serialWrite(String(L("Yavaş görev (Ç1): ", "Slow task (C1): ")) + slowDelayMs + L(" ms, ", " ms, ") + (unsigned long)slowRuns + L(" adım", " steps"));
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);

  // Pin modlarını ayarla / set the pin modes
  for (int i = 0; i < 3; i++) pinMode(ledPinsFast[i], OUTPUT);
  for (int i = 0; i < 2; i++) pinMode(ledPinsSlow[i], OUTPUT);

  iotbot.lcdShowLoading(L("Çift çekirdek demo", "Dual core demo"));
  iotbot.lcdClear();

  // Görevleri başlat / start the tasks
  iotbot.createLoopTask(TaskFastLEDs, "FastLED", 0, 1); // Görev 1 -> Çekirdek 0 (hızlı) / Task 1 -> Core 0 (fast)
  iotbot.createLoopTask(TaskSlowLEDs, "SlowLED", 1, 1); // Görev 2 -> Çekirdek 1 (yavaş) / Task 2 -> Core 1 (slow)
  iotbot.createLoopTask(TaskInfo, "Info", 1, 1);        // Görev 3 -> Çekirdek 1 (bilgi) / Task 3 -> Core 1 (info)

  iotbot.serialWrite(L("Görevler başladı! LED'leri izleyin.", "Tasks started! Watch the LEDs."));
  printHelp();
}

void loop() {
  // loop() da bir görevdir: B3 ve seri komutları burada dinliyoruz.
  // loop() is a task too: we listen for B3 and serial commands here.
  static bool lastB3 = false;
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) setFastPaused(!fastPaused);
  lastB3 = b3;

  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
  iotbot.taskDelay(20);
}
