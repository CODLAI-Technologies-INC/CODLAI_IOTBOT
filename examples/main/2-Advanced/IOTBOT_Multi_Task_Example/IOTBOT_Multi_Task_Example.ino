/*
 * TR: ÇOKLU GÖREV (MULTI-TASKING) - ESP32'nin iki çekirdeğiyle aynı anda birden fazla iş
 *  - IOTBOT kütüphanesinin 'createLoopTask' fonksiyonu sayesinde karmaşık FreeRTOS
 *    kodu yazmadan birden fazla görevi aynı anda çalıştırabilirsiniz.
 *  - Görevler:
 *      Görev 1 (Çekirdek 0): LCD ekranını günceller (süre, pot değeri, bip sayısı).
 *      Görev 2 (Çekirdek 1): Potansiyometreyi okur; değer değişince Seri Port'a yazar.
 *      Görev 3 (Çekirdek 1): 2 saniyede bir buzzer ile bip yapar.
 *      loop() de ayrı bir görevdir: seri komutları dinler.
 *  - Nasıl kullanılır?
 *    1) Görev fonksiyonunu yazın: void GorevAdi() { ... }  İçine sonsuz döngü
 *       (while(1) / for(;;)) YAZMAYIN - kütüphane fonksiyonu sürekli tekrar çağırır.
 *    2) setup() içinde başlatın: iotbot.createLoopTask(Fonksiyon, "İsim", Çekirdek, Öncelik, YığınBoyutu);
 *       Çekirdek 0: genelde WiFi/arka plan işleri, Çekirdek 1: loop() burada çalışır.
 *       Öncelik: 1-9 (varsayılan 1), Yığın: bellek boyutu (varsayılan 10000).
 *    3) Görevin içinde beklemek için iotbot.taskDelay(ms) kullanın (işlemciyi meşgul etmez).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help    -> komut listesi
 *      sessiz / mute    -> Görev 3'ün bip sesini aç/kapat
 *      durum  / status  -> her görev kaç kez çalıştı
 *      dil    / lang    -> dili değiştir (Türkçe <-> English)
 *
 * EN: MULTI-TASKING - several jobs at the same time with the ESP32's two cores
 *  - Thanks to the IOTBOT library's 'createLoopTask' you can run several tasks at the
 *    same time without writing complex FreeRTOS code.
 *  - Tasks:
 *      Task 1 (Core 0): updates the LCD (uptime, pot value, beep count).
 *      Task 2 (Core 1): reads the potentiometer; prints to Serial when it changes.
 *      Task 3 (Core 1): beeps the buzzer every 2 seconds.
 *      loop() is a task too: it listens for serial commands.
 *  - How to use?
 *    1) Write the task function: void TaskName() { ... }  Do NOT write an endless
 *       loop (while(1) / for(;;)) inside - the library calls the function again and again.
 *    2) Start it in setup(): iotbot.createLoopTask(Function, "Name", Core, Priority, StackSize);
 *       Core 0: usually WiFi/background work, Core 1: loop() runs here.
 *       Priority: 1-9 (default 1), Stack: memory size (default 10000).
 *    3) Use iotbot.taskDelay(ms) to wait inside a task (it does not keep the CPU busy).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim  -> command list
 *      mute   / sessiz  -> turn Task 3's beep on/off
 *      status / durum   -> how many times each task ran
 *      lang   / dil     -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. / NO extra module needed. (Sadece ESP32 / ESP32 only)
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

// Görevler arasında paylaşılan değişkenler (volatile: başka görev değiştirebilir)
// Variables shared between tasks (volatile: another task may change them)
volatile int potValue = 0;
volatile bool beepEnabled = true;
volatile uint32_t task1Runs = 0, task2Runs = 0, task3Runs = 0, beepCount = 0;

// LCD satırı yazar (Türkçe harfler doğru görünür). LCD'yi SADECE Görev 1 kullanır -
// iki görev aynı anda LCD'ye yazarsa ekran bozulur.
// Writes an LCD row (Turkish letters show correctly). ONLY Task 1 uses the LCD -
// if two tasks wrote to the LCD at the same time the screen would get corrupted.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

// ---------------------------------------------------------------------------
// Görev 1: LCD güncelleme (Çekirdek 0) / Task 1: LCD update (Core 0)
// for(;;) ya da while(1) gerekmez: createLoopTask bunu sizin için yapar.
// No for(;;) or while(1) needed: createLoopTask does it for you.
// ---------------------------------------------------------------------------
void Task1code() {
  char line[41];
  task1Runs++;
  lcdRow(0, L("Görev 1: LCD (Ç0)", "Task 1: LCD (C0)"));
  snprintf(line, sizeof(line), L("Süre: %lu sn", "Uptime: %lu s"), (unsigned long)(millis() / 1000));
  lcdRow(1, line);
  snprintf(line, sizeof(line), L("Görev 2 pot: %4d", "Task 2 pot: %4d"), potValue);
  lcdRow(2, line);
  snprintf(line, sizeof(line), L("Görev 3 bip: %lu%s", "Task 3 beep: %lu%s"), (unsigned long)beepCount, beepEnabled ? "" : L(" (sessiz)", " (mute)"));
  lcdRow(3, line);
  iotbot.taskDelay(500); // 0,5 sn bekle (işlemciyi meşgul etmez) / wait 0.5 s (does not keep the CPU busy)
}

// ---------------------------------------------------------------------------
// Görev 2: Sensör okuma (Çekirdek 1) / Task 2: sensor reading (Core 1)
// Değer belirgin değişince yazdırır (Seri Port dolup taşmasın).
// Prints only when the value clearly changes (so the Serial Monitor does not overflow).
// ---------------------------------------------------------------------------
void Task2code() {
  static int lastPrinted = -1000;
  task2Runs++;
  potValue = iotbot.potentiometerRead();
  if (abs(potValue - lastPrinted) >= 100) {
    lastPrinted = potValue;
    iotbot.serialWrite(String(L("Görev 2 - Pot değeri: ", "Task 2 - Pot value: ")) + potValue);
  }
  iotbot.taskDelay(200); // 200 ms bekle / wait 200 ms
}

// ---------------------------------------------------------------------------
// Görev 3: Buzzer (Çekirdek 1) / Task 3: buzzer (Core 1)
// ---------------------------------------------------------------------------
void Task3code() {
  task3Runs++;
  if (beepEnabled) {
    iotbot.buzzerPlayTone(1000, 100);
    beepCount++;
  }
  iotbot.taskDelay(2000); // 2 sn bekle / wait 2 s
}

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "SESSİZ" -> "sessiz"
// Lower-cases and simplifies Turkish letters: "SESSİZ" -> "sessiz"
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
  iotbot.serialWrite(L("---- ÇOKLU GÖREV - Komutlar ----", "---- MULTI-TASK - Commands ----"));
  iotbot.serialWrite(L("  yardim : bu liste", "  help   : this list"));
  iotbot.serialWrite(L("  sessiz : Görev 3'ün bip sesini aç/kapat", "  mute   : turn Task 3's beep on/off"));
  iotbot.serialWrite(L("  durum  : görevlerin çalışma sayıları", "  status : how many times each task ran"));
  iotbot.serialWrite(L("  dil    : English'e geç", "  lang   : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "sessiz" || cmd == "mute") {
    beepEnabled = !beepEnabled;
    iotbot.serialWrite(beepEnabled ? L("Görev 3: bip AÇIK.", "Task 3: beep ON.") : L("Görev 3: bip KAPALI (görev yine çalışıyor).", "Task 3: beep OFF (the task still runs)."));
  } else if (cmd == "durum" || cmd == "status") {
    iotbot.serialWrite(String(L("Görev 1 (LCD, Ç0): ", "Task 1 (LCD, C0): ")) + (unsigned long)task1Runs + L(" kez", " runs"));
    iotbot.serialWrite(String(L("Görev 2 (Pot, Ç1): ", "Task 2 (Pot, C1): ")) + (unsigned long)task2Runs + L(" kez", " runs"));
    iotbot.serialWrite(String(L("Görev 3 (Bip, Ç1): ", "Task 3 (Beep, C1): ")) + (unsigned long)task3Runs + L(" kez", " runs"));
    iotbot.serialWrite(String(L("loop() bu çekirdekte çalışıyor: ", "loop() is running on core: ")) + xPortGetCoreID());
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

  iotbot.lcdShowLoading(L("Görevler başlıyor", "Starting tasks"));
  iotbot.lcdClear();

  // Görevleri başlat / start the tasks
  // Kullanım / Usage: iotbot.createLoopTask(Fonksiyon/Function, "İsim/Name", Çekirdek/Core, Öncelik/Priority, Yığın/Stack);
  iotbot.createLoopTask(Task1code, "Task1", 0, 1); // Çekirdek 0: arka plan işleri için önerilir / Core 0: for background work
  iotbot.createLoopTask(Task2code, "Task2", 1, 1); // Çekirdek 1: ana işler ve sensörler / Core 1: main work and sensors
  iotbot.createLoopTask(Task3code, "Task3", 1, 1);

  iotbot.serialWrite(L("Tüm görevler başladı!", "All tasks started!"));
  printHelp();
}

void loop() {
  // loop() da bir görevdir: burada seri komutları dinliyoruz. LCD'ye burada yazmıyoruz (Görev 1'in işi).
  // loop() is a task too: here we listen for serial commands. We do not write to the LCD here (that is Task 1's job).
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
  iotbot.taskDelay(20);
}
