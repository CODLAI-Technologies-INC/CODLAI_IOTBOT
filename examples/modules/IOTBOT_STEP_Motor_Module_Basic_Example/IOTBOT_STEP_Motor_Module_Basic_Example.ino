/*
 * TR: STEP MOTOR MODÜLÜ - Otomatik demo + Manuel kontrol
 *  - Açılışta OTOMATİK mod çalışır: motor 1 tur ileri, mola, 1 tur geri, mola,
 *    yarım tur hızlı ileri/geri ... şeklinde kendi kendine döner.
 *  - B3 butonuna basınca MANUEL moda geçer:
 *      B1 basılı tut = ileri döndür, B2 basılı tut = geri döndür,
 *      potansiyometre = hız (RPM). B3'e tekrar basınca otomatik moda döner.
 *  - Motor boşta kalınca bobinlerin akımı kesilir (motor ve sürücü ısınmaz).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim     / help        -> komut listesi
 *      oto        / auto        -> otomatik mod
 *      manuel     / manual      -> manuel mod (B1/B2 + potansiyometre)
 *      adim 100   / step 100    -> 100 adım ileri (eksi = geri, manuel moda geçer)
 *      hiz 60     / speed 60    -> hız RPM (30-120)
 *      dur        / stop        -> hareketi durdur
 *      dil        / lang        -> dili değiştir (Türkçe <-> English)
 *
 * EN: STEP MOTOR MODULE - Automatic demo + Manual control
 *  - At startup AUTO mode runs: the motor turns 1 rev forward, rests, 1 rev
 *    back, rests, a fast half rev forward/back ... all by itself.
 *  - Press B3 to switch to MANUAL mode:
 *      hold B1 = turn forward, hold B2 = turn backward,
 *      potentiometer = speed (RPM). Press B3 again to go back to auto mode.
 *  - When the motor is idle the coil current is cut (motor and driver stay cool).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help       / yardim      -> command list
 *      auto       / oto         -> auto mode
 *      manual     / manuel      -> manual mode (B1/B2 + potentiometer)
 *      step 100   / adim 100    -> 100 steps forward (minus = backward, switches to manual)
 *      speed 60   / hiz 60      -> speed in RPM (30-120)
 *      stop       / dur         -> stop the move
 *      lang       / dil         -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Step motoru P6 motor soketine takın. IO26, IO33, IO32 ve
 * IO27 pinlerini kullanır; bu pinlere başka modül takmayın. / Plug the step
 * motor into the P6 motor socket. It uses IO26, IO33, IO32 and IO27; do not plug
 * other modules into these pins.
 */

#define USE_STEP_MOTOR
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Ayarlar / Settings
// Motorunuzun bir turdaki adım sayısı. 4'ün katı olmalı (aşağıdaki "neden 4?" notuna bakın).
// Steps per revolution of your motor. Must be a multiple of 4 (see the "why 4?" note below).
const int STEPS_PER_REV = 48;
const int CHUNK_STEPS = 4;      // Tek seferde atılan adım / steps per chunk
const int MIN_RPM = 30;         // En düşük hız / lowest speed
const int MAX_RPM = 120;        // En yüksek hız / highest speed
const uint32_t RELEASE_MS = 300; // Bu kadar boşta kalınca bobinleri bırak / release coils after this idle time

// Otomatik demo: adım (eksi = geri), hız (RPM), sonra mola (ms)
// Auto demo: steps (minus = backward), speed (RPM), then rest (ms)
const int DEMO_STEPS[] = {STEPS_PER_REV, -STEPS_PER_REV, STEPS_PER_REV / 2, -STEPS_PER_REV / 2};
const int DEMO_RPM[] = {60, 60, 120, 120};
const uint32_t DEMO_REST[] = {1000, 1000, 500, 1500};
const int DEMO_COUNT = sizeof(DEMO_STEPS) / sizeof(DEMO_STEPS[0]);

bool manualMode = false;   // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
int rpm = 60;              // Şu anki hız / current speed
long stepsLeft = 0;        // Atılacak adım (eksi = geri) / steps still to go (minus = backward)
long position = 0;         // Başlangıçtan beri toplam adım / total steps since start
bool coilsOn = false;      // Bobinlerde akım var mı? / are the coils powered?
uint32_t idleSinceMs = 0;  // Hareketin bittiği an / when the motion ended
uint32_t lastChunkUs = 0;  // Son parçanın bittiği an / when the last chunk ended
int demoIndex = 0;
bool demoResting = false;
uint32_t demoRestUntilMs = 0;
uint32_t lastScreenMs = 0;
bool lastB3 = false;
int lastPotRpm = -1;       // Potansiyometrenin son hızı / last potentiometer speed

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "ADIM" -> "adim"
// Lower-cases and simplifies Turkish letters: "ADIM" -> "adim"
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
// Motor yardımcıları / Motor helpers
// ---------------------------------------------------------------------------
// moduleStepMotorMotion() hareket bitene kadar BEKLER (blocking). Bu yüzden motoru
// 4'er adımlık küçük parçalarla döndürürüz; parçalar arasında buton ve seri port okunur.
// NEDEN 4? Kütüphane her çağrıda bobin sırasını baştan başlatır; 4 adım = bir tam bobin
// döngüsü olduğundan her parça bir öncekinin kaldığı yerden devam eder (adım kaçmaz).
// moduleStepMotorMotion() WAITS until the move ends (blocking). So we turn the motor in
// small 4-step chunks and read the buttons and serial port in between.
// WHY 4? The library restarts the coil sequence on every call; 4 steps = one full coil
// cycle, so every chunk continues exactly where the previous one stopped (no lost steps).

// Bir adımın süresi (mikrosaniye) / time of one step (microseconds)
uint32_t stepIntervalUs() { return 60000000UL / ((uint32_t)STEPS_PER_REV * rpm); }

// Hareketten sonra bobinlerdeki akımı keseriz: Stepper kütüphanesi son adımı enerjili bırakır,
// motor ve sürücü boşuna ısınır. / After moving we cut the coil current: the Stepper library
// leaves the last step energized and the motor and driver heat up for nothing.
void releaseCoils() {
  const int coilPins[] = {IO26, IO33, IO32, IO27};
  for (int pin : coilPins) digitalWrite(pin, LOW);
  coilsOn = false;
}

// Adım sayısını 4'ün katına yuvarlar / rounds a step count to a multiple of 4
long roundToChunk(long steps) {
  long half = (steps >= 0) ? CHUNK_STEPS / 2 : -CHUNK_STEPS / 2;
  return (steps + half) / CHUNK_STEPS * CHUNK_STEPS;
}

void stopMotor() {
  stepsLeft = 0;
  releaseCoils();
}

// Sırada adım varsa ve zamanı geldiyse 4 adımlık bir parça at.
// If there are steps to go and it is time, run one 4-step chunk.
void runMotor() {
  if (stepsLeft == 0) return;
  if (micros() - lastChunkUs < stepIntervalUs()) return; // Parçalar arası bir adım süresi bekle / one step time between chunks
  bool forward = stepsLeft > 0;
  iotbot.moduleStepMotorMotion(STEPS_PER_REV, forward, CHUNK_STEPS, rpm); // ~3 adım süresi bloklar / blocks ~3 step times
  lastChunkUs = micros();
  coilsOn = true;
  stepsLeft += forward ? -CHUNK_STEPS : CHUNK_STEPS;
  position += forward ? CHUNK_STEPS : -CHUNK_STEPS;
  if (stepsLeft == 0) idleSinceMs = millis();
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
// lcdWriteFixedTxt Türkçe harfleri (ç, ğ, ı, ö, ş, ü) LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- STEP MOTOR - Komutlar ----", "---- STEP MOTOR - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oto           : otomatik mod", "  auto          : auto mode"));
  iotbot.serialWrite(L("  manuel        : manuel mod (B1/B2 + pot)", "  manual        : manual mode (B1/B2 + pot)"));
  iotbot.serialWrite(L("  adim N        : N adım döndür (eksi = geri)", "  step N        : turn N steps (minus = backward)"));
  iotbot.serialWrite(L("  hiz 30-120    : hız (RPM)", "  speed 30-120  : speed (RPM)"));
  iotbot.serialWrite(L("  dur           : hareketi durdur", "  stop          : stop the move"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : OTOMATİK <-> MANUEL", "  B3 button     : AUTO <-> MANUAL"));
  iotbot.serialWrite(L("  B1 / B2 basılı: ileri / geri (manuel)", "  hold B1 / B2  : forward / backward (manual)"));
}

void drawStaticScreen() {
  lcdRow(0, "     STEP MOTOR");
  lcdRow(3, manualMode ? L("B1/B2:döndür B3:oto", "B1/B2:turn  B3:auto") : L("B3: manuel kontrol", "B3: manual control"));
  lastScreenMs = 0; // Değerleri hemen çiz / draw the values right away
}

void startDemoStep() {
  stepsLeft = DEMO_STEPS[demoIndex];
  rpm = DEMO_RPM[demoIndex];
  demoResting = false;
  iotbot.serialWrite(String(L("Otomatik: ", "Auto: ")) + stepsLeft + L(" adım, ", " steps, ") + rpm + " RPM");
}

void setMode(bool manual) {
  manualMode = manual;
  stopMotor(); // Güvenlik: mod değişince motor durur / safety: motor stops on mode change
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  iotbot.serialWrite(manual ? L(">> MANUEL mod: B1 = ileri, B2 = geri (basılı tutun), pot = hız.", ">> MANUAL mode: B1 = forward, B2 = backward (hold), pot = speed.")
                            : L(">> OTOMATİK mod: motor demo hareketlerini yapıyor.", ">> AUTO mode: the motor runs the demo moves."));
  lastPotRpm = -1; // Manuelde pot hemen geçerli olsun / pot takes effect immediately in manual
  if (!manual) {
    demoIndex = 0;
    startDemoStep();
  }
  drawStaticScreen();
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  long value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oto" || word == "otomatik" || word == "auto") {
    setMode(false);
  } else if (word == "manuel" || word == "manual") {
    setMode(true);
  } else if ((word == "adim" || word == "step") && hasValue) {
    if (!manualMode) setMode(true);
    long steps = roundToChunk(constrain(value, -100000L, 100000L));
    stepsLeft = steps;
    iotbot.serialWrite(String(L("Adım: ", "Steps: ")) + steps + (steps != value ? L("  (4'ün katına yuvarlandı)", "  (rounded to a multiple of 4)") : ""));
  } else if ((word == "hiz" || word == "speed") && hasValue) {
    if (!manualMode) setMode(true);
    rpm = constrain((int)value, MIN_RPM, MAX_RPM);
    // Seri komutla verilen hızı potansiyometre hemen ezmesin diye pot konumunu kaydet.
    // Remember the pot position so it does not override the serial speed right away.
    lastPotRpm = map(iotbot.potentiometerRead(), 0, 4095, MIN_RPM, MAX_RPM);
    iotbot.serialWrite(String(L("Hız: ", "Speed: ")) + rpm + " RPM");
  } else if (word == "dur" || word == "stop") {
    if (!manualMode) setMode(true);
    stopMotor();
    iotbot.serialWrite(L("Motor durdu.", "Motor stopped."));
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    drawStaticScreen();
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();             // IoTBot başlatılıyor / Initialize IoTBot
  iotbot.serialStart(115200); // Seri haberleşme / Serial communication
  const int coilPins[] = {IO26, IO33, IO32, IO27};
  for (int pin : coilPins) pinMode(pin, OUTPUT);
  releaseCoils();             // Güvenlik: bobinler boşta / safety: coils off
  iotbot.lcdClear();
  drawStaticScreen();
  iotbot.serialWrite(L("Step motor testi başladı.", "Step motor test started."));
  printHelp();
  startDemoStep();
}

void loop() {
  uint32_t now = millis();

  // 1) B3 -> mod değiştir (sadece basıldığı an) / B3 -> toggle mode (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) setMode(!manualMode);
  lastB3 = b3;

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Ne kadar adım atılacağını belirle / Decide how many steps to go
  if (manualMode) {
    // Pot sadece gerçekten çevrilince hızı değiştirir (titreşimi yok saymak için 3 RPM eşik).
    // The pot changes the speed only when really turned (3 RPM threshold ignores jitter).
    int potRpm = map(iotbot.potentiometerRead(), 0, 4095, MIN_RPM, MAX_RPM);
    if (lastPotRpm < 0 || abs(potRpm - lastPotRpm) >= 3) {
      lastPotRpm = potRpm;
      rpm = potRpm;
    }
    // B1 / B2 basılı tutuldukça bir parça daha ekle (bırakınca motor hemen durur).
    // While B1 / B2 is held keep adding one chunk (release it and the motor stops at once).
    if (stepsLeft == 0) {
      if (iotbot.button1Read()) stepsLeft = CHUNK_STEPS;
      else if (iotbot.button2Read()) stepsLeft = -CHUNK_STEPS;
    }
  } else if (stepsLeft == 0) {
    if (!demoResting) { // Hareket bitti: mola başlasın / move finished: start the rest
      demoResting = true;
      demoRestUntilMs = now + DEMO_REST[demoIndex];
    } else if ((int32_t)(now - demoRestUntilMs) >= 0) { // Mola bitti: sonraki hareket / rest over: next move
      demoIndex = (demoIndex + 1) % DEMO_COUNT;
      startDemoStep();
    }
  }

  // 4) Motoru bir parça döndür ve boşta kalınca bobinleri bırak.
  // 4) Turn the motor one chunk and release the coils when it stays idle.
  runMotor();
  if (stepsLeft == 0 && coilsOn && millis() - idleSinceMs >= RELEASE_MS) releaseCoils();

  // 5) LCD (250 ms'de bir, titremesiz) / LCD (every 250 ms, no flicker)
  if (millis() - lastScreenMs >= 250) {
    lastScreenMs = millis();
    char line[41];
    snprintf(line, sizeof(line), L("Mod: %s", "Mode: %s"), manualMode ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO"));
    lcdRow(1, line);
    snprintf(line, sizeof(line), L("Hız:%3d RPM Poz:%ld", "Spd:%3d RPM Pos:%ld"), rpm, position);
    lcdRow(2, line);
  }
}
