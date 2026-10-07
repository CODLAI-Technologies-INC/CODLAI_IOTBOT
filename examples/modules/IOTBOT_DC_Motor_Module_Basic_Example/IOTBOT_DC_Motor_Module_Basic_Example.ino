/*
 * TR: DC MOTOR MODÜLÜ - Otomatik demo + Manuel kontrol
 *  - Açılışta motor DURUR, sonra OTOMATİK demo başlar: ileri %50 -> dur ->
 *    geri %75 -> dur ... Hız her zaman yumuşak rampayla değişir; yön
 *    değişirken motor önce 0'a iner, kısa bir mola verir, sonra ters döner.
 *  - B3 butonuna basınca MANUEL moda geçer: hızı potansiyometre ile ayarlarsınız.
 *      Orta = DUR, sağa çevir = İLERİ, sola çevir = GERİ (ortada "ölü bölge" var).
 *    Güvenlik: manuele geçince motor durur; potu bir kez ORTAYA getirene kadar
 *    pot motoru çalıştırmaz. B3'e tekrar basınca otomatik moda döner.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim     / help        -> komut listesi
 *      oto        / auto        -> otomatik mod
 *      manuel     / manual      -> manuel mod (potansiyometre)
 *      hiz 50     / speed 50    -> hız -100..100 (eksi = geri, manuel moda geçer)
 *      dur        / stop        -> motoru yumuşakça durdur
 *      dil        / lang        -> dili değiştir (Türkçe <-> English)
 *
 * EN: DC MOTOR MODULE - Automatic demo + Manual control
 *  - At startup the motor is STOPPED, then the AUTO demo begins: forward 50% ->
 *    stop -> reverse 75% -> stop ... The speed always changes on a soft ramp;
 *    when reversing, the motor first slows to 0, rests briefly, then turns back.
 *  - Press B3 to switch to MANUAL mode: set the speed with the potentiometer.
 *      Center = STOP, turn right = FORWARD, turn left = REVERSE (with a dead zone).
 *    Safety: entering manual stops the motor; the pot does nothing until you
 *    bring it to the CENTER once. Press B3 again to go back to auto mode.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help       / yardim      -> command list
 *      auto       / oto         -> auto mode
 *      manual     / manuel      -> manual mode (potentiometer)
 *      speed 50   / hiz 50      -> speed -100..100 (minus = reverse, switches to manual)
 *      stop       / dur         -> stop the motor softly
 *      lang       / dil         -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: DC motoru P6 motor sürücü soketine takın. Motor IO26 ve
 * IO27'yi kullanır; bu pinlere başka modül takmayın. / Plug the DC motor into the
 * P6 motor driver socket. It uses IO26 and IO27; do not plug other modules there.
 */

#define USE_DC_MOTOR
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Ayarlar / Settings
const int MIN_PWM = 60;             // Motorun dönmeye başladığı PWM (0-255) / PWM where the motor starts turning
const int POT_DEAD_ZONE = 250;      // Pot ortası +/- bu kadar = DUR (0-4095) / pot center +/- this = STOP
const int RAMP_STEP = 2;            // Her rampa adımında % değişim / % change per ramp step
const uint32_t RAMP_MS = 15;        // Rampa adım aralığı (0->%100 ~0.75 sn) / ramp step interval
const uint32_t REVERSE_REST_MS = 300; // Ters dönmeden önce mola / rest before reversing

// Otomatik demo adımları: hız (%) ve süre (ms) / Auto demo steps: speed (%) and time (ms)
const int DEMO_SPEED[] = {0, 50, 0, -75, 0, 100, 0, -40};
const uint32_t DEMO_TIME[] = {2000, 3000, 1500, 3000, 1500, 2500, 1500, 3000};
const int DEMO_COUNT = sizeof(DEMO_SPEED) / sizeof(DEMO_SPEED[0]);

bool manualMode = false;    // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
int targetSpeed = 0;        // İstenen hız % (-100..100) / wanted speed %
int currentSpeed = 0;       // Motora şu an verilen hız % / speed applied right now %
int appliedPwm = 9999;      // Motora en son yazılan işaretli PWM / last signed PWM written to the motor
bool potWaitCenter = true;  // Güvenlik kilidi: pot önce ortaya gelmeli / safety lock: pot must be centered first
bool potFollow = false;     // Hız potu takip ediyor mu? / is the speed following the pot?
int potRef = 0;             // Seri komut anındaki pot hızı / pot speed at the time of the serial command
int demoIndex = 0;
uint32_t demoStepMs = 0;    // Demo adımının başladığı an / when the demo step started
uint32_t lastRampMs = 0;
uint32_t restUntilMs = 0;   // Bu ana kadar 0'da bekle / stay at 0 until this time
uint32_t lastScreenMs = 0;
bool lastB3 = false;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "HIZ" -> "hiz"
// Lower-cases and simplifies Turkish letters: "HIZ" -> "hiz"
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
// Yüzdeyi PWM'e çevirir: %1 bile motoru döndürsün diye MIN_PWM'den başlar.
// Converts percent to PWM: starts at MIN_PWM so even 1% really turns the motor.
int percentToPwm(int pct) {
  if (pct == 0) return 0;
  int pwm = map(abs(pct), 1, 100, MIN_PWM, 255);
  return pct > 0 ? pwm : -pwm;
}

// İşaretli hızı motora uygular: + saat yönü (ileri), - ters yön (geri), 0 dur.
// Applies the signed speed: + clockwise (forward), - counter-clockwise (reverse), 0 stop.
void applyMotor(int pct) {
  int pwm = percentToPwm(pct);
  if (pwm == appliedPwm) return; // Değişmediyse tekrar yazma / don't rewrite an unchanged value
  appliedPwm = pwm;
  if (pwm > 0) iotbot.moduleDCMotorGOClockWise(pwm);
  else if (pwm < 0) iotbot.moduleDCMotorGOCounterClockWise(-pwm);
  else iotbot.moduleDCMotorStop();
}

// Pot değerini -100..100 hıza çevirir (ortada ölü bölge). / Pot value -> speed -100..100 (dead zone in the middle).
int potToSpeed(int raw) {
  const int center = 2048;
  if (raw > center + POT_DEAD_ZONE) return map(raw, center + POT_DEAD_ZONE, 4095, 1, 100);
  if (raw < center - POT_DEAD_ZONE) return -map(raw, center - POT_DEAD_ZONE, 0, 1, 100);
  return 0;
}

const char *directionText(int pct) {
  if (pct > 0) return L("İLERİ", "FORWARD");
  if (pct < 0) return L("GERİ", "REVERSE");
  return L("DURUYOR", "STOPPED");
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
// lcdWriteFixedTxt Türkçe harfleri (ç, ğ, ı, ö, ş, ü) LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- DC MOTOR - Komutlar ----", "---- DC MOTOR - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oto           : otomatik mod", "  auto          : auto mode"));
  iotbot.serialWrite(L("  manuel        : manuel mod (potansiyometre)", "  manual        : manual mode (potentiometer)"));
  iotbot.serialWrite(L("  hiz -100..100 : hız % (eksi = geri)", "  speed -100..100: speed % (minus = reverse)"));
  iotbot.serialWrite(L("  dur           : motoru durdur", "  stop          : stop the motor"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : OTOMATİK <-> MANUEL", "  B3 button     : AUTO <-> MANUAL"));
}

void drawStaticScreen() {
  lcdRow(0, "      DC MOTOR");
  lastScreenMs = 0; // Değerleri hemen çiz / draw the values right away
}

void setMode(bool manual) {
  manualMode = manual;
  targetSpeed = 0;   // Güvenlik: mod değişince motor durur / safety: motor stops on mode change
  potWaitCenter = true; // Pot önce ortaya getirilmeli / pot must be centered first
  potFollow = false;
  demoIndex = 0;     // Demo baştan (önce 'dur' adımı) / demo restarts (with a 'stop' step first)
  demoStepMs = millis();
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  iotbot.serialWrite(manual ? L(">> MANUEL mod: potu ortaya getirin, sonra sağa = ileri, sola = geri.", ">> MANUAL mode: center the pot, then right = forward, left = reverse.")
                            : L(">> OTOMATİK mod: motor demo hareketlerini yapıyor.", ">> AUTO mode: the motor runs the demo moves."));
  drawStaticScreen();
}

void setTarget(int pct) {
  targetSpeed = constrain(pct, -100, 100);
  iotbot.serialWrite(String(L("Hedef hız: ", "Target speed: ")) + targetSpeed + "% " + directionText(targetSpeed));
}

// Seri komutla verilen hızı pot hemen ezmesin: pot ancak gerçekten çevrilince (%5) kontrolü geri alır.
// Keep the serial speed: the pot takes control back only when really turned (5%).
void holdSerialSpeed() {
  potWaitCenter = false;
  potFollow = false;
  potRef = potToSpeed(iotbot.potentiometerRead());
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oto" || word == "otomatik" || word == "auto") {
    setMode(false);
  } else if (word == "manuel" || word == "manual") {
    setMode(true);
  } else if ((word == "hiz" || word == "speed") && hasValue) {
    if (!manualMode) setMode(true);
    holdSerialSpeed();
    setTarget(value);
  } else if (word == "dur" || word == "stop") {
    if (!manualMode) setMode(true);
    holdSerialSpeed();
    setTarget(0);
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
  applyMotor(0);              // Güvenlik: motor duruyor / safety: motor stopped
  iotbot.lcdClear();
  demoStepMs = millis();
  drawStaticScreen();
  iotbot.serialWrite(L("DC motor testi başladı.", "DC motor test started."));
  printHelp();
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

  // 3) Hedef hızı belirle / Decide the target speed
  if (manualMode) {
    int potSpeed = potToSpeed(iotbot.potentiometerRead());
    if (potWaitCenter) {
      if (potSpeed == 0) { // Pot ortada: artık kontrol potta / pot centered: pot is in control now
        potWaitCenter = false;
        potFollow = true;
        iotbot.serialWrite(L("Pot hazır: sağa = ileri, sola = geri.", "Pot ready: right = forward, left = reverse."));
      }
    } else if (!potFollow && abs(potSpeed - potRef) >= 5) {
      potFollow = true; // Pot çevrildi, kontrolü geri aldı / pot turned, it takes control back
    }
    if (potFollow) targetSpeed = potSpeed;
  } else if (now - demoStepMs >= DEMO_TIME[demoIndex]) {
    demoStepMs = now;
    demoIndex = (demoIndex + 1) % DEMO_COUNT;
    iotbot.serialWrite(String(L("Otomatik: ", "Auto: ")) + DEMO_SPEED[demoIndex] + "% " + directionText(DEMO_SPEED[demoIndex]));
  }
  if (!manualMode) targetSpeed = DEMO_SPEED[demoIndex];

  // 4) Yumuşak rampa: hız her 15 ms'de biraz değişir. Yön tersse önce 0'a iner ve bekler.
  // 4) Soft ramp: the speed changes a little every 15 ms. When reversing it first goes to 0 and rests.
  if (now - lastRampMs >= RAMP_MS) {
    lastRampMs = now;
    bool reversing = (currentSpeed > 0 && targetSpeed < 0) || (currentSpeed < 0 && targetSpeed > 0);
    bool resting = (int32_t)(now - restUntilMs) < 0;
    int goal = (reversing || resting) ? 0 : targetSpeed;
    int before = currentSpeed;
    if (currentSpeed < goal) currentSpeed = min(currentSpeed + RAMP_STEP, goal);
    else if (currentSpeed > goal) currentSpeed = max(currentSpeed - RAMP_STEP, goal);
    if (reversing && before != 0 && currentSpeed == 0) restUntilMs = now + REVERSE_REST_MS; // Tam dursun / let it fully stop
    applyMotor(currentSpeed);
  }

  // 5) LCD (200 ms'de bir, titremesiz) / LCD (every 200 ms, no flicker)
  if (now - lastScreenMs >= 200) {
    lastScreenMs = now;
    char line[41];
    snprintf(line, sizeof(line), L("Mod: %s", "Mode: %s"), manualMode ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO"));
    lcdRow(1, line);
    snprintf(line, sizeof(line), L("Hız:%+4d%% %s", "Spd:%+4d%% %s"), currentSpeed, directionText(currentSpeed));
    lcdRow(2, line);
    if (!manualMode) lcdRow(3, L("B3: manuel kontrol", "B3: manual control"));
    else if (potWaitCenter) lcdRow(3, L("Potu ortaya getirin", "Center the pot"));
    else lcdRow(3, L("Pot:hız  B3:otomatik", "Pot:speed  B3:auto"));
  }
}
