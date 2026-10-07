/*
 * TR: GERÇEK PROJE - DC Motor Hız Kontrolü (Otomatik demo + Manuel kontrol)
 *  - Açılışta OTOMATİK mod çalışır: motor kendi kendine yavaşça hızlanır,
 *    yavaşlar, durur ve ters yönde döner (bir gösteri programı).
 *  - B3 butonuna basınca MANUEL moda geçer: hızı potansiyometre ile siz
 *    ayarlarsınız, B1 (veya B2) dönüş yönünü değiştirir. B3'e tekrar basınca
 *    otomatik moda döner.
 *  - LCD hızı yüzde (%) ve 20 kutuluk bir çubuk olarak gösterir. Potansiyometre
 *    en alttayken motor tamamen durur ("ölü bölge").
 *  - YUMUŞAK YÖN DEĞİŞTİRME: motor ANINDA ters dönmez: önce yavaşça durur,
 *    kısa bir mola verir, sonra yavaşça ters yönde hızlanır. Neden? Dönen bir
 *    motoru aniden ters çevirmek çok yüksek bir akım darbesi oluşturur (motor o
 *    an jeneratör gibi çalışır) ve dişliler bir anda zorlanır. Bu da motor
 *    sürücüsünü (L293D) ısıtıp bozabilir, dişli dişlerini kırabilir. Yumuşak
 *    geçiş ikisini de korur.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help        -> komut listesi
 *      oto     / auto        -> otomatik mod
 *      manuel  / manual      -> manuel mod (potansiyometre + B1)
 *      hiz 50  / speed 50    -> %50 hız (eksi değer = ters yön, manuel moda geçer)
 *      dur     / stop        -> motoru yumuşakça durdur (manuel moda geçer)
 *      ters    / reverse     -> yönü değiştir (manuel moda geçer)
 *      dil     / lang        -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - DC Motor Speed Control (Auto demo + Manual control)
 *  - At startup AUTO mode runs: the motor speeds up, slows down, stops and
 *    turns the other way by itself (a demo program).
 *  - Press B3 to switch to MANUAL mode: you set the speed with the
 *    potentiometer and B1 (or B2) reverses the direction. Press B3 again to
 *    go back to auto mode.
 *  - The LCD shows the speed as a percentage (%) and as a 20-cell bar. With
 *    the potentiometer at the bottom the motor stops completely ("dead zone").
 *  - SOFT REVERSE: the motor does NOT reverse instantly: it first slows to a
 *    stop, rests briefly, then speeds up the other way. Why? Suddenly
 *    reversing a spinning motor causes a huge current spike (for a moment the
 *    motor acts like a generator) and a shock on the gears. That can overheat
 *    and damage the motor driver (L293D) and break gear teeth. The soft
 *    transition protects both.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help     / yardim     -> command list
 *      auto     / oto        -> auto mode
 *      manual   / manuel     -> manual mode (potentiometer + B1)
 *      speed 50 / hiz 50     -> 50% speed (negative = reverse, switches to manual)
 *      stop     / dur        -> stop the motor softly (switches to manual)
 *      reverse  / ters       -> flip the direction (switches to manual)
 *      lang     / dil        -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: DC motoru P6 (motor sürücü) soketine takın. Motor IO26
 * ve IO27'yi kullanır, bu yüzden P2/P3'e ve trafik lambasına başka modül
 * takmayın. Potansiyometre, B1 ve B3 kart üzerindedir. / Plug the DC motor
 * into socket P6 (motor driver). It uses IO26 and IO27, so do not plug other
 * modules into P2/P3 or use the traffic light. The potentiometer, B1 and B3
 * are on the board.
 */

#include <IOTBOT.h>

IOTBOT iotbot;

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

namespace {
  constexpr int kDeadZoneRaw = 200;          // Pot bunun altında -> motor durur / below this: motor stops
  constexpr int kMinPwm = 60;                // Motorun dönmeye başladığı PWM / PWM where the motor starts turning
  constexpr int kRampStep = 5;               // Her adımda hız değişimi / speed change per step
  constexpr uint32_t kRampIntervalMs = 15;   // 0->255 yaklaşık 0.8 sn / 0->255 in about 0.8 s
  constexpr uint32_t kReverseRestMs = 300;   // Ters dönmeden önce mola / rest before reversing
  constexpr uint32_t kUiIntervalMs = 200;    // LCD yenileme aralığı / LCD refresh interval
  constexpr int kPotMoveRaw = 150;           // Pot "gerçekten çevrildi" eşiği / pot "really turned" threshold

  // Otomatik gösteri: {işaretli PWM hedefi, bekleme süresi}. + saat yönü, - ters yön.
  // Auto demo: {signed PWM target, hold time}. + clockwise, - counter-clockwise.
  struct DemoStep { int speed; uint32_t holdMs; };
  const DemoStep kDemo[] = {
    {150, 3000}, {255, 2000}, {0, 1500}, {-150, 3000}, {-255, 2000}, {0, 1500},
  };
  constexpr int kDemoCount = sizeof(kDemo) / sizeof(kDemo[0]);

  bool manualMode = false;    // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
  int direction = 1;          // Manuel yön: +1 saat yönü, -1 ters yön / manual direction
  int magnitude = 0;          // Manuel hız büyüklüğü (0-255) / manual speed magnitude (0-255)
  int lastPotRaw = -1000;     // Potun son kabul edilen değeri / last accepted pot value
  int demoIndex = 0;
  uint32_t demoStepMs = 0;    // Gösteri adımının başladığı an / when the demo step started
  int currentSpeed = 0;       // Motora verilen hız, işaretli (-255..255) / applied speed, signed
  uint32_t lastRampMs = 0;
  uint32_t restUntilMs = 0;   // Bu ana kadar dur / stay stopped until this time
  uint32_t lastUiMs = 0;
  bool lastB3 = false;
  bool lastB1 = false;
  uint32_t lastButtonMs = 0;
  int lastPrintedTarget = 9999;
  uint32_t lastPrintMs = 0;
}

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
// İşaretli hızı motora uygular: + saat yönü, - ters yön, 0 dur.
// Applies the signed speed: + clockwise, - counter-clockwise, 0 stop.
void applyMotor(int speed) {
  if (speed > 0) iotbot.moduleDCMotorGOClockWise(speed);
  else if (speed < 0) iotbot.moduleDCMotorGOCounterClockWise(-speed);
  else iotbot.moduleDCMotorStop();
}

// value'yu hedefe doğru en fazla step kadar yaklaştırır.
// Moves value toward goal by at most step.
int stepToward(int value, int goal, int step) {
  if (value < goal) return min(value + step, goal);
  if (value > goal) return max(value - step, goal);
  return value;
}

// Pot değeri -> hız büyüklüğü (ölü bölge ile) / pot value -> speed magnitude (with a dead zone)
int potToMagnitude(int raw) {
  if (raw < kDeadZoneRaw) return 0;
  return constrain(map(raw, kDeadZoneRaw, 4095, kMinPwm, 255), 0, 255);
}

// Şu an istenen işaretli hız / the signed speed wanted right now
int targetSpeed() {
  return manualMode ? magnitude * direction : kDemo[demoIndex].speed;
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
// lcdWriteFixedTxt Türkçe harfleri (ç, ğ, ı, ö, ş, ü) LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- DC MOTOR HIZ KONTROLÜ - Komutlar ----", "---- DC MOTOR SPEED CONTROL - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oto           : otomatik mod (gösteri)", "  auto          : auto mode (demo)"));
  iotbot.serialWrite(L("  manuel        : manuel mod (pot = hız, B1 = yön)", "  manual        : manual mode (pot = speed, B1 = direction)"));
  iotbot.serialWrite(L("  hiz -100..100 : hız yüzdesi (eksi = ters yön)", "  speed -100..100: speed percent (negative = reverse)"));
  iotbot.serialWrite(L("  dur           : motoru yumuşakça durdur", "  stop          : stop the motor softly"));
  iotbot.serialWrite(L("  ters          : yönü değiştir", "  reverse       : flip the direction"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : OTOMATİK <-> MANUEL", "  B3 button     : AUTO <-> MANUAL"));
}

void drawStaticScreen() {
  lcdRow(0, manualMode ? L("DC MOTOR - MANUEL", "DC MOTOR - MANUAL") : L("DC MOTOR - OTOMATİK", "DC MOTOR - AUTO"));
  lastUiMs = 0; // Değerleri hemen çiz / draw the values right away
}

void setMode(bool manual) {
  manualMode = manual;
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  if (manual) {
    // Manuelde motor, şu anki pot konumuyla devam eder / in manual the motor follows the current pot
    lastPotRaw = iotbot.potentiometerRead();
    magnitude = potToMagnitude(lastPotRaw);
    iotbot.serialWrite(L(">> MANUEL mod: hız = potansiyometre, yön = B1.", ">> MANUAL mode: speed = potentiometer, direction = B1."));
  } else {
    demoIndex = 0;
    demoStepMs = millis();
    iotbot.serialWrite(L(">> OTOMATİK mod: motor gösteri programını çalıştırıyor.", ">> AUTO mode: the motor runs the demo program."));
  }
  drawStaticScreen();
}

void flipDirection() {
  direction = -direction;
  iotbot.buzzerPlayTone(1200, 40);
  iotbot.serialWrite(direction > 0 ? L("Yön: SAAT YÖNÜ (yumuşak geçiş...)", "Direction: CLOCKWISE (soft change...)")
                                   : L("Yön: TERS YÖN (yumuşak geçiş...)", "Direction: COUNTER-CW (soft change...)"));
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
    value = constrain(value, -100, 100);
    direction = (value < 0) ? -1 : 1;
    magnitude = (value == 0) ? 0 : map(abs(value), 1, 100, kMinPwm, 255);
    // Seri komutla verilen hızı pot hemen ezmesin diye pot konumunu kaydet.
    // Remember the pot position so it does not override the serial speed right away.
    lastPotRaw = iotbot.potentiometerRead();
    iotbot.serialWrite(String(L("Hedef hız: %", "Target speed: ")) + value + L("", "%"));
  } else if (word == "dur" || word == "stop") {
    if (!manualMode) setMode(true);
    magnitude = 0;
    lastPotRaw = iotbot.potentiometerRead();
    iotbot.serialWrite(L("Motor yumuşakça duruyor.", "Motor stopping softly."));
  } else if (word == "ters" || word == "reverse") {
    if (!manualMode) setMode(true);
    flipDirection();
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
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.moduleDCMotorStop(); // Motor her zaman DURUK başlar / the motor always starts STOPPED
  iotbot.lcdClear();
  demoStepMs = millis();
  drawStaticScreen();
  iotbot.serialWrite(L("DC motor hız kontrolü hazır.", "DC motor speed control ready."));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) B3 -> mod değiştir (sadece basıldığı an) / B3 -> toggle mode (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3 && now - lastButtonMs > 200) {
    lastButtonMs = now;
    setMode(!manualMode);
  }
  lastB3 = b3;

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Hedef hız / Target speed
  if (manualMode) {
    // Pot -> hız büyüklüğü; sadece gerçekten çevrilince (titreşim eşiği) ya da ölü bölgeye
    // girip çıkınca. / Pot -> speed magnitude; only when really turned (jitter threshold) or
    // when it enters/leaves the dead zone.
    int raw = iotbot.potentiometerRead();
    bool crossedDeadZone = (raw < kDeadZoneRaw) != (lastPotRaw < kDeadZoneRaw);
    if (abs(raw - lastPotRaw) >= kPotMoveRaw || crossedDeadZone) {
      lastPotRaw = raw;
      magnitude = potToMagnitude(raw);
    }
    // B1 (veya B2) -> yönü değiştir. Sadece istenen yön değişir, rampa gerisini yapar.
    // B1 (or B2) -> flip direction. Only the wish changes, the ramp does the rest.
    bool b1 = iotbot.button1Read() || iotbot.button2Read();
    if (b1 && !lastB1 && now - lastButtonMs > 200) {
      lastButtonMs = now;
      flipDirection();
    }
    lastB1 = b1;
  } else if (now - demoStepMs >= kDemo[demoIndex].holdMs) {
    // Otomatik gösteride sıradaki adım / next step of the auto demo
    demoIndex = (demoIndex + 1) % kDemoCount;
    demoStepMs = now;
  }
  int target = targetSpeed();

  // Hedef değişince seri porta yaz (en fazla 300 ms'de bir) / print the target when it changes
  int targetPct = (target * 100) / 255;
  if (abs(targetPct - lastPrintedTarget) >= 5 && now - lastPrintMs >= 300) {
    lastPrintedTarget = targetPct;
    lastPrintMs = now;
    iotbot.serialWrite(String(manualMode ? L("Manuel", "Manual") : L("Otomatik", "Auto")) + L(": hedef hız %", ": target speed ") +
                       targetPct + L("", "%"));
  }

  // 4) Rampa: hız her 15 ms'de biraz değişir. Yön tersse önce 0'a iner.
  // 4) Ramp: speed changes a little every 15 ms. If reversing, go to 0 first.
  bool reversing = (currentSpeed > 0 && target < 0) || (currentSpeed < 0 && target > 0);
  bool resting = (int32_t)(now - restUntilMs) < 0;
  if (now - lastRampMs >= kRampIntervalMs) {
    lastRampMs = now;
    int goal = (reversing || resting) ? 0 : target;
    int before = currentSpeed;
    currentSpeed = stepToward(currentSpeed, goal, kRampStep);
    if (reversing && before != 0 && currentSpeed == 0) {
      restUntilMs = now + kReverseRestMs;  // Motor tam dursun / let the motor fully stop
    }
    applyMotor(currentSpeed);
  }

  // 5) LCD: yüzde, yön, çubuk ve ipucu (200 ms'de bir, titremesiz)
  // 5) LCD: percent, direction, bar and hint (every 200 ms, no flicker)
  if (now - lastUiMs >= kUiIntervalMs) {
    lastUiMs = now;
    int speedAbs = abs(currentSpeed);
    int percent = speedAbs * 100 / 255;
    const char *dirText;
    if (currentSpeed > 0) dirText = L("SAAT YÖNÜ", "CLOCKWISE");
    else if (currentSpeed < 0) dirText = L("TERS YÖN", "COUNTER-CW");
    else dirText = L("DURUYOR", "STOPPED");

    char line[41];
    snprintf(line, sizeof(line), L("Hız:%3d%% %s", "Spd:%3d%% %s"), percent, dirText);
    lcdRow(1, line);

    // 20 kutuluk çubuk: 255 = 20 dolu kutu. (char)255 LCD'de tam dolu kare.
    // 20-cell bar: 255 = 20 filled cells. (char)255 is a solid block on the LCD.
    int filled = speedAbs * 20 / 255;
    for (int i = 0; i < 20; i++) line[i] = (i < filled) ? (char)255 : '-';
    line[20] = '\0';
    lcdRow(2, line);

    if (reversing || resting) lcdRow(3, L("Yön değişiyor...", "Reversing..."));
    else if (manualMode) lcdRow(3, L("B1:yön  B3:otomatik", "B1:reverse  B3:auto"));
    else lcdRow(3, L("B3: manuel kontrol", "B3: manual control"));
  }
  delay(2);
}
