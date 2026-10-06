// TR: GERCEK PROJE - DC Motor Hiz Kontrolu. Potansiyometreyi cevirerek
// DC motorun hizini ayarlayin; LCD hizi yuzde (%) olarak ve 20 karakterlik
// bir cubuk olarak gosterir. Potansiyometre en alttayken motor tamamen durur
// ("olu bolge"). B3 butonu donus yonunu degistirir - ama motor ANINDA ters
// donmez: once yavasca durur, kisa bir mola verir, sonra yavasca ters yonde
// hizlanir ("yumusak yon degistirme"). Neden? Donen bir motoru aniden ters
// cevirmek cok yuksek bir akim darbesi olusturur (motor o an jenerator gibi
// calisir) ve disliler bir anda zorlanir. Bu da motor surucusunu (L293D)
// isitip bozabilir, disli dislerini kirabilir. Yumusak gecis ikisini de korur.
// EN: A REAL PROJECT - DC Motor Speed Control. Turn the potentiometer to set
// the DC motor speed; the LCD shows it as a percentage (%) and as a 20-char
// bar. With the potentiometer at the bottom the motor stops completely
// ("dead zone"). B3 reverses the direction - but NOT instantly: the motor
// first slows to a stop, rests briefly, then speeds up the other way ("soft
// reverse"). Why? Suddenly reversing a spinning motor causes a huge current
// spike (for a moment the motor acts like a generator) and a shock on the
// gears. That can overheat and damage the motor driver (L293D) and break
// gear teeth. The soft transition protects both.
//
// Baglanti / Wiring: DC motoru P6 (motor surucu) soketine takin. Motor IO26
// ve IO27'yi kullanir, bu yuzden P2/P3'e ve trafik lambasina baska modul
// takmayin. Potansiyometre ve B3 kart uzerindedir. / Plug the DC motor into
// socket P6 (motor driver). It uses IO26 and IO27, so do not plug other
// modules into P2/P3 or use the traffic light. The potentiometer and B3 are
// on the board.

#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

namespace {
  constexpr int kDeadZoneRaw = 200;          // Pot bunun altinda -> motor durur / below this: motor stops
  constexpr int kMinPwm = 60;                // Motorun donmeye basladigi PWM / PWM where the motor starts turning
  constexpr int kRampStep = 5;               // Her adimda hiz degisimi / speed change per step
  constexpr uint32_t kRampIntervalMs = 15;   // 0->255 yaklasik 0.8 sn / 0->255 in about 0.8 s
  constexpr uint32_t kReverseRestMs = 300;   // Ters donmeden once mola / rest before reversing
  constexpr uint32_t kUiIntervalMs = 200;    // LCD yenileme araligi / LCD refresh interval

  int direction = 1;          // +1 = saat yonu, -1 = ters yon / +1 = clockwise, -1 = counter-clockwise
  int currentSpeed = 0;       // Motora verilen hiz, isaretli (-255..255) / applied speed, signed
  uint32_t lastRampMs = 0;
  uint32_t restUntilMs = 0;   // Bu ana kadar dur / stay stopped until this time
  uint32_t lastUiMs = 0;
  bool lastB3 = false;
  uint32_t lastB3Ms = 0;
}

// Isaretli hizi motora uygular: + saat yonu, - ters yon, 0 dur.
// Applies the signed speed: + clockwise, - counter-clockwise, 0 stop.
void applyMotor(int speed) {
  if (speed > 0) iotbot.moduleDCMotorGOClockWise(speed);
  else if (speed < 0) iotbot.moduleDCMotorGOCounterClockWise(-speed);
  else iotbot.moduleDCMotorStop();
}

// value'yu hedefe dogru en fazla step kadar yaklastirir.
// Moves value toward goal by at most step.
int stepToward(int value, int goal, int step) {
  if (value < goal) return min(value + step, goal);
  if (value > goal) return max(value - step, goal);
  return value;
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.moduleDCMotorStop();
  iotbot.lcdClear();
  iotbot.lcdWriteFixed(0, turkish ? "DC MOTOR HIZ KONTROL" : "DC MOTOR SPEED CTRL");
  iotbot.serialWrite(turkish ? "DC motor hiz kontrolu hazir." : "DC motor speed control ready.");
}

void loop() {
  uint32_t now = millis();

  // 1) Potansiyometre -> hedef hiz (olu bolge ile)
  // 1) Potentiometer -> target speed (with a dead zone)
  int raw = iotbot.potentiometerRead();
  int magnitude = 0;
  if (raw >= kDeadZoneRaw) magnitude = map(raw, kDeadZoneRaw, 4095, kMinPwm, 255);
  magnitude = constrain(magnitude, 0, 255);

  // 2) B3 -> yonu degistir (sadece istenen yon degisir, motor rampayla gecer)
  // 2) B3 -> flip direction (only the wish changes, the ramp does the rest)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3 && now - lastB3Ms > 200) {
    lastB3Ms = now;
    direction = -direction;
    iotbot.buzzerPlayTone(1200, 40);
    iotbot.serialWrite(turkish ? "Yon degisiyor..." : "Reversing...");
  }
  lastB3 = b3;
  int target = magnitude * direction;

  // 3) Rampa: hiz her 15 ms'de biraz degisir. Yon tersse once 0'a iner.
  // 3) Ramp: speed changes a little every 15 ms. If reversing, go to 0 first.
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

  // 4) LCD: yuzde, yon, cubuk ve durum (200 ms'de bir, titremesiz)
  // 4) LCD: percent, direction, bar and status (every 200 ms, no flicker)
  if (now - lastUiMs >= kUiIntervalMs) {
    lastUiMs = now;
    int speedAbs = abs(currentSpeed);
    int percent = speedAbs * 100 / 255;
    const char *dirText;
    if (currentSpeed > 0) dirText = turkish ? "SAAT YONU" : "CLOCKWISE";
    else if (currentSpeed < 0) dirText = turkish ? "TERS YON" : "COUNTER-CW";
    else dirText = turkish ? "DURUYOR" : "STOPPED";

    char line[21];
    snprintf(line, sizeof(line), turkish ? "Hiz:%3d%% %s" : "Spd:%3d%% %s", percent, dirText);
    iotbot.lcdWriteFixed(1, line);

    // 20 kutuluk cubuk: 255 = 20 dolu kutu. (char)255 LCD'de tam dolu kare.
    // 20-cell bar: 255 = 20 filled cells. (char)255 is a solid block on the LCD.
    int filled = speedAbs * 20 / 255;
    for (int i = 0; i < 20; i++) line[i] = (i < filled) ? (char)255 : '-';
    line[20] = '\0';
    iotbot.lcdWriteFixed(2, line);

    if (reversing || resting) snprintf(line, sizeof(line), "%s", turkish ? "Yon degisiyor..." : "Reversing...");
    else snprintf(line, sizeof(line), "%s", turkish ? "B3: Yonu degistir" : "B3: Reverse");
    iotbot.lcdWriteFixed(3, line);
  }
  delay(2);
}
