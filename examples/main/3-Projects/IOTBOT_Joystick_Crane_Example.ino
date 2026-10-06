// TR: GERCEK PROJE - Joystick Kumandali Vinc. Joystick'i SAGA-SOLA itince
// servo motor vincin kolunu 0-180 derece arasinda dondurur; kol yumusakca,
// her adimda birkac derece ilerleyerek hedefe gider (gercek vincler de ani
// hareket etmez, yuk sallanmasin diye). Joystick'i ILERI-GERI itince DC
// motor (vinc makarasi) ipi SARAR ya da BIRAKIR; ne kadar cok iterseniz o
// kadar hizli doner. Acilista joystick'in orta noktasi olculur ve ortadaki
// kucuk bir "olu bolge" yok sayilir, boylece birakinca vinc kendi kendine
// kaymaz. Joystick'in DUGMESINE basmak ACIL STOP'tur: motor hemen durur;
// tekrar basinca devam eder (makara ancak kol ortaya donunce calisir).
// EN: A REAL PROJECT - Joystick Crane. Push the joystick LEFT-RIGHT and the
// servo turns the crane's boom between 0 and 180 degrees; the boom moves
// smoothly, a few degrees per step (real cranes don't jerk either, so the
// load doesn't swing). Push the joystick FORWARD-BACK and the DC motor (the
// winch) WINDS or UNWINDS the rope; the further you push, the faster it
// turns. At startup the joystick's center point is measured and a small
// "dead zone" around it is ignored, so the crane doesn't creep when you let
// go. Pressing the joystick BUTTON is the EMERGENCY STOP: the motor stops at
// once; press again to continue (the winch only runs after the stick
// returns to the center).
//
// Baglanti / Wiring: Servo motoru P1 soketine (IO25), DC motoru P6 motor
// soketine takin (IO26 + IO27 kullanir - bu yuzden P2/P3'e baska modul
// TAKMAYIN). Joystick kart uzerindedir. / Plug the servo into socket P1
// (IO25) and the DC motor into the P6 motor socket (it uses IO26 + IO27 -
// so do NOT plug other modules into P2/P3). The joystick is on the board.

#define USE_SERVO
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define SERVO_PIN IO25 // P1 soketi / socket P1

namespace {
  constexpr int kDeadZone = 250;        // Merkezde yok sayilan bolge / ignored zone around the center
  constexpr int kServoStepDeg = 3;      // Her dongude en fazla derece / max degrees per loop
  constexpr int kMinMotorSpeed = 90;    // Motorun donmeye basladigi hiz / speed where the motor starts to turn
  constexpr bool kInvertWinch = false;  // Makara ters donuyorsa true / true if the winch turns the wrong way
  constexpr uint32_t kLoopMs = 20;      // Dongu araligi / loop interval
  constexpr uint32_t kScreenMs = 200;   // LCD yenileme araligi / LCD refresh interval

  int xCenter = 2048;
  int yCenter = 2048;
  int boomAngle = 90;         // Kolun su anki acisi / current boom angle
  int winchPercent = 0;       // -100..+100: + yukari, - asagi / + up, - down
  bool emergencyStop = false;
  bool waitForCenter = false; // Acil stoptan sonra kol ortalanana kadar makara calismaz / winch waits for a centered stick
  bool joyBtnWasDown = false;
  uint32_t lastJoyBtnMs = 0;
  uint32_t lastScreenMs = 0;
}

// Joystick'in merkezden sapmasini -100..+100 yuzdeye cevirir; olu bolgedeyse 0 doner.
// Turns the joystick's offset from the center into -100..+100 percent; 0 inside the dead zone.
int axisPercent(int raw, int center) {
  int offset = raw - center;
  if (abs(offset) < kDeadZone) return 0;
  if (offset > 0) return constrain(map(offset, kDeadZone, 4095 - center, 1, 100), 1, 100);
  return -constrain(map(-offset, kDeadZone, center, 1, 100), 1, 100);
}

void writeRow(int row, const char *text) {
  char line[21];
  snprintf(line, sizeof(line), "%-20s", text); // 20'ye bosluklarla tamamla / pad to 20 with spaces
  iotbot.lcdWriteFixed(row, line);
}

void showRunScreen() {
  iotbot.lcdWriteMid(turkish ? "VINC KUMANDASI" : "CRANE CONTROL", "", "",
                      turkish ? "Dugme = ACIL STOP" : "Button = E-STOP");
  lastScreenMs = 0;
}

void setWinch(int percent) {
  winchPercent = percent;
  if (percent == 0) {
    iotbot.moduleDCMotorStop();
    return;
  }
  // Hiz, itme miktariyla orantili: az it = yavas, tam it = tam hiz.
  // Speed is proportional to the push: a little = slow, all the way = full speed.
  int speed = map(abs(percent), 1, 100, kMinMotorSpeed, 255);
  bool up = (percent > 0) != kInvertWinch;
  if (up) iotbot.moduleDCMotorGOClockWise(speed);
  else iotbot.moduleDCMotorGOCounterClockWise(speed);
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.moduleDCMotorStop();
  iotbot.moduleServoGoAngle(SERVO_PIN, boomAngle, 5); // Kolu ortaya al / center the boom

  // Kalibrasyon: joystick'e dokunulmadan orta noktasini olc. / Calibrate: measure the center untouched.
  iotbot.lcdWriteMid(turkish ? "VINC KUMANDASI" : "CRANE CONTROL", "", turkish ? "Joystick'e" : "Do not touch",
                      turkish ? "dokunmayin..." : "the joystick...");
  delay(800);
  iotbot.calibrateJoystick(xCenter, yCenter); // 20 okuma ortalamasi / average of 20 readings
  // Kalibrasyon sirasinda kol itiliyduysa orta nokta sacma olur; varsayilana don.
  // If the stick was pushed during calibration the center is nonsense; fall back to the default.
  if (xCenter < 1000 || xCenter > 3100 || yCenter < 1000 || yCenter > 3100) {
    iotbot.serialWrite(turkish ? "Kalibrasyon hatali, 2048 kullaniliyor." : "Bad calibration, using 2048.");
    xCenter = 2048;
    yCenter = 2048;
  }
  char msg[48];
  snprintf(msg, sizeof(msg), turkish ? "Orta nokta X=%d Y=%d" : "Center X=%d Y=%d", xCenter, yCenter);
  iotbot.serialWrite(msg);
  showRunScreen();
}

void loop() {
  uint32_t now = millis();

  // Joystick dugmesi INPUT_PULLUP: basiliyken false (LOW) doner. / Joystick button: false when pressed.
  bool joyBtnDown = !iotbot.joystickButtonRead();
  if (joyBtnDown && !joyBtnWasDown && now - lastJoyBtnMs > 300) {
    lastJoyBtnMs = now;
    emergencyStop = !emergencyStop;
    if (emergencyStop) {
      setWinch(0); // Once motoru durdur! / stop the motor first!
      iotbot.lcdWriteMid(turkish ? "!!! ACIL STOP !!!" : "!!! E-STOP !!!", turkish ? "Motor durduruldu" : "Motor stopped", "",
                          turkish ? "Devam: dugmeye bas" : "Resume: press button");
      iotbot.serialWrite(turkish ? "ACIL STOP!" : "EMERGENCY STOP!");
      iotbot.buzzerPlayTone(400, 300);
    } else {
      waitForCenter = true; // Kol itili kaldiysa makara aniden firlamasin / no sudden jump if the stick is held
      iotbot.serialWrite(turkish ? "Devam ediliyor" : "Resuming");
      iotbot.buzzerPlayTone(1200, 80);
      showRunScreen();
    }
  }
  joyBtnWasDown = joyBtnDown;

  if (emergencyStop) {
    delay(kLoopMs);
    return;
  }

  // X ekseni -> kolun HEDEF acisi. Kol her dongude en fazla birkac derece yaklasir (yumusak).
  // X axis -> the boom's TARGET angle. The boom moves at most a few degrees per loop (smooth).
  int target = 90 + axisPercent(iotbot.joystickXRead(), xCenter) * 90 / 100;
  int step = constrain(target - boomAngle, -kServoStepDeg, kServoStepDeg);
  if (step != 0) {
    boomAngle += step;
    iotbot.moduleServoGoAngle(SERVO_PIN, boomAngle, 1); // En fazla 3 ms bekler / waits at most 3 ms
  }

  // Y ekseni -> makara yonu ve hizi. / Y axis -> winch direction and speed.
  int yPercent = axisPercent(iotbot.joystickYRead(), yCenter);
  if (waitForCenter && yPercent == 0) waitForCenter = false;
  int wanted = waitForCenter ? 0 : yPercent;
  if (wanted != winchPercent) setWinch(wanted); // Sadece degisince motora yaz / only write the motor on change

  if (now - lastScreenMs >= kScreenMs) {
    lastScreenMs = now;
    char line[21];
    snprintf(line, sizeof(line), turkish ? " Kol acisi: %3d der" : " Boom angle: %3d deg", boomAngle);
    writeRow(1, line);
    if (waitForCenter) {
      snprintf(line, sizeof(line), "%s", turkish ? " Kolu ortaya birak" : " Center the stick");
    } else if (winchPercent > 0) {
      snprintf(line, sizeof(line), turkish ? " Vinc: YUKARI %3d%%" : " Winch: UP   %3d%%", winchPercent);
    } else if (winchPercent < 0) {
      snprintf(line, sizeof(line), turkish ? " Vinc: ASAGI  %3d%%" : " Winch: DOWN %3d%%", -winchPercent);
    } else {
      snprintf(line, sizeof(line), "%s", turkish ? " Vinc: DURUYOR" : " Winch: STOPPED");
    }
    writeRow(2, line);
  }
  delay(kLoopMs);
}
