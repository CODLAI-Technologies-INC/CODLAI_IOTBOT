/*
 * TR: GERÇEK PROJE - Joystick Kumandalı Vinç.
 *  - OTOMATİK mod (açılışta): vinç kendi kendine bir gösteri yapar: kol
 *    yavaşça sağa-sola döner, makara ipi biraz sarar ve aynı kadar bırakır.
 *  - B3 butonu MANUEL moda geçer (B3'e tekrar basınca otomatiğe döner):
 *    Joystick'i SAĞA-SOLA itince servo motor vincin kolunu 0-180 derece
 *    arasında döndürür; kol yumuşakça, her adımda birkaç derece ilerleyerek
 *    hedefe gider (gerçek vinçler de ani hareket etmez, yük sallanmasın diye).
 *    Joystick'i İLERİ-GERİ itince DC motor (vinç makarası) ipi SARAR ya da
 *    BIRAKIR; ne kadar çok iterseniz o kadar hızlı döner. Açılışta joystick'in
 *    orta noktası ölçülür ve ortadaki küçük bir "ölü bölge" yok sayılır,
 *    böylece bırakınca vinç kendi kendine kaymaz.
 *  - Joystick'in DÜĞMESİNE basmak her iki modda da ACİL STOP'tur: motor hemen
 *    durur; tekrar basınca devam eder (manuelde makara ancak kol ortaya dönünce
 *    çalışır).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim   / help       -> komut listesi
 *      oto      / auto       -> otomatik mod (gösteri)
 *      manuel   / manual     -> manuel mod (joystick)
 *      aci 90   / angle 90   -> kolu 90°'ye döndür (manuel moda geçer)
 *      vinc 50  / winch 50   -> makarayı %50 hızla sar (eksi = bırak, manuel moda geçer)
 *      dur      / stop       -> makarayı durdur (manuel moda geçer)
 *      acil     / estop      -> ACİL STOP
 *      devam    / resume     -> acil stoptan çık
 *      dil      / lang       -> dili değiştir (Türkçe <-> English)
 *    Joystick'i oynatınca seri porttan verilen açı/hız bırakılır, joystick geçerli olur.
 *
 * EN: A REAL PROJECT - Joystick Crane.
 *  - AUTO mode (at startup): the crane runs a demo by itself: the boom turns
 *    slowly left and right, the winch winds the rope a bit and unwinds it by
 *    the same amount.
 *  - Button B3 switches to MANUAL mode (press B3 again to go back to auto):
 *    Push the joystick LEFT-RIGHT and the servo turns the crane's boom between
 *    0 and 180 degrees; the boom moves smoothly, a few degrees per step (real
 *    cranes don't jerk either, so the load doesn't swing). Push the joystick
 *    FORWARD-BACK and the DC motor (the winch) WINDS or UNWINDS the rope; the
 *    further you push, the faster it turns. At startup the joystick's center
 *    point is measured and a small "dead zone" around it is ignored, so the
 *    crane doesn't creep when you let go.
 *  - Pressing the joystick BUTTON is the EMERGENCY STOP in both modes: the
 *    motor stops at once; press again to continue (in manual the winch only
 *    runs after the stick returns to the center).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help     / yardim     -> command list
 *      auto     / oto        -> auto mode (demo)
 *      manual   / manuel     -> manual mode (joystick)
 *      angle 90 / aci 90     -> turn the boom to 90° (switches to manual)
 *      winch 50 / vinc 50    -> wind the winch at 50% (negative = unwind, switches to manual)
 *      stop     / dur        -> stop the winch (switches to manual)
 *      estop    / acil       -> EMERGENCY STOP
 *      resume   / devam      -> leave the emergency stop
 *      lang     / dil        -> switch language (Turkish <-> English)
 *    Moving the joystick drops the serial angle/speed and the joystick takes over.
 *
 * Bağlantı / Wiring: Servo motoru P1 soketine (IO25), DC motoru P6 motor
 * soketine takın (IO26 + IO27 kullanır - bu yüzden P2/P3'e başka modül
 * TAKMAYIN). Joystick ve B3 kart üzerindedir. / Plug the servo into socket P1
 * (IO25) and the DC motor into the P6 motor socket (it uses IO26 + IO27 - so
 * do NOT plug other modules into P2/P3). The joystick and B3 are on the board.
 */

#define USE_SERVO
#include <IOTBOT.h>

IOTBOT iotbot;

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

#define SERVO_PIN IO25 // P1 soketi / socket P1

namespace {
  constexpr int kDeadZone = 250;        // Merkezde yok sayılan bölge / ignored zone around the center
  constexpr int kServoStepDeg = 3;      // Manuelde her döngüde en fazla derece / max degrees per loop in manual
  constexpr int kMinMotorSpeed = 90;    // Motorun dönmeye başladığı hız / speed where the motor starts to turn
  constexpr bool kInvertWinch = false;  // Makara ters dönüyorsa true / true if the winch turns the wrong way
  constexpr uint32_t kLoopMs = 20;      // Döngü aralığı / loop interval
  constexpr uint32_t kScreenMs = 200;   // LCD yenileme aralığı / LCD refresh interval

  // Otomatik gösteri: kol 30° <-> 150°, makara {hız %, süre}. Sarma ve bırakma eşit, ip yerinde kalır.
  // Auto demo: boom 30° <-> 150°, winch {speed %, time}. Wind and unwind are equal, the rope stays put.
  constexpr int kAutoBoomLow = 30;
  constexpr int kAutoBoomHigh = 150;
  constexpr uint32_t kAutoBoomPauseMs = 800;
  struct WinchStep { int percent; uint32_t ms; };
  const WinchStep kWinchDemo[] = {{60, 1500}, {0, 1000}, {-60, 1500}, {0, 1000}};
  constexpr int kWinchDemoCount = sizeof(kWinchDemo) / sizeof(kWinchDemo[0]);

  bool manualMode = false;    // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
  int xCenter = 2048;
  int yCenter = 2048;
  int boomAngle = 90;         // Kolun şu anki açısı / current boom angle
  int winchPercent = 0;       // -100..+100: + yukarı, - aşağı / + up, - down
  bool emergencyStop = false;
  bool waitForCenter = false; // Kol ortalanana kadar makara çalışmaz / winch waits for a centered stick
  bool joyBtnWasDown = false;
  uint32_t lastJoyBtnMs = 0;
  bool lastB3 = false;
  uint32_t lastB3Ms = 0;
  uint32_t lastScreenMs = 0;
  // Seri komut değerleri (joystick oynayınca bırakılır) / serial command values (dropped when the joystick moves)
  bool serialBoom = false;
  int serialBoomAngle = 90;
  int serialWinch = 0;
  // Otomatik gösteri durumu / auto demo state
  int autoBoomTarget = kAutoBoomHigh;
  uint32_t boomPauseUntilMs = 0;
  int winchStep = 0;
  uint32_t winchStepMs = 0;
}

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
// Joystick ve motor / Joystick and motor
// ---------------------------------------------------------------------------
// Joystick'in merkezden sapmasını -100..+100 yüzdeye çevirir; ölü bölgedeyse 0 döner.
// Turns the joystick's offset from the center into -100..+100 percent; 0 inside the dead zone.
int axisPercent(int raw, int center) {
  int offset = raw - center;
  if (abs(offset) < kDeadZone) return 0;
  if (offset > 0) return constrain(map(offset, kDeadZone, 4095 - center, 1, 100), 1, 100);
  return -constrain(map(-offset, kDeadZone, center, 1, 100), 1, 100);
}

void setWinch(int percent) {
  winchPercent = percent;
  if (percent == 0) {
    iotbot.moduleDCMotorStop();
    return;
  }
  // Hız, itme miktarıyla orantılı: az it = yavaş, tam it = tam hız.
  // Speed is proportional to the push: a little = slow, all the way = full speed.
  int speed = map(abs(percent), 1, 100, kMinMotorSpeed, 255);
  bool up = (percent > 0) != kInvertWinch;
  if (up) iotbot.moduleDCMotorGOClockWise(speed);
  else iotbot.moduleDCMotorGOCounterClockWise(speed);
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
// lcdWriteFixedTxt Türkçe harfleri (ç, ğ, ı, ö, ş, ü) LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- JOYSTICK VİNÇ - Komutlar ----", "---- JOYSTICK CRANE - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oto           : otomatik mod (gösteri)", "  auto          : auto mode (demo)"));
  iotbot.serialWrite(L("  manuel        : manuel mod (joystick)", "  manual        : manual mode (joystick)"));
  iotbot.serialWrite(L("  aci 0-180     : kol açısı", "  angle 0-180   : boom angle"));
  iotbot.serialWrite(L("  vinc -100..100: makara hızı (+ sar, - bırak)", "  winch -100..100: winch speed (+ wind, - unwind)"));
  iotbot.serialWrite(L("  dur           : makarayı durdur", "  stop          : stop the winch"));
  iotbot.serialWrite(L("  acil / devam  : ACİL STOP / devam et", "  estop / resume: EMERGENCY STOP / continue"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : OTOMATİK <-> MANUEL", "  B3 button     : AUTO <-> MANUAL"));
  iotbot.serialWrite(L("  Joystick düğmesi: ACİL STOP", "  Joystick button: EMERGENCY STOP"));
}

void showRunScreen() {
  lcdRow(0, manualMode ? L("   VİNÇ - MANUEL", "   CRANE - MANUAL") : L("  VİNÇ - OTOMATİK", "   CRANE - AUTO"));
  lcdRow(3, manualMode ? L("B3:oto  Düğme:STOP", "B3:auto  Btn:E-STOP") : L("B3:manuel Düğme:STOP", "B3:manual Btn:E-STOP"));
  lastScreenMs = 0;
}

void setEmergency(bool on) {
  if (on == emergencyStop) return;
  emergencyStop = on;
  if (on) {
    setWinch(0); // Önce motoru durdur! / stop the motor first!
    iotbot.lcdWriteMid(L("!!! ACİL STOP !!!", "!!! E-STOP !!!"), L("Motor durduruldu", "Motor stopped"), "",
                       L("Devam: düğmeye bas", "Resume: press button"));
    iotbot.serialWrite(L("ACİL STOP!", "EMERGENCY STOP!"));
    iotbot.buzzerPlayTone(400, 300);
  } else {
    waitForCenter = true; // Kol itili kaldıysa makara aniden fırlamasın / no sudden jump if the stick is held
    iotbot.serialWrite(L("Devam ediliyor", "Resuming"));
    iotbot.buzzerPlayTone(1200, 80);
    showRunScreen();
  }
}

void setMode(bool manual) {
  manualMode = manual;
  setWinch(0);            // Mod değişirken makara dursun / stop the winch when changing modes
  waitForCenter = true;   // Manuelde kol ortalanana kadar bekle / in manual wait for a centered stick
  serialBoom = false;
  serialWinch = 0;
  winchStep = 0;
  winchStepMs = millis();
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  iotbot.serialWrite(manual ? L(">> MANUEL mod: joystick X = kol, Y = makara.", ">> MANUAL mode: joystick X = boom, Y = winch.")
                            : L(">> OTOMATİK mod: vinç gösteri yapıyor.", ">> AUTO mode: the crane runs a demo."));
  if (!emergencyStop) showRunScreen();
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
  } else if ((word == "aci" || word == "angle") && hasValue) {
    if (!manualMode) setMode(true);
    serialBoom = true;
    serialBoomAngle = constrain(value, 0, 180);
    iotbot.serialWrite(String(L("Kol hedefi: ", "Boom target: ")) + serialBoomAngle + "°");
  } else if ((word == "vinc" || word == "winch") && hasValue) {
    if (!manualMode) setMode(true);
    serialWinch = constrain(value, -100, 100);
    waitForCenter = false;
    iotbot.serialWrite(String(L("Makara: %", "Winch: ")) + serialWinch + L("", "%"));
  } else if (word == "dur" || word == "stop") {
    if (!manualMode) setMode(true);
    serialWinch = 0;
    iotbot.serialWrite(L("Makara durdu.", "Winch stopped."));
  } else if (word == "acil" || word == "estop") {
    setEmergency(true);
  } else if (word == "devam" || word == "resume") {
    setEmergency(false);
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    if (emergencyStop) {
      emergencyStop = false; // Acil stop ekranını yeni dilde yeniden çiz / redraw the E-STOP screen in the new language
      setEmergency(true);
    } else {
      showRunScreen();
    }
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.moduleDCMotorStop();
  iotbot.moduleServoGoAngle(SERVO_PIN, boomAngle, 5); // Kolu ortaya al / center the boom

  // Kalibrasyon: joystick'e dokunulmadan orta noktasını ölç. / Calibrate: measure the center untouched.
  iotbot.lcdWriteMid(L("VİNÇ KUMANDASI", "CRANE CONTROL"), "", L("Joystick'e", "Do not touch"), L("dokunmayın...", "the joystick..."));
  delay(800);
  iotbot.calibrateJoystick(xCenter, yCenter); // 20 okuma ortalaması / average of 20 readings
  // Kalibrasyon sırasında kol itiliydiyse orta nokta saçma olur; varsayılana dön.
  // If the stick was pushed during calibration the center is nonsense; fall back to the default.
  if (xCenter < 1000 || xCenter > 3100 || yCenter < 1000 || yCenter > 3100) {
    iotbot.serialWrite(L("Kalibrasyon hatalı, 2048 kullanılıyor.", "Bad calibration, using 2048."));
    xCenter = 2048;
    yCenter = 2048;
  }
  char msg[48];
  snprintf(msg, sizeof(msg), L("Orta nokta X=%d Y=%d", "Center X=%d Y=%d"), xCenter, yCenter);
  iotbot.serialWrite(msg);
  winchStepMs = millis();
  showRunScreen();
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) Joystick düğmesi = ACİL STOP. INPUT_PULLUP: basılıyken false (LOW) döner.
  // 1) Joystick button = EMERGENCY STOP. INPUT_PULLUP: false when pressed.
  bool joyBtnDown = !iotbot.joystickButtonRead();
  if (joyBtnDown && !joyBtnWasDown && now - lastJoyBtnMs > 300) {
    lastJoyBtnMs = now;
    setEmergency(!emergencyStop);
  }
  joyBtnWasDown = joyBtnDown;

  // 2) B3 -> mod değiştir (sadece basıldığı an) / B3 -> toggle mode (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3 && now - lastB3Ms > 200) {
    lastB3Ms = now;
    setMode(!manualMode);
  }
  lastB3 = b3;

  // 3) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  if (emergencyStop) {
    delay(kLoopMs);
    return;
  }

  int target;     // Kolun hedef açısı / boom target angle
  int wanted;     // İstenen makara hızı / wanted winch speed
  int stepLimit;  // Döngü başına en fazla derece / max degrees per loop
  if (manualMode) {
    // X ekseni -> kolun HEDEF açısı. Kol her döngüde en fazla birkaç derece yaklaşır (yumuşak).
    // X axis -> the boom's TARGET angle. The boom moves at most a few degrees per loop (smooth).
    int xPercent = axisPercent(iotbot.joystickXRead(), xCenter);
    if (xPercent != 0) serialBoom = false; // Joystick oynadı: seri açıyı bırak / joystick moved: drop the serial angle
    target = serialBoom ? serialBoomAngle : 90 + xPercent * 90 / 100;
    stepLimit = kServoStepDeg;

    // Y ekseni -> makara yönü ve hızı. / Y axis -> winch direction and speed.
    int yPercent = axisPercent(iotbot.joystickYRead(), yCenter);
    if (waitForCenter && yPercent == 0) waitForCenter = false;
    if (yPercent != 0) serialWinch = 0;    // Joystick oynadı: seri hızı bırak / joystick moved: drop the serial speed
    wanted = waitForCenter ? 0 : (yPercent != 0 ? yPercent : serialWinch);
  } else {
    // OTOMATİK gösteri: kol uçlar arasında gidip gelir, makara sırayla sarar / durur / bırakır.
    // AUTO demo: the boom goes between the ends, the winch winds / stops / unwinds in turn.
    if (boomAngle == autoBoomTarget && (int32_t)(now - boomPauseUntilMs) >= 0) {
      if (boomPauseUntilMs == 0) {
        boomPauseUntilMs = now + kAutoBoomPauseMs; // Uçta kısa mola / short rest at the end
      } else {
        boomPauseUntilMs = 0;
        autoBoomTarget = (autoBoomTarget == kAutoBoomHigh) ? kAutoBoomLow : kAutoBoomHigh;
      }
    }
    target = autoBoomTarget;
    stepLimit = 1; // Gösteride daha yavaş / slower in the demo
    if (now - winchStepMs >= kWinchDemo[winchStep].ms) {
      winchStep = (winchStep + 1) % kWinchDemoCount;
      winchStepMs = now;
    }
    wanted = kWinchDemo[winchStep].percent;
  }

  int step = constrain(target - boomAngle, -stepLimit, stepLimit);
  if (step != 0) {
    boomAngle += step;
    iotbot.moduleServoGoAngle(SERVO_PIN, boomAngle, 1); // En fazla 3 ms bekler / waits at most 3 ms
  }
  if (wanted != winchPercent) setWinch(wanted); // Sadece değişince motora yaz / only write the motor on change

  if (now - lastScreenMs >= kScreenMs) {
    lastScreenMs = now;
    char line[41];
    snprintf(line, sizeof(line), L(" Kol açısı: %3d der", " Boom angle: %3d deg"), boomAngle);
    lcdRow(1, line);
    if (manualMode && waitForCenter) {
      snprintf(line, sizeof(line), "%s", L(" Kolu ortaya bırak", " Center the stick"));
    } else if (winchPercent > 0) {
      snprintf(line, sizeof(line), L(" Vinç: YUKARI %3d%%", " Winch: UP   %3d%%"), winchPercent);
    } else if (winchPercent < 0) {
      snprintf(line, sizeof(line), L(" Vinç: AŞAĞI  %3d%%", " Winch: DOWN %3d%%"), -winchPercent);
    } else {
      snprintf(line, sizeof(line), "%s", L(" Vinç: DURUYOR", " Winch: STOPPED"));
    }
    lcdRow(2, line);
  }
  delay(kLoopMs);
}
