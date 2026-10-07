/*
 * TR: SERVO MOTOR MODÜLÜ - Otomatik demo + Manuel kontrol
 *  - Açılışta OTOMATİK mod çalışır: servo 0° ile 180° arasında yavaşça gidip gelir.
 *  - B3 butonuna basınca MANUEL moda geçer: açıyı kart üzerindeki potansiyometre
 *    ile siz ayarlarsınız. B3'e tekrar basınca otomatik moda döner.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help       -> komut listesi
 *      oto     / auto       -> otomatik mod
 *      manuel  / manual     -> manuel mod (potansiyometre)
 *      aci 90  / angle 90   -> servoyu 90°'ye götür (manuel moda geçer)
 *      dil     / lang       -> dili değiştir (Türkçe <-> English)
 *
 * EN: SERVO MOTOR MODULE - Automatic demo + Manual control
 *  - At startup AUTO mode runs: the servo sweeps slowly between 0° and 180°.
 *  - Press B3 to switch to MANUAL mode: you set the angle with the onboard
 *    potentiometer. Press B3 again to go back to auto mode.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help    / yardim     -> command list
 *      auto    / oto        -> auto mode
 *      manual  / manuel     -> manual mode (potentiometer)
 *      angle 90 / aci 90    -> move the servo to 90° (switches to manual)
 *      lang    / dil        -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Servoyu IO27'ye bağlı sokete takın.
 * Desteklenen pinler / Supported pins: IO25 - IO26 - IO27 - IO32 - IO33
 */

#define USE_SERVO
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

#define SERVO_PIN IO27 // Servonun bağlı olduğu pin / Pin the servo is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

bool manualMode = false;   // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
int angle = 90;            // Servonun şu anki açısı / current servo angle
int targetAngle = 90;      // Gitmek istediği açı / angle it is heading to
int sweepDir = 1;          // Otomatik moddaki yön (+1 / -1) / sweep direction in auto mode
uint32_t lastStepMs = 0;   // Son adım zamanı / time of the last step
uint32_t pauseUntilMs = 0; // Uçlarda kısa mola / short rest at the ends
uint32_t lastScreenMs = 0;
bool lastB3 = false;
int lastPotAngle = -1;     // Potansiyometrenin son açısı / last potentiometer angle

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
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
// lcdWriteFixedTxt Türkçe harfleri (ç, ğ, ı, ö, ş, ü) LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- SERVO MOTOR - Komutlar ----", "---- SERVO MOTOR - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oto           : otomatik mod", "  auto          : auto mode"));
  iotbot.serialWrite(L("  manuel        : manuel mod (potansiyometre)", "  manual        : manual mode (potentiometer)"));
  iotbot.serialWrite(L("  aci 0-180     : servoyu o açıya götür", "  angle 0-180   : move the servo to that angle"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : OTOMATİK <-> MANUEL", "  B3 button     : AUTO <-> MANUAL"));
}

void drawStaticScreen() {
  lcdRow(0, "    SERVO MOTOR");
  lcdRow(3, manualMode ? L("Pot:açı  B3:otomatik", "Pot:angle  B3:auto") : L("B3: manuel kontrol", "B3: manual control"));
  lastScreenMs = 0; // Değerleri hemen çiz / draw the values right away
}

void setMode(bool manual) {
  manualMode = manual;
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  iotbot.serialWrite(manual ? L(">> MANUEL mod: açıyı potansiyometre ile ayarlayın.", ">> MANUAL mode: set the angle with the potentiometer.")
                            : L(">> OTOMATİK mod: servo kendi kendine gidip geliyor.", ">> AUTO mode: the servo sweeps by itself."));
  drawStaticScreen();
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
    targetAngle = constrain(value, 0, 180);
    // Seri komutla verilen açıyı potansiyometre hemen ezmesin diye pot konumunu kaydet.
    // Remember the pot position so it does not override the serial angle right away.
    lastPotAngle = map(iotbot.potentiometerRead(), 0, 4095, 0, 180);
    iotbot.serialWrite(String(L("Hedef açı: ", "Target angle: ")) + targetAngle);
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
  iotbot.lcdClear();
  iotbot.moduleServoGoAngle(SERVO_PIN, angle, 1); // Ortadan başla / start at the middle
  drawStaticScreen();
  iotbot.serialWrite(L("Servo motor testi başladı.", "Servo motor test started."));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) B3 -> mod değiştir (sadece basıldığı an) / B3 -> toggle mode (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) {
    setMode(!manualMode);
    lastPotAngle = -1; // Manuelde pot hemen geçerli olsun / pot takes effect immediately in manual
  }
  lastB3 = b3;

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Hedef açıyı belirle / Decide the target angle
  if (manualMode) {
    // Pot sadece gerçekten çevrilince hedefi değiştirir (titreşimi yok saymak için 3° eşik).
    // The pot changes the target only when really turned (3° threshold ignores jitter).
    int potAngle = map(iotbot.potentiometerRead(), 0, 4095, 0, 180);
    if (lastPotAngle < 0 || abs(potAngle - lastPotAngle) >= 3) {
      lastPotAngle = potAngle;
      targetAngle = potAngle;
    }
  } else if (now >= pauseUntilMs && angle == targetAngle) {
    // Uca varınca yön değiştir ve yarım saniye bekle / at an end: reverse and rest 0.5 s
    if (angle >= 180) sweepDir = -1;
    if (angle <= 0) sweepDir = 1;
    targetAngle = (sweepDir > 0) ? 180 : 0;
    iotbot.serialWrite(String(L("Otomatik: hedef ", "Auto: heading to ")) + targetAngle + "°");
  }

  // 4) Servoyu her 15 ms'de 1° hedefe yaklaştır (loop hiç bloklanmaz, B3 anında çalışır).
  // 4) Move the servo 1° toward the target every 15 ms (loop never blocks, B3 reacts instantly).
  if (angle != targetAngle && now - lastStepMs >= 15) {
    lastStepMs = now;
    angle += (targetAngle > angle) ? 1 : -1;
    iotbot.moduleServoGoAngle(SERVO_PIN, angle, 1);
    if (!manualMode && angle == targetAngle) pauseUntilMs = now + 500;
  }

  // 5) LCD (200 ms'de bir, titremesiz) / LCD (every 200 ms, no flicker)
  if (now - lastScreenMs >= 200) {
    lastScreenMs = now;
    char line[41];
    snprintf(line, sizeof(line), L("Mod: %s", "Mode: %s"), manualMode ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO"));
    lcdRow(1, line);
    snprintf(line, sizeof(line), L("Açı: %3d derece", "Angle: %3d deg"), angle);
    lcdRow(2, line);
  }
}
