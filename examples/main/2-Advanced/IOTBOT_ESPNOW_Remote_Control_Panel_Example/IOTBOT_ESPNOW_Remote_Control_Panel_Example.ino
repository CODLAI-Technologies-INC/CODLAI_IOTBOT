/*
 * TR: GERÇEK PROJE - Kablosuz Uzaktan Kumanda Paneli
 *  IOTBOT bir kumanda masasına dönüşür ve komutları ESP-NOW ile yayınlar:
 *  "servo" (0-180 derece), "role1", "role2", "led" (1 = aç, 0 = kapat).
 *  - Açılışta OTOMATİK mod (gösteri): servo kendi kendine gidip gelir, röleler ve
 *    LED sırayla açılıp kapanır - alıcı kartlar tek başına hareket eder.
 *  - B3 butonu OTOMATİK <-> MANUEL geçişi yapar. MANUEL modda:
 *      Potansiyometre      -> "servo" (0-180 derece)
 *      Joystick Y (it/çek) -> "role1" (aç/kapa)
 *      Joystick butonu     -> "role2" (aç/kapa)
 *      Encoder butonu      -> "led"   (aç/kapa)
 *  - Bu kodu IOTBOT'a, MINIBOT_ESPNOW_Remote_Servo_Receiver_Example.ino dosyasını bir
 *    MINIBOT'a (servo + LED) ve ROLEBOT_ESPNOW_Remote_Relay_Receiver_Example.ino
 *    dosyasını bir ROLEBOT'a (2 röle + LED) yükleyin - hepsini tek panelden yönetin!
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help                   -> komut listesi
 *      oto    / auto                   -> otomatik mod (gösteri)
 *      manuel / manual                 -> manuel mod
 *      servo 90 / aci 90 / angle 90    -> servo açısı (manuele geçer)
 *      role1 ac / relay1 on            -> röle 1 aç (role1 kapat / relay1 off)
 *      role2 ac / relay2 on            -> röle 2 aç (role2 kapat / relay2 off)
 *      led ac / led on                 -> LED aç (led kapat / led off)
 *      dur    / stop                   -> hepsini kapat
 *      durum  / status                 -> son durumlar
 *      dil    / lang                   -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Wireless Remote Control Panel
 *  The IOTBOT becomes a control desk and broadcasts commands over ESP-NOW:
 *  "servo" (0-180 degrees), "role1", "role2", "led" (1 = on, 0 = off).
 *  - At startup AUTO mode (show) runs: the servo sweeps by itself, the relays and the
 *    LED switch on and off in turn - the receiver boards move on their own.
 *  - The B3 button toggles AUTO <-> MANUAL. In MANUAL mode:
 *      Potentiometer       -> "servo" (0-180 degrees)
 *      Joystick Y (push/pull) -> "role1" (toggle)
 *      Joystick button     -> "role2" (toggle)
 *      Encoder button      -> "led"   (toggle)
 *  - Upload this to an IOTBOT, MINIBOT_ESPNOW_Remote_Servo_Receiver_Example.ino to a
 *    MINIBOT (servo + LED) and ROLEBOT_ESPNOW_Remote_Relay_Receiver_Example.ino to a
 *    ROLEBOT (2 relays + LED) - control them all from one panel!
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim                 -> command list
 *      auto   / oto                    -> auto mode (show)
 *      manual / manuel                 -> manual mode
 *      servo 90 / angle 90 / aci 90    -> servo angle (switches to manual)
 *      relay1 on / role1 ac            -> relay 1 on (relay1 off / role1 kapat)
 *      relay2 on / role2 ac            -> relay 2 on (relay2 off / role2 kapat)
 *      led on / led ac                 -> LED on (led off / led kapat)
 *      stop   / dur                    -> switch everything off
 *      status / durum                  -> latest states
 *      lang   / dil                    -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ - potansiyometre, joystick, encoder ve B3
 * kartın üzerindedir. / NO extra module needed - the potentiometer, joystick, encoder
 * and B3 are all on the board.
 * NOT: B1/B2 ve joystick X KULLANILMIYOR - ESP-NOW açıkken B1/B2 birbirinden ayırt
 * edilemez, joystick X (ADC2) okunamaz. Joystick Y ve potansiyometre (ADC1) çalışır.
 * NOTE: B1/B2 and joystick X are NOT used - with ESP-NOW on, B1/B2 cannot be told
 * apart and joystick X (ADC2) cannot be read. Joystick Y and the pot (ADC1) work.
 */

#define USE_ESPNOW
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

const int kEspNowChannel = 1;          // Alıcılarla AYNI kanal / SAME channel as the receivers
const uint32_t kGapMs = 40;            // İki mesaj arası en az bekleme (alıcı yetişsin) / min gap between messages (let receivers keep up)
const uint32_t kServoGapMs = 100;      // Servo: saniyede en fazla 10 mesaj / servo: at most 10 messages per second
const int kServoMinChange = 3;         // Bu kadar derece değişmeden gönderme / do not send below this change
const uint32_t kRefreshMs = 500;       // Her 500 ms'de bir durumu tekrar gönder (kayıp paketi düzeltir) / resend one state every 500 ms (fixes lost packets)
const uint32_t kDebounceMs = 30;

// Kenar algılama + debounce: basıldığı anı SADECE BİR KEZ bildirir. Önce
// "bırakılmış" görmeden basmayı kabul etmez (açılışta yanlış tetik olmasın).
// Edge detect + debounce: reports the press moment ONLY ONCE. It ignores
// presses until it has seen "released" once (no false trigger at power-up).
struct EdgeButton {
  bool raw = false, stable = false, armed = false;
  uint32_t changedMs = 0;
  bool pressed(bool downNow) {
    if (downNow != raw) { raw = downNow; changedMs = millis(); }
    if (raw == stable || millis() - changedMs < kDebounceMs) return false;
    stable = raw;
    if (!stable) { armed = true; return false; }
    return armed;
  }
};

EdgeButton b3Button, joyUp, joyButton, encButton;
bool manualMode = false;               // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
bool role1 = false, role2 = false, led = false;
bool dirty1 = false, dirty2 = false, dirtyLed = false;
int servoSent = -100;                  // Son gönderilen açı (-100 = hiç) / last angle sent (-100 = never)
int servoWanted = 90;
int lastPotAngle = -1;                 // Manuelde potun son açısı / last pot angle in manual
int demoDir = 1;                       // Otomatik gösteride servo yönü / servo direction in the auto show
int demoStep = 0;
uint32_t lastSendMs = 0, lastServoMs = 0, lastRefreshMs = 0, lastLcdMs = 0;
uint32_t lastDemoServoMs = 0, lastDemoStepMs = 0;
int refreshIndex = 0;
const char *lastName = "-";

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "RÖLE1 AÇ" -> "role1 ac"
// Lower-cases and simplifies Turkish letters: "RÖLE1 AÇ" -> "role1 ac"
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
// Gönderim ve çıkışlar / Sending and outputs
// ---------------------------------------------------------------------------
// Ad ve değer alıcılarla AYNI kalmalı ("servo", "role1", "role2", "led").
// The names and values must stay the SAME as on the receivers ("servo", "role1", "role2", "led").
void send(const char *name, int value) {
  iotbot.espNowSendNumber(name, value);
  lastSendMs = millis();
  lastName = name;
}

void setRole1(bool on) { if (role1 != on) { role1 = on; dirty1 = true; } }
void setRole2(bool on) { if (role2 != on) { role2 = on; dirty2 = true; } }
void setLed(bool on) { if (led != on) { led = on; dirtyLed = true; } }

void allOff() {
  setRole1(false);
  setRole2(false);
  setLed(false);
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

const char *onOff(bool on) { return on ? L("AÇ ", "ON ") : L("KAP", "OFF"); }

void printHelp() {
  iotbot.serialWrite(L("---- UZAKTAN KUMANDA PANELİ - Komutlar ----", "---- REMOTE CONTROL PANEL - Commands ----"));
  iotbot.serialWrite(L("  yardim              : bu liste", "  help                : this list"));
  iotbot.serialWrite(L("  oto / manuel        : mod seç", "  auto / manual       : choose mode"));
  iotbot.serialWrite(L("  servo 90            : servo açısı (0-180)", "  servo 90            : servo angle (0-180)"));
  iotbot.serialWrite(L("  role1 ac / kapat    : röle 1", "  relay1 on / off     : relay 1"));
  iotbot.serialWrite(L("  role2 ac / kapat    : röle 2", "  relay2 on / off     : relay 2"));
  iotbot.serialWrite(L("  led ac / kapat      : LED", "  led on / off        : LED"));
  iotbot.serialWrite(L("  dur                 : hepsini kapat", "  stop                : switch everything off"));
  iotbot.serialWrite(L("  durum, dil", "  status, lang"));
  iotbot.serialWrite(L("  B3: OTOMATİK <-> MANUEL. Manuel: Pot=servo, Joy Y=röle1, Joy btn=röle2, Enc btn=LED",
                       "  B3: AUTO <-> MANUAL. Manual: Pot=servo, Joy Y=relay1, Joy btn=relay2, Enc btn=LED"));
}

void drawHint() {
  lcdRow(3, manualMode ? L("JoyY:R1 Joy:R2 Enc:L", "JoyY:R1 Joy:R2 Enc:L") : L("B3: manuel kontrol", "B3: manual control"));
  lastLcdMs = 0;
}

void setMode(bool manual) {
  manualMode = manual;
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  allOff(); // Mod değişince röleler/LED güvenli şekilde kapanır / relays/LED switch off safely on a mode change
  if (manual) {
    lastPotAngle = -1; // Pot hemen geçerli olsun / the pot takes effect right away
    iotbot.serialWrite(L(">> MANUEL mod: Pot=servo, Joystick Y (it/çek)=röle1, Joystick butonu=röle2, Encoder butonu=LED.",
                         ">> MANUAL mode: Pot=servo, Joystick Y (push/pull)=relay1, Joystick button=relay2, Encoder button=LED."));
  } else {
    demoStep = 0;
    lastDemoStepMs = millis();
    iotbot.serialWrite(L(">> OTOMATİK mod: gösteri - servo gidip gelir, röleler ve LED sırayla yanar.",
                         ">> AUTO mode: show - the servo sweeps, relays and LED switch in turn."));
  }
  drawHint();
}

void printStatus() {
  iotbot.serialWrite(String(L("Mod: ", "Mode: ")) + (manualMode ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO")) +
                     "   Servo: " + servoWanted + L("°   Röle1: ", "°   Relay1: ") + onOff(role1) +
                     L("  Röle2: ", "  Relay2: ") + onOff(role2) + "  LED: " + onOff(led) +
                     L("   Son gönderilen: ", "   Last sent: ") + lastName);
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  String arg = (space < 0) ? "" : cmd.substring(space + 1);
  arg.trim();
  bool on = (arg == "ac" || arg == "on" || arg == "1");
  bool off = (arg == "kapat" || arg == "off" || arg == "0");

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oto" || word == "otomatik" || word == "auto") {
    setMode(false);
  } else if (word == "manuel" || word == "manual") {
    setMode(true);
  } else if ((word == "servo" || word == "aci" || word == "angle") && arg.length() > 0) {
    if (!manualMode) setMode(true);
    servoWanted = constrain(arg.toInt(), 0, 180);
    // Seri komutla verilen açıyı potansiyometre hemen ezmesin / so the pot does not override the serial angle right away
    lastPotAngle = map(iotbot.potentiometerRead(), 0, 4095, 0, 180);
    iotbot.serialWrite(String(L("Servo hedefi: ", "Servo target: ")) + servoWanted + "°");
  } else if ((word == "role1" || word == "relay1") && (on || off)) {
    if (!manualMode) setMode(true);
    setRole1(on);
    iotbot.serialWrite(String(L("Röle 1: ", "Relay 1: ")) + onOff(on));
  } else if ((word == "role2" || word == "relay2") && (on || off)) {
    if (!manualMode) setMode(true);
    setRole2(on);
    iotbot.serialWrite(String(L("Röle 2: ", "Relay 2: ")) + onOff(on));
  } else if (word == "led" && (on || off)) {
    if (!manualMode) setMode(true);
    setLed(on);
    iotbot.serialWrite(String("LED: ") + onOff(on));
  } else if (word == "dur" || word == "stop") {
    if (!manualMode) setMode(true);
    allOff();
    iotbot.serialWrite(L("Hepsi kapatıldı.", "Everything switched off."));
  } else if (word == "durum" || word == "status") {
    printStatus();
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    drawHint();
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.espNowBegin(kEspNowChannel);
  iotbot.lcdClear();
  drawHint();
  iotbot.serialWrite(L("Kumanda paneli hazır.", "Control panel ready."));
  printHelp();
  lastDemoStepMs = millis();
}

void loop() {
  uint32_t now = millis();

  // 1) B3 -> mod değiştir. B3: true = basılı. / B3 -> toggle mode. B3: true = pressed.
  if (b3Button.pressed(iotbot.button3Read())) setMode(!manualMode);

  // 2) Seri komutlar / serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  if (manualMode) {
    // 3a) MANUEL: joystick/encoder butonları INPUT_PULLUP: LOW = basılı, bu yüzden "!" ile çeviriyoruz.
    // 3a) MANUAL: joystick/encoder buttons are INPUT_PULLUP: LOW = pressed, so we invert them with "!".
    int joyY = iotbot.joystickYRead(); // Ortada ~2000 / ~2000 in the middle
    if (joyUp.pressed(joyY > 3500 || joyY < 500))       { setRole1(!role1); iotbot.buzzerPlayTone(1500, 30); }
    if (joyButton.pressed(!iotbot.joystickButtonRead()))     { setRole2(!role2); iotbot.buzzerPlayTone(1800, 30); }
    if (encButton.pressed(!iotbot.encoderButtonRead()))      { setLed(!led);     iotbot.buzzerPlayTone(2100, 30); }

    // Potansiyometre -> açı; sadece gerçekten çevrilince (3° eşik) / pot -> angle; only when really turned (3° threshold)
    int potAngle = map(iotbot.potentiometerRead(), 0, 4095, 0, 180);
    if (lastPotAngle < 0 || abs(potAngle - lastPotAngle) >= kServoMinChange) {
      lastPotAngle = potAngle;
      servoWanted = potAngle;
    }
  } else {
    // 3b) OTOMATİK gösteri: servo 150 ms'de 5° kayar; her 2 sn'de röle/LED deseni değişir
    // 3b) AUTO show: the servo moves 5° every 150 ms; the relay/LED pattern changes every 2 s
    if (now - lastDemoServoMs >= 150) {
      lastDemoServoMs = now;
      servoWanted += 5 * demoDir;
      if (servoWanted >= 180) { servoWanted = 180; demoDir = -1; }
      if (servoWanted <= 0) { servoWanted = 0; demoDir = 1; }
    }
    if (now - lastDemoStepMs >= 2000) {
      lastDemoStepMs = now;
      demoStep = (demoStep + 1) % 4;
      setRole1(demoStep == 1 || demoStep == 2);
      setRole2(demoStep == 2 || demoStep == 3);
      setLed(demoStep % 2 == 1);
    }
  }

  // 4) En fazla 40 ms'de bir TEK mesaj: önce değişen anahtarlar, sonra servo, boş kalınca
  //    da sırayla durum tazeleme.
  // 4) At most ONE message every 40 ms: changed switches first, then the servo, and when
  //    idle, a rotating state refresh.
  bool servoDirty = abs(servoWanted - servoSent) >= kServoMinChange || (servoWanted != servoSent && (servoWanted == 0 || servoWanted == 180));
  if (now - lastSendMs >= kGapMs) {
    if (dirty1)        { send("role1", role1); dirty1 = false; }
    else if (dirty2)   { send("role2", role2); dirty2 = false; }
    else if (dirtyLed) { send("led", led);     dirtyLed = false; }
    else if (servoDirty && now - lastServoMs >= kServoGapMs) {
      servoSent = servoWanted;
      lastServoMs = now;
      send("servo", servoSent);
    } else if (now - lastRefreshMs >= kRefreshMs) {
      lastRefreshMs = now;
      refreshIndex = (refreshIndex + 1) % 4;
      if (refreshIndex == 0 && servoSent >= 0) send("servo", servoSent);
      else if (refreshIndex == 1) send("role1", role1);
      else if (refreshIndex == 2) send("role2", role2);
      else if (refreshIndex == 3) send("led", led);
    }
  }

  // 5) LCD (200 ms'de bir, titremeden). / LCD (every 200 ms, no flicker).
  if (now - lastLcdMs >= 200) {
    lastLcdMs = now;
    char text[41];
    snprintf(text, sizeof(text), L("%-8s  Servo:%3d", "%-8s  Servo:%3d"), manualMode ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO"), servoWanted);
    lcdRow(0, text);
    snprintf(text, sizeof(text), L("Röle1:%s  Röle2:%s", "Rly1:%s   Rly2:%s"), onOff(role1), onOff(role2));
    lcdRow(1, text);
    snprintf(text, sizeof(text), L("LED:%s Gönd:%s", "LED:%s  Sent:%s"), onOff(led), lastName);
    lcdRow(2, text);
  }

  delay(5);
}
