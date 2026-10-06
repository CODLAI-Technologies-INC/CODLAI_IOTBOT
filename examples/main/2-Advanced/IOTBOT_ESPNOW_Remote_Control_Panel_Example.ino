// TR: GERCEK PROJE - Kablosuz Uzaktan Kumanda Paneli. IOTBOT bir kumanda
// masasina donusur ve komutlari ESP-NOW ile yayinlar:
//   Potansiyometre -> "servo" (0-180 derece)   B3 butonu    -> "role1" (ac/kapa)
//   Joystick butonu -> "role2" (ac/kapa)       Encoder butonu -> "led" (ac/kapa)
// LCD her komutun son durumunu gosterir. Bu kodu IOTBOT'a,
// MINIBOT_ESPNOW_Remote_Servo_Receiver_Example.ino dosyasini bir MINIBOT'a
// (servo + LED) ve ROLEBOT_ESPNOW_Remote_Relay_Receiver_Example.ino dosyasini
// bir ROLEBOT'a (2 role + LED) yukleyin - hepsini tek panelden yonetin!
// EN: A REAL PROJECT - Wireless Remote Control Panel. The IOTBOT becomes a
// control desk and broadcasts commands over ESP-NOW:
//   Potentiometer -> "servo" (0-180 degrees)   B3 button      -> "role1" (toggle)
//   Joystick button -> "role2" (toggle)        Encoder button -> "led" (toggle)
// The LCD shows the latest state of every command. Upload this to an IOTBOT,
// MINIBOT_ESPNOW_Remote_Servo_Receiver_Example.ino to a MINIBOT (servo + LED)
// and ROLEBOT_ESPNOW_Remote_Relay_Receiver_Example.ino to a ROLEBOT (2 relays
// + LED) - control them all from one panel!
//
// Baglanti / Wiring: Ek modul GEREKMEZ - potansiyometre, joystick, encoder
// ve B3 kartin uzerindedir. / NO extra module needed - the potentiometer,
// joystick, encoder and B3 are all on the board.

#define USE_ESPNOW
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

// NOT: B1/B2 KULLANILMIYOR - ikisi ayni analog pini (IO4) paylasir; USE_ESPNOW
// acikken ikisi de sadece digitalRead(IO4) dondurur, yani birbirinden ayirt
// edilemez. / NOTE: B1/B2 are NOT used - they share one analog pin (IO4); with
// USE_ESPNOW both just return digitalRead(IO4), so they cannot be told apart.

namespace {
  constexpr int kEspNowChannel = 1;        // Alicilarla AYNI kanal / SAME channel as the receivers
  constexpr uint32_t kGapMs = 40;          // Iki mesaj arasi en az bekleme (alici yetissin) / min gap between messages (let receivers keep up)
  constexpr uint32_t kServoGapMs = 100;    // Servo: saniyede en fazla 10 mesaj / servo: at most 10 messages per second
  constexpr int kServoMinChange = 3;       // Bu kadar derece degismeden gonderme / do not send below this change
  constexpr uint32_t kRefreshMs = 500;     // Her 500 ms'de bir durumu tekrar gonder (kayip paketi duzeltir) / resend one state every 500 ms (fixes lost packets)
  constexpr uint32_t kDebounceMs = 30;

  // Kenar algilama + debounce: basildigi ani SADECE BIR KEZ bildirir. Once
  // "birakilmis" gormeden basmayi kabul etmez (acilista yanlis tetik olmasin).
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

  EdgeButton b3Button, joyButton, encButton;
  bool role1 = false, role2 = false, led = false;
  bool dirty1 = false, dirty2 = false, dirtyLed = false;
  int servoSent = -100;                    // Son gonderilen aci (-100 = hic) / last angle sent (-100 = never)
  int servoWanted = 90;
  uint32_t lastSendMs = 0, lastServoMs = 0, lastRefreshMs = 0, lastLcdMs = 0;
  int refreshIndex = 0;
  const char *lastName = "-";

  void send(const char *name, int value) {
    iotbot.espNowSendNumber(name, value);
    lastSendMs = millis();
    lastName = name;
  }

  void showRow(int row, const char *text) {
    char line[21];
    snprintf(line, sizeof(line), "%-20s", text);
    iotbot.lcdWriteFixed(row, line);
  }

  const char *onOff(bool on) { return on ? (turkish ? "AC " : "ON ") : (turkish ? "KAP" : "OFF"); }
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.espNowBegin(kEspNowChannel);
  iotbot.lcdWriteMid(turkish ? "UZAKTAN KUMANDA" : "REMOTE CONTROL",
                      turkish ? "Pot=Servo  B3=Role1" : "Pot=Servo B3=Relay1",
                      turkish ? "Joy btn=Role2" : "Joy btn=Relay2",
                      turkish ? "Encoder btn=LED" : "Encoder btn=LED");
  delay(2500); // Yardim ekranini okumak icin / time to read the help screen
  iotbot.lcdWriteMid(turkish ? "UZAKTAN KUMANDA" : "REMOTE CONTROL", "", "", "");
  iotbot.serialWrite(turkish ? "Kumanda paneli hazir." : "Control panel ready.");
}

void loop() {
  uint32_t now = millis();

  // 1) Butonlar. B3: true = basili. Joystick/encoder butonu INPUT_PULLUP:
  // false (LOW) = basili, bu yuzden "!" ile ceviriyoruz.
  // 1) Buttons. B3: true = pressed. Joystick/encoder buttons are INPUT_PULLUP:
  // false (LOW) = pressed, so we invert them with "!".
  if (b3Button.pressed(iotbot.button3Read()))         { role1 = !role1; dirty1 = true;   iotbot.buzzerPlayTone(1500, 30); }
  if (joyButton.pressed(!iotbot.joystickButtonRead())) { role2 = !role2; dirty2 = true;   iotbot.buzzerPlayTone(1800, 30); }
  if (encButton.pressed(!iotbot.encoderButtonRead()))  { led = !led;     dirtyLed = true; iotbot.buzzerPlayTone(2100, 30); }

  // 2) Potansiyometre -> aci; sadece 3 dereceden fazla degisince gonder.
  // 2) Potentiometer -> angle; send only when it changes by 3 degrees or more.
  servoWanted = map(iotbot.potentiometerRead(), 0, 4095, 0, 180);
  bool servoDirty = abs(servoWanted - servoSent) >= kServoMinChange;

  // 3) En fazla 40 ms'de bir TEK mesaj: once degisen butonlar, sonra servo,
  // bos kalinca da sirayla durum tazeleme.
  // 3) At most ONE message every 40 ms: changed buttons first, then the servo,
  // and when idle, a rotating state refresh.
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

  // 4) LCD (200 ms'de bir, titremeden). / LCD (every 200 ms, no flicker).
  if (now - lastLcdMs >= 200) {
    lastLcdMs = now;
    char text[32];
    snprintf(text, sizeof(text), turkish ? "Servo (Pot): %3d dr" : "Servo (Pot): %3d deg", servoWanted);
    showRow(1, text);
    snprintf(text, sizeof(text), turkish ? "Role1:%s  Role2:%s" : "Rly1:%s   Rly2:%s", onOff(role1), onOff(role2));
    showRow(2, text);
    snprintf(text, sizeof(text), turkish ? "LED:%s Gonder:%s" : "LED:%s  Sent:%s", onOff(led), lastName);
    showRow(3, text);
  }

  delay(5);
}
