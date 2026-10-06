// TR: GERCEK PROJE - Temassiz Cop Kutusu. Elinizi cop kutusunun uzerindeki
// ultrasonik sensore 20 cm'den fazla yaklastirin: kapak (servo motor) kendi
// kendine acilir ve kisa bir "bip" sesi gelir. Eliniz yakindayken kapak acik
// kalir; elinizi cektikten 3 saniye sonra yavasca kapanir. Kapaga hic
// dokunmadiginiz icin mikrop bulasmaz! LCD kutunun kac kez kullanildigini sayar.
// EN: A REAL PROJECT - Touchless Trash Can. Bring your hand closer than
// 20 cm to the ultrasonic sensor on top of the bin: the lid (servo motor)
// opens by itself with a short "beep". The lid stays open while your hand
// is near and closes gently 3 seconds after you pull your hand away. You
// never touch the lid, so no germs are spread! The LCD counts how many
// times the bin was used.
//
// Baglanti / Wiring: Ultrasonik sensor sabit pinler kullanir (TRIG=IO27,
// ECHO=IO32), soket secmenize gerek yok. Kapak servosunu P2 soketine (IO26)
// takin. Trafik lambasini bu ornekte KULLANMAYIN (IO32'yi paylasir). / The
// ultrasonic sensor uses fixed pins (TRIG=IO27, ECHO=IO32), no socket choice
// needed. Plug the lid servo into socket P2 (IO26). Do NOT use the traffic
// light in this example (it shares IO32).

#define USE_SERVO
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define SERVO_PIN IO26 // Kapak servosu: P2 / lid servo: P2

namespace {
  constexpr int kNearCm = 20;                   // Bu mesafenin alti "el var" / closer than this = hand
  constexpr uint32_t kConfirmMs = 150;          // El bu kadar sure kalmali / hand must stay this long
  constexpr uint32_t kCloseDelayMs = 3000;      // El gidince kapanma gecikmesi / close delay after hand leaves
  constexpr uint32_t kMeasureIntervalMs = 60;   // Sensor en az 60 ms arayla olcmeli / sensor needs 60 ms between pings
  constexpr uint32_t kUiIntervalMs = 300;       // LCD yenileme araligi / LCD refresh interval
  constexpr int kOpenAngle = 90;                // Kapak acik aci / lid open angle
  constexpr int kClosedAngle = 0;               // Kapak kapali aci / lid closed angle
  constexpr int kOpenMsPerDeg = 3;              // Hizli acilis (~0.3 sn) / fast opening (~0.3 s)
  constexpr int kCloseMsPerDeg = 6;             // Yavas kapanis (~0.5 sn) / gentle closing (~0.5 s)

  bool lidOpen = false;
  bool handSeen = false;        // Su an el algilaniyor mu? / is a hand detected right now?
  uint32_t handSinceMs = 0;     // El ne zamandan beri var / since when the hand is there
  uint32_t lastHandMs = 0;      // Eli en son ne zaman gorduk / last time we saw the hand
  uint32_t lastMeasureMs = 0;
  uint32_t lastUiMs = 0;
  unsigned long useCount = 0;
  int distanceCm = 0;
}

void showMainScreen() {
  char line[21];
  snprintf(line, sizeof(line), turkish ? "Kullanim: %lu" : "Used: %lu times", useCount);
  iotbot.lcdWriteMid(turkish ? "TEMASSIZ COP KUTUSU" : "TOUCHLESS TRASH CAN",
                     lidOpen ? (turkish ? "Kapak: ACIK" : "Lid: OPEN")
                             : (turkish ? "Kapak: KAPALI" : "Lid: CLOSED"),
                     line, "");
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.moduleServoGoAngle(SERVO_PIN, kClosedAngle, 1);  // Baslangicta kapali / start closed
  showMainScreen();
  iotbot.serialWrite(turkish ? "Temassiz cop kutusu hazir." : "Touchless trash can ready.");
}

void loop() {
  uint32_t now = millis();

  // 1) Olcum: 0 = "yanki yok / menzil disi" demektir, el yok sayilir.
  // 1) Measure: 0 means "no echo / out of range", treated as no hand.
  if (now - lastMeasureMs >= kMeasureIntervalMs) {
    lastMeasureMs = now;
    distanceCm = iotbot.moduleUltrasonicDistanceRead();
    bool near = distanceCm > 0 && distanceCm < kNearCm;
    if (near) {
      if (!handSeen) { handSeen = true; handSinceMs = now; }
      lastHandMs = now;
    } else {
      handSeen = false;
    }
  }

  // 2) Acma: el 150 ms boyunca kesintisiz yakin olmali (tek hatali olcum kapagi acmasin).
  // 2) Open: the hand must stay near for 150 ms (one bad reading must not open the lid).
  if (!lidOpen && handSeen && now - handSinceMs >= kConfirmMs) {
    lidOpen = true;
    useCount++;
    iotbot.buzzerPlayTone(1500, 40);
    iotbot.moduleServoGoAngle(SERVO_PIN, kOpenAngle, kOpenMsPerDeg);
    showMainScreen();
    iotbot.serialWrite(turkish ? "Kapak acildi." : "Lid opened.");
  }

  // 3) Kapama: el 3 sn boyunca hic gorunmediyse. El geri gelirse sure sifirlanir.
  // 3) Close: when no hand was seen for 3 s. If the hand comes back the timer restarts.
  if (lidOpen && millis() - lastHandMs >= kCloseDelayMs) {
    lidOpen = false;
    iotbot.moduleServoGoAngle(SERVO_PIN, kClosedAngle, kCloseMsPerDeg);
    showMainScreen();
    iotbot.serialWrite(turkish ? "Kapak kapandi." : "Lid closed.");
  }

  // 4) Alt satir: canli mesafe ya da kapanmaya kalan sure.
  // 4) Bottom row: live distance or time left until closing.
  if (now - lastUiMs >= kUiIntervalMs) {
    lastUiMs = now;
    char line[21];
    if (lidOpen && !handSeen) {
      uint32_t gone = millis() - lastHandMs;
      uint32_t left = gone < kCloseDelayMs ? (kCloseDelayMs - gone + 999) / 1000 : 0;
      snprintf(line, sizeof(line), turkish ? "Kapaniyor: %lu sn" : "Closing in: %lu s", (unsigned long)left);
    } else if (distanceCm > 0) {
      snprintf(line, sizeof(line), turkish ? "Mesafe: %d cm" : "Distance: %d cm", distanceCm);
    } else {
      snprintf(line, sizeof(line), "%s", turkish ? "Elini yaklastir" : "Bring your hand");
    }
    iotbot.lcdWriteFixed(3, line);
  }
  delay(5);
}
