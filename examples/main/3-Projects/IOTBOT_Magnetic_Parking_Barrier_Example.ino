// TR: GERCEK PROJE - Manyetik Otopark Bariyeri. Gercek otoparklarda yolun
// altinda arabanin metal govdesini algilayan manyetik sensorler vardir. Bu
// projede manyetik sensor "araba dedektoru" olur: sensore bir miknatis
// (= araba) yaklastirin ve 0.3 saniye bekletin. Bariyer (servo motor) bir
// "bip" sesiyle 0'dan 90 dereceye kalkar, araba oradayken acik kalir ve
// araba gittikten 2 saniye sonra iner. LCD "BARIYER ACIK/KAPALI" yazar ve
// gecen araclari sayar. Bariyer acikken kart uzerindeki role de acilir -
// buraya bir ikaz lambasi baglayabilirsiniz.
// EN: A REAL PROJECT - Magnetic Parking Barrier. Real car parks have magnetic
// sensors under the road that detect a car's metal body. In this project
// the magnetic sensor is the "car detector": bring a magnet (= a car) near
// the sensor and hold it for 0.3 seconds. The barrier (servo motor) rises
// from 0 to 90 degrees with a "beep", stays open while the car is there
// and goes down 2 seconds after the car leaves. The LCD shows "BARRIER
// OPEN/CLOSED" and counts the cars. While the barrier is open the onboard
// relay is also on - you can wire a warning lamp to it.
//
// Baglanti / Wiring: Manyetik sensoru P1 soketine (IO25), bariyer servosunu
// P2 soketine (IO26) takin. Role kart uzerindedir (istege bagli ikaz lambasi
// icin). / Plug the magnetic sensor into socket P1 (IO25) and the barrier
// servo into socket P2 (IO26). The relay is on the board (optional warning
// lamp).

#define USE_SERVO
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define MAGNET_PIN IO25 // Manyetik sensor: P1 / magnetic sensor: P1
#define SERVO_PIN IO26  // Bariyer servosu: P2 / barrier servo: P2

namespace {
  constexpr uint32_t kConfirmMs = 300;      // Araba bu kadar sure algilanmali / car must be seen this long
  constexpr uint32_t kCloseDelayMs = 2000;  // Araba gidince kapanma gecikmesi / close delay after car leaves
  constexpr uint32_t kUiIntervalMs = 250;   // LCD yenileme araligi / LCD refresh interval
  constexpr int kOpenAngle = 90;            // Bariyer yukarida / barrier up
  constexpr int kClosedAngle = 0;           // Bariyer asagida / barrier down
  constexpr int kMsPerDeg = 5;              // Yavas, gercekci hareket (~0.45 sn) / slow, realistic motion

  bool barrierOpen = false;
  bool rawSeen = false;         // Ham sensor okumasi true mu? / raw sensor reading true?
  uint32_t rawSinceMs = 0;
  uint32_t lastCarMs = 0;       // Arabayi en son ne zaman gorduk / last time the car was seen
  uint32_t lastUiMs = 0;
  unsigned long carCount = 0;
}

void showMainScreen() {
  char line[21];
  snprintf(line, sizeof(line), turkish ? "Arac sayisi: %lu" : "Cars: %lu", carCount);
  iotbot.lcdWriteMid(turkish ? "OTOPARK BARIYERI" : "PARKING BARRIER",
                     barrierOpen ? (turkish ? "BARIYER ACIK" : "BARRIER OPEN")
                                 : (turkish ? "BARIYER KAPALI" : "BARRIER CLOSED"),
                     line, "");
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.relayWrite(false);
  iotbot.moduleServoGoAngle(SERVO_PIN, kClosedAngle, 1);  // Baslangicta kapali / start closed
  showMainScreen();
  iotbot.serialWrite(turkish ? "Otopark bariyeri hazir." : "Parking barrier ready.");
}

void loop() {
  uint32_t now = millis();

  // 1) Araba algilama: sensor girisi bosta "yuzebilir" (rastgele okuyabilir),
  // bu yuzden sinyal 300 ms KESINTISIZ true kalmadan araba sayilmaz.
  // 1) Car detection: the input can "float" (read random values), so the
  // signal must stay true for 300 ms WITHOUT a break before it counts.
  bool raw = iotbot.moduleMagneticRead(MAGNET_PIN);
  if (raw && !rawSeen) rawSinceMs = now;
  rawSeen = raw;
  bool carPresent = raw && now - rawSinceMs >= kConfirmMs;
  if (carPresent) lastCarMs = now;

  // 2) Araba geldi -> bariyeri kaldir, bip, ikaz lambasini yak.
  // 2) Car arrived -> raise the barrier, beep, turn on the warning lamp.
  if (!barrierOpen && carPresent) {
    barrierOpen = true;
    carCount++;
    iotbot.relayWrite(true);
    iotbot.buzzerPlayTone(1400, 50);
    iotbot.moduleServoGoAngle(SERVO_PIN, kOpenAngle, kMsPerDeg);
    showMainScreen();
    iotbot.serialWrite(turkish ? "Arac geldi, bariyer acildi." : "Car arrived, barrier opened.");
  }

  // 3) Araba 2 sn'dir yok -> bariyeri indir. Araba geri gelirse sure sifirlanir.
  // 3) No car for 2 s -> lower the barrier. If the car comes back the timer restarts.
  if (barrierOpen && millis() - lastCarMs >= kCloseDelayMs) {
    barrierOpen = false;
    iotbot.buzzerPlayTone(800, 50);
    iotbot.moduleServoGoAngle(SERVO_PIN, kClosedAngle, kMsPerDeg);
    iotbot.relayWrite(false);  // Bariyer tamamen inince lamba soner / lamp off once fully down
    showMainScreen();
    iotbot.serialWrite(turkish ? "Bariyer kapandi." : "Barrier closed.");
  }

  // 4) Alt satir: canli durum.
  // 4) Bottom row: live status.
  if (now - lastUiMs >= kUiIntervalMs) {
    lastUiMs = now;
    char line[21];
    if (barrierOpen && carPresent) {
      snprintf(line, sizeof(line), "%s", turkish ? "Arac geciyor..." : "Car passing...");
    } else if (barrierOpen) {
      uint32_t gone = millis() - lastCarMs;
      uint32_t left = gone < kCloseDelayMs ? (kCloseDelayMs - gone + 999) / 1000 : 0;
      snprintf(line, sizeof(line), turkish ? "Kapaniyor: %lu sn" : "Closing in: %lu s", (unsigned long)left);
    } else {
      snprintf(line, sizeof(line), "%s", turkish ? "Arac bekleniyor" : "Waiting for a car");
    }
    iotbot.lcdWriteFixed(3, line);
  }
  delay(10);
}
