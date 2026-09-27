// TR: GERCEK PROJE - Park Sensoru. Ultrasonik mesafe sensoru bir cismi
// (ornegin bir duvari ya da baska bir araci) algiladikca buzzer'i
// GIDEREK HIZLANAN bir sekilde "bip" sesi cikartir - tipki arabalardaki
// park sensoru gibi. Cok yaklasinca (10cm alti) sesi surekli/sabit hale
// gelir ve LCD "DUR!" yazar, ayrica kart uzerindeki roleyi (ornegin bir
// kirmizi ikaz lambasi baglayabilirsiniz) tetikler.
// EN: A REAL PROJECT - Parking Sensor. As the ultrasonic distance sensor
// detects an object getting closer (e.g. a wall or another car), the
// buzzer beeps FASTER AND FASTER - just like a real car's parking
// sensor. When very close (under 10cm) the sound becomes constant and
// the LCD shows "STOP!", and it also triggers the board's onboard relay
// (you can wire a red warning lamp to it).
//
// Baglanti / Wiring: Ultrasonik sensoru herhangi bir P1-P5 soketine
// takmaniza GEREK YOK - bu modul sabit pinler kullanir (TRIG=IO27,
// ECHO=IO32). / You do NOT need to plug the ultrasonic sensor into any
// P1-P5 socket - this module uses fixed pins (TRIG=IO27, ECHO=IO32).

#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

namespace {
  constexpr int kStopDistanceCm = 10;   // Bu mesafenin altinda "DUR!" / below this: "STOP!"
  constexpr int kMaxUsefulDistanceCm = 100; // Bu mesafenin ustunde sessiz / above this: silent
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.relayWrite(false);
  iotbot.lcdWriteMid(turkish ? "PARK SENSORU" : "PARKING SENSOR",
                      turkish ? "Bir cisme yaklastirin" : "Bring an object near",
                      turkish ? "sensoru test edin" : "to test the sensor",
                      "");
  iotbot.serialWrite(turkish ? "Park sensoru hazir." : "Parking sensor ready.");
}

void loop() {
  int distance = iotbot.moduleUltrasonicDistanceRead();
  bool valid = distance > 0 && distance < 400;

  if (!valid || distance > kMaxUsefulDistanceCm) {
    // Cok uzak ya da okuma gecersiz - sessiz / too far or invalid reading - stay quiet
    iotbot.relayWrite(false);
    iotbot.lcdWriteMid(turkish ? "PARK SENSORU" : "PARKING SENSOR",
                        turkish ? "Yol acik" : "Path clear",
                        "", "");
    delay(200);
    return;
  }

  char line[21];
  if (distance <= kStopDistanceCm) {
    // Cok yakin: surekli ses + role tetikle / very close: continuous beep + trigger relay
    iotbot.relayWrite(true);
    snprintf(line, sizeof(line), turkish ? "DUR! %dcm" : "STOP! %dcm", distance);
    iotbot.lcdWriteMid(turkish ? "PARK SENSORU" : "PARKING SENSOR", line, "", "");
    iotbot.buzzerPlayTone(1800, 300);
  } else {
    // Mesafeye gore bip hizini ayarla: yaklastikca daha sik bip / beep
    // rate scales with distance: closer = faster beeping
    iotbot.relayWrite(false);
    snprintf(line, sizeof(line), turkish ? "Mesafe: %dcm" : "Distance: %dcm", distance);
    iotbot.lcdWriteMid(turkish ? "PARK SENSORU" : "PARKING SENSOR", line, "", "");
    int beepGapMs = map(distance, kStopDistanceCm, kMaxUsefulDistanceCm, 60, 600);
    iotbot.buzzerPlayTone(1800, 60);
    delay(beepGapMs);
    return;
  }
  delay(100);
}
