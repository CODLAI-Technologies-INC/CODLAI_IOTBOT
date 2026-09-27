// TR: GERCEK PROJE - Guvenlik Alarmi. PIR hareket sensoru bir hareket
// algiladiginda: buzzer surekli alarm sesi calar, LCD "ALARM!" yazar ve
// kart uzerindeki role tetiklenir (ornegin bir siren ya da isik
// baglayabilirsiniz). B3 butonuna basarak alarmi susturabilirsiniz -
// tipki gercek bir alarm sisteminin "iptal" tusu gibi.
// EN: A REAL PROJECT - Security Alarm. When the PIR motion sensor
// detects movement: the buzzer sounds a continuous alarm, the LCD shows
// "ALARM!" and the board's relay is triggered (you can wire a siren or a
// light to it). Press B3 to silence the alarm - just like the "cancel"
// button on a real alarm system.
//
// Baglanti / Wiring: PIR sensorunu P1-P5 soketlerinden BIRINE takin ve
// asagidaki PIR_PIN degerini o soketin sinyaline gore ayarlayin. / Plug
// the PIR sensor into ONE of the P1-P5 sockets and set PIR_PIN below to
// match that socket's signal.

#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define PIR_PIN IO27 // PIR sensorunun bagli oldugu pin / Pin the PIR sensor is connected to
// Desteklenen pinler: IO25 - IO26 - IO27 - IO32 - IO33
// Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

namespace {
  bool alarmActive = false;
  uint32_t lastBeepMs = 0;
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.relayWrite(false);
  iotbot.lcdWriteMid(turkish ? "GUVENLIK ALARMI" : "SECURITY ALARM",
                      turkish ? "Sistem aktif" : "System armed",
                      turkish ? "Hareket bekleniyor" : "Watching for motion",
                      "");
  iotbot.serialWrite(turkish ? "Guvenlik sistemi aktif." : "Security system armed.");
}

void loop() {
  bool motionDetected = iotbot.moduleMotionRead(PIR_PIN);

  if (motionDetected && !alarmActive) {
    alarmActive = true;
    iotbot.relayWrite(true);
    iotbot.lcdWriteMid(turkish ? "!!! ALARM !!!" : "!!! ALARM !!!",
                        turkish ? "Hareket algilandi" : "Motion detected",
                        turkish ? "Susturmak icin B3" : "Press B3 to silence",
                        "");
    iotbot.serialWrite(turkish ? "ALARM: hareket algilandi!" : "ALARM: motion detected!");
  }

  if (alarmActive) {
    if (iotbot.button3Read()) {
      // Alarmi sustur / silence the alarm
      alarmActive = false;
      iotbot.relayWrite(false);
      iotbot.lcdWriteMid(turkish ? "GUVENLIK ALARMI" : "SECURITY ALARM",
                          turkish ? "Susturuldu" : "Silenced",
                          turkish ? "Sistem aktif" : "System armed",
                          "");
      delay(500); // Buton birakilana kadar bekle / debounce
    } else if (millis() - lastBeepMs >= 300) {
      lastBeepMs = millis();
      iotbot.buzzerPlayTone(2000, 150);
    }
  }

  delay(50);
}
