// TR: GERCEK PROJE - Duman/Gaz Alarmi. Duman sensoru havadaki gaz
// yogunlugunu surekli olcer. Deger, dinlenme (temiz hava) degerinin
// belirgin sekilde ustune ciktiginda: buzzer alarm calar, LCD "DUMAN/GAZ
// ALGILANDI!" yazar ve kart uzerindeki role tetiklenir (ornegin bir
// egzoz fani ya da uyari lambasi baglayabilirsiniz).
// EN: A REAL PROJECT - Smoke/Gas Alarm. The smoke sensor continuously
// measures the gas concentration in the air. When the value rises
// noticeably above the resting (clean air) baseline: the buzzer sounds
// an alarm, the LCD shows "SMOKE/GAS DETECTED!" and the board's relay is
// triggered (you can wire an exhaust fan or a warning light to it).
//
// GUVENLIK NOTU / SAFETY NOTE: Bu bir OYUNCAK/EGITIM projesidir, gercek
// bir yangin alarmi YERINE KULLANILMAMALIDIR. / This is a TOY/EDUCATIONAL
// project and must NOT be used as a substitute for a real fire alarm.
//
// Baglanti / Wiring: Duman sensorunu P1-P5 soketlerinden BIRINE takin ve
// asagidaki SMOKE_PIN degerini o soketin sinyaline gore ayarlayin. /
// Plug the smoke sensor into ONE of the P1-P5 sockets and set SMOKE_PIN
// below to match that socket's signal.

#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define SMOKE_PIN IO27 // Duman sensorunun bagli oldugu pin / Pin the smoke sensor is connected to
// Desteklenen pinler: IO25 - IO26 - IO27 - IO32 - IO33
// Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

namespace {
  int cleanAirBaseline = -1;
  // Bu degerin UZERINE cikildiginda alarm calar - gercek donanimla test
  // edip ayarlayin. / Alarm triggers ABOVE this margin - tune by testing
  // with real hardware.
  constexpr int kAlarmMargin = 400;
  bool alarmActive = false;
  uint32_t lastBeepMs = 0;
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.relayWrite(false);

  cleanAirBaseline = iotbot.moduleSmokeRead(SMOKE_PIN);
  iotbot.lcdWriteMid(turkish ? "DUMAN/GAZ ALARMI" : "SMOKE/GAS ALARM",
                      turkish ? "Sistem aktif" : "System armed",
                      ("Baz: " + String(cleanAirBaseline)).c_str(),
                      "");
  iotbot.serialWrite(turkish ? "Duman/gaz alarmi hazir." : "Smoke/gas alarm ready.");
}

void loop() {
  int value = iotbot.moduleSmokeRead(SMOKE_PIN);
  bool dangerDetected = (value - cleanAirBaseline) >= kAlarmMargin;

  if (dangerDetected && !alarmActive) {
    alarmActive = true;
    iotbot.relayWrite(true);
    iotbot.serialWrite(turkish ? "ALARM: duman/gaz algilandi!" : "ALARM: smoke/gas detected!");
  } else if (!dangerDetected && alarmActive) {
    alarmActive = false;
    iotbot.relayWrite(false);
  }

  char line[21];
  snprintf(line, sizeof(line), turkish ? "Deger: %d" : "Value: %d", value);

  if (alarmActive) {
    iotbot.lcdWriteMid(turkish ? "!! DUMAN/GAZ !!" : "!! SMOKE/GAS !!",
                        turkish ? "ALGILANDI!" : "DETECTED!",
                        line,
                        "");
    if (millis() - lastBeepMs >= 200) {
      lastBeepMs = millis();
      iotbot.buzzerPlayTone(2200, 100);
    }
  } else {
    iotbot.lcdWriteMid(turkish ? "DUMAN/GAZ ALARMI" : "SMOKE/GAS ALARM",
                        turkish ? "Hava temiz" : "Air is clean",
                        line,
                        "");
  }

  delay(150);
}
