// TR: GERCEK PROJE - Bitki Sulama Hatirlaticisi. Toprak nemi sensorunu
// saksinizin topragina yerlestirin. Toprak kuruduysa (nem degeri
// dusukse), LCD "SULAMA ZAMANI!" yazar ve her birkac saniyede bir kisa
// bir hatirlatma sesi calar - toprak nemlenene kadar devam eder.
// EN: A REAL PROJECT - Plant Watering Reminder. Place the soil moisture
// sensor's probe in your plant pot's soil. When the soil dries out (low
// moisture reading), the LCD shows "TIME TO WATER!" and it plays a short
// reminder tone every few seconds - it keeps going until the soil is
// moist again.
//
// Baglanti / Wiring: Toprak nemi sensorunu P1-P5 soketlerinden BIRINE
// takin ve asagidaki SOIL_PIN degerini o soketin sinyaline gore
// ayarlayin. / Plug the soil moisture sensor into ONE of the P1-P5
// sockets and set SOIL_PIN below to match that socket's signal.

#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define SOIL_PIN IO27 // Toprak nemi sensorunun bagli oldugu pin / Pin the soil moisture sensor is connected to
// Desteklenen pinler: IO25 - IO26 - IO27 - IO32 - IO33
// Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

namespace {
  int wetBaseline = -1; // Islak/normal toprak degeri (ilk baglantida olculur) / wet/normal baseline (measured at startup)
  // Bu degerden ("wetBaseline") ne kadar UZAKLASIRSA toprak o kadar
  // kurumus demektir - gercek toprakla test edip ayarlayin.
  // The FARTHER the reading drifts from this baseline, the drier the
  // soil is - tune this by testing with real soil.
  constexpr int kDryDifference = 300;
  uint32_t lastReminderMs = 0;
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);

  wetBaseline = iotbot.moduleSoilMoistureRead(SOIL_PIN);
  iotbot.lcdWriteMid(turkish ? "BITKI SULAMA" : "PLANT WATERING",
                      turkish ? "HATIRLATICISI" : "REMINDER",
                      turkish ? "Sensoru toprak" : "Insert the sensor",
                      turkish ? "icine yerlestirin" : "into the soil");
  iotbot.serialWrite(turkish ? "Bitki sulama hatirlaticisi hazir."
                             : "Plant watering reminder ready.");
  delay(2000);
}

void loop() {
  int value = iotbot.moduleSoilMoistureRead(SOIL_PIN);
  bool isDry = abs(value - wetBaseline) >= kDryDifference;

  char line[21];
  snprintf(line, sizeof(line), turkish ? "Deger: %d" : "Value: %d", value);

  if (isDry) {
    iotbot.lcdWriteMid(turkish ? "SULAMA ZAMANI!" : "TIME TO WATER!",
                        line,
                        turkish ? "Toprak kurumus" : "Soil is dry",
                        "");
    if (millis() - lastReminderMs >= 3000) {
      lastReminderMs = millis();
      iotbot.buzzerPlayTone(1000, 150);
      iotbot.buzzerPlayTone(1300, 150);
    }
  } else {
    iotbot.lcdWriteMid(turkish ? "BITKI SULAMA" : "PLANT WATERING",
                        line,
                        turkish ? "Toprak nemli, iyi!" : "Soil is moist, good!",
                        "");
  }

  delay(500);
}
