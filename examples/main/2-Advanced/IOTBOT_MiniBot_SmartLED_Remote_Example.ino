// TR: EGLENCELI KABLOSUZ ORNEK - Bir MINIBOT'un butonuyla, uzaktaki bir
// IOTBOT'un akilli LED serisinin (NeoPixel) rengini/efektini
// degistiriyoruz. Iki kart arasinda hicbir kablo yok - sadece ESP-NOW.
// Once bu kodu bir IOTBOT'a, sonra IOTBOT_MiniBot_SmartLED_Remote_
// Kumanda_Example.ino'yu (asagida ayni dosyada anlatilir, MINIBOT
// tarafi ayri dosyadadir) bir MINIBOT'a yukleyin.
// EN: A FUN WIRELESS EXAMPLE - use a MINIBOT's button to remotely change
// the color/effect of an IOTBOT's smart LED strip (NeoPixel). No wire
// between the two boards - just ESP-NOW. Upload this to an IOTBOT, and
// upload the MINIBOT-side sketch (see MINIBOT_IoTBot_SmartLED_Remote_
// Example.ino) to a MINIBOT.

#define USE_ESPNOW
#define USE_NEOPIXEL
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

namespace {
  const char* namesTr[] = {"Gokkusagi", "Gokkusagi Gecit", "Gecit", "Renk Dolgusu"};
  const char* namesEn[] = {"Rainbow", "Rainbow Chase", "Chase", "Color Wipe"};

  void playEffect(uint8_t choice) {
    char line[21];
    snprintf(line, sizeof(line), "%s%s", turkish ? "Oynuyor: " : "Playing: ",
             turkish ? namesTr[choice % 4] : namesEn[choice % 4]);
    iotbot.lcdWriteMid(turkish ? "UZAKTAN KUMANDALI LED" : "REMOTE-CONTROLLED LED",
                        turkish ? "MiniBot'tan komut geldi!" : "Command from MiniBot!",
                        line, "");
    uint32_t colors[] = {0xFF0000, 0x00FF00, 0x0000FF, 0xFFAA00};
    uint32_t color = colors[choice % 4];
    switch (choice % 4) {
      case 0: iotbot.moduleSmartLEDRainbowEffect(4); break;
      case 1: iotbot.moduleSmartLEDRainbowTheaterChaseEffect(10); break;
      case 2: iotbot.moduleSmartLEDTheaterChaseEffect(color, 20); break;
      default: iotbot.moduleSmartLEDColorWipeEffect(color, 20); break;
    }
  }
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.moduleSmartLEDPrepare(IO27); // Akilli LED serisi P1-P5'ten IO27 sinyaline takili olmali.

  iotbot.initESPNow();
  iotbot.startListening();

  iotbot.lcdWriteMid(turkish ? "UZAKTAN KUMANDALI LED" : "REMOTE-CONTROLLED LED",
                      turkish ? "MiniBot'un butonuna" : "Press the MiniBot's",
                      turkish ? "basip bekleyin..." : "button and wait...",
                      "");
  iotbot.serialWrite(turkish ? "Hazir - MiniBot'tan komut bekleniyor." : "Ready - waiting for a command from MiniBot.");
}

void loop() {
  if (iotbot.newData) {
    iotbot.newData = false;
    // action alani MiniBot'un butonuna kac kez basildigini tasir; her
    // basista bir SONRAKI efekte geciyoruz. / The action field carries
    // how many times the MiniBot's button was pressed; each press moves
    // to the NEXT effect.
    if (iotbot.receivedData.deviceType == 20) { // 20 = MINIBOT
      playEffect((uint8_t)iotbot.receivedData.action);
    }
  }
}
