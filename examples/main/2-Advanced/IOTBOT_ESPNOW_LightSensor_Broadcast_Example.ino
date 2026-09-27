// TR: KABLOSUZ AKILLI EV FIKRI - IOTBOT'un uzerindeki isik sensoru (LDR)
// degerini surekli olarak ESP-NOW ile yayinlar (broadcast). Bu, tek basina
// bir sey yapmaz ama ayni odadaki BASKA kartlar bu veriyi dinleyip kendi
// kararlarini verebilir - ornegin "hava kararinca lambayi ac" gibi. Bkz.
// MINIBOT_ESPNOW_NightLight_Reactive_Example.ino ve ROLEBOT_ESPNOW_
// NightLight_Reactive_Example.ino - onlari calistirip bu ornekle
// eslestirin, IOTBOT'un uzerini elinizle kapatinca uzaktaki LED/lamba
// otomatik yanacak!
// EN: A WIRELESS SMART HOME IDEA - continuously broadcasts IOTBOT's light
// sensor (LDR) reading over ESP-NOW. By itself this does nothing, but
// OTHER boards in the room can listen to this data and make their own
// decisions - like "turn on the lamp when it gets dark". See
// MINIBOT_ESPNOW_NightLight_Reactive_Example.ino and ROLEBOT_ESPNOW_
// NightLight_Reactive_Example.ino - run one of them alongside this
// example, cover IOTBOT's light sensor with your hand, and watch the
// remote LED/lamp turn on automatically!

#define USE_ESPNOW
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

namespace {
  uint32_t lastSendMs = 0;
  constexpr uint32_t kSendIntervalMs = 500;
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.initESPNow();
  iotbot.lcdWriteMid(turkish ? "ISIK SENSORU YAYINI" : "LIGHT SENSOR BROADCAST",
                      turkish ? "LDR degeri yayinlaniyor" : "Broadcasting LDR value",
                      turkish ? "(uzaktaki kartlar" : "(remote boards can",
                      turkish ? "dinleyebilir)" : "listen to it)");
  iotbot.serialWrite(turkish ? "Isik sensoru yayini basladi." : "Light sensor broadcast started.");
}

void loop() {
  if (millis() - lastSendMs >= kSendIntervalMs) {
    lastSendMs = millis();
    int lightValue = iotbot.ldrRead();

    CodlaiESPNowMessage outgoing;
    outgoing.deviceType = 10; // 10 = IOTBOT (bu ornekte kullanilan kimlik / id used in this example)
    outgoing.axis1 = lightValue; // Isik degeri / light value
    outgoing.axis2 = 0;
    outgoing.axis3 = 0;
    outgoing.gripper = 0;
    outgoing.action = 0;
    iotbot.sendESPNow(broadcastAddress, (const uint8_t *)&outgoing, sizeof(outgoing));

    char line[21];
    snprintf(line, sizeof(line), turkish ? "Isik: %d" : "Light: %d", lightValue);
    iotbot.lcdWriteMid(turkish ? "ISIK SENSORU YAYINI" : "LIGHT SENSOR BROADCAST",
                        line,
                        turkish ? "(uzaktaki kartlar" : "(remote boards can",
                        turkish ? "dinleyebilir)" : "listen to it)");
  }
}
