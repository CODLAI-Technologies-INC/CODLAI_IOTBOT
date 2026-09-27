// TR: KABLOSUZ AKILLI EV FIKRI - IOTBOT'a takili DHT sicaklik sensorunun
// degerini surekli olarak ESP-NOW ile yayinlar (broadcast). Bu, tek
// basina bir sey yapmaz ama ayni odadaki BASKA kartlar bu veriyi
// dinleyip kendi kararlarini verebilir - ornegin "sicaklik yukselince
// vantilatoru ac" gibi. Bkz. MINIBOT_ESPNOW_Fan_Control_Reactive_
// Example.ino ve ROLEBOT_ESPNOW_Fan_Control_Reactive_Example.ino -
// onlari calistirip bu ornekle eslestirin, DHT sensorunu elinizle
// isitinca uzaktaki role/vantilator otomatik calisacak!
// EN: A WIRELESS SMART HOME IDEA - continuously broadcasts the reading of
// IOTBOT's DHT temperature sensor over ESP-NOW. By itself this does
// nothing, but OTHER boards in the room can listen to this data and make
// their own decisions - like "turn on the fan when it gets hot". See
// MINIBOT_ESPNOW_Fan_Control_Reactive_Example.ino and ROLEBOT_ESPNOW_
// Fan_Control_Reactive_Example.ino - run one of them alongside this
// example, warm up the DHT sensor with your hand, and watch the remote
// relay/fan turn on automatically!
//
// Baglanti / Wiring: DHT sensorunu P1-P5 soketlerinden BIRINE takin ve
// asagidaki DHT_PIN degerini o soketin sinyaline gore ayarlayin. / Plug
// the DHT sensor into ONE of the P1-P5 sockets and set DHT_PIN below to
// match that socket's signal.

#define USE_ESPNOW
#define USE_DHT
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define DHT_PIN IO27 // DHT sensorunun bagli oldugu pin / Pin the DHT sensor is connected to
// Desteklenen pinler: IO25 - IO26 - IO27 - IO32 - IO33
// Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

namespace {
  uint32_t lastSendMs = 0;
  constexpr uint32_t kSendIntervalMs = 1000;
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.initESPNow();
  iotbot.lcdWriteMid(turkish ? "SICAKLIK YAYINI" : "TEMPERATURE BROADCAST",
                      turkish ? "DHT degeri yayinlaniyor" : "Broadcasting DHT value",
                      turkish ? "(uzaktaki kartlar" : "(remote boards can",
                      turkish ? "dinleyebilir)" : "listen to it)");
  iotbot.serialWrite(turkish ? "Sicaklik yayini basladi." : "Temperature broadcast started.");
}

void loop() {
  if (millis() - lastSendMs >= kSendIntervalMs) {
    lastSendMs = millis();
    int tempC = iotbot.moduleDhtTempReadC(DHT_PIN);

    CodlaiESPNowMessage outgoing;
    outgoing.deviceType = 11; // 11 = IOTBOT sicaklik yayini (bu ornekte kullanilan kimlik) / IOTBOT temperature broadcast (id used in this example)
    outgoing.axis1 = tempC; // Sicaklik (C) / temperature (C)
    outgoing.axis2 = 0;
    outgoing.axis3 = 0;
    outgoing.gripper = 0;
    outgoing.action = 0;
    iotbot.sendESPNow(broadcastAddress, (const uint8_t *)&outgoing, sizeof(outgoing));

    char line[21];
    snprintf(line, sizeof(line), turkish ? "Sicaklik: %d C" : "Temperature: %d C", tempC);
    iotbot.lcdWriteMid(turkish ? "SICAKLIK YAYINI" : "TEMPERATURE BROADCAST",
                        line,
                        turkish ? "(uzaktaki kartlar" : "(remote boards can",
                        turkish ? "dinleyebilir)" : "listen to it)");
  }
}
