// TR: ESP-NOW'A ILK ADIM - en basit kablosuz haberlesme ornegi. Bir MAC
// adresi bilmenize gerek YOK: bu kod bir sayaci "yayin" (broadcast)
// olarak havaya gonderir, ve aninda ayni odada ESP-NOW ile dinleyen
// HERHANGI bir CODLAI karti (baska bir IOTBOT, bir MINIBOT ya da bir
// ROLEBOT - hepsi ayni veri yapisini kullanir) bunu duyabilir. Ayni anda
// hem gonderiyor hem dinliyoruz. Daha sonra IOTBOT_MiniBot_ESPNOW_Pair_
// Example.ino ile IKI KART ARASINDA OZEL (MAC adresine dayali) bir
// eslesme yapmayi ogrenebilirsiniz.
// EN: FIRST STEP INTO ESP-NOW - the simplest wireless example. You do
// NOT need to know any MAC address: this code broadcasts a counter into
// the air, and ANY nearby CODLAI board listening over ESP-NOW (another
// IOTBOT, a MINIBOT, or a ROLEBOT - they all share the same data
// structure) can hear it. We both send AND listen at the same time.
// Later, see IOTBOT_MiniBot_ESPNOW_Pair_Example.ino to learn how to pair
// TWO SPECIFIC boards together using their MAC addresses.

#define USE_ESPNOW
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

namespace {
  uint32_t counter = 0;
  uint32_t lastSendMs = 0;
  constexpr uint32_t kSendIntervalMs = 1000;
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.lcdWriteMid(turkish ? "ESP-NOW YAYIN" : "ESP-NOW BROADCAST",
                      turkish ? "Baslatiliyor..." : "Starting...",
                      "", "");

  iotbot.initESPNow();
  iotbot.startListening(); // Gelen HERHANGI bir yayini iotbot.receivedData'ya yazar.

  iotbot.serialWrite(turkish ? "Yayin modu hazir - herkese aciyoruz!"
                             : "Broadcast mode ready - open to everyone!");
}

void loop() {
  // ---- Gonderim: her saniye sayaci yayinla / Sending: broadcast counter every second ----
  if (millis() - lastSendMs >= kSendIntervalMs) {
    lastSendMs = millis();
    counter++;
    CodlaiESPNowMessage outgoing;
    outgoing.deviceType = 10; // 10 = IOTBOT (bu ornekte kullanilan kimlik / id used in this example)
    outgoing.axis1 = counter;
    outgoing.axis2 = 0;
    outgoing.axis3 = 0;
    outgoing.gripper = 0;
    outgoing.action = 0;
    iotbot.sendESPNow(broadcastAddress, (const uint8_t *)&outgoing, sizeof(outgoing));

    char line[21];
    snprintf(line, sizeof(line), turkish ? "Gonderilen: %lu" : "Sent: %lu", (unsigned long)counter);
    iotbot.lcdWriteMid(turkish ? "ESP-NOW YAYIN" : "ESP-NOW BROADCAST", line, "", "");
  }

  // ---- Alis: baska bir karttan gelen HERHANGI bir yayin / Receiving: ANY broadcast from another board ----
  if (iotbot.newData) {
    iotbot.newData = false;
    const char* senderName = "?";
    switch (iotbot.receivedData.deviceType) {
      case 10: senderName = "IOTBOT"; break;
      case 20: senderName = "MINIBOT"; break;
      case 30: senderName = "ROLEBOT"; break;
      default: break;
    }
    Serial.print(turkish ? "Yayin alindi -> gonderen: " : "Broadcast received -> from: ");
    Serial.print(senderName);
    Serial.print(turkish ? ", deger: " : ", value: ");
    Serial.println(iotbot.receivedData.axis1);
    iotbot.buzzerPlayTone(1000, 40);
  }
}
