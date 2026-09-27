// TR: IOTBOT'u, ayni odadaki bir MINIBOT ile ROUTER/WIFI AGI OLMADAN
// (ESP-NOW ile) dogrudan haberlestirir. IOTBOT potansiyometre degerini ve
// B3 butonunun durumunu MINIBOT'a gonderir; MINIBOT'un butonuna basilip
// basilmadigini ve gonderdigi sayaci geri alip LCD'de gosterir.
// EN: Talks directly (peer-to-peer, no router/WiFi network needed) with a
// MINIBOT in the same room over ESP-NOW. Sends IOTBOT's potentiometer
// value and B3 button state to the MINIBOT; receives whether the MINIBOT's
// button is pressed and its counter, and shows them on the LCD.
//
// Eslenecek MINIBOT'a bu klasordeki MINIBOT_IoTBot_ESPNOW_Pair_Example.ino
// dosyasini yukleyin / Upload MINIBOT_IoTBot_ESPNOW_Pair_Example.ino (in
// the CODLAI_MINIBOT library's examples) to the MINIBOT you want to pair with.
//
// ONEMLI / IMPORTANT: Asagidaki kPeerMac dizisini, GERCEK MINIBOT'unuzun
// MAC adresiyle degistirin (MINIBOT tarafindaki kod Seri Port'a kendi
// MAC'ini yazdirir). / Replace kPeerMac below with your actual MINIBOT's
// MAC address (the MINIBOT-side sketch prints its own MAC to Serial).

#define USE_ESPNOW
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

// Test icin kullanilan gercek bir MINIBOT'un MAC adresi - kendi kartiniza
// gore degistirin. / A real MINIBOT's MAC address used for testing -
// change this to match your own board.
uint8_t kPeerMac[] = {0x8C, 0x4F, 0x00, 0x5C, 0x84, 0x9E};

namespace {
  uint32_t lastSendMs = 0;
  constexpr uint32_t kSendIntervalMs = 500;
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.lcdShowLoading(turkish ? "ESP-NOW baslatiliyor" : "Starting ESP-NOW");

  iotbot.initESPNow();
  iotbot.setWiFiChannel(1); // Iki taraf da AYNI kanalda olmali / Both sides must use the SAME channel.
  iotbot.startListening();  // Gelen mesajlari iotbot.receivedData'ya yazar / Fills iotbot.receivedData on arrival.

  String myMac = WiFi.macAddress();
  iotbot.serialWrite(turkish ? "Benim MAC adresim: " + myMac : "My MAC address: " + myMac);
  iotbot.lcdWriteMid(turkish ? "ESP-NOW HAZIR" : "ESP-NOW READY",
                      turkish ? "MiniBot ile" : "Paired with",
                      turkish ? "eslesme aktif" : "MiniBot",
                      turkish ? "Veri gonderiliyor..." : "Sending data...");
}

void loop() {
  // ---- Gonderim: potansiyometre + B3 durumu / Sending: pot + B3 state ----
  if (millis() - lastSendMs >= kSendIntervalMs) {
    lastSendMs = millis();
    CodlaiESPNowMessage outgoing;
    outgoing.deviceType = 10; // 10 = IOTBOT (bu ornekte kullanilan kimlik / id used in this example)
    outgoing.axis1 = iotbot.potentiometerRead();
    outgoing.axis2 = 0;
    outgoing.axis3 = 0;
    outgoing.gripper = 0;
    outgoing.action = iotbot.button3Read() ? 1 : 0;
    iotbot.sendESPNow(kPeerMac, (const uint8_t *)&outgoing, sizeof(outgoing));
  }

  // ---- Alis: MINIBOT'tan gelen veri / Receiving: data from the MINIBOT ----
  if (iotbot.newData) {
    iotbot.newData = false;
    bool minibotButtonPressed = iotbot.receivedData.action == 1;
    int minibotCounter = iotbot.receivedData.axis1;

    char line2[21];
    snprintf(line2, sizeof(line2), turkish ? "Sayac: %d" : "Counter: %d", minibotCounter);

    Serial.print(turkish ? "MiniBot'tan alindi -> sayac: " : "Received from MiniBot -> counter: ");
    Serial.print(minibotCounter);
    Serial.print(turkish ? ", buton: " : ", button: ");
    Serial.println(minibotButtonPressed ? (turkish ? "BASILI" : "PRESSED") : (turkish ? "serbest" : "released"));

    iotbot.lcdWriteMid(turkish ? "MINIBOT'TAN VERI" : "DATA FROM MINIBOT",
                        line2,
                        minibotButtonPressed ? (turkish ? "Buton: BASILI" : "Button: PRESSED")
                                             : (turkish ? "Buton: serbest" : "Button: released"),
                        turkish ? "Gonderiliyor..." : "Sending...");

    // MiniBot'un butonuna basildiginda kisa bir bip / short beep when the
    // MiniBot's button is pressed.
    if (minibotButtonPressed) {
      iotbot.buzzerPlayTone(1200, 60);
    }
  }
}
