// TR: Telefonunuzdaki bir Bluetooth terminal uygulamasindan (ornegin
// "Serial Bluetooth Terminal") tek harfli komutlar gonderip IOTBOT'un
// LED'lerini ve roleyi kontrol ettigimiz, sensor degerini de geri
// okuyabildigimiz basit bir uzaktan kumanda ornegi.
// EN: A simple remote-control example: send single-letter commands from a
// Bluetooth terminal app on your phone (e.g. "Serial Bluetooth Terminal")
// to control IOTBOT's LEDs and relay, and read a sensor value back.
//
// Komutlar / Commands (harf + Enter / letter + Enter):
//   1 -> LED'leri ac / turn LEDs on
//   0 -> LED'leri kapat / turn LEDs off
//   R -> Roleyi ac / turn relay on
//   r -> Roleyi kapat / turn relay off
//   ? -> Isik (LDR) degerini oku / read the light (LDR) value
//
// Kurulum / Setup: Telefonunuzda Bluetooth'u acin, "IOTBOT_BT" cihazina
// baglanin (PIN: 1234) / Turn on Bluetooth on your phone, pair with the
// "IOTBOT_BT" device (PIN: 1234).

#define USE_BLUETOOTH
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

namespace {
  constexpr uint8_t kLedPins[] = {IO32, IO33, IO25, IO26, IO27};
  constexpr uint8_t kLedCount = 5;

  void allLeds(bool state) {
    for (uint8_t pin : kLedPins) {
      iotbot.digitalWritePin(pin, state);
    }
  }

  void handleCommand(char command) {
    switch (command) {
      case '1':
        allLeds(true);
        iotbot.bluetoothWrite(turkish ? "LED'ler acildi" : "LEDs turned on");
        break;
      case '0':
        allLeds(false);
        iotbot.bluetoothWrite(turkish ? "LED'ler kapatildi" : "LEDs turned off");
        break;
      case 'R':
        iotbot.relayWrite(true);
        iotbot.bluetoothWrite(turkish ? "Role acildi" : "Relay turned on");
        break;
      case 'r':
        iotbot.relayWrite(false);
        iotbot.bluetoothWrite(turkish ? "Role kapatildi" : "Relay turned off");
        break;
      case '?': {
        String reply = (turkish ? "Isik degeri: " : "Light value: ") + String(iotbot.ldrRead());
        iotbot.bluetoothWrite(reply);
        break;
      }
      default:
        iotbot.bluetoothWrite(turkish ? "Bilinmeyen komut. 1/0/R/r/?" : "Unknown command. 1/0/R/r/?");
        break;
    }
  }
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  allLeds(false);
  iotbot.relayWrite(false);

  iotbot.lcdWriteMid(turkish ? "BLUETOOTH KONTROL" : "BLUETOOTH CONTROL",
                      turkish ? "Cihaz: IOTBOT_BT" : "Device: IOTBOT_BT",
                      turkish ? "PIN: 1234" : "PIN: 1234",
                      turkish ? "Baglanti bekleniyor" : "Waiting for connection");
  iotbot.bluetoothStart("IOTBOT_BT", "1234");
  iotbot.serialWrite(turkish ? "Bluetooth baslatildi: IOTBOT_BT (PIN 1234)"
                             : "Bluetooth started: IOTBOT_BT (PIN 1234)");
}

void loop() {
  String incoming = iotbot.bluetoothRead();
  if (incoming.length() > 0) {
    incoming.trim();
    if (incoming.length() > 0) {
      char command = incoming.charAt(0);
      handleCommand(command);
      iotbot.lcdWriteMid(turkish ? "BLUETOOTH KONTROL" : "BLUETOOTH CONTROL",
                          turkish ? "Alinan komut:" : "Command received:",
                          String(command).c_str(),
                          "");
    }
  }
  iotbot.taskDelay(20);
}
