// TR: COCUKLAR ICIN BASIT KABLOSUZ MESAJLASMA. Bu ornek her 2 saniyede
// bir artan bir sayi ve kisa bir metin mesaji yayinlar (broadcast); ayni
// anda baska bir CODLAI karti (IOTBOT/MINIBOT/ROLEBOT) gonderdiginiz bir
// mesaji alirsa onu da ekrana/seri porta yazdirir. Iki kart arasinda
// (router/WiFi agi OLMADAN) "sohbet" etmenin en basit yolu budur.
// EN: SIMPLE WIRELESS MESSAGING FOR KIDS. This example broadcasts an
// increasing number and a short text message every 2 seconds; if
// another CODLAI board (IOTBOT/MINIBOT/ROLEBOT) sends a message at the
// same time, it prints it too. This is the simplest way for two boards
// to "chat" with each other (with NO router/WiFi network needed).

#define USE_ESPNOW
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

namespace {
  uint32_t lastSendMs = 0;
  int counter = 0;
}

void setup() {
  iotbot.serialStart(115200);
  iotbot.espNowBegin(1); // Kanal 1 - dinleyen diger kartla AYNI kanal olmali / channel 1 - must match the listening board's channel
  iotbot.lcdWriteMid(turkish ? "BASIT MESAJLASMA" : "SIMPLE MESSAGING",
                      turkish ? "Hazir!" : "Ready!",
                      "", "");
  iotbot.serialWrite(turkish ? "Basit ESP-NOW mesajlasma hazir." : "Simple ESP-NOW messaging ready.");
}

void loop() {
  // Her 2 saniyede bir mesaj gonder / send a message every 2 seconds
  if (millis() - lastSendMs >= 2000) {
    lastSendMs = millis();
    counter++;
    iotbot.espNowSendText(turkish ? "Merhaba!" : "Hello!");
    iotbot.espNowSendNumber("sayac", counter);
    iotbot.serialWrite(turkish ? ("Gonderildi -> sayac: " + String(counter)) : ("Sent -> counter: " + String(counter)));
  }

  // Gelen bir mesaj var mi kontrol et / check for an incoming message
  if (iotbot.espNowAvailable()) {
    String text = iotbot.espNowReadText();
    if (text.length() > 0) {
      iotbot.serialWrite(turkish ? ("Metin alindi: " + text) : ("Text received: " + text));
      iotbot.lcdWriteMid(turkish ? "MESAJ ALINDI" : "MESSAGE RECEIVED", text.c_str(), "", "");
    }
  }

  delay(20);
}
