// TR: BLUETOOTH'A ILK ADIM - en basit ornek. Telefonunuzdan bir Bluetooth
// terminal uygulamasiyla (ornegin "Serial Bluetooth Terminal") IOTBOT'a
// baglanin ve ne yazarsaniz yazin, IOTBOT aynisini size geri gonderir ve
// LCD'de gosterir - klasik bir "yankı" (echo) ornegi. Hicbir komut
// islenmez, sadece kablosuz metin alisverisinin nasil calistigini
// gosterir. Daha sonra IOTBOT_Bluetooth_TR_EN_Control_Example.ino ile
// gercek komutlarla LED/role kontrol etmeyi ogrenebilirsiniz.
// EN: FIRST STEP INTO BLUETOOTH - the simplest example. Connect to
// IOTBOT from a Bluetooth terminal app on your phone (e.g. "Serial
// Bluetooth Terminal") and whatever you type, IOTBOT sends it right back
// to you and shows it on the LCD - a classic "echo" example. No commands
// are processed, it just shows how wireless text exchange works. Later,
// see IOTBOT_Bluetooth_TR_EN_Control_Example.ino to control LEDs/relay
// with real commands.
//
// Kurulum / Setup: Telefonunuzda Bluetooth'u acin, "IOTBOT_ECHO" cihazina
// baglanin (PIN gerekmez) / Turn on Bluetooth on your phone, pair with
// the "IOTBOT_ECHO" device (no PIN required).

#define USE_BLUETOOTH
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.lcdWriteMid(turkish ? "BLUETOOTH YANKI" : "BLUETOOTH ECHO",
                      turkish ? "Cihaz: IOTBOT_ECHO" : "Device: IOTBOT_ECHO",
                      turkish ? "Baglanti bekleniyor" : "Waiting for connection",
                      "");
  iotbot.bluetoothStart("IOTBOT_ECHO");
  iotbot.serialWrite(turkish ? "Bluetooth baslatildi: IOTBOT_ECHO" : "Bluetooth started: IOTBOT_ECHO");
}

void loop() {
  String message = iotbot.bluetoothRead();
  if (message.length() > 0) {
    message.trim();
    iotbot.serialWrite(turkish ? "Alinan: " + message : "Received: " + message);
    // Aynisini geri gonder / Send it right back
    iotbot.bluetoothWrite((turkish ? "Yanki: " : "Echo: ") + message);
    iotbot.lcdWriteMid(turkish ? "ALINAN MESAJ" : "MESSAGE RECEIVED",
                        message.c_str(),
                        turkish ? "Geri gonderildi" : "Sent back",
                        "");
    iotbot.buzzerPlayTone(1200, 50);
  }
  iotbot.taskDelay(20);
}
