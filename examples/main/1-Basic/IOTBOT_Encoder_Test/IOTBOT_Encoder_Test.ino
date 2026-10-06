#include <IOTBOT.h>

IOTBOT iotbot;

void setup()
{
    iotbot.begin();
    iotbot.serialStart(115200);

    iotbot.lcdClear();
    iotbot.lcdWriteMid("Encoder Test", "--- IoTBot ---", "Baslatildi!", "Started!");
    iotbot.serialWrite("🔄 Encoder testi baslatildi / Encoder test started.");

    delay(3000);
}

void loop()
{
    static unsigned long previousMillis = 0;
    const long interval = 500;

    unsigned long currentMillis = millis();
    int encoderValue = iotbot.encoderRead();
    int buttonStatus = iotbot.encoderButtonRead();

    if (currentMillis - previousMillis >= interval)
    {
        previousMillis = currentMillis;

        iotbot.serialWrite("🔄 Encoder: " + String(encoderValue) + " | Buton: " + String(buttonStatus));

        iotbot.lcdClear();
        iotbot.lcdWriteCR(0, 0, "Encoder Test");
        iotbot.lcdWriteCR(0, 1, "Deger / Value:");
        iotbot.lcdWriteCR(15, 1, String(encoderValue));
        iotbot.lcdWriteCR(0, 2, "Buton:");
        // encoderButtonRead() ham pin seviyesini verir: pin pull-up oldugu icin
        // BASILIYKEN false (LOW) doner. / encoderButtonRead() returns the raw pin
        // level: the pin is pulled up, so it is false (LOW) while PRESSED.
        iotbot.lcdWriteCR(10, 2, !buttonStatus ? "BASILI " : "SERBEST");
    }
}
