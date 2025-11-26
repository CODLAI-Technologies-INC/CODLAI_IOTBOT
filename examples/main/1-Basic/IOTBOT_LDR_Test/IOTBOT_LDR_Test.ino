#include <IOTBOT.h>

IOTBOT iotbot;

void setup()
{
    iotbot.begin();
    iotbot.serialStart(115200);

    iotbot.lcdClear();
    iotbot.lcdWriteMid("LDR Test", "--- IoTBot ---", "Baslatildi!", "Started!");
    iotbot.serialWrite("💡 LDR testi baslatildi / LDR test started.");

    delay(3000);
}

void loop()
{
    int ldrValue = iotbot.ldrRead();

    iotbot.serialWrite("💡 LDR Degeri / LDR Value: " + String(ldrValue));

    iotbot.lcdClear();
    iotbot.lcdWriteCR(0, 0, "LDR Test");
    iotbot.lcdWriteCR(0, 1, "LDR Degeri:");
    iotbot.lcdWriteCR(10, 1, String(ldrValue));

    delay(500);
}
