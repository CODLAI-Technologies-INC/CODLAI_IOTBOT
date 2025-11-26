#include <IOTBOT.h>

IOTBOT iotbot;

void setup()
{
    iotbot.begin();
    iotbot.serialStart(115200);
    iotbot.serialWrite("🔌 Role testi başlatildi / Relay test started.");
}

void loop()
{
    iotbot.serialWrite("🔌 Role açik / Relay ON.");
    iotbot.relayWrite(true);
    delay(2000);

    iotbot.serialWrite("🔌 Role kapali / Relay OFF.");
    iotbot.relayWrite(false);
    delay(2000);
}
