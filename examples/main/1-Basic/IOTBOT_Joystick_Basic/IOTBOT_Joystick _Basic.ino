#include <IOTBOT.h>

IOTBOT iotbot;

void setup()
{
    iotbot.begin();
    iotbot.serialStart(115200);

    iotbot.lcdClear();
    iotbot.lcdWriteMid("Joystick Test", "--- IoTBot ---", "Baslatildi!", "Started!");
    iotbot.serialWrite("🕹 Joystick testi baslatildi / Joystick test started.");

    delay(3000);
}

void loop()
{
    int xValue = iotbot.joystickXRead();
    int yValue = iotbot.joystickYRead();
    int button = iotbot.joystickButtonRead();

    iotbot.serialWrite("🕹 X: " + String(xValue) + " | Y: " + String(yValue) + " | Buton: " + String(button));

    iotbot.lcdClear();
    iotbot.lcdWriteCR(0, 0, "Joystick Test");
    iotbot.lcdWriteCR(0, 1, "X:");
    iotbot.lcdWriteCR(3, 1, String(xValue));
    iotbot.lcdWriteCR(10, 1, "Y:");
    iotbot.lcdWriteCR(13, 1, String(yValue));
    iotbot.lcdWriteCR(0, 2, "Buton:");
    iotbot.lcdWriteCR(7, 2, button ? "BASILI" : "SERBEST");

    delay(500);
}
