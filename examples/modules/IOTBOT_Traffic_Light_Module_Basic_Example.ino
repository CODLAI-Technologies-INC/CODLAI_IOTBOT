#include <IOTBOT.h> // IoTBot kutuphanesi / IoTBot library

// IoTBot nesnesi olusturuluyor / Create an IoTBot object
IOTBOT iotbot;

void setup()
{
    iotbot.begin(); // IoTBot baslatiliyor / Initialize IoTBot

    iotbot.serialStart(115200); // Seri haberlesmeyi baslat / Start serial communication
    // Bilgisayar ile seri haberlesme icin 115200 baud hizinda baslatilir.
    // Starts serial communication at 115200 baud for computer connection.

    iotbot.lcdClear(); // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("Trafik Isigi", "--- IoTBot ---", "Test Basladi", "Test Started");
    // LCD'ye test basladigini yazdir / Display test start message on LCD

    iotbot.serialWrite("Trafik Isigi Testi Basladi / Traffic Light Test Started.");
    // Test basladigini seri porta yazdir / Print test start message to serial port

    delay(3000); // Baslangic icin bekleme suresi / Initial delay
}

void loop()
{
    // 1️⃣ **Kirmizi Isik / Red Light**
    iotbot.moduleTraficLightWrite(true, false, false);
    // Kirmizi isigi yak, digerlerini sondur / Turn on red light, turn off others

    iotbot.lcdClear();
    // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("Kirmizi Isik", "Red Light", "", "");
    // LCD'ye kirmizi isik bilgisini yazdir / Display "Red Light" status on LCD

    iotbot.serialWrite("🚦 Kirmizi Isik Yandi / Red Light ON");
    // Seri porta kirmizi isigin yandigini yazdir / Print "Red Light ON" message to serial port

    delay(3000); // 3 saniye bekle / Wait for 3 seconds

    // 2️⃣ **Sari Isik / Yellow Light**
    iotbot.moduleTraficLightWrite(false, true, false);
    // Sari isigi yak, digerlerini sondur / Turn on yellow light, turn off others

    iotbot.lcdClear();
    // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("Sari Isik", "Yellow Light", "", "");
    // LCD'ye sari isik bilgisini yazdir / Display "Yellow Light" status on LCD

    iotbot.serialWrite("🚦 Sari Isik Yandi / Yellow Light ON");
    // Seri porta sari isigin yandigini yazdir / Print "Yellow Light ON" message to serial port

    delay(3000); // 3 saniye bekle / Wait for 3 seconds

    // 3️⃣ **Yesil Isik / Green Light**
    iotbot.moduleTraficLightWrite(false, false, true);
    // Yesil isigi yak, digerlerini sondur / Turn on green light, turn off others

    iotbot.lcdClear();
    // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("Yesil Isik", "Green Light", "", "");
    // LCD'ye yesil isik bilgisini yazdir / Display "Green Light" status on LCD

    iotbot.serialWrite("🚦 Yesil Isik Yandi / Green Light ON");
    // Seri porta yesil isigin yandigini yazdir / Print "Green Light ON" message to serial port

    delay(3000); // 3 saniye bekle / Wait for 3 seconds
}
