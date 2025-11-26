#include <IOTBOT.h> // IoTBot kutuphanesi / IoTBot library

// IoTBot nesnesi olusturuluyor / Create an IoTBot object
IOTBOT iotbot;

#define SENSOR_PIN IO27 // Rolenin bagli oldugu pini secin / Select the pin connected to the relay
// Desteklenen pinler: IO25 - IO26 - IO27 - IO32 - IO33
// Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

void setup()
{
    iotbot.begin(); // IoTBot baslatiliyor / Initialize IoTBot

    iotbot.serialStart(115200); // Seri haberlesmeyi baslat / Start serial communication
    // Bilgisayar ile seri haberlesme icin 115200 baud hizinda baslatilir.
    // Starts serial communication at 115200 baud for computer connection.

    iotbot.serialWrite("Role Testi Baslatildi / Relay Test Started.");
    // Role testinin basladigini seri porta yazdir / Print relay test start message to the serial port

    iotbot.lcdClear(); // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("Role Modulu", "--- IoTBot ---", "Test Basladi", "Test Started");
    // LCD'ye test basladigini yazdir / Display test start message on LCD

    delay(3000); // Baslangic icin bekleme suresi / Initial delay
}

void loop()
{
    // 1️⃣ **Roleyi Ac / Activate the Relay**
    iotbot.moduleRelayWrite(SENSOR_PIN, true);
    // Roleyi aktif hale getir / Turn on the relay

    iotbot.lcdClear();
    // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("Role Durumu:", "Relay Status:", "Acik / ON", "");
    // LCD'ye rolenin acik oldugunu yazdir / Display "Relay ON" status on LCD

    iotbot.serialWrite("🔌 Role Acildi! / 🔌 Relay Activated!");
    // Seri porta rolenin acik oldugunu yazdir / Print "Relay ON" status to serial port

    delay(3000); // 3 saniye bekle / Wait for 3 seconds

    // 2️⃣ **Roleyi Kapat / Deactivate the Relay**
    iotbot.moduleRelayWrite(SENSOR_PIN, false);
    // Roleyi pasif hale getir / Turn off the relay

    iotbot.lcdClear();
    // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("Role Durumu:", "Relay Status:", "Kapali / OFF", "");
    // LCD'ye rolenin kapali oldugunu yazdir / Display "Relay OFF" status on LCD

    iotbot.serialWrite("⚡ Role Kapandi! / ⚡ Relay Deactivated!");
    // Seri porta rolenin kapali oldugunu yazdir / Print "Relay OFF" status to serial port

    delay(3000); // 3 saniye bekle / Wait for 3 seconds
}
