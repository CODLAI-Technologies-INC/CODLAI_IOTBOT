#define USE_HCSR04
#include <IOTBOT.h> // IoTBot kutuphanesi / IoTBot library

// IoTBot nesnesi olusturuluyor / Create an IoTBot object
IOTBOT iotbot;

void setup()
{
    iotbot.begin(); // IoTBot baslatiliyor / Initialize IoTBot

    iotbot.serialStart(115200); // Seri haberlesmeyi baslat / Start serial communication
    // Bilgisayar ile seri haberlesme icin 115200 baud hizinda baslatilir.
    // Starts serial communication at 115200 baud for computer connection.

    iotbot.serialWrite("Ultrasonik sensor testi baslatildi / Ultrasonic sensor test started.");
    // Test basladigini seri porta yazdir / Print test start message to serial port

    iotbot.lcdClear(); // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("Ultrasonik Sensor", "--- IoTBot ---", "Test Basladi", "Test Started");
    // LCD'ye test basladigini yazdir / Display test start message on LCD

    delay(3000); // Baslangic icin bekleme suresi / Initial delay
}

void loop()
{
    int distance = iotbot.moduleUltrasonicDistanceRead();
    // Ultrasonik sensorden mesafeyi oku / Measure distance using the ultrasonic sensor

    iotbot.serialWrite("Mesafe / Distance: ");
    iotbot.serialWrite(distance);
    // Seri porta mesafe bilgisini yazdir / Print distance value to serial port

    iotbot.lcdClear();
    // LCD ekrani temizle / Clear the LCD screen

    if (distance < 10) // Tehlikeli mesafe kontrolu / Check for dangerous distance
    {
        iotbot.lcdWriteMid("⚠️ Cok Yakin!", "⚠️ Too Close!", "Mesafe: ", String(distance).c_str());
        // Tehlikeli derecede yakin mesafe icin uyari goster / Display warning if too close

        iotbot.serialWrite("⚠️ Cok Yakin! / ⚠️ Too Close!");
        // Seri porta uyari mesaji yazdir / Print warning message to serial port
    }
    else
    {
        iotbot.lcdWriteMid("Mesafe:", "Distance:", String(distance).c_str(), " cm");
        // Mesafeyi LCD'ye yazdir / Display distance value on LCD

        iotbot.serialWrite("Guvenli Mesafe / Safe Distance");
        // Seri porta guvenli mesafe mesaji yazdir / Print safe distance message to serial port
    }

    delay(1000);
    // Yeni okuma yapmadan once 1 saniye bekle / Wait for 1 second before reading again
}
