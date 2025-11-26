#include <IOTBOT.h> // IoTBot kutuphanesi / IoTBot library

// IoTBot nesnesi olusturuluyor / Create an IoTBot object
IOTBOT iotbot;

#define SENSOR_PIN IO27 // Manyetik sensorun bagli oldugu pini secin / Select the pin connected to the magnetic sensor
// Desteklenen pinler: IO25 - IO26 - IO27 - IO32 - IO33
// Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

void setup()
{
    iotbot.begin(); // IoTBot baslatiliyor / Initialize IoTBot

    iotbot.serialStart(115200); // Seri haberlesmeyi baslat / Start serial communication
    // Bilgisayar ile seri haberlesme icin 115200 baud hizinda baslatilir.
    // Starts serial communication at 115200 baud for computer connection.

    iotbot.serialWrite("Manyetik sensor testi baslatildi / Magnetic sensor test started.");
    // Manyetik sensor testinin basladigini seri porta yazdir / Display test start message on the serial port

    iotbot.lcdClear(); // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("Manyetik Sensor", "--- IoTBot ---", "Test Basladi", "Test Started");
    // LCD'ye test basladigini yazdir / Display test start message on LCD

    delay(3000); // Baslangic icin bekleme suresi / Initial delay
}

void loop()
{
    int magneticStatus = iotbot.moduleMagneticRead(SENSOR_PIN);
    // Manyetik sensorun durumunu oku / Read magnetic sensor status

    iotbot.serialWrite("Manyetik Sensor Durumu / Magnetic Sensor Status: ");
    iotbot.serialWrite(magneticStatus ? "Tespit Edildi / Detected" : "Yok / Not Detected");
    // Manyetik alan algilandiysa "Tespit Edildi", aksi halde "Yok" yazdir
    // Print "Detected" if a magnetic field is detected, otherwise print "Not Detected"

    iotbot.lcdClear(); // LCD ekrani temizle / Clear the LCD screen

    if (magneticStatus)
    {
        iotbot.lcdWriteMid("Manyetik Alan", "Tespit Edildi!", "Magnetic Field", "Detected!");
        // Manyetik alan algilandiginda LCD'ye uyari yazdir / Display warning if a magnetic field is detected

        iotbot.serialWrite("⚠️ Manyetik Alan Algilandi! / ⚠️ Magnetic Field Detected!");
        // Seri porta uyari mesaji yazdir / Print warning message to serial port
    }
    else
    {
        iotbot.lcdWriteMid("Manyetik Alan", "Yok", "Magnetic Field", "Not Detected");
        // Manyetik alan yoksa LCD'ye bilgi yazdir / Display "Not Detected" if no magnetic field is found

        iotbot.serialWrite("Manyetik Alan Yok / No Magnetic Field");
        // Seri porta normal durum mesaji yazdir / Print normal status message to serial port
    }

    delay(500); // Yeni okuma yapmadan once 500 ms bekle / Wait for 500 ms before reading again
}
