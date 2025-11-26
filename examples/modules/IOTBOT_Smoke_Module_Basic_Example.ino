#include <IOTBOT.h> // IoTBot kutuphanesi / IoTBot library

// IoTBot nesnesi olusturuluyor / Create an IoTBot object
IOTBOT iotbot;

#define SENSOR_PIN IO27 // Duman sensorunun bagli oldugu pini secin / Select the pin connected to the smoke sensor
// Desteklenen pinler: IO25 - IO26 - IO27 - IO32 - IO33
// Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

void setup()
{
    iotbot.begin(); // IoTBot baslatiliyor / Initialize IoTBot

    iotbot.serialStart(115200); // Seri haberlesmeyi baslat / Start serial communication
    // Bilgisayar ile seri haberlesme icin 115200 baud hizinda baslatilir.
    // Starts serial communication at 115200 baud for computer connection.

    iotbot.lcdClear(); // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("Duman Sensoru", "--- IoTBot ---", "Test Basladi", "Test Started");
    // LCD'ye test basladigini yazdir / Display test start message on LCD

    iotbot.serialWrite("Duman Sensoru Testi Basladi / Smoke Sensor Test Started.");
    // Seri porta test basladigini yazdir / Print test start message to serial port

    delay(3000); // Baslangic icin bekleme suresi / Initial delay
}

void loop()
{
    int smokeLevel = iotbot.moduleSmokeRead(SENSOR_PIN);
    // Duman sensorunden veri oku / Read data from the smoke sensor

    iotbot.lcdClear();
    // LCD ekrani temizle / Clear the LCD screen

    iotbot.serialWrite("Duman Seviyesi / Smoke Level: ");
    iotbot.serialWrite(smokeLevel);
    // Seri porta duman seviyesini yazdir / Print smoke level to serial port

    if (smokeLevel > 1400) // Kritik esik degeri kontrolu / Check critical threshold
    {
        iotbot.lcdWriteMid("Duman Algilandi!", "Smoke Detected!", "", "");
        // Duman algilandi mesajini LCD ekrana yazdir / Display "Smoke Detected!" message on LCD

        iotbot.serialWrite("⚠️ Duman Algilandi! / Smoke Detected!");
        // Seri porta uyari mesaji yazdir / Print warning message to serial port
    }
    else
    {
        iotbot.lcdWriteMid("Duman Yok", "No Smoke", "Seviye: ", String(smokeLevel).c_str());
        // Duman algilanmadiginda LCD'de seviye goster / Display smoke level on LCD if no smoke detected

        iotbot.serialWrite("Duman Yok / No Smoke");
        // Seri porta duman olmadigini yazdir / Print "No Smoke" message to serial port
    }

    delay(1000);
    // Yeni okuma yapmadan once 1 saniye bekle / Wait for 1 second before reading again
}
