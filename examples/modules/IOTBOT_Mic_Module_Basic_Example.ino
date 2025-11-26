#include <IOTBOT.h> // IoTBot kutuphanesi / IoTBot library

// IoTBot nesnesi olusturuluyor / Create an IoTBot object
IOTBOT iotbot;

#define SENSOR_PIN IO27 // Mikrofon sensorunun bagli oldugu pini secin / Select the pin connected to the microphone sensor
// Desteklenen pinler: IO25 - IO26 - IO27 - IO32 - IO33
// Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

void setup()
{
    iotbot.begin(); // IoTBot baslatiliyor / Initialize IoTBot

    iotbot.serialStart(115200); // Seri haberlesmeyi baslat / Start serial communication
    // Bilgisayar ile seri haberlesme icin 115200 baud hizinda baslatilir.
    // Starts serial communication at 115200 baud for computer connection.

    iotbot.lcdClear(); // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("Mikrofon Modulu", "--- IoTBot ---", "Test Basladi", "Test Started");
    // LCD'ye test basladigini yazdir / Display test start message on LCD

    iotbot.serialWrite("Mikrofon Sensor Testi Baslatildi / Microphone Sensor Test Started.");
    // Test basladigini seri porta yazdir / Print test start message to the serial port

    delay(3000); // Baslangic icin bekleme suresi / Initial delay
}

void loop()
{
    int micValue = iotbot.moduleMicRead(SENSOR_PIN);
    // Mikrofon sensorunden gelen veriyi oku / Read data from the microphone sensor

    iotbot.serialWrite("Mikrofon Sensor Degeri / Microphone Sensor Value: " + String(micValue));
    // Mikrofon sensor degerini seri porta yazdir / Print microphone sensor value to serial port

    iotbot.lcdClear(); // LCD ekrani temizle / Clear the LCD screen

    if (micValue > 1000) // Gurultu seviyesi esik deger kontrolu / Noise level threshold check
    {
        iotbot.lcdWriteMid("⚠️ Yuksek Ses!", "⚠️ High Noise!", "Seviye: ", String(micValue).c_str());
        // Gurultu seviyesi yuksekse uyari mesaji goster / Display warning message if noise level is high

        iotbot.serialWrite("⚠️ Yuksek Ses Algilandi! / ⚠️ High Noise Detected!");
        // Seri porta uyari mesaji yazdir / Print warning message to serial port
    }
    else
    {
        iotbot.lcdWriteMid("Mikrofon Degeri:", "Microphone Value:", String(micValue).c_str(), "");
        // Mikrofon degerini LCD ekrana yazdir / Display microphone value on LCD

        iotbot.serialWrite("Ses Seviyesi Normal / Sound Level Normal");
        // Seri porta normal ses seviyesi mesaji yazdir / Print normal sound level message to serial port
    }

    delay(500); // Yeni okuma yapmadan once 500 ms bekle / Wait for 500 ms before reading again
}
