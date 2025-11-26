#include <IOTBOT.h> // IoTBot kutuphanesi / IoTBot library

// IoTBot nesnesi olusturuluyor / Create an IoTBot object
IOTBOT iotbot;

#define SENSOR_PIN IO27 // Titresim sensorunun bagli oldugu pini secin / Select the pin connected to the vibration sensor
// Desteklenen pinler: IO25 - IO26 - IO27 - IO32 - IO33
// Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

void setup()
{
    iotbot.begin(); // IoTBot baslatiliyor / Initialize IoTBot

    iotbot.serialStart(115200); // Seri haberlesmeyi baslat / Start serial communication
    // Bilgisayar ile seri haberlesme icin 115200 baud hizinda baslatilir.
    // Starts serial communication at 115200 baud for computer connection.

    iotbot.serialWrite("Titresim sensor testi baslatildi / Vibration sensor test started.");
    // Test basladigini seri porta yazdir / Print test start message to serial port

    iotbot.lcdClear(); // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("Titresim Sensoru", "--- IoTBot ---", "Test Basladi", "Test Started");
    // LCD'ye test basladigini yazdir / Display test start message on LCD

    delay(3000); // Baslangic icin bekleme suresi / Initial delay
}

void loop()
{
    int vibrationStatus = iotbot.moduleVibrationAnalogRead(SENSOR_PIN);
    // Titresim sensorunden veri oku / Read data from the vibration sensor

    iotbot.serialWrite("Titresim Degeri / Vibration Value: ");
    iotbot.serialWrite(vibrationStatus);
    // Seri porta titresim degerini yazdir / Print vibration value to serial port

    iotbot.lcdClear();
    // LCD ekrani temizle / Clear the LCD screen

    if (vibrationStatus > 1000) // Yuksek titresim kontrolu / Check for high vibration level
    {
        iotbot.lcdWriteMid("Yuksek Titresim!", "High Vibration!", "Seviye: ", String(vibrationStatus).c_str());
        // Yuksek titresim algilandiginda LCD'de uyari goster / Display warning on LCD if high vibration detected

        iotbot.serialWrite("⚠️ Yuksek Titresim Algilandi! / ⚠️ High Vibration Detected!");
        // Seri porta uyari mesaji yazdir / Print warning message to serial port
    }
    else
    {
        iotbot.lcdWriteMid("Titresim Seviyesi:", "Vibration Level:", String(vibrationStatus).c_str(), "");
        // Titresim seviyesini LCD'ye yazdir / Display vibration level on LCD

        iotbot.serialWrite("Titresim Normal / Normal Vibration");
        // Seri porta normal durum mesaji yazdir / Print normal vibration status to serial port
    }

    delay(500);
    // Yeni okuma yapmadan once 500 ms bekle / Wait for 500 ms before reading again
}
