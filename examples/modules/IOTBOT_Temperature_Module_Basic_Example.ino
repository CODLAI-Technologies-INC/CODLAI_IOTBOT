#include <IOTBOT.h> // IoTBot kutuphanesi / IoTBot library

// IoTBot nesnesi olusturuluyor / Create an IoTBot object
IOTBOT iotbot;

#define SENSOR_PIN IO27 // NTC sicaklik sensorunun bagli oldugu pini secin / Select the pin connected to the NTC temperature sensor
// Desteklenen pinler: IO25 - IO26 - IO27 - IO32 - IO33
// Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

void setup()
{
    iotbot.begin(); // IoTBot baslatiliyor / Initialize IoTBot

    iotbot.serialStart(115200); // Seri haberlesmeyi baslat / Start serial communication
    // Bilgisayar ile seri haberlesme icin 115200 baud hizinda baslatilir.
    // Starts serial communication at 115200 baud for computer connection.

    iotbot.lcdClear(); // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("NTC Sicaklik", "--- IoTBot ---", "Test Basladi", "Test Started");
    // LCD'ye test basladigini yazdir / Display test start message on LCD

    iotbot.serialWrite("NTC Sicaklik Sensoru Testi Basladi / NTC Temperature Sensor Test Started.");
    // Test basladigini seri porta yazdir / Print test start message to serial port

    delay(3000); // Baslangic icin bekleme suresi / Initial delay
}

void loop()
{
    float temp = iotbot.moduleNtcTempRead(SENSOR_PIN);
    // NTC sicaklik sensorunden veri oku / Read data from the NTC temperature sensor

    iotbot.serialWrite("Sicaklik / Temperature: ");
    iotbot.serialWrite(temp);
    // Seri porta sicaklik degerini yazdir / Print temperature value to serial port

    iotbot.lcdClear();
    // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("Sicaklik:", "Temperature:", String(temp, 1).c_str(), "C");
    // Sicaklik degerini LCD ekrana yazdir / Display temperature value on LCD

    delay(1000);
    // Yeni okuma yapmadan once 1 saniye bekle / Wait for 1 second before reading again
}
