#include <IOTBOT.h> // IoTBot kutuphanesi / IoTBot library

// IoTBot nesnesi olusturuluyor / Create an IoTBot object
IOTBOT iotbot;

#define SENSOR_PIN IO27 // Toprak nem sensorunun bagli oldugu pini secin / Select the pin connected to the soil moisture sensor
// Desteklenen pinler: IO25 - IO26 - IO27 - IO32 - IO33
// Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

void setup()
{
    iotbot.begin(); // IoTBot baslatiliyor / Initialize IoTBot

    iotbot.serialStart(115200); // Seri haberlesmeyi baslat / Start serial communication
    // Bilgisayar ile seri haberlesme icin 115200 baud hizinda baslatilir.
    // Starts serial communication at 115200 baud for computer connection.

    iotbot.serialWrite("Toprak nem sensor testi baslatildi / Soil moisture sensor test started.");
    // Test basladigini seri porta yazdir / Print test start message to serial port

    iotbot.lcdClear(); // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("Toprak Nem Sensoru", "--- IoTBot ---", "Test Basladi", "Test Started");
    // LCD'ye test basladigini yazdir / Display test start message on LCD

    delay(3000); // Baslangic icin bekleme suresi / Initial delay
}

void loop()
{
    int soilMoisture = iotbot.moduleSoilMoistureRead(SENSOR_PIN);
    // Toprak nem sensorunden veri oku / Read data from the soil moisture sensor

    iotbot.serialWrite("Toprak Nem Degeri / Soil Moisture Value: ");
    iotbot.serialWrite(soilMoisture);
    // Seri porta toprak nem degerini yazdir / Print soil moisture value to serial port

    iotbot.lcdClear();
    // LCD ekrani temizle / Clear the LCD screen

    if (soilMoisture < 500) // Dusuk nem seviyesi kontrolu / Check for low moisture level
    {
        iotbot.lcdWriteMid("Toprak Kurak!", "Soil Dry!", "Nem: ", String(soilMoisture).c_str());
        // Nem seviyesi dusukse LCD'de uyari goster / Display warning if soil is dry

        iotbot.serialWrite("⚠️ Toprak kuru! / Soil is dry!");
        // Seri porta uyari mesaji yazdir / Print warning message to serial port
    }
    else
    {
        iotbot.lcdWriteMid("Toprak Nemli", "Soil Moist", "Nem: ", String(soilMoisture).c_str());
        // Toprak yeterince nemliyse LCD'de goster / Display moisture level on LCD

        iotbot.serialWrite("Toprak yeterince nemli / Soil is sufficiently moist.");
        // Seri porta durumu yazdir / Print status message to serial port
    }

    delay(1000);
    // Yeni okuma yapmadan once 1 saniye bekle / Wait for 1 second before reading again
}
