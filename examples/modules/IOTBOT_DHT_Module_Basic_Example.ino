/*
 * 📌 DHT Sensör özelliklerini kullanabilmek için, IOTBOT_Config.h dosyasında 'USE_DHT' tanımının başındaki
 * yorum satırlarını (//) kaldırın. Aksi takdirde, sensör işlevleri devre dışı kalacaktır.
 *
 * 📌 To enable DHT Sensor features, remove the comment (//) before the 'USE_DHT' definition in
 * IOTBOT_Config.h. Otherwise, sensor functions will be disabled.
 */
#define USE_DHT
#include <IOTBOT.h> // IoTBot kutuphanesi / IoTBot library

// IoTBot nesnesi olusturuluyor / Create an IoTBot object
IOTBOT iotbot;

#define DHT_PIN IO27 // DHT sensorunun bagli oldugu pini secin / Select the pin connected to the DHT sensor
// Desteklenen pinler: IO25 - IO26 - IO27 - IO32 - IO33
// Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

void setup()
{
    iotbot.begin(); // IoTBot baslatiliyor / Initialize IoTBot

    iotbot.serialStart(115200); // Seri haberlesmeyi baslat / Start serial communication
    // Bilgisayar ile seri haberlesme icin 115200 baud hizinda baslatilir.
    // Starts serial communication at 115200 baud for computer connection.

    iotbot.serialWrite("DHT Sensor Testi Baslatildi / DHT Sensor Test Started.");
    // DHT sensor testinin basladigini seri porta yazdir / Print DHT sensor test start message to the serial port

    iotbot.lcdClear(); // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("DHT Sensor", "--- IoTBot ---", "Test Basladi", "Test Started");
    // LCD'ye test basladigini yazdir / Display test start message on LCD

    delay(3000); // Baslangic icin bekleme suresi / Initial delay
}

void loop()
{
    // Sıcaklık ve Nem değerlerini oku / Read Temperature and Humidity values
    int tempC = iotbot.moduleDhtTempReadC(DHT_PIN);
    int hum = iotbot.moduleDhtHumRead(DHT_PIN);
    int feelingTempC = iotbot.moduleDthFeelingTempC(DHT_PIN);

    // Seri porta yazdır / Print to serial port
    iotbot.serialWrite("Sicaklik / Temperature: " + String(tempC) + " C");
    iotbot.serialWrite("Nem / Humidity: %" + String(hum));
    iotbot.serialWrite("Hissedilen / Feeling: " + String(feelingTempC) + " C");
    iotbot.serialWrite("------------------------------------------------");

    // LCD ekrana yazdır / Print to LCD screen
    iotbot.lcdClear();
    iotbot.lcdWriteCR(0, 0, "Temp: " + String(tempC) + " C");
    iotbot.lcdWriteCR(0, 1, "Hum: %" + String(hum));
    iotbot.lcdWriteCR(0, 2, "Feel: " + String(feelingTempC) + " C");
    iotbot.lcdWriteCR(0, 3, "--- IoTBot ---");

    delay(2000); // Yeni okuma yapmadan once 2 saniye bekle / Wait for 2 seconds before reading again
}
