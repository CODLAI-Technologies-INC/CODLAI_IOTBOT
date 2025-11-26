/*
 * 📌 IR Sensör özelliklerini kullanabilmek için, IOTBOT_Config.h dosyasında 'USE_IR' tanımının başındaki
 * yorum satırlarını (//) kaldırın. Aksi takdirde, sensör işlevleri devre dışı kalacaktır.
 *
 * 📌 To enable IR Sensor  features, remove the comment (//) before the 'USE_IR' definition in
 * IOTBOT_Config.h. Otherwise, sensor functions will be disabled.
 */
#define USE_IR
#include <IOTBOT.h> // IOTBOT kutuphanesi / IOTBOT library

// IOTBOT nesnesi olustur / Create an IOTBOT object
IOTBOT iotbot;

#define SENSOR_PIN IO27 // IR sensorunun bagli oldugu pini secin / Select the pin connected to the IR sensor
// Desteklenen pinler: IO25 - IO26 - IO27 - IO32 - IO33
// Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

void setup()
{
    iotbot.begin(); // IOTBOT baslatiliyor / Initialize IOTBOT

    iotbot.serialStart(115200); // Seri haberlesmeyi 115200 baud hiziyla baslat / Start serial communication at 115200 baud
    // Bilgisayar ile seri haberlesme icin 115200 baud hizinda baslatilir.
    // Starts serial communication at 115200 baud for computer connection.

    iotbot.serialWrite("IR Okuyucu testi baslatildi / IR Reader test started.");
    // IR sensor testinin basladigini seri porta yazdir / Print IR sensor test start message to the serial port

    delay(2000); // Baslangic gecikmesi (2 saniye) / Initial delay (2 seconds)

    iotbot.lcdClear(); // LCD ekrani temizle / Clear the LCD screen

    // **LCD ekrana baslik ve aciklamalari yazdir / Display title and description on the LCD**
    iotbot.lcdWriteMid("IotBot", "IR Reader Test", "", "HEX value:        ");
}

void loop()
{
    // **IR sensorunden gelen HEX formatindaki veriyi oku / Read HEX formatted data from IR sensor**
    String value = iotbot.moduleIRReadHex(SENSOR_PIN);

    // **Eger gecerli bir veri alindiysa (0 degilse) ekrana ve seri porta yazdir / If valid data is received (not "0"), print to LCD and serial port**
    if (value != "0")
    {
        // **LCD ekrana HEX kodunu yaz (4. satir, 11. sutundan itibaren) / Write HEX code to LCD (Row 4, Column 11)**
        iotbot.lcdWriteCR(11, 3, value);

        // **Seri port uzerinden HEX kodunu gonder / Send HEX code via serial port**
        iotbot.serialWrite("Hex Code:");
        iotbot.serialWrite(value);
    }
    else
    {
        // **Eger sinyal alinmazsa ekranda bekleme durumu goster / If no signal is received, show waiting status on LCD**
        iotbot.lcdWriteCR(11, 3, "Waiting...");
        iotbot.serialWrite("No IR Signal Received.");
    }

    delay(500); // 500ms bekleme suresi / 500ms delay
}
