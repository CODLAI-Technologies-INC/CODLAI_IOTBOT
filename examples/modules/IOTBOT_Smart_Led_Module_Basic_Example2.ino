/*
 * 📌 Akıllı Led özelliklerini kullanabilmek için, IOTBOT_Config.h dosyasında 'USE_NEOPIXEL' tanımının başındaki
 * yorum satırlarını (//) kaldırın. Aksi takdirde, sensör işlevleri devre dışı kalacaktır.
 *
 * 📌 To enable Smart Led features, remove the comment (//) before the 'USE_NEOPIXEL' definition in
 * IOTBOT_Config.h. Otherwise, sensor functions will be disabled.
 */
#define USE_NEOPIXEL
#include <IOTBOT.h> // IoTBot kutuphanesi / IoTBot library

// IoTBot nesnesi olusturuluyor / Create an IoTBot object
IOTBOT iotbot;

#define SENSOR_PIN IO27 // Akilli LED'in bagli oldugu pini secin / Select the pin connected to the Smart LED
// Desteklenen pinler: IO25 - IO26 - IO27 - IO32 - IO33
// Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

void setup()
{
    iotbot.begin(); // IoTBot baslatiliyor / Initialize IoTBot

    iotbot.serialStart(115200); // Seri haberlesmeyi baslat / Start serial communication
    // Bilgisayar ile seri haberlesme icin 115200 baud hizinda baslatilir.
    // Starts serial communication at 115200 baud for computer connection.

    iotbot.serialWrite("🟢 Smart LED testi baslatildi / Smart LED test started.");
    // Test basladigini seri porta yazdir / Print test start message to serial port

    iotbot.lcdClear(); // LCD ekrani temizle / Clear the LCD screen
    iotbot.lcdWriteMid("Smart LED", "--- IoTBot ---", "Test Basladi", "Test Started");
    // LCD'ye test basladigini yazdir / Display test start message on LCD

    delay(2000); // Baslangic bekleme suresi / Initial delay

    iotbot.moduleSmartLEDPrepare(SENSOR_PIN);
    // Akilli LED'leri baslat / Initialize Smart LED
}

void loop()
{
    // 🔴 **Kirmizi LED**
    iotbot.serialWrite("🔴 Kirmizi LED yaniyor / Red LED ON.");
    // Seri porta mesaj yazdir / Print message to serial port

    iotbot.lcdClear();
    iotbot.lcdWriteMid("Smart LED", "LED:", "Kirmizi / Red", "");
    // LCD'ye kirmizi LED bilgisini yazdir / Display red LED status on LCD

    iotbot.moduleSmartLEDWrite(0, 255, 0, 0);
    // Kirmizi LED'i yak / Turn on the red LED

    delay(1000); // 1 saniye bekle / Wait for 1 second

    // 🟢 **Yesil LED**
    iotbot.serialWrite("🟢 Yesil LED yaniyor / Green LED ON.");
    // Seri porta mesaj yazdir / Print message to serial port

    iotbot.lcdClear();
    iotbot.lcdWriteMid("Smart LED", "LED:", "Yesil / Green", "");
    // LCD'ye yesil LED bilgisini yazdir / Display green LED status on LCD

    iotbot.moduleSmartLEDWrite(1, 0, 255, 0);
    // Yesil LED'i yak / Turn on the green LED

    delay(1000); // 1 saniye bekle / Wait for 1 second

    // 🔵 **Mavi LED**
    iotbot.serialWrite("🔵 Mavi LED yaniyor / Blue LED ON.");
    // Seri porta mesaj yazdir / Print message to serial port

    iotbot.lcdClear();
    iotbot.lcdWriteMid("Smart LED", "LED:", "Mavi / Blue", "");
    // LCD'ye mavi LED bilgisini yazdir / Display blue LED status on LCD

    iotbot.moduleSmartLEDWrite(2, 0, 0, 255);
    // Mavi LED'i yak / Turn on the blue LED

    delay(1000); // 1 saniye bekle / Wait for 1 second

    // 🚫 **Tum LED'leri kapat / Turn off all LEDs**
    iotbot.serialWrite("🚫 Tum LED'ler kapatiliyor / Turning OFF all LEDs.");
    // Seri porta LED kapatma mesaji yazdir / Print LED off message to serial port

    iotbot.lcdClear();
    iotbot.lcdWriteMid("Smart LED", "LED:", "Kapali / OFF", "");
    // LCD'ye LED'lerin kapatildigini yazdir / Display LED off status on LCD

    iotbot.moduleSmartLEDWrite(0, 0, 0, 0);
    iotbot.moduleSmartLEDWrite(1, 0, 0, 0);
    iotbot.moduleSmartLEDWrite(2, 0, 0, 0);
    // Tum LED'leri kapat / Turn off all LEDs

    delay(1000); // 1 saniye bekle / Wait for 1 second
}
