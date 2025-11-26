#include <IOTBOT.h> // IoTBot kutuphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi olusturuluyor / Create IoTBot object

void setup()
{
    iotbot.begin();             // IoTBot baslatiliyor / Initialize IoTBot
    iotbot.serialStart(115200); // Seri haberlesmeyi baslat / Start serial communication
    iotbot.serialWrite("📺 LCD testi baslatildi / LCD test started.");

    // LCD giris mesaji / LCD startup message
    iotbot.lcdClear();
    iotbot.lcdWriteMid("LCD Test", "--- IoTBot ---", "Test Basladi!", "Starting Test!");
    delay(3000); // 3 saniye bekle / Wait for 3 seconds
}

void loop()
{
    // 📌 **Temel LCD Mesajlari / Basic LCD Messages**
    iotbot.lcdClear();
    iotbot.lcdWriteCR(0, 0, "IoTBot LCD Test");
    iotbot.lcdWriteCR(0, 1, "Satir 1 / Row 1");
    iotbot.lcdWriteCR(0, 2, "Satir 2 / Row 2");
    iotbot.lcdWriteCR(0, 3, "Satir 3 / Row 3");
    iotbot.serialWrite("🖥 LCD temel metinler yazildi.");
    delay(2000);

    // 📌 **LCD'de Hareketli Yazi Testi / Scrolling Text Test**
    iotbot.lcdClear();
    iotbot.serialWrite("📜 Hareketli metin baslatildi...");
    for (int i = 0; i < 16; i++) // 16 sutun kaydirma yap
    {
        iotbot.lcdWriteCR(i, 1, "-> IoTBot <-"); // Yaziyi saga kaydir / Shift text to the right
        delay(150);                              // 150 ms bekle / Wait for 150 ms
        iotbot.lcdClear();
    }

    // 📌 **Sayi Sayaci Testi / Number Counter Test**
    iotbot.lcdClear();
    iotbot.lcdWriteCR(0, 0, "Sayi Test / Numbers:");
    for (int i = 0; i <= 10; i++)
    {
        iotbot.lcdWriteCR(12, 1, String(i)); // Sayilari yazdir / Print numbers
        iotbot.serialWrite("🔢 Sayi: " + String(i));
        delay(500);
    }

    // 📌 **Basari Mesaji / Success Message**
    iotbot.lcdClear();
    iotbot.lcdWriteMid("IoTBot Testi", "LCD Modulu", "BAsARILI!", "SUCCESSFUL!");
    iotbot.serialWrite("✅ LCD testi basariyla tamamlandi.");
    delay(5000);
}
