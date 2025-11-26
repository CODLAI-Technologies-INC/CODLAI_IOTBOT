/*
 * 📌 RFID özelliklerini kullanabilmek için, IOTBOT_Config.h dosyasında 'USE_SERVO' tanımının başındaki
 * yorum satırlarını (//) kaldırın. Aksi takdirde, sensör işlevleri devre dışı kalacaktır.
 *
 * 📌 To enable RFID features, remove the comment (//) before the 'USE_SERVO' definition in
 * IOTBOT_Config.h. Otherwise, sensor functions will be disabled.
 */
#define USE_RFID
#include <IOTBOT.h> // 📌 IoTBot kutuphanesi

// 📌 **IoTBot nesnesi olustur**
IOTBOT iotbot;

void setup()
{
    iotbot.begin();    // 📌 IoTBot baslatiliyor / Initialize IoTBot
    iotbot.lcdClear(); // 📌 LCD ekrani temizle / Clear LCD screen

    // 📌 **Seri port ve LCD'ye baslangic mesaji**
    iotbot.serialWrite("RFID Okuyucu Baslatiliyor... / Starting RFID Reader...");
    iotbot.lcdWriteMid("RFID Modulu", "--- IoTBot ---", "Hazir!", "Ready!");

    delay(2000); // 2 saniye bekle / Wait 2 seconds
}

void loop()
{
    int rfidID = iotbot.moduleRFIDRead(); // 📌 RFID karti oku / Read RFID card

    if (rfidID != 0) // 📌 **Kart okutulduysa islem yap**
    {
        // 📌 **Seri porta RFID bilgisini yazdir**
        iotbot.serialWrite("📌 Okunan RFID ID: " + String(rfidID));

        // 📌 **LCD ekranda RFID bilgisini goster**
        iotbot.lcdClear();
        iotbot.lcdWriteMid("RFID Kart Okundu!", "RFID Scanned!", "ID:", String(rfidID).c_str());

        delay(2000); // 2 saniye bekle / Wait 2 seconds
    }
}
