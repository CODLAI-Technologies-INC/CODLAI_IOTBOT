#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi oluşturuluyor / Create IoTBot object

bool prevButton1 = false, prevButton2 = false, prevButton3 = false; // Önceki buton durumları / Previous button states

void setup()
{
    iotbot.begin();             // IoTBot başlatılıyor / Initialize IoTBot
    iotbot.serialStart(115200); // Seri iletişim başlatılıyor / Start serial communication
    iotbot.serialWrite("🎛 Buton testi başlatıldı! / Button test started!");

    // **LCD başlangıç mesajı / LCD startup message**
    iotbot.lcdClear();
    iotbot.lcdWriteMid("Button Test", "--- IoTBot ---", "Test Basladi!", "Starting Test!");
    delay(3000); // **3 saniye bekle / Wait for 3 seconds**
}

void loop()
{
    // 🟢 **Buton değerlerini oku / Read button values**
    bool button1 = iotbot.button1Read(); // **B1 butonu / Button 1**
    bool button2 = iotbot.button2Read(); // **B2 butonu / Button 2**
    bool button3 = iotbot.button3Read(); // **B3 butonu / Button 3**

    // **LCD'ye sadece değişiklik olduğunda yaz / Update LCD only if there's a change**
    if (button1 != prevButton1 || button2 != prevButton2 || button3 != prevButton3)
    {
        iotbot.lcdClear();
        iotbot.lcdWriteCR(0, 0, "Button Test:");
        iotbot.lcdWriteCR(0, 1, "B1: " + String(button1 ? "Pressed" : "Released"));
        iotbot.lcdWriteCR(0, 2, "B2: " + String(button2 ? "Pressed" : "Released"));
        iotbot.lcdWriteCR(0, 3, "B3: " + String(button3 ? "Pressed" : "Released"));
    }

    // **Seri porta değişiklik olduğunda yaz / Print to serial only if there's a change**
    if (button1 != prevButton1)
    {
        iotbot.serialWrite("🔘 B1: " + String(button1 ? "Pressed" : "Released"));
    }
    if (button2 != prevButton2)
    {
        iotbot.serialWrite("🔘 B2: " + String(button2 ? "Pressed" : "Released"));
    }
    if (button3 != prevButton3)
    {
        iotbot.serialWrite("🔘 B3: " + String(button3 ? "Pressed" : "Released"));
    }

    // **Önceki buton durumlarını güncelle / Update previous button states**
    prevButton1 = button1;
    prevButton2 = button2;
    prevButton3 = button3;

    delay(100); // **Gereksiz işlem yükünü azaltmak için 100 ms bekle / Reduce CPU load with 100ms delay**
}
