#include <IOTBOT.h> // IoTBot kutuphanesi / IoTBot library

// IoTBot nesnesi olusturuluyor / Create an IoTBot object
IOTBOT iotbot;

#define SENSOR_PIN IO27 // Matris butonunun bagli oldugu pini secin / Select the pin connected to the matrix button
// Desteklenen pinler: IO25 - IO26 - IO27 - IO32 - IO33
// Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

void setup()
{
    iotbot.begin(); // IoTBot baslatiliyor / Initialize IoTBot

    iotbot.serialStart(115200); // Seri haberlesmeyi baslat / Start serial communication
    // Bilgisayar ile seri haberlesme icin 115200 baud hizinda baslatilir.
    // Starts serial communication at 115200 baud for computer connection.

    iotbot.lcdClear(); // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("Matris Buton", "--- IoTBot ---", "Test Basladi", "Test Started");
    // LCD'ye test basladigini yazdir / Display test start message on LCD

    iotbot.serialWrite("Matris Buton Testi Baslatildi / Matrix Button Test Started.");
    // Test basladigini seri porta yazdir / Print test start message to the serial port

    delay(3000); // Baslangic icin bekleme suresi / Initial delay
}

void loop()
{
    int buttonNumber = iotbot.moduleMatrisButtonNumberRead(SENSOR_PIN);
    // Matris buton verisini oku / Read matrix button value

    iotbot.lcdClear(); // LCD ekrani temizle / Clear the LCD screen

    if (buttonNumber > 0)
    {
        iotbot.lcdWriteMid("Buton:", "Button:", String(buttonNumber).c_str(), "");
        // Basilan buton numarasini LCD ekrana yazdir / Display pressed button number on LCD

        iotbot.serialWrite("Basilan Buton / Pressed Button: " + String(buttonNumber));
        // Seri porta basilan buton numarasini yazdir / Print pressed button number to the serial port
    }
    else
    {
        iotbot.lcdWriteMid("Buton Basilmadi", "No Button Pressed", "", "");
        // Eger butona basilmadiysa ekrana mesaj yazdir / Display "No Button Pressed" message on LCD

        iotbot.serialWrite("Herhangi bir butona basilmadi / No button pressed.");
        // Seri porta butona basilmadigini yazdir / Print "No button pressed" message to the serial port
    }

    delay(500); // Yeni okuma yapmadan once 500 ms bekle / Wait for 500 ms before reading again
}
