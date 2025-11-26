#include <IOTBOT.h> // IoTBot kutuphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi olusturuluyor / Create IoTBot object

void setup()
{
    iotbot.begin();             // IoTBot baslatiliyor / Initialize IoTBot
    iotbot.serialStart(115200); // Seri iletisim baslatiliyor / Start serial communication
    iotbot.serialWrite("🔊 Buzzer testi baslatildi / Buzzer test started.");

    // **LCD baslangic mesaji / LCD startup message**
    iotbot.lcdClear();
    iotbot.lcdWriteMid("Buzzer Test", "--- IoTBot ---", "Test Basladi!", "Starting Test!");
    delay(2000); // Baslangic icin 2 saniye bekle / Wait 2 seconds for startup
}

void loop()
{
    // 🎵 **Farkli tonlar ve surelerle buzzer calma / Play buzzer with different tones and durations**
    int tones[][2] = {
        {1000, 300}, // **1000 Hz - 300 ms**
        {1500, 300}, // **1500 Hz - 300 ms**
        {2000, 300}, // **2000 Hz - 300 ms**
        {2500, 300}, // **2500 Hz - 300 ms**
    };

    // 🎼 **Her ton icin donguye gir / Loop through each tone**
    for (int i = 0; i < 4; i++)
    {
        int freq = tones[i][0];
        int duration = tones[i][1];

        // **Seri porta mesaj yazdir / Print message to serial port**
        iotbot.serialWrite("🎶 Buzzer playing at " + String(freq) + "Hz for " + String(duration) + "ms");

        // **LCD ekrani guncelle / Update LCD screen**
        iotbot.lcdClear();
        iotbot.lcdWriteMid("Buzzer caliyor", "Playing Tone", String(freq).c_str(), "Hz");

        // **Buzzer cal / Play buzzer**
        iotbot.buzzerPlayTone(freq, duration);
        delay(500); // 500 ms bekle / Wait 500 ms
    }

    // **1 saniye bekle ve tekrar et / Wait 1 second and repeat**
    delay(1000);
}
