#include <IOTBOT.h> // IoTBot kutuphanesi / IoTBot library

// IoTBot nesnesi olusturuluyor / Create an IoTBot object
IOTBOT iotbot;

#define SENSOR_PIN IO27 // PIR sensorunun bagli oldugu pini secin / Select the pin connected to the PIR sensor
// Desteklenen pinler: IO25 - IO26 - IO27 - IO32 - IO33
// Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

void setup()
{
    iotbot.begin(); // IoTBot baslatiliyor / Initialize IoTBot

    iotbot.serialStart(115200); // Seri haberlesmeyi baslat / Start serial communication
    // Bilgisayar ile seri haberlesme icin 115200 baud hizinda baslatilir.
    // Starts serial communication at 115200 baud for computer connection.

    iotbot.serialWrite("PIR Sensor Testi Baslatildi / PIR Sensor Test Started.");
    // PIR sensor testinin basladigini seri porta yazdir / Print PIR sensor test start message to the serial port

    iotbot.lcdClear(); // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("PIR Sensor", "--- IoTBot ---", "Test Basladi", "Test Started");
    // LCD'ye test basladigini yazdir / Display test start message on LCD

    delay(3000); // Baslangic icin bekleme suresi / Initial delay
}

void loop()
{
    int motionDetected = iotbot.moduleMotionRead(SENSOR_PIN);
    // PIR sensorunden hareket algilama verisini oku / Read motion detection data from PIR sensor

    iotbot.serialWrite("PIR Sensor Durumu / PIR Sensor Status: " + String(motionDetected ? "Hareket Algilandi / Motion Detected" : "Hareket Yok / No Motion"));
    // Hareket durumu bilgisini seri porta yazdir / Print motion status to serial port

    iotbot.lcdClear(); // LCD ekrani temizle / Clear the LCD screen

    if (motionDetected)
    {
        iotbot.lcdWriteMid("Hareket Algilandi!", "Motion Detected!", "", "");
        // Eger hareket algilandiysa mesaji LCD ekrana yazdir / Display message on LCD if motion is detected

        iotbot.serialWrite("PIR Sensor Hareket Algiladi! / PIR Sensor Detected Motion!");
        // Seri porta uyari mesaji yazdir / Print warning message to serial port
    }
    else
    {
        iotbot.lcdWriteMid("Hareket Yok", "No Motion", "", "");
        // Eger hareket algilanmadiysa mesaji LCD ekrana yazdir / Display message on LCD if no motion is detected

        iotbot.serialWrite("PIR Sensor Hareket Algilamadi / PIR Sensor Detected No Motion");
        // Seri porta bilgilendirme mesaji yazdir / Print no motion detected message to serial port
    }

    delay(500); // Yeni okuma yapmadan once 500 ms bekle / Wait for 500 ms before reading again
}
