#define USE_STEP_MOTOR
#include <IOTBOT.h> // IoTBot kutuphanesi / IoTBot library

// IoTBot nesnesi olusturuluyor / Create an IoTBot object
IOTBOT iotbot;

void setup()
{
    iotbot.begin(); // IoTBot baslatiliyor / Initialize IoTBot

    iotbot.serialStart(115200); // Seri haberlesmeyi baslat / Start serial communication
    // Bilgisayar ile seri haberlesme icin 115200 baud hizinda baslatilir.
    // Starts serial communication at 115200 baud for computer connection.

    iotbot.lcdClear(); // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("Stepper Motor", "--- IoTBot ---", "Test Basladi", "Test Started");
    // LCD'ye test basladigini yazdir / Display test start message on LCD

    iotbot.serialWrite("Stepper Motor Testi Basladi / Stepper Motor Test Started.");
    // Test basladigini seri porta yazdir / Print test start message to serial port

    delay(3000); // Baslangic icin bekleme suresi / Initial delay
}

void loop()
{
    // 1️⃣ **Saat Yonunde Hareket / Clockwise Motion**
    iotbot.lcdClear();
    // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("Stepper Motor", "Direction:", "Clockwise", "Steps: 100");
    // LCD'ye saat yonunde hareket bilgisi yazdir / Display clockwise motion info on LCD

    iotbot.serialWrite("Stepper Motor Saat Yonunde Hareket Ediyor / Stepper Motor Moving Clockwise");
    // Saat yonunde hareket mesajini seri porta yazdir / Print clockwise motion message to serial port

    iotbot.moduleStepMotorMotion(50, true, 100, 60);
    // Step motoru 100 adim saat yonunde dondur, hiz 60 RPM / Rotate stepper motor 100 steps clockwise at 60 RPM

    delay(3000); // 3 saniye bekle / Wait for 3 seconds

    // 2️⃣ **Saat Yonunun Tersine Hareket / Counterclockwise Motion**
    iotbot.lcdClear();
    // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("Stepper Motor", "Direction:", "CounterClockwise", "Steps: 100");
    // LCD'ye saat yonunun tersi hareket bilgisi yazdir / Display counterclockwise motion info on LCD

    iotbot.serialWrite("Stepper Motor Saat Yonunun Tersine Hareket Ediyor / Stepper Motor Moving Counterclockwise");
    // Saat yonunun tersine hareket mesajini seri porta yazdir / Print counterclockwise motion message to serial port

    iotbot.moduleStepMotorMotion(50, false, 100, 60);
    // Step motoru 100 adim saat yonunun tersine dondur, hiz 60 RPM / Rotate stepper motor 100 steps counterclockwise at 60 RPM

    delay(3000); // 3 saniye bekle / Wait for 3 seconds
}
