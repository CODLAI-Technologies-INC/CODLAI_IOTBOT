#define USE_DC_MOTOR
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

    iotbot.lcdWriteMid("DC Motor", "--- IoTBot ---", "Test Basladi", "Test Started");
    // LCD'ye test basladigini yazdir / Display test start message on LCD

    iotbot.serialWrite("DC Motor Testi Basladi / DC Motor Test Started.");
    // Test basladigini seri porta yazdir / Print test start message to serial port

    delay(3000); // Baslangic icin bekleme suresi / Initial delay
}

void loop()
{
    // 1️⃣ **Saat Yonunde Hareket / Clockwise Motion**
    iotbot.moduleDCMotorGOClockWise(50);
    // Motoru saat yonunde %50 hizda dondur / Rotate motor clockwise at 50% speed

    iotbot.lcdClear();
    // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("DC Motor", "Direction:", "Clockwise", "Speed: 50%");
    // LCD'ye yon ve hiz bilgilerini yazdir / Display direction and speed on LCD

    iotbot.serialWrite("🔄 DC Motor Saat Yonunde Dondu / DC Motor Rotating Clockwise at 50% Speed");
    // Seri porta saat yonunde dondugunu yazdir / Print clockwise rotation message to serial port

    delay(3000); // 3 saniye bekle / Wait for 3 seconds

    // 2️⃣ **Saat Yonunun Tersine Hareket / Counterclockwise Motion**
    iotbot.moduleDCMotorGOCounterClockWise(75);
    // Motoru saat yonunun tersine %75 hizda dondur / Rotate motor counterclockwise at 75% speed

    iotbot.lcdClear();
    // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("DC Motor", "Direction:", "CounterClockwise", "Speed: 75%");
    // LCD'ye yon ve hiz bilgilerini yazdir / Display direction and speed on LCD

    iotbot.serialWrite("🔄 DC Motor Saat Yonunun Tersine Dondu / DC Motor Rotating Counterclockwise at 75% Speed");
    // Seri porta saat yonunun tersine dondugunu yazdir / Print counterclockwise rotation message to serial port

    delay(3000); // 3 saniye bekle / Wait for 3 seconds

    // 3️⃣ **Motoru Durdur / Stop the Motor**
    iotbot.moduleDCMotorStop();
    // Motoru durdur / Stop the motor

    iotbot.lcdClear();
    // LCD ekrani temizle / Clear the LCD screen

    iotbot.lcdWriteMid("DC Motor", "Motor Stopped", "", "");
    // LCD'ye motorun durdugunu yazdir / Display motor stop message on LCD

    iotbot.serialWrite("🛑 DC Motor Durduruldu / DC Motor Stopped");
    // Seri porta motorun durdugunu yazdir / Print motor stop message to serial port

    delay(3000); // 3 saniye bekle / Wait for 3 seconds
}
