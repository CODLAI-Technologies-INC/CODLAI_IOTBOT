#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

// IoTBot nesnesi oluşturuluyor / Create an IoTBot object
IOTBOT iotbot;

void setup()
{
  // 🟢 **IoTBot Başlatılıyor / Initialize IoTBot**
  iotbot.begin();
  iotbot.serialStart(115200);

  // 🟡 **LCD Ekranda Başlangıç Mesajı / Display Startup Message on LCD**
  iotbot.lcdClear();
  iotbot.lcdWriteMid("IoTBot Disco!", "CODLAI Style", "Let's Dance!", "");
  iotbot.serialWrite("🎶 IoTBot Disco Basladi / IoTBot Disco Started!");
  delay(2000);
}

void loop()
{
  // 🎤 **LCD Güncelle - Şov Başlıyor! / Update LCD - Show Starts!**
  iotbot.lcdClear();
  iotbot.lcdWriteMid("Enjoy the", "CODLAI Beat!", "Dance Now!", "");
  iotbot.serialWrite("🎼 Ritim Basliyor! / Beat is Starting!");

  // 🎵 **Ritim Tonları, Süreleri ve LED Pinleri / Rhythm Tones, Durations, and LED Pins**
  int ritimTonlari[] = {600, 800, 600, 700, 900, 700, 800, 600, 700, 600, 500, 700, 800, 600, 500, 600};
  int delayTimes[] = {150, 150, 150, 200, 150, 200, 150, 300, 200, 150, 150, 200, 250, 150, 100, 400};
  int ledPins[] = {IO25, IO26, IO27, IO32, IO33}; // LED pinleri / LED pins

  // 🕺 **LED ve Müzik Şovu Başlıyor / LED and Music Show Starts!**
  for (int i = 0; i < 16; i++)
  {
    int currentLed = ledPins[i % 5]; // **Mevcut LED'i Seç / Select the Current LED**

    // 🔴 **LED'i Aç / Turn On LED**
    iotbot.digitalWritePin(currentLed, HIGH);
    iotbot.serialWrite("💡 LED Açıldı: Pin " + String(currentLed));

    // 🔊 **Buzzer Çalsın / Play Buzzer**
    iotbot.buzzerPlayTone(ritimTonlari[i], delayTimes[i]);
    iotbot.serialWrite("🎵 Çalan Nota: " + String(ritimTonlari[i]) + " Hz, Süre: " + String(delayTimes[i]) + " ms");

    // 🔄 **Röle Tetiklemesi / Relay Triggering**
    if (i % 8 == 0)
    {
      iotbot.relayWrite(true); // **Röleyi Aç / Turn Relay On**
      delay(50);
      iotbot.relayWrite(false); // **Röleyi Kapat / Turn Relay Off**
      iotbot.serialWrite("⚡ Röle Tetiklendi! / Relay Triggered!");
    }

    // 🔵 **LCD Güncelle - Mevcut Durum / Update LCD - Current Step**
    iotbot.lcdClear();
    iotbot.lcdWriteMid(
        "IoTBot Disco!",
        ("Step: " + String(i + 1)).c_str(),
        ("LED: " + String(currentLed)).c_str(),
        ("Tone: " + String(ritimTonlari[i]) + " Hz").c_str());

    // 🔻 **Bekleme Süresi ve LED Kapatma / Wait Time and Turn Off LED**
    delay(delayTimes[i]);
    iotbot.digitalWritePin(currentLed, LOW);
    iotbot.serialWrite("🔌 LED Kapandı: Pin " + String(currentLed));
  }

  // 🎊 **Final Mesajı / Display Final Message**
  iotbot.lcdClear();
  iotbot.lcdWriteMid("That's It!", "IoTBot Disco", "", "See You Again!");
  iotbot.serialWrite("🎶 IoTBot Disco Tamamlandı! / IoTBot Disco Completed!");
  delay(3000);
}
