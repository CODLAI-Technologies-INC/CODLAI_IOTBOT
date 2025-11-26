/*
 * IOTBOT Dual-Core Multi-Tasking & LED Visual Example
 * 
 * 🇹🇷 Bu örnek, ESP32'nin çift çekirdekli (Dual-Core) yapısını görsel olarak kanıtlamak için hazırlanmıştır.
 * Farklı çekirdeklerde çalışan görevler, farklı LED gruplarını birbirinden bağımsız hızlarda kontrol eder.
 * 
 * 🇬🇧 This example is designed to visually demonstrate ESP32's Dual-Core structure.
 * Tasks running on different cores control different LED groups at independent speeds.
 * 
 * 📌 Görsel Kanıt / Visual Proof:
 * - Core 0 (Task 1): 25, 26, 27 numaralı LED'leri çok hızlı bir şekilde "Kara Şimşek" (Knight Rider) efektiyle yakar.
 * - Core 1 (Task 2): 32 ve 33 numaralı LED'leri yavaşça (1 saniye arayla) yakıp söndürür.
 * -> Sonuç: Biri çok hızlı, diğeri çok yavaş çalışmasına rağmen birbirlerini HİÇ etkilemezler!
 * 
 * 📌 Bağlantılar / Connections:
 * - LED 1 -> Pin 25
 * - LED 2 -> Pin 26
 * - LED 3 -> Pin 27
 * - LED 4 -> Pin 32
 * - LED 5 -> Pin 33
 */

#include <IOTBOT.h>

// 📌 Sadece ESP32 için geçerlidir / Valid only for ESP32
#if !defined(ESP32)
  #error "This example is designed for ESP32 only! / Bu örnek sadece ESP32 içindir!"
#endif

IOTBOT iotbot;

// LED Pinleri
const int ledPinsFast[] = {25, 26, 27}; // Core 0 tarafından kontrol edilecek (Hızlı)
const int ledPinsSlow[] = {32, 33};     // Core 1 tarafından kontrol edilecek (Yavaş)

// 📌 Görev 1: Hızlı LED Efekti (Core 0)
// Bu görev Core 0 üzerinde çalışır ve diğer görevlerden bağımsız olarak LED'leri hızlıca kaydırır.
// This task runs on Core 0 and shifts LEDs rapidly, independent of other tasks.
void TaskFastLEDs() {
  static int direction = 1;
  static int currentLed = 0;

  // Tüm hızlı LED'leri söndür / Turn off all fast LEDs
  for(int i=0; i<3; i++) digitalWrite(ledPinsFast[i], LOW);
  
  // Şu anki LED'i yak / Turn on current LED
  digitalWrite(ledPinsFast[currentLed], HIGH);

  // Bir sonraki LED'i hesapla / Calculate next LED
  currentLed += direction;
  if(currentLed >= 2 || currentLed <= 0) direction *= -1;

  // Hangi çekirdekte çalıştığını yazdır (Sadece debug için, çok hızlı olduğu için her zaman yazdırılmaz)
  // Print which core is running (Only for debug, not printed always due to speed)
  // Serial.print("Fast Task Core: "); Serial.println(xPortGetCoreID());

  iotbot.taskDelay(100); // 100ms bekle (Çok hızlı / Very fast)
}

// 📌 Görev 2: Yavaş Yanıp Sönme (Core 1)
// Bu görev Core 1 üzerinde çalışır ve LED'leri yavaşça yakıp söndürür.
// This task runs on Core 1 and blinks LEDs slowly.
void TaskSlowLEDs() {
  static bool state = false;
  state = !state;

  digitalWrite(ledPinsSlow[0], state);
  digitalWrite(ledPinsSlow[1], !state); // Ters çalışsın / Toggle opposite

  iotbot.serialWrite("Slow Task (Blink) running on Core: " + String(xPortGetCoreID()));
  
  iotbot.taskDelay(1000); // 1000ms bekle (Yavaş / Slow)
}

// 📌 Görev 3: Bilgi Ekranı (Core 1)
// LCD ekranını günceller.
// Updates LCD screen.
void TaskInfo() {
  iotbot.lcdWriteCR(0, 0, "Core 0: Fast LEDs");
  iotbot.lcdWriteCR(0, 1, "Core 1: Slow LEDs");
  iotbot.lcdWriteCR(0, 2, "Multi-Tasking...");
  iotbot.lcdWriteCR(0, 3, "Time: " + String(millis() / 1000) + "s");
  
  iotbot.taskDelay(500);
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  
  // Pin Modlarını Ayarla / Set Pin Modes
  for(int i=0; i<3; i++) pinMode(ledPinsFast[i], OUTPUT);
  for(int i=0; i<2; i++) pinMode(ledPinsSlow[i], OUTPUT);

  iotbot.lcdShowLoading("Dual Core Demo...");
  delay(1000);
  iotbot.lcdClear();

  // 📌 Görevleri Başlat / Start Tasks
  
  // Task 1 -> Core 0 (Hızlı / Fast)
  iotbot.createLoopTask(TaskFastLEDs, "FastLED", 0, 1); 
  
  // Task 2 -> Core 1 (Yavaş / Slow)
  iotbot.createLoopTask(TaskSlowLEDs, "SlowLED", 1, 1); 
  
  // Task 3 -> Core 1 (Bilgi / Info)
  iotbot.createLoopTask(TaskInfo, "Info", 1, 1); 

  iotbot.serialWrite("Tasks started! Watch the LEDs.");
}

void loop() {
  // Loop boş. Her şey görevlerde yapılıyor.
  // Loop is empty. Everything is done in tasks.
  iotbot.taskDelay(1000);
}
