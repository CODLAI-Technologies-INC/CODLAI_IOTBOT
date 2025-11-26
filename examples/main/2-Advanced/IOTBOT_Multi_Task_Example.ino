/*
 * IOTBOT Multi-Tasking Example (ESP32 Only)
 * 
 * 🇹🇷 Bu örnek, ESP32'nin çift çekirdekli işlemcisini kullanarak birden fazla görevi aynı anda nasıl çalıştıracağınızı gösterir.
 * IOTBOT kütüphanesinin basitleştirilmiş 'createLoopTask' fonksiyonu sayesinde, karmaşık FreeRTOS kodları yazmadan
 * kolayca çoklu görev (multi-tasking) uygulamaları geliştirebilirsiniz.
 * 
 * 🇬🇧 This example demonstrates how to run multiple tasks simultaneously using ESP32's dual-core processor.
 * Thanks to the simplified 'createLoopTask' function of the IOTBOT library, you can easily develop 
 * multi-tasking applications without writing complex FreeRTOS code.
 * 
 * 📌 Nasıl Kullanılır? / How to Use?
 * 1. Görev Fonksiyonunu Tanımlayın / Define Task Function:
 *    - void GorevAdi() { ... } şeklinde bir fonksiyon oluşturun.
 *    - Create a function like void TaskName() { ... }.
 *    - Bu fonksiyonun içine yapılacak işi yazın. Sonsuz döngü (while(1) veya for(;;)) kullanmanıza GEREK YOKTUR.
 *    - Write the task logic inside. You do NOT need to use an infinite loop (while(1) or for(;;)).
 *    - Kütüphane bu fonksiyonu otomatik olarak sürekli tekrar edecektir.
 *    - The library will automatically repeat this function continuously.
 * 
 * 2. Görevi Başlatın / Start the Task:
 *    - setup() içinde 'iotbot.createLoopTask(...)' fonksiyonunu kullanın.
 *    - Use 'iotbot.createLoopTask(...)' inside setup().
 * 
 * 📌 Parametreler / Parameters:
 *    iotbot.createLoopTask(Function, "Name", CoreID, Priority, StackSize);
 * 
 *    - Function: Görev fonksiyonunun adı (örn: Task1code). / Name of the task function.
 *    - Name: Görevin metin olarak adı (örn: "Task1"). / Name of the task as string.
 *    - CoreID: Hangi çekirdekte çalışacak? (0 veya 1). / Which core to run on? (0 or 1).
 *      * Core 0: Genellikle WiFi ve arka plan işleri için kullanılır. / Usually for WiFi and background tasks.
 *      * Core 1: Arduino'nun ana döngüsü (loop) burada çalışır. / Arduino main loop runs here.
 *    - Priority: Öncelik seviyesi (1-9). Varsayılan: 1. / Priority level. Default: 1.
 *    - StackSize: Bellek boyutu. Varsayılan: 10000. / Memory size. Default: 10000.
 * 
 * 📌 Gereksinimler / Requirements:
 * - ESP32 Development Board (IOTBOT)
 * - IOTBOT Library
 * 
 * 📌 Görevler / Tasks:
 * 1. Task 1 (Core 0): LCD Ekranını günceller. / Updates the LCD Screen.
 * 2. Task 2 (Core 1): Sensör verilerini okur (Potansiyometre). / Reads sensor data (Potentiometer).
 * 3. Task 3 (Core 1): Buzzer ile sesli uyarı verir. / Beeps with the Buzzer.
 */

#include <IOTBOT.h>

// 📌 Sadece ESP32 için geçerlidir / Valid only for ESP32
#if !defined(ESP32)
  #error "This example is designed for ESP32 only! / Bu örnek sadece ESP32 içindir!"
#endif

IOTBOT iotbot;

// 📌 Görev Tanımlayıcıları / Task Handles
// (Not needed for simple usage / Basit kullanım için gerekli değil)

// 📌 Görev 1: LCD Güncelleme (Core 0)
// Bu görev LCD ekranına sürekli olarak zamanı yazar.
// This task continuously writes time to the LCD screen.
void Task1code(){
  // 🇹🇷 Not: for(;;) veya while(1) döngüsü kullanmanıza gerek yoktur.
  // 'createLoopTask' fonksiyonu bunu sizin için otomatik yapar.
  // 🇬🇧 Note: You don't need to use for(;;) or while(1) loops.
  // 'createLoopTask' function handles this automatically for you.
  
  iotbot.lcdWriteCR(0, 0, "Task 1: Running");
  iotbot.lcdWriteCR(0, 1, "Time: " + String(millis()));
  
  // 🇹🇷 Görev zamanlaması: 1 saniye bekle.
  // 'iotbot.taskDelay' kullanmak, işlemciyi meşgul etmeden bekletir.
  // 🇬🇧 Task timing: Wait for 1 second.
  // Using 'iotbot.taskDelay' waits without blocking the processor.
  iotbot.taskDelay(1000); 
}

// 📌 Görev 2: Sensör Okuma (Core 1)
// Bu görev potansiyometre değerini okur ve Seri Port'a yazar.
// This task reads potentiometer value and writes to Serial Port.
void Task2code(){
  int potValue = iotbot.potentiometerRead();
  iotbot.serialWrite("Task 2 - Pot Value: " + String(potValue));
  
  iotbot.taskDelay(500); // 500 ms bekle / wait 500 ms
}

// 📌 Görev 3: Buzzer Kontrolü (Core 1)
// Bu görev belirli aralıklarla buzzer'ı öttürür.
// This task beeps the buzzer at intervals.
void Task3code(){
  iotbot.buzzerPlayTone(1000, 100);
  iotbot.serialWrite("Task 3 - Beep!");
  
  iotbot.taskDelay(2000); // 2 saniye bekle / wait 2 seconds
}

void setup() {
  // 📌 IOTBOT Başlat / Initialize IOTBOT
  iotbot.begin();
  iotbot.serialStart(115200);
  
  iotbot.lcdShowLoading("Starting Tasks...");
  delay(1000);
  iotbot.lcdClear();

  // 📌 Görevleri Başlat / Start Tasks
  // 🇹🇷 Kullanım: iotbot.createLoopTask(Fonksiyon, "İsim", ÇekirdekID, Öncelik, YığınBoyutu);
  // 🇬🇧 Usage: iotbot.createLoopTask(Function, "Name", CoreID, Priority, StackSize);
  
  // Core 0: Arka plan işlemleri için önerilir (WiFi vb.) / Recommended for background tasks.
  iotbot.createLoopTask(Task1code, "Task1", 0, 1); 
  
  // Core 1: Ana işlemler ve sensör okumaları için önerilir. / Recommended for main tasks and sensors.
  iotbot.createLoopTask(Task2code, "Task2", 1, 1); 
  iotbot.createLoopTask(Task3code, "Task3", 1, 1); 
                    
  iotbot.serialWrite("All tasks started!");
}

void loop() {
  // 📌 Loop boş bırakılabilir çünkü işlemler görevlerde yapılıyor.
  // Loop can be left empty as operations are handled in tasks.
  iotbot.taskDelay(1000);
}
