/*
 * IOTBOT Bluetooth Serial Example / IOTBOT Bluetooth Seri Haberleşme Örneği
 * 
 * This example demonstrates how to use Bluetooth Serial on IOTBOT (ESP32).
 * Bu örnek, IOTBOT (ESP32) üzerinde Bluetooth Seri haberleşmenin nasıl kullanılacağını gösterir.
 * 
 * You can control the robot or send messages using a Bluetooth Terminal app on your phone.
 * Telefonunuzdaki bir Bluetooth Terminal uygulaması kullanarak robotu kontrol edebilir veya mesaj gönderebilirsiniz.
 * 
 * Instructions / Talimatlar:
 * 1. Upload the code to your IOTBOT. / Kodu IOTBOT'a yükleyin.
 * 2. Open a Bluetooth Terminal app on your smartphone. / Akıllı telefonunuzda bir Bluetooth Terminal uygulaması açın.
 * 3. Connect to "IOTBOT_BT". / "IOTBOT_BT" cihazına bağlanın.
 * 4. Enter PIN "1234" if requested. / İstenirse PIN olarak "1234" girin.
 * 5. Send "LED_ON" to turn on the built-in LED (if available/mapped) or see messages on LCD. / Dahili LED'i yakmak için "LED_ON" gönderin veya mesajları LCD'de görün.
 * 6. Send any text to display it on the LCD. / LCD'de görüntülemek için herhangi bir metin gönderin.
 */

#define USE_BLUETOOTH // Enable Bluetooth feature / Bluetooth özelliğini etkinleştir
#include <IOTBOT.h>

IOTBOT iotbot;

void setup() {
  iotbot.serialStart(115200);
  iotbot.begin();
  iotbot.playIntro();
  
  iotbot.lcdClear();
  iotbot.lcdWriteMid("Bluetooth", "Mode", "Waiting...", "Connection");

  // Start Bluetooth with a name and PIN / Bluetooth'u isim ve PIN ile başlat
  iotbot.bluetoothStart("IOTBOT_BT", "1234");
  
  iotbot.serialWrite("Bluetooth Started! Connect to 'IOTBOT_BT' with PIN 1234 / Bluetooth Başlatıldı! 'IOTBOT_BT'ye 1234 PIN'i ile bağlanın");
}

void loop() {
  // Check if data is available from Bluetooth / Bluetooth'tan veri gelip gelmediğini kontrol et
  String message = iotbot.bluetoothRead();
  
  if (message != "") {
    message.trim(); // Remove whitespace / Boşlukları temizle
    iotbot.serialWrite("Received / Alındı: " + message);
    
    // Display on LCD / LCD'de göster
    iotbot.lcdClear();
    iotbot.lcdWriteMid("BT Received:", message.c_str(), "", "");
    
    // Echo back to phone / Telefona geri gönder
    iotbot.bluetoothWrite("IOTBOT Received / Alındı: " + message);
    
    // Example command / Örnek komut
    if (message == "HELLO") {
      iotbot.bluetoothWrite("Hello from IOTBOT! / IOTBOT'tan Merhaba!");
      iotbot.buzzerPlayTone(1000, 200);
    }
  }
  
  // Send data from Serial Monitor to Bluetooth (for testing) / Seri Monitörden Bluetooth'a veri gönder (test için)
  if (iotbot.serialAvailable()) {
    String serialMsg = iotbot.serialReadStringUntil('\n');
    serialMsg.trim();
    iotbot.bluetoothWrite(serialMsg);
    iotbot.lcdClear();
    iotbot.lcdWriteMid("Sent to BT:", serialMsg.c_str(), "", "");
  }
  
  delay(20);
}
