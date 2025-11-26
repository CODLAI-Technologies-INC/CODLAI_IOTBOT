/*
 * IOTBOT Telegram Notification Example / IOTBOT Telegram Bildirim Örneği
 * 
 * This example demonstrates how to send notifications to a Telegram chat using the IOTBOT library.
 * Bu örnek, IOTBOT kütüphanesini kullanarak bir Telegram sohbetine nasıl bildirim gönderileceğini gösterir.
 * 
 * Requirements / Gereksinimler:
 * 1. Telegram Bot Token (Get from @BotFather) / Telegram Bot Token (@BotFather'dan alın)
 * 2. Chat ID (Get from @userinfobot or similar) / Chat ID (@userinfobot veya benzerinden alın)
 * 3. WiFi Connection / WiFi Bağlantısı
 * 
 * Instructions / Talimatlar:
 * 1. Enter your WiFi credentials. / WiFi bilgilerinizi girin.
 * 2. Enter your Telegram Bot Token and Chat ID. / Telegram Bot Token ve Chat ID bilgilerinizi girin.
 * 3. Upload the code to your IOTBOT. / Kodu IOTBOT'a yükleyin.
 * 4. Open the Serial Monitor (115200 baud) to see the status. / Durumu görmek için Seri Monitörü (115200 baud) açın.
 */

#define USE_TELEGRAM // Enable Telegram feature / Telegram özelliğini etkinleştir
#define USE_WIFI     // Enable WiFi feature / WiFi özelliğini etkinleştir
#include <IOTBOT.h>

IOTBOT iotbot;

// WiFi Credentials / WiFi Bilgileri
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Telegram Credentials / Telegram Bilgileri
String botToken = "YOUR_BOT_TOKEN";
String chatID = "YOUR_CHAT_ID";

void setup() {
  iotbot.serialStart(115200);
  iotbot.begin();
  iotbot.playIntro();
  
  iotbot.lcdClear();
  iotbot.lcdWriteMid("Telegram", "Example", "Connecting...", "WiFi");

  // Connect to WiFi / WiFi'ye bağlan
  iotbot.wifiStartAndConnect(ssid, password);
  
  if (iotbot.wifiConnectionControl()) {
    iotbot.lcdClear();
    iotbot.lcdWriteMid("WiFi", "Connected!", "Sending", "Message...");
    
    // Send a startup message / Başlangıç mesajı gönder
    iotbot.sendTelegram(botToken, chatID, "Hello from IOTBOT! System is online. / IOTBOT'tan merhaba! Sistem çevrimiçi.");
    
    iotbot.lcdClear();
    iotbot.lcdWriteMid("Message", "Sent!", "Check", "Telegram");
  } else {
    iotbot.lcdClear();
    iotbot.lcdWriteMid("WiFi", "Connection", "Failed!", "Check SSID/Pass");
  }
}

void loop() {
  // Example: Send a message when a button is pressed / Örnek: Bir düğmeye basıldığında mesaj gönder
  // Assuming Button 1 is used / Düğme 1'in kullanıldığı varsayılıyor
  if (iotbot.button1Read()) {
    iotbot.serialWrite("Button 1 Pressed! Sending Telegram... / Düğme 1 Basıldı! Telegram Gönderiliyor...");
    iotbot.lcdClear();
    iotbot.lcdWriteMid("Button 1", "Pressed", "Sending...", "Telegram");
    
    iotbot.sendTelegram(botToken, chatID, "Alert: Button 1 was pressed on IOTBOT! / Uyarı: IOTBOT üzerinde Düğme 1'e basıldı!");
    
    iotbot.lcdClear();
    iotbot.lcdWriteMid("Message", "Sent!", "Waiting...", "...");
    delay(2000); // Debounce delay / Debounce gecikmesi
  }
  
  delay(100);
}
