/*
 * IOTBOT Email Sender Example / IOTBOT E-posta Gönderici Örneği
 * 
 * This example demonstrates how to send an email using the IOTBOT library.
 * Bu örnek, IOTBOT kütüphanesini kullanarak nasıl e-posta gönderileceğini gösterir.
 * 
 * Requirements / Gereksinimler:
 * 1. WiFi Connection / WiFi Bağlantısı
 * 2. Sender Email & App Password (Not your regular password!) / Gönderici E-postası ve Uygulama Şifresi (Normal şifreniz değil!)
 * 
 * Note: For Gmail, you must enable 2-Step Verification and generate an "App Password".
 * Not: Gmail için, 2 Adımlı Doğrulamayı etkinleştirmeli ve bir "Uygulama Şifresi" oluşturmalısınız.
 */

#define USE_EMAIL // Enable Email feature / E-posta özelliğini etkinleştir
#define USE_WIFI  // Enable WiFi feature / WiFi özelliğini etkinleştir
#include <IOTBOT.h>

IOTBOT iotbot;

// WiFi Credentials / WiFi Bilgileri
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// Email Credentials / E-posta Bilgileri
#define SMTP_HOST "smtp.gmail.com"
#define SMTP_PORT 465
#define AUTHOR_EMAIL "YOUR_EMAIL@gmail.com"
#define AUTHOR_PASSWORD "YOUR_APP_PASSWORD"
#define RECIPIENT_EMAIL "RECIPIENT_EMAIL@example.com"

void setup() {
  iotbot.serialStart(115200);
  iotbot.begin();
  iotbot.playIntro();
  
  iotbot.lcdClear();
  iotbot.lcdWriteMid("Email", "Sender", "Example", "Starting...");
  
  iotbot.serialWrite("Email Sender Example / E-posta Gönderici Örneği");

  // Connect to WiFi / WiFi'ye bağlan
  iotbot.wifiStartAndConnect(WIFI_SSID, WIFI_PASSWORD);
  
  if(iotbot.wifiConnectionControl()) {
      iotbot.lcdClear();
      iotbot.lcdWriteMid("WiFi", "Connected!", "Sending", "Email...");
      iotbot.serialWrite("Sending Email... / E-posta Gönderiliyor...");
      
      // Send Email / E-posta Gönder
      // Arguments: SMTP Host, Port, Author Email, Author Password, Recipient Email, Subject, Message
      iotbot.sendEmail(SMTP_HOST, SMTP_PORT, AUTHOR_EMAIL, AUTHOR_PASSWORD, RECIPIENT_EMAIL, "IOTBOT Test", "Hello from IOTBOT! / IOTBOT'tan Merhaba!");
      
      iotbot.lcdClear();
      iotbot.lcdWriteMid("Email", "Process", "Completed", "Check Inbox");
      iotbot.serialWrite("Process Completed / İşlem Tamamlandı");
  } else {
      iotbot.lcdClear();
      iotbot.lcdWriteMid("WiFi", "Connection", "Failed!", "Check Info");
  }
}

void loop() {
  // Nothing to do here / Burada yapılacak bir şey yok
}
