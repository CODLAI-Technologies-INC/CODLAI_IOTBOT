/*
 * 📌 Wikipedia özelliğini kullanabilmek için, IOTBOT_Config.h dosyasında 'USE_WIKIPEDIA' tanımının başındaki
 * yorum satırlarını (//) kaldırın.
 *
 * 📌 To enable Wikipedia feature, remove the comment (//) before the 'USE_WIKIPEDIA' definition in
 * IOTBOT_Config.h.
 */
#define USE_WIKIPEDIA
#include <IOTBOT.h>

IOTBOT iotbot;

// WiFi Bilgileri / WiFi Credentials
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);

  // LCD Bilgilendirme / LCD Info
  iotbot.lcdShowLoading("WiFi Baglaniyor..."); // Connecting WiFi...
  iotbot.buzzerPlayTone(1000, 200);

  // WiFi Bağlantısı / WiFi Connection
  iotbot.wifiStartAndConnect(ssid, password);

  // WiFi Bağlandı / WiFi Connected
  iotbot.lcdShowStatus("WiFi Durumu", "Baglandi", true); // WiFi Status, Connected
  iotbot.buzzerPlayTone(1500, 200);
  delay(1000);

  // Wikipedia'dan bilgi al / Get info from Wikipedia
  String query = "Arduino";
  iotbot.lcdShowLoading("Araniyor: " + query); // Searching: ...
  Serial.println("Searching Wikipedia for: " + query);
  
  // Wikipedia'dan özet al (Türkçe karakter desteği ile)
  // Get summary from Wikipedia (with Turkish character support)
  String summary = iotbot.getWikipedia(query, "tr"); // "tr" for Turkish, "en" for English
  
  Serial.println("Ozet / Summary:");
  Serial.println(summary);

  // LCD'ye yazdır / Print to LCD
  iotbot.lcdClear();
  iotbot.lcdWrite("Ozet / Summary:");
  iotbot.lcdWriteCR(0, 1, summary.substring(0, 20)); // İlk 20 karakteri göster / Display first 20 chars
  iotbot.lcdWriteCR(0, 2, summary.substring(20, 40)); // Sonraki 20 karakteri göster / Display next 20 chars
  iotbot.lcdWriteCR(0, 3, summary.substring(40, 60)); // Sonraki 20 karakteri göster / Display next 20 chars
  iotbot.buzzerPlayTone(2000, 500);
}

void loop() {
  // Döngüde bir şey yapmaya gerek yok
  // Nothing to do in loop
}
