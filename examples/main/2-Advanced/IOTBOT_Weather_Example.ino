/*
 * 📌 Hava Durumu özelliğini kullanabilmek için, IOTBOT_Config.h dosyasında 'USE_WEATHER' tanımının başındaki
 * yorum satırlarını (//) kaldırın.
 *
 * 📌 To enable Weather feature, remove the comment (//) before the 'USE_WEATHER' definition in
 * IOTBOT_Config.h.
 */
#define USE_WEATHER
#include <IOTBOT.h>

IOTBOT iotbot;

// WiFi Bilgileri / WiFi Credentials
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

// OpenWeatherMap API Anahtarı / API Key
// Eğer API anahtarınız yoksa boş bırakın, sistem otomatik olarak ücretsiz servisi (wttr.in) kullanacaktır.
// If you don't have an API key, leave it empty. The system will automatically use the free service (wttr.in).
String apiKey = ""; 
String city = "Istanbul";

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);

  // LCD Bilgilendirme / LCD Info
  iotbot.lcdShowLoading("Connecting WiFi");
  iotbot.buzzerPlayTone(1000, 200);

  // WiFi Bağlantısı / WiFi Connection
  iotbot.wifiStartAndConnect(ssid, password);

  // WiFi Bağlandı / WiFi Connected
  iotbot.lcdShowStatus("WiFi Status", "Connected", true);
  iotbot.buzzerPlayTone(2000, 500);
  delay(1000);
  
  iotbot.lcdShowLoading("Fetching Weather");
}

void loop() {
  // Hava durumunu al / Get weather
  String weatherData = iotbot.getWeather(city, apiKey);
  
  iotbot.serialWrite("Weather in " + city + ": " + weatherData);
  
  // LCD'ye yazdır / Print to LCD
  iotbot.lcdClear();
  iotbot.lcdWriteMid("Weather Report", city.c_str(), weatherData.c_str(), "Updated");
  iotbot.buzzerPlayTone(1500, 100);

  delay(60000); // 1 dakika bekle / Wait 1 minute
}
