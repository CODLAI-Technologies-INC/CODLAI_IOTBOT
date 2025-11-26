/*
 * 📌 Server özelliklerini kullanabilmek için, IOTBOT_Config.h dosyasında 'USE_SERVER' tanımının başındaki
 * yorum satırlarını (//) kaldırın. Aksi takdirde, web sunucu işlevleri devre dışı kalacaktır.
 *
 * 📌 To enable server features, remove the comment (//) before the 'USE_SERVER' definition in
 * IOTBOT_Config.h. Otherwise, web server functions will be disabled.
 */
#define USE_SERVER
#include <IOTBOT.h> // 📌 IoTBot Kütüphanesi / IoTBot Library

IOTBOT iotbot; // 📌 IoTBot Nesnesi / IoTBot Object

// 📡 WiFi Settings | 📡 WiFi Ayarları
#define WIFI_SSID "WIFI_SSID" // Enter the name of the Wi-Fi network you want to connect to here. | Buraya bağlanmak istediğiniz Wi-Fi ağının adını yazınız.
#define WIFI_PASS "WiFi_PASS" // Enter the password of the Wi-Fi network you want to connect to here. | Buraya bağlanmak istediğiniz Wi-Fi ağının şifresini yazınız.

// 📌 **Erişim Noktası (AP) Modu İçin Wi-Fi Bilgileri**
#define AP_SSID "CODLAI Server" // 📌 AP Modu için SSID / AP Mode SSID
#define AP_PASS "12345678"      // 📌 AP Modu için Şifre / AP Mode Password (En az 8 karakter)

// 📌 **Web Sayfası İçeriği (HTML, CSS, JavaScript)**
// ✅ **JavaScript (Web Sayfası İçin)**
// Kullanıcı butona tıkladığında bir mesaj gösterecek.
const char WEBPageScript[] PROGMEM = R"rawliteral(
<script>
  function sayHello() {
    alert("Merhaba IOTBOT!");
  }
</script>
)rawliteral";

// ✅ **CSS (Web Sayfası Stili)**
// Web sayfasının görünümünü ayarlamak için CSS kullanıyoruz.
const char WEBPageCSS[] PROGMEM = R"rawliteral(
<style>
  body { text-align: center; font-family: Arial, sans-serif; }
  button { font-size: 20px; padding: 10px; margin: 20px; }
</style>
)rawliteral";

// ✅ **HTML (Web Sayfası İçeriği)**
// IOTBOT tarafından sunulacak HTML kodu
const char WEBPageHTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="tr">
<head>
  <meta charset="UTF-8">
  <title>IOTBOT Web Server</title>
  %s <!-- CSS Dosyası Buraya Eklenir -->
  %s <!-- JavaScript Dosyası Buraya Eklenir -->
</head>
<body>
  <h1>IOTBOT Web Sayfası</h1>
  <button onclick="sayHello()">Tıklayın</button>
</body>
</html>
)rawliteral";

// 📌 **Kurulum Fonksiyonu (Setup)**
void setup()
{
  iotbot.begin(); // 📌 **IoTBot Başlat / Initialize IoTBot**
  // 📌 **Seri Haberleşmeyi Başlat / Start Serial Communication**
  iotbot.serialStart(115200);

  // LCD Bilgilendirme / LCD Info
  iotbot.lcdShowLoading("Connecting WiFi");
  iotbot.buzzerPlayTone(1000, 200);

  // 📌 **STA veya AP Modunda Bağlantı Kur**
  iotbot.serverStart("STA", WIFI_SSID, WIFI_PASS);

  // 📌 **Eğer STA bağlantısı başarısız olursa AP moduna geç**
  if (!iotbot.wifiConnectionControl())
  {
    iotbot.serverStart("AP", AP_SSID, AP_PASS);
    // LCD Bilgilendirme / LCD Info
    iotbot.lcdShowStatus("WiFi Failed", "AP Mode Started", false);
    iotbot.buzzerPlayTone(500, 500);
    delay(2000);
    iotbot.lcdWriteMid("AP Mode Active", "SSID: " AP_SSID, "IP: 192.168.4.1", "Go to /demopage");
  } else {
    // LCD Bilgilendirme / LCD Info
    iotbot.lcdShowStatus("WiFi Connected", "IP: " + iotbot.wifiGetIPAddress(), true);
    iotbot.buzzerPlayTone(2000, 500);
    delay(2000);
    iotbot.lcdWriteMid("Server Ready", "IP: " + iotbot.wifiGetIPAddress(), "Go to /demopage", "...");
  }

  // 📌 **IOTBOT Üzerinde Web Sayfasını Yayınla**
  iotbot.serverCreateLocalPage("demopage", WEBPageScript, WEBPageCSS, WEBPageHTML); // Cihaza Bağlanın ve linke gidin: 192.168.4.1/demo / Connect to device and goto link: 192.168.4.1/demo
}

// 📌 **Ana Döngü (Loop)**
void loop()
{
  iotbot.serverContinue(); // 📌 **AP modunda DNS yönlendirmeyi sürdür**
}
