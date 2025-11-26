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

// 📌 **Erişim Noktası (AP) Modu İçin Wi-Fi Bilgileri**
// 📌 **Wi-Fi Information for Access Point (AP) Mode**
#define AP_SSID "CODLAI Server" // 📌 AP Modu için SSID / AP Mode SSID
#define AP_PASS "12345678"      // 📌 AP Modu için Şifre / AP Mode Password (En az 8 karakter)

// 📌 **Web Sayfası İçeriği (HTML, CSS, JavaScript)**
// 📌 **Web Page Content (HTML, CSS, JavaScript)**
// ✅ **JavaScript (Web Sayfası İçin)**
// ✅ **JavaScript (For Web Page)**
// Kullanıcı butona tıkladığında bir mesaj gösterecek.
// Shows a message when the user clicks the button.
const char WEBPageScript[] PROGMEM = R"rawliteral(
<script>
  function sayHello() {
    alert("Merhaba IOTBOT!");
  }
</script>
)rawliteral";

// ✅ **CSS (Web Sayfası Stili)**
// ✅ **CSS (Web Page Style)**
// Web sayfasının görünümünü ayarlamak için CSS kullanıyoruz.
// We use CSS to style the appearance of the web page.
const char WEBPageCSS[] PROGMEM = R"rawliteral(
<style>
  body { text-align: center; font-family: Arial, sans-serif; }
  button { font-size: 20px; padding: 10px; margin: 20px; }
</style>
)rawliteral";

// ✅ **HTML (Web Sayfası İçeriği)**
// ✅ **HTML (Web Page Content)**
// IOTBOT tarafından sunulacak HTML kodu
// HTML code to be served by IOTBOT
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
// 📌 **Setup Function**
void setup()
{
  iotbot.begin(); // 📌 **IoTBot Başlat / Initialize IoTBot**
  // 📌 **Seri Haberleşmeyi Başlat / Start Serial Communication**
  iotbot.serialStart(115200);

  // LCD Bilgilendirme / LCD Info
  iotbot.lcdShowLoading("Starting AP Mode");
  iotbot.buzzerPlayTone(1000, 200);

  // 📌 **IOTBOT'u Erişim Noktası (AP) Olarak Başlat**
  // 📌 **Start IOTBOT as Access Point (AP)**
  iotbot.serverStart("AP", AP_SSID, AP_PASS);

  // 📌 **IOTBOT Üzerinde Web Sayfasını Yayınla**
  // 📌 **Publish Web Page on IOTBOT**
  iotbot.serverCreateLocalPage("demopage", WEBPageScript, WEBPageCSS, WEBPageHTML); // Cihaza Bağlanın ve linke gidin: 192.168.4.1/demo / Connect to device and goto link: 192.168.4.1/demo

  // LCD Bilgilendirme / LCD Info
  iotbot.lcdShowStatus("AP Started", "IP: 192.168.4.1", true);
  iotbot.buzzerPlayTone(2000, 500);
  delay(2000);
  
  iotbot.lcdWriteMid("AP Mode Active", "SSID: " AP_SSID, "IP: 192.168.4.1", "Go to /demopage");
}

// 📌 **Ana Döngü (Loop)**
// 📌 **Main Loop**
void loop()
{
  iotbot.serverContinue(); // 📌 **AP modunda DNS yönlendirmeyi sürdür / Continue DNS redirection in AP mode**
}
