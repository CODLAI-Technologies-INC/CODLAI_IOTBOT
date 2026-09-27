// TR: KABLOSUZ ILETISIME ILK ADIM - En basit WiFi ornegi. IOTBOT'u evinizin
// WiFi agina baglar, baglanti basarili olursa aldigi IP adresini ve sinyal
// gucunu LCD'de ve Seri Port'ta gosterir. Sunucu YOK, web sayfasi YOK -
// sadece "ag'a katilmak" ne demek onu ogretir. Once bunu deneyin, sonra
// IOTBOT_WiFi_Web_Control_Example.ino'ya gecin.
// EN: FIRST STEP INTO WIRELESS COMMUNICATION - the simplest WiFi example.
// Connects IOTBOT to your home WiFi network and, once connected, shows the
// IP address and signal strength on the LCD and Serial. NO server, NO web
// page - just teaches what "joining a network" means. Try this first,
// then move on to IOTBOT_WiFi_Web_Control_Example.ino.

#define USE_WIFI
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

// ONEMLI: Kendi WiFi agranizin adini ve sifresini yazin.
// IMPORTANT: Fill in your own WiFi network's name and password.
#define WIFI_SSID "WIFI_SSID"
#define WIFI_PASS "WIFI_PASSWORD"

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.lcdShowLoading(turkish ? "WiFi'ye baglaniyor" : "Connecting to WiFi");

  iotbot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS);

  if (iotbot.wifiConnectionControl()) {
    String ip = iotbot.wifiGetIPAddress();
    iotbot.serialWrite(turkish ? "Baglandi! IP adresi: " + ip : "Connected! IP address: " + ip);
    iotbot.buzzerPlayTone(2000, 300);
    iotbot.lcdWriteMid(turkish ? "WIFI BAGLANDI" : "WIFI CONNECTED",
                        turkish ? "Ag: " WIFI_SSID : "Network: " WIFI_SSID,
                        ("IP: " + ip).c_str(),
                        turkish ? "Basarili!" : "Success!");
  } else {
    iotbot.serialWrite(turkish ? "Baglanti basarisiz! SSID/sifreyi kontrol edin."
                               : "Connection failed! Check SSID/password.");
    iotbot.buzzerPlayTone(400, 500);
    iotbot.lcdWriteMid(turkish ? "BAGLANTI BASARISIZ" : "CONNECTION FAILED",
                        turkish ? "SSID/sifre" : "Check SSID/",
                        turkish ? "kontrol edin" : "password",
                        "");
  }
}

void loop() {
  // Bu basit ornekte yapilacak baska bir sey yok - baglanti bilgisi zaten
  // setup()'ta gosterildi. / Nothing else to do in this simple example -
  // connection info was already shown in setup().
  iotbot.taskDelay(1000);
}
