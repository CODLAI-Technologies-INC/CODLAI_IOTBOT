/*
  CODLAI IOTBOT - OTA Update Example (ESP32)
  -------------------------------------------------
  TR: OTA kullanmak icin once WiFi baglantisi kurun ve loop icinde otaHandle() cagirin.
  EN: Connect to WiFi first and call otaHandle() continuously in loop.

  Not: USE_OTA ve USE_WIFI define'lari kutuphane dahil edilmeden once yazilmalidir.
*/

#define USE_WIFI
#define USE_OTA

#include <IOTBOT.h>

IOTBOT iotbot;

const char *WIFI_SSID = "YOUR_WIFI_SSID";
const char *WIFI_PASS = "YOUR_WIFI_PASSWORD";

const char *OTA_HOST = "IOTBOT-OTA"; // Cihaz adi (opsiyonel)
const char *OTA_PASS = "1234";       // Sifre (varsayilan 1234)

void setup()
{
  iotbot.serialStart(115200);
  iotbot.begin();

  iotbot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS);
  iotbot.otaBegin(OTA_HOST, OTA_PASS, 3232);
}

void loop()
{
  iotbot.otaHandle();
  delay(10);
}
