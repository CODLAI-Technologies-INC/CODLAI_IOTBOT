/*
  CODLAI IOTBOT - OTA + WiFi + NTP + Remote Info (ESP32)
  -------------------------------------------------
  TR: OTA kullanmak icin once WiFi baglantisi kurun ve loop icinde otaHandle() cagirin.
  EN: Connect to WiFi first and call otaHandle() continuously in loop.

  Not: USE_OTA ve USE_WIFI define'lari kutuphane dahil edilmeden once yazilmalidir.
*/

#define USE_WIFI
#define USE_OTA
#define USE_WIKIPEDIA

#include <IOTBOT.h>

IOTBOT iotbot;

const char *WIFI_SSID = "YOUR_WIFI_SSID";
const char *WIFI_PASS = "YOUR_WIFI_PASSWORD";

const char *OTA_HOST = "IOTBOT-OTA"; // Cihaz adi (opsiyonel)
const char *OTA_PASS = "1234";       // Sifre (varsayilan 1234)

const char *WIKI_QUERY = "Istanbul";
const char *WIKI_LANG = "tr";

unsigned long lastUiUpdateMs = 0;
unsigned long lastRemoteFetchMs = 0;
String lastWikiSnippet = "";

void setup()
{
  iotbot.serialStart(115200);
  iotbot.begin();

  iotbot.lcdClear();
  iotbot.lcdWriteCR(0, 0, "WiFi baglan...");

  iotbot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS);
  iotbot.otaBegin(OTA_HOST, OTA_PASS, 3232);

  // TR: Turkiye icin UTC+3
  iotbot.ntpBegin(3);

  iotbot.lcdClear();
  iotbot.lcdWriteCR(0, 0, "IP: " + iotbot.wifiGetIPAddress());
  iotbot.lcdWriteCR(0, 1, "OTA hazir");
}

void loop()
{
  iotbot.otaHandle();

  const unsigned long nowMs = millis();

  if (nowMs - lastUiUpdateMs >= 2000)
  {
    lastUiUpdateMs = nowMs;

    String timeStr = iotbot.ntpGetDateTimeString();
    if (timeStr.length() == 0)
    {
      timeStr = "NTP bekleniyor";
    }

    iotbot.lcdWriteCR(0, 2, timeStr);

    if (lastWikiSnippet.length() > 0)
    {
      String line = lastWikiSnippet;
      if (line.length() > 20)
      {
        line = line.substring(0, 20);
      }
      iotbot.lcdWriteCR(0, 3, line);
    }
  }

  if (nowMs - lastRemoteFetchMs >= 60000)
  {
    lastRemoteFetchMs = nowMs;
    lastWikiSnippet = iotbot.getWikipedia(WIKI_QUERY, WIKI_LANG);
    if (lastWikiSnippet.length() == 0)
    {
      lastWikiSnippet = "Bilgi alinamadi";
    }
  }

  delay(10);
}
