/*
  CODLAI IOTBOT - NTP Time Advanced Example

  TR:
  - Bu örnek, NTP ile saat senkronizasyonunu gosterir.
  - WiFi baglantisi kurulduktan sonra `ntpBegin()` cagrilmalidir.

  EN:
  - This example demonstrates time synchronization via NTP.
  - Call `ntpBegin()` AFTER connecting to WiFi.
*/

#define USE_WIFI
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Fill in your WiFi credentials
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

// TR/EN: Turkey is UTC+3, no DST (usually)
static const int TIMEZONE_HOURS = 3;

void setup()
{
  iotbot.serialStart(115200);
  delay(200);

  iotbot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS);

  if (!iotbot.wifiConnectionControl())
  {
    Serial.println("[WiFi] Not connected");
    return;
  }

  // TR/EN: Single-call setup (recommended)
  bool ok = iotbot.ntpBegin(TIMEZONE_HOURS);
  Serial.println(ok ? "[NTP] Synced" : "[NTP] Sync failed");

  Serial.print("Epoch: ");
  Serial.println((unsigned long)iotbot.ntpGetEpoch());

  Serial.print("DateTime: ");
  Serial.println(iotbot.ntpGetDateTimeString());
}

void loop()
{
}
