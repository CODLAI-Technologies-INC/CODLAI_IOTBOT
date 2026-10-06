// TR: INTERNET SAATI - Saat, Alarm ve Gece Lambasi. IOTBOT WiFi'ye baglanip
// saati internetten (NTP) ceker ve LCD'de saat, tarih ve gunu gosterir.
//  - "Saat 07:30 OLUNCA" alarm melodisi BIR KEZ calar (ntpTimeReached).
//  - "Saat 22:00 ile 06:00 ARASINDA ISE" kart uzerindeki role (gece lambasi)
//    acik kalir (ntpTimeIsBetween - gece yarisini asan aralik da olur).
//  - Saat her 6 saatte bir kendiliginden, B3'e basinca da hemen GUNCELLENIR
//    (ntpUpdate).
// Bu fonksiyonlar editordeki "Internet saatini kullan / guncelle", "saat ...
// ise", "saat ... olunca" bloklarinin karsiligidir.
// EN: INTERNET TIME - Clock, Alarm and Night Lamp. The IOTBOT connects to WiFi,
// gets the time from the internet (NTP) and shows time, date and weekday on
// the LCD.
//  - "WHEN it is 07:30" the alarm melody plays ONCE (ntpTimeReached).
//  - "IF the time is BETWEEN 22:00 and 06:00" the onboard relay (night lamp)
//    stays on (ntpTimeIsBetween - ranges crossing midnight work too).
//  - The time is UPDATED by itself every 6 hours, and right away when you
//    press B3 (ntpUpdate).
// These functions are what the editor's "use / update internet time", "if
// time is ...", "when time is ..." blocks call.
//
// Baglanti / Wiring: Ek modul GEREKMEZ. Asagiya WiFi adinizi ve sifrenizi
// yazin. / NO extra module needed. Fill in your WiFi name and password below.

#define USE_WIFI
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

namespace {
  constexpr int kTimezoneHours = 3;                 // Turkiye UTC+3 / Turkey UTC+3
  constexpr int kAlarmHour = 7, kAlarmMinute = 30;  // Alarm saati / alarm time
  constexpr int kNightStartHour = 22, kNightStartMinute = 0; // Gece lambasi baslangic / night lamp start
  constexpr int kNightEndHour = 6, kNightEndMinute = 0;      // Gece lambasi bitis / night lamp end
  constexpr uint32_t kAutoUpdateMs = 6UL * 60UL * 60UL * 1000UL; // 6 saatte bir guncelle / update every 6 hours

  const char *kDaysTr[] = {"", "Pazartesi", "Sali", "Carsamba", "Persembe", "Cuma", "Cumartesi", "Pazar"};
  const char *kDaysEn[] = {"", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};

  uint32_t lastUpdateMs = 0;
  uint32_t lastDrawMs = 0;
  bool b3WasDown = false;

  void showRow(int row, const char *text) {
    char line[21];
    snprintf(line, sizeof(line), "%-20s", text);
    iotbot.lcdWriteFixed(row, line);
  }

  void updateTime() {
    iotbot.lcdWriteMid(turkish ? "Saat guncelleniyor" : "Updating time", "...", "", "");
    bool ok = iotbot.ntpUpdate(); // "Internet saatini guncelle" / "update internet time"
    iotbot.serialWrite(ok ? (turkish ? "Saat guncellendi." : "Time updated.")
                          : (turkish ? "Guncelleme basarisiz." : "Update failed."));
    lastUpdateMs = millis();
  }
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.lcdWriteMid(turkish ? "INTERNET SAATI" : "INTERNET CLOCK",
                     turkish ? "WiFi'ye baglaniyor" : "Connecting WiFi", "", "");
  iotbot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS);
  if (!iotbot.wifiConnectionControl()) {
    iotbot.lcdWriteMid(turkish ? "WiFi YOK" : "NO WiFi",
                       turkish ? "Ad/sifreyi kontrol" : "Check name/password",
                       turkish ? "edip tekrar yukleyin" : "and upload again", "");
    return;
  }

  // "Internet saatini kullan" / "use internet time"
  iotbot.ntpBegin(kTimezoneHours);
  lastUpdateMs = millis();
  iotbot.lcdClear();
}

void loop() {
  uint32_t now = millis();

  // B3 = saati hemen guncelle / B3 = update the time now
  bool b3Down = iotbot.button3Read();
  if (b3Down && !b3WasDown) updateTime();
  b3WasDown = b3Down;

  if (now - lastUpdateMs >= kAutoUpdateMs) updateTime();

  // "Saat 07:30 olunca" - o dakikada SADECE BIR KEZ true doner, bu yuzden
  // melodi bir dakika boyunca tekrar tekrar calmaz.
  // "When it is 07:30" - true only ONCE in that minute, so the melody does not
  // repeat for a whole minute.
  if (iotbot.ntpTimeReached(kAlarmHour, kAlarmMinute)) {
    iotbot.serialWrite("ALARM!");
    iotbot.lcdWriteMid(turkish ? "GUNAYDIN!" : "GOOD MORNING!", "07:30", "", "");
    iotbot.buzzerPlayMelody(5);
    iotbot.lcdClear();
  }

  // "Saat 22:00 ile 06:00 arasinda ise" / "if the time is between 22:00 and 06:00"
  bool night = iotbot.ntpTimeIsBetween(kNightStartHour, kNightStartMinute, kNightEndHour, kNightEndMinute);
  iotbot.relayWrite(night);

  if (now - lastDrawMs >= 250) {
    lastDrawMs = now;
    char text[24];
    snprintf(text, sizeof(text), "      %s", iotbot.ntpGetTimeString().c_str());
    showRow(0, text);
    snprintf(text, sizeof(text), "     %s", iotbot.ntpGetDateString().c_str());
    showRow(1, text);
    int wd = iotbot.ntpGetWeekday();
    showRow(2, wd > 0 ? (turkish ? kDaysTr[wd] : kDaysEn[wd]) : (turkish ? "Saat bekleniyor..." : "Waiting for time..."));
    // "L" = gece lambasi (role) / "L" = night lamp (relay)
    snprintf(text, sizeof(text), "Alarm %02d:%02d L:%s",
             kAlarmHour, kAlarmMinute, night ? (turkish ? "ACIK" : "ON") : (turkish ? "KAPALI" : "OFF"));
    showRow(3, text);
  }
  delay(10);
}
