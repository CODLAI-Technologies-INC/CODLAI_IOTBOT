/*
 * TR: İNTERNET SAATİ - Saat, Alarm ve Gece Lambası
 *  - IOTBOT WiFi'ye bağlanıp saati internetten (NTP) çeker ve LCD'de saat, tarih ve
 *    günü gösterir.
 *  - "Saat 07:30 OLUNCA" alarm melodisi BİR KEZ çalar (ntpTimeReached).
 *  - Gece lambası (karttaki röle) OTOMATİK modda "saat 22:00 ile 06:00 ARASINDA İSE"
 *    açık kalır (ntpTimeIsBetween - gece yarısını aşan aralık da olur).
 *  - B3 butonu lambayı OTOMATİK <-> MANUEL yapar. MANUEL modda lambayı joystick
 *    butonu (veya "lamba ac/kapat" komutu) açıp kapatır.
 *  - Saat her 6 saatte bir kendiliğinden, encoder butonuna basınca da hemen GÜNCELLENİR
 *    (ntpUpdate).
 *  - Bu fonksiyonlar editördeki "İnternet saatini kullan / güncelle", "saat ... ise",
 *    "saat ... olunca" bloklarının karşılığıdır.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help                     -> komut listesi
 *      saat   / time                     -> saati yazdır
 *      alarm 07:30                       -> alarm saatini ayarla
 *      alarm kapat / alarm off           -> alarmı kapat (alarm ac / alarm on: aç)
 *      test                              -> alarm melodisini şimdi çal
 *      gece 22:00 06:00 / night 22:00 06:00 -> gece lambası aralığı
 *      oto    / auto                     -> lamba otomatik (saate göre)
 *      manuel / manual                   -> lamba manuel
 *      lamba ac / lamp on, lamba kapat / lamp off -> lambayı aç/kapat (manuele geçer)
 *      guncelle / update                 -> saati şimdi güncelle
 *      durum  / status                   -> ayarlar ve durum
 *      dil    / lang                     -> dili değiştir (Türkçe <-> English)
 *  - Not: Alarm melodisi çalarken (yaklaşık 25 sn) kart başka işe bakmaz.
 *
 * EN: INTERNET TIME - Clock, Alarm and Night Lamp
 *  - The IOTBOT connects to WiFi, gets the time from the internet (NTP) and shows the
 *    time, date and weekday on the LCD.
 *  - "WHEN it is 07:30" the alarm melody plays ONCE (ntpTimeReached).
 *  - In AUTO mode the night lamp (the onboard relay) stays on "IF the time is BETWEEN
 *    22:00 and 06:00" (ntpTimeIsBetween - ranges crossing midnight work too).
 *  - The B3 button switches the lamp AUTO <-> MANUAL. In MANUAL mode the joystick
 *    button (or the "lamp on/off" command) switches the lamp.
 *  - The time is UPDATED by itself every 6 hours, and right away when you press the
 *    encoder button (ntpUpdate).
 *  - These functions are what the editor's "use / update internet time", "if time is
 *    ...", "when time is ..." blocks call.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim                   -> command list
 *      time   / saat                     -> print the time
 *      alarm 07:30                       -> set the alarm time
 *      alarm off / alarm kapat           -> turn the alarm off (alarm on / alarm ac: on)
 *      test                              -> play the alarm melody now
 *      night 22:00 06:00 / gece 22:00 06:00 -> night lamp time range
 *      auto   / oto                      -> lamp automatic (by the clock)
 *      manual / manuel                   -> lamp manual
 *      lamp on / lamba ac, lamp off / lamba kapat -> switch the lamp (goes to manual)
 *      update / guncelle                 -> update the time now
 *      status / durum                    -> settings and state
 *      lang   / dil                      -> switch language (Turkish <-> English)
 *  - Note: while the alarm melody plays (about 25 s) the board does nothing else.
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. Aşağıya WiFi adınızı ve şifrenizi yazın.
 * NO extra module needed. Fill in your WiFi name and password below.
 */

#define USE_WIFI
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

const int kTimezoneHours = 3;                                  // Türkiye UTC+3 / Turkey UTC+3
const uint32_t kAutoUpdateMs = 6UL * 60UL * 60UL * 1000UL;     // 6 saatte bir güncelle / update every 6 hours

int alarmHour = 7, alarmMinute = 30;                           // Alarm saati / alarm time
bool alarmEnabled = true;
int nightStartHour = 22, nightStartMinute = 0;                 // Gece lambası başlangıç / night lamp start
int nightEndHour = 6, nightEndMinute = 0;                      // Gece lambası bitiş / night lamp end

const char *kDaysTr[] = {"", "Pazartesi", "Salı", "Çarşamba", "Perşembe", "Cuma", "Cumartesi", "Pazar"};
const char *kDaysEn[] = {"", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};

bool manualLamp = false;   // false = OTOMATİK (saate göre), true = MANUEL / false = AUTO (by the clock), true = MANUAL
bool lampOn = false;
uint32_t lastUpdateMs = 0, lastDrawMs = 0;
bool lastB3 = false, lastJoy = false, lastEnc = false;
bool wifiFailed = false;   // WiFi yoksa uyarı ekranı kalsın / keep the warning screen when there is no WiFi

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "GÜNCELLE" -> "guncelle"
// Lower-cases and simplifies Turkish letters: "GÜNCELLE" -> "guncelle"
String normalizeCommand(String s) {
  s.trim();
  s.replace("İ", "i"); s.replace("I", "i"); s.replace("ı", "i");
  s.replace("Ş", "s"); s.replace("ş", "s");
  s.replace("Ğ", "g"); s.replace("ğ", "g");
  s.replace("Ü", "u"); s.replace("ü", "u");
  s.replace("Ö", "o"); s.replace("ö", "o");
  s.replace("Ç", "c"); s.replace("ç", "c");
  s.toLowerCase();
  return s;
}

bool readCommand(String &cmd) {
  while (iotbot.serialAvailable() > 0) {
    char c = Serial.read();
    lastCharMs = millis();
    if (c == '\n' || c == '\r') {
      if (cmdBuffer.length() == 0) continue;
      cmd = normalizeCommand(cmdBuffer);
      cmdBuffer = "";
      return true;
    }
    if (cmdBuffer.length() < 40) cmdBuffer += c;
  }
  // "Satır sonu yok" seçiliyse: 150 ms sessizlikten sonra komutu kabul et.
  // "No line ending" selected: accept the command after 150 ms of silence.
  if (cmdBuffer.length() > 0 && millis() - lastCharMs > 150) {
    cmd = normalizeCommand(cmdBuffer);
    cmdBuffer = "";
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- İNTERNET SAATİ - Komutlar ----", "---- INTERNET CLOCK - Commands ----"));
  iotbot.serialWrite(L("  yardim             : bu liste", "  help               : this list"));
  iotbot.serialWrite(L("  saat               : saati yazdır", "  time               : print the time"));
  iotbot.serialWrite(L("  alarm 07:30        : alarm saati", "  alarm 07:30        : alarm time"));
  iotbot.serialWrite(L("  alarm kapat / ac   : alarmı kapat / aç", "  alarm off / on     : alarm off / on"));
  iotbot.serialWrite(L("  test               : alarm melodisini çal", "  test               : play the alarm melody"));
  iotbot.serialWrite(L("  gece 22:00 06:00   : gece lambası aralığı", "  night 22:00 06:00  : night lamp range"));
  iotbot.serialWrite(L("  oto / manuel       : lamba modu", "  auto / manual      : lamp mode"));
  iotbot.serialWrite(L("  lamba ac / kapat   : lambayı aç / kapat", "  lamp on / off      : lamp on / off"));
  iotbot.serialWrite(L("  guncelle           : saati şimdi güncelle", "  update             : update the time now"));
  iotbot.serialWrite(L("  durum, dil", "  status, lang"));
  iotbot.serialWrite(L("  B3: lamba OTO <-> MANUEL, Joystick btn: lamba aç/kapa (manuel), Encoder btn: saati güncelle",
                       "  B3: lamp AUTO <-> MANUAL, Joystick btn: lamp on/off (manual), Encoder btn: update time"));
}

void updateTime() {
  lcdRow(3, L("Saat güncelleniyor..", "Updating time..."));
  bool ok = iotbot.ntpUpdate(); // "İnternet saatini güncelle" / "update internet time"
  iotbot.serialWrite(ok ? L("Saat güncellendi.", "Time updated.") : L("Güncelleme başarısız.", "Update failed."));
  lastUpdateMs = millis();
  lastDrawMs = 0;
}

void setLampMode(bool manual) {
  manualLamp = manual;
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  iotbot.serialWrite(manual ? L(">> Lamba MANUEL: joystick butonu veya 'lamba ac/kapat'.", ">> Lamp MANUAL: joystick button or 'lamp on/off'.")
                            : L(">> Lamba OTOMATİK: gece aralığında açık.", ">> Lamp AUTO: on during the night range."));
  lastDrawMs = 0;
}

void setLamp(bool on) {
  lampOn = on;
  iotbot.relayWrite(on);
  lastDrawMs = 0;
}

void playAlarm() {
  iotbot.serialWrite("ALARM!");
  lcdRow(0, L("     GÜNAYDIN!", "   GOOD MORNING!"));
  iotbot.buzzerPlayMelody(5); // "Daha Dün Annemizin" (bitene kadar bekler / waits until it ends)
  lastDrawMs = 0;
}

void printStatus() {
  char line[100];
  snprintf(line, sizeof(line), L("Alarm: %02d:%02d (%s)   Gece: %02d:%02d - %02d:%02d", "Alarm: %02d:%02d (%s)   Night: %02d:%02d - %02d:%02d"),
           alarmHour, alarmMinute, alarmEnabled ? L("açık", "on") : L("kapalı", "off"),
           nightStartHour, nightStartMinute, nightEndHour, nightEndMinute);
  iotbot.serialWrite(line);
  snprintf(line, sizeof(line), L("Lamba: %s (%s)   WiFi: %s", "Lamp: %s (%s)   WiFi: %s"),
           lampOn ? L("AÇIK", "ON") : L("KAPALI", "OFF"), manualLamp ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO"),
           WiFi.status() == WL_CONNECTED ? L("bağlı", "connected") : L("YOK", "NONE"));
  iotbot.serialWrite(line);
}

// "07:30" gibi bir saati okur / parses a time like "07:30"
bool parseTime(const String &text, int &h, int &m) {
  int hh = -1, mm = -1;
  if (sscanf(text.c_str(), "%d:%d", &hh, &mm) != 2 || hh < 0 || hh > 23 || mm < 0 || mm > 59) return false;
  h = hh;
  m = mm;
  return true;
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  String arg = (space < 0) ? "" : cmd.substring(space + 1);
  arg.trim();

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "saat" || word == "time") {
    iotbot.serialWrite(iotbot.ntpGetDateString() + "  " + iotbot.ntpGetTimeString());
  } else if (word == "alarm") {
    if (arg == "kapat" || arg == "off") {
      alarmEnabled = false;
      iotbot.serialWrite(L("Alarm kapatıldı.", "Alarm turned off."));
    } else if (arg == "ac" || arg == "on") {
      alarmEnabled = true;
      iotbot.serialWrite(L("Alarm açıldı.", "Alarm turned on."));
    } else if (parseTime(arg, alarmHour, alarmMinute)) {
      alarmEnabled = true;
      char line[41];
      snprintf(line, sizeof(line), L("Alarm: %02d:%02d", "Alarm: %02d:%02d"), alarmHour, alarmMinute);
      iotbot.serialWrite(line);
    } else {
      iotbot.serialWrite(L("Kullanım: alarm 07:30  /  alarm kapat", "Usage: alarm 07:30  /  alarm off"));
    }
    lastDrawMs = 0;
  } else if (word == "test") {
    playAlarm();
  } else if (word == "gece" || word == "night") {
    int sp = arg.indexOf(' ');
    int sh, sm, eh, em;
    if (sp > 0 && parseTime(arg.substring(0, sp), sh, sm) && parseTime(arg.substring(sp + 1), eh, em)) {
      nightStartHour = sh; nightStartMinute = sm; nightEndHour = eh; nightEndMinute = em;
      printStatus();
    } else {
      iotbot.serialWrite(L("Kullanım: gece 22:00 06:00", "Usage: night 22:00 06:00"));
    }
  } else if (word == "oto" || word == "otomatik" || word == "auto") {
    setLampMode(false);
  } else if (word == "manuel" || word == "manual") {
    setLampMode(true);
  } else if ((word == "lamba" || word == "lamp") && (arg == "ac" || arg == "on" || arg == "kapat" || arg == "off")) {
    if (!manualLamp) setLampMode(true);
    setLamp(arg == "ac" || arg == "on");
    iotbot.serialWrite(lampOn ? L("Lamba açıldı.", "Lamp on.") : L("Lamba kapatıldı.", "Lamp off."));
  } else if (word == "guncelle" || word == "update") {
    updateTime();
  } else if (word == "durum" || word == "status") {
    printStatus();
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    lastDrawMs = 0;
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  setLamp(false);
  iotbot.lcdClear();
  lcdRow(0, L("İNTERNET SAATİ", "INTERNET CLOCK"));
  lcdRow(1, L("WiFi'ye bağlanıyor", "Connecting WiFi"));
  iotbot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS);
  if (!iotbot.wifiConnectionControl()) {
    wifiFailed = true;
    lcdRow(0, L("WiFi YOK", "NO WiFi"));
    lcdRow(1, L("Ad/şifreyi kontrol", "Check name/password"));
    lcdRow(2, L("edip tekrar yükleyin", "and upload again"));
    iotbot.serialWrite(L("WiFi bağlantısı yok! Ad/şifreyi kontrol edip tekrar yükleyin.", "No WiFi! Check the name/password and upload again."));
  } else {
    // "İnternet saatini kullan" / "use internet time"
    iotbot.ntpBegin(kTimezoneHours);
  }
  lastUpdateMs = millis();
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) Butonlar (sadece basıldığı an) / buttons (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) setLampMode(!manualLamp);
  lastB3 = b3;
  bool joy = !iotbot.joystickButtonRead(); // LOW = basılı / LOW = pressed
  if (joy && !lastJoy && manualLamp) {
    setLamp(!lampOn);
    iotbot.serialWrite(lampOn ? L("Lamba açıldı.", "Lamp on.") : L("Lamba kapatıldı.", "Lamp off."));
  }
  lastJoy = joy;
  bool enc = !iotbot.encoderButtonRead(); // LOW = basılı / LOW = pressed
  if (enc && !lastEnc) updateTime();
  lastEnc = enc;

  // 2) Seri komutlar / serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  if (now - lastUpdateMs >= kAutoUpdateMs) updateTime();

  // 3) "Saat 07:30 olunca" - o dakikada SADECE BİR KEZ true döner, bu yüzden melodi
  //    bir dakika boyunca tekrar tekrar çalmaz.
  // 3) "When it is 07:30" - true only ONCE in that minute, so the melody does not
  //    repeat for a whole minute.
  if (iotbot.ntpTimeReached(alarmHour, alarmMinute) && alarmEnabled) playAlarm();

  // 4) "Saat 22:00 ile 06:00 arasında ise" -> lamba (sadece OTOMATİK modda)
  // 4) "If the time is between 22:00 and 06:00" -> lamp (only in AUTO mode)
  if (!manualLamp) {
    bool night = iotbot.ntpTimeIsBetween(nightStartHour, nightStartMinute, nightEndHour, nightEndMinute);
    if (night != lampOn) setLamp(night);
  }

  // 5) LCD (250 ms'de bir, titremesiz) / LCD (every 250 ms, no flicker)
  if (now - lastDrawMs >= 250 && !(wifiFailed && !iotbot.ntpIsTimeValid())) {
    lastDrawMs = now;
    char text[41];
    snprintf(text, sizeof(text), "      %s", iotbot.ntpGetTimeString().c_str());
    lcdRow(0, text);
    snprintf(text, sizeof(text), "     %s", iotbot.ntpGetDateString().c_str());
    lcdRow(1, text);
    int wd = iotbot.ntpGetWeekday();
    if (wd > 0) snprintf(text, sizeof(text), "%s  %s", turkish ? kDaysTr[wd] : kDaysEn[wd], manualLamp ? L("MANUEL", "MANUAL") : L("OTO", "AUTO"));
    else snprintf(text, sizeof(text), "%s", L("Saat bekleniyor...", "Waiting for time..."));
    lcdRow(2, text);
    // "L" = gece lambası (röle) / "L" = night lamp (relay)
    if (alarmEnabled) snprintf(text, sizeof(text), "Alarm %02d:%02d L:%s", alarmHour, alarmMinute, lampOn ? L("AÇIK", "ON") : L("KAPALI", "OFF"));
    else snprintf(text, sizeof(text), L("Alarm yok  L:%s", "No alarm  L:%s"), lampOn ? L("AÇIK", "ON") : L("KAPALI", "OFF"));
    lcdRow(3, text);
  }
  delay(10);
}
