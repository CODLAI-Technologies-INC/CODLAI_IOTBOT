/*
 * TR: NTP İLE İNTERNET SAATİ - Gelişmiş örnek
 *  - IOTBOT WiFi'ye bağlanır, saati internetten (NTP) çeker ve LCD'de tarih/saat ile
 *    "epoch" değerini (1 Ocak 1970'ten beri geçen saniye) gösterir.
 *  - ntpBegin() WiFi bağlantısından SONRA çağrılmalıdır.
 *  - B3 butonu: saati hemen yeniden eşitle (ntpUpdate).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help                -> komut listesi
 *      saat   / time                -> tarih, saat ve epoch'u yazdır
 *      guncelle / update            -> saati şimdi yeniden eşitle
 *      dilim 3 / timezone 3         -> saat dilimi (UTC+3 = Türkiye; -12 ... +14)
 *      durum  / status              -> WiFi ve NTP durumu
 *      dil    / lang                -> dili değiştir (Türkçe <-> English)
 *
 * EN: INTERNET TIME WITH NTP - Advanced example
 *  - The IOTBOT connects to WiFi, gets the time from the internet (NTP) and shows the
 *    date/time and the "epoch" value (seconds since 1 January 1970) on the LCD.
 *  - Call ntpBegin() AFTER connecting to WiFi.
 *  - B3 button: re-sync the time right now (ntpUpdate).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim              -> command list
 *      time   / saat                -> print the date, time and epoch
 *      update / guncelle            -> re-sync the time now
 *      timezone 3 / dilim 3         -> time zone (UTC+3 = Turkey; -12 ... +14)
 *      status / durum               -> WiFi and NTP state
 *      lang   / dil                 -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. Aşağıya WiFi adınızı ve şifrenizi yazın.
 * NO extra module needed. Fill in your WiFi name and password below.
 */

#define USE_WIFI
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// WiFi bilgileri / WiFi credentials
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

int timezoneHours = 3; // Türkiye UTC+3, yaz saati yok / Turkey is UTC+3, no daylight saving
uint32_t lastDrawMs = 0;
bool lastB3 = false;

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

bool wifiOk() { return WiFi.status() == WL_CONNECTED; }

void printHelp() {
  iotbot.serialWrite(L("---- NTP SAATİ - Komutlar ----", "---- NTP TIME - Commands ----"));
  iotbot.serialWrite(L("  yardim    : bu liste", "  help      : this list"));
  iotbot.serialWrite(L("  saat      : tarih, saat, epoch", "  time      : date, time, epoch"));
  iotbot.serialWrite(L("  guncelle  : saati şimdi eşitle", "  update    : sync the time now"));
  iotbot.serialWrite(L("  dilim 3   : saat dilimi (UTC+3)", "  timezone 3: time zone (UTC+3)"));
  iotbot.serialWrite(L("  durum     : WiFi ve NTP durumu", "  status    : WiFi and NTP state"));
  iotbot.serialWrite(L("  dil       : English'e geç", "  lang      : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu : saati şimdi eşitle", "  B3 button : sync the time now"));
}

void printTime() {
  if (!iotbot.ntpIsTimeValid()) {
    iotbot.serialWrite(L("Saat henüz geçerli değil (NTP bekleniyor).", "Time not valid yet (waiting for NTP)."));
    return;
  }
  iotbot.serialWrite(String(L("Tarih/Saat: ", "Date/Time: ")) + iotbot.ntpGetDateTimeString() + "  (UTC" + (timezoneHours >= 0 ? "+" : "") + timezoneHours + ")");
  iotbot.serialWrite(String("Epoch: ") + (unsigned long)iotbot.ntpGetEpoch());
}

void syncNow() {
  if (!wifiOk()) {
    iotbot.serialWrite(L("WiFi yok, eşitleme yapılamadı.", "No WiFi, cannot sync."));
    return;
  }
  lcdRow(3, L("Eşitleniyor...", "Syncing..."));
  bool ok = iotbot.ntpBegin(timezoneHours); // En fazla 10 sn bekler / waits at most 10 s
  iotbot.serialWrite(ok ? L("[NTP] Eşitlendi", "[NTP] Synced") : L("[NTP] Eşitleme başarısız", "[NTP] Sync failed"));
  if (ok) iotbot.buzzerPlayTone(1800, 60);
  printTime();
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "saat" || word == "time") {
    printTime();
  } else if (word == "guncelle" || word == "update" || word == "sync") {
    syncNow();
  } else if ((word == "dilim" || word == "timezone") && hasValue) {
    timezoneHours = constrain(value, -12, 14);
    iotbot.serialWrite(String(L("Saat dilimi: UTC", "Time zone: UTC")) + (timezoneHours >= 0 ? "+" : "") + timezoneHours);
    syncNow();
  } else if (word == "durum" || word == "status") {
    iotbot.serialWrite(String("WiFi: ") + (wifiOk() ? String(L("bağlı, IP ", "connected, IP ")) + iotbot.wifiGetIPAddress() : String(L("YOK", "NONE"))));
    iotbot.serialWrite(String("NTP: ") + (iotbot.ntpIsTimeValid() ? L("saat geçerli", "time valid") : L("saat geçersiz", "time not valid")));
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.lcdClear();
  lcdRow(0, L("    NTP SAATİ", "    NTP TIME"));
  lcdRow(1, L("WiFi'ye bağlanıyor", "Connecting WiFi"));

  iotbot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS);

  if (!wifiOk()) {
    iotbot.serialWrite(L("[WiFi] Bağlanamadı - SSID/şifreyi kontrol edin.", "[WiFi] Not connected - check SSID/password."));
    lcdRow(1, L("WiFi YOK", "NO WiFi"));
    lcdRow(2, L("Ad/şifreyi kontrol", "Check name/password"));
  } else {
    // Tek satırda kurulum (önerilen) / single-call setup (recommended)
    lcdRow(1, L("Saat alınıyor...", "Getting time..."));
    bool ok = iotbot.ntpBegin(timezoneHours);
    iotbot.serialWrite(ok ? L("[NTP] Eşitlendi", "[NTP] Synced") : L("[NTP] Eşitleme başarısız", "[NTP] Sync failed"));
    printTime();
  }
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // B3 = saati hemen eşitle (sadece basıldığı an) / B3 = sync now (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) syncNow();
  lastB3 = b3;

  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // LCD saniyede bir / LCD once a second
  if (now - lastDrawMs >= 1000 && wifiOk()) {
    lastDrawMs = now;
    char line[41];
    lcdRow(0, L("    NTP SAATİ", "    NTP TIME"));
    if (iotbot.ntpIsTimeValid()) {
      snprintf(line, sizeof(line), "%s  %s", iotbot.ntpGetDateString().c_str(), iotbot.ntpGetTimeString().c_str());
      lcdRow(1, line);
      snprintf(line, sizeof(line), "Epoch: %lu", (unsigned long)iotbot.ntpGetEpoch());
      lcdRow(2, line);
    } else {
      lcdRow(1, L("Saat bekleniyor...", "Waiting for time..."));
      lcdRow(2, "");
    }
    snprintf(line, sizeof(line), L("UTC%+d  B3:eşitle", "UTC%+d  B3:sync"), timezoneHours);
    lcdRow(3, line);
  }
}
