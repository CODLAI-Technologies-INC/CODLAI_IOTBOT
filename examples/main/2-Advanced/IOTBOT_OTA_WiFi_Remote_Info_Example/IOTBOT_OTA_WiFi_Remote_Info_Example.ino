/*
 * TR: OTA + WiFi + NTP + İNTERNETTEN BİLGİ
 *  - IOTBOT WiFi'ye bağlanır, OTA'yı (kablosuz kod yükleme) başlatır, saati NTP'den
 *    alır ve her dakika Wikipedia'dan bir konunun özetini çeker.
 *  - LCD: IP adresi, OTA durumu, tarih/saat ve özetin bir parçası. B3 butonu özetin
 *    SONRAKİ parçasını gösterir.
 *  - Wikipedia dili kartın diliyle aynıdır (Türkçe -> tr.wikipedia, English -> en.wikipedia).
 *  - OTA ile yükleme: Arduino IDE > Araçlar > Port > "IOTBOT-OTA at <IP>", şifre 1234.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help                 -> komut listesi
 *      ara <konu> / search <topic>   -> Wikipedia'da başka bir konu ara (ör. "ara Ankara")
 *      guncelle / update             -> özeti şimdi yeniden çek
 *      ozet   / summary              -> özetin tamamını yazdır
 *      durum  / status               -> IP, OTA, saat bilgisi
 *      dil    / lang                 -> dili değiştir (Wikipedia dili de değişir)
 *
 * EN: OTA + WiFi + NTP + INFO FROM THE INTERNET
 *  - The IOTBOT joins WiFi, starts OTA (wireless code upload), gets the time from NTP
 *    and fetches a topic summary from Wikipedia every minute.
 *  - LCD: IP address, OTA state, date/time and a part of the summary. The B3 button
 *    shows the NEXT part of the summary.
 *  - The Wikipedia language follows the board's language (Turkish -> tr.wikipedia,
 *    English -> en.wikipedia).
 *  - OTA upload: Arduino IDE > Tools > Port > "IOTBOT-OTA at <IP>", password 1234.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim               -> command list
 *      search <topic> / ara <konu>   -> look up another topic (e.g. "search Ankara")
 *      update / guncelle             -> fetch the summary again now
 *      summary / ozet                -> print the whole summary
 *      status / durum                -> IP, OTA, time information
 *      lang   / dil                  -> switch language (the Wikipedia language too)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. / NO extra module needed.
 * Not / Note: USE_OTA ve USE_WIFI tanımları kütüphaneden ÖNCE yazılmalıdır. Wikipedia
 * isteği birkaç saniye sürebilir; o sırada kart bekler. / USE_OTA and USE_WIFI must be
 * defined BEFORE including the library. A Wikipedia request can take a few seconds;
 * the board waits meanwhile.
 */

#define USE_WIFI
#define USE_OTA
#define USE_WIKIPEDIA
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

const char *WIFI_SSID = "YOUR_WIFI_SSID";
const char *WIFI_PASS = "YOUR_WIFI_PASSWORD";

const char *OTA_HOST = "IOTBOT-OTA"; // Cihaz adı / device name
const char *OTA_PASS = "1234";       // OTA şifresi / OTA password

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

String wikiQuery = "Istanbul";  // Aranan konu / topic
String wikiSummary = "";        // Son özet / last summary
int summaryPage = 0;            // LCD'de gösterilen 20 karakterlik parça / 20-char part shown on the LCD
uint32_t lastUiUpdateMs = 0;
uint32_t lastRemoteFetchMs = 0;
bool fetchNow = true;           // İlk özeti hemen çek / fetch the first summary right away
bool lastB3 = false;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
String rawLine; // Komutun orijinal hali (konu için) / original command text (for the topic)
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
      rawLine = cmdBuffer; rawLine.trim();
      cmd = normalizeCommand(cmdBuffer);
      cmdBuffer = "";
      return true;
    }
    if (cmdBuffer.length() < 80) cmdBuffer += c;
  }
  // "Satır sonu yok" seçiliyse: 150 ms sessizlikten sonra komutu kabul et.
  // "No line ending" selected: accept the command after 150 ms of silence.
  if (cmdBuffer.length() > 0 && millis() - lastCharMs > 150) {
    rawLine = cmdBuffer; rawLine.trim();
    cmd = normalizeCommand(cmdBuffer);
    cmdBuffer = "";
    return true;
  }
  return false;
}

// Komuttan sonraki metin (orijinal harflerle) / the text after the command word (original letters)
String argText() {
  int space = rawLine.indexOf(' ');
  if (space < 0) return "";
  String t = rawLine.substring(space + 1);
  t.trim();
  return t;
}

// ---------------------------------------------------------------------------
// Yardımcılar / Helpers
// ---------------------------------------------------------------------------
// Not: getWikipedia() konu adını kendisi adrese uygun hale getirir (boşluk -> "_",
// Türkçe harf -> %XX); konuyu olduğu gibi (ör. "Mustafa Kemal Atatürk") verin.
// Note: getWikipedia() makes the topic URL-safe itself (space -> "_", Turkish letters
// -> %XX); pass the topic as it is (e.g. "Ada Lovelace").

// UTF-8 metinden "startChar"dan başlayan "count" karakteri alır (Türkçe harf 2 bayt olsa da 1 sayılır)
// Takes "count" characters starting at "startChar" from a UTF-8 text (a Turkish letter counts as 1 even though it is 2 bytes)
String utf8Slice(const String &s, int startChar, int count) {
  int ch = 0, startByte = -1, endByte = s.length();
  for (unsigned int i = 0; i < s.length();) {
    if (ch == startChar && startByte < 0) startByte = i;
    if (ch == startChar + count) { endByte = i; break; }
    unsigned char c = s[i];
    i += (c >= 0xF0) ? 4 : (c >= 0xE0) ? 3 : (c >= 0xC0) ? 2 : 1;
    ch++;
  }
  if (startByte < 0) return "";
  return s.substring(startByte, endByte);
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

bool wifiOk() { return WiFi.status() == WL_CONNECTED; }

void drawSummaryRow() {
  String part = utf8Slice(wikiSummary, summaryPage * 20, 20);
  if (part.length() == 0 && summaryPage > 0) { // Sona gelince başa dön / back to the start at the end
    summaryPage = 0;
    part = utf8Slice(wikiSummary, 0, 20);
  }
  lcdRow(3, part.length() ? part.c_str() : L("Özet bekleniyor...", "Waiting for summary"));
}

void printHelp() {
  iotbot.serialWrite(L("---- OTA + WiFi + BİLGİ - Komutlar ----", "---- OTA + WiFi + INFO - Commands ----"));
  iotbot.serialWrite(L("  yardim      : bu liste", "  help        : this list"));
  iotbot.serialWrite(L("  ara <konu>  : Wikipedia'da ara", "  search <topic> : look up on Wikipedia"));
  iotbot.serialWrite(L("  guncelle    : özeti şimdi çek", "  update      : fetch the summary now"));
  iotbot.serialWrite(L("  ozet        : özetin tamamını yazdır", "  summary     : print the whole summary"));
  iotbot.serialWrite(L("  durum       : IP, OTA, saat", "  status      : IP, OTA, time"));
  iotbot.serialWrite(L("  dil         : English'e geç (Wikipedia da)", "  lang        : switch to Turkish (Wikipedia too)"));
  iotbot.serialWrite(L("  B3 butonu   : özetin sonraki parçası", "  B3 button   : next part of the summary"));
}

void fetchSummary() {
  lastRemoteFetchMs = millis();
  fetchNow = false;
  if (!wifiOk()) {
    wikiSummary = L("WiFi yok", "No WiFi");
    return;
  }
  lcdRow(3, L("Wikipedia...", "Wikipedia..."));
  String result = iotbot.getWikipedia(wikiQuery, L("tr", "en"));
  result.replace("\n", " ");
  if (result.length() == 0 || result == "No Summary Found" || result.startsWith("Error")) {
    iotbot.serialWrite(String(L("Bilgi alınamadı: ", "Could not get info: ")) + result);
    wikiSummary = L("Bilgi alınamadı", "No info");
  } else {
    wikiSummary = result;
    iotbot.serialWrite(String(L("Wikipedia (", "Wikipedia (")) + wikiQuery + "): " + utf8Slice(wikiSummary, 0, 120) + "...");
  }
  summaryPage = 0;
  drawSummaryRow();
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "ara" || word == "search") {
    String topic = argText();
    if (topic.length() == 0) {
      iotbot.serialWrite(L("Kullanım: ara Ankara", "Usage: search Ankara"));
    } else {
      wikiQuery = topic;
      iotbot.serialWrite(String(L("Aranıyor: ", "Searching: ")) + wikiQuery);
      fetchSummary();
    }
  } else if (word == "guncelle" || word == "update") {
    fetchSummary();
  } else if (word == "ozet" || word == "summary") {
    iotbot.serialWrite(wikiSummary);
  } else if (word == "durum" || word == "status") {
    iotbot.serialWrite(String("WiFi: ") + (wifiOk() ? String(L("bağlı, IP ", "connected, IP ")) + iotbot.wifiGetIPAddress() : String(L("YOK", "NONE"))));
    iotbot.serialWrite(String("OTA: ") + OTA_HOST + L("  (şifre ", "  (password ") + OTA_PASS + ")");
    String t = iotbot.ntpGetDateTimeString();
    iotbot.serialWrite(String(L("Saat: ", "Time: ")) + (t.length() ? t : String(L("NTP bekleniyor", "waiting for NTP"))));
    iotbot.serialWrite(String(L("Konu: ", "Topic: ")) + wikiQuery + L("  (dil: ", "  (language: ") + L("tr)", "en)"));
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe (Wikipedia: tr)", "Language: English (Wikipedia: en)"));
    lastUiUpdateMs = 0;
    printHelp();
    fetchNow = true; // Özeti yeni dilde çek / fetch the summary in the new language
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.lcdClear();
  lcdRow(0, L("WiFi'ye bağlanıyor", "Connecting WiFi"));

  iotbot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS);
  iotbot.otaBegin(OTA_HOST, OTA_PASS, 3232); // OTA'yı başlat / start OTA

  if (wifiOk()) {
    lcdRow(1, L("Saat alınıyor...", "Getting time..."));
    iotbot.ntpBegin(3); // Türkiye için UTC+3 / UTC+3 for Turkey
  } else {
    iotbot.serialWrite(L("WiFi bağlantısı yok! SSID/şifreyi kontrol edin.", "No WiFi connection! Check SSID/password."));
  }
  iotbot.lcdClear();
  printHelp();
}

void loop() {
  iotbot.otaHandle(); // OTA - SÜREKLİ çağrılmalı / OTA - must be called ALL the time

  const uint32_t nowMs = millis();

  // B3 = özetin sonraki parçası / B3 = next part of the summary
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) {
    summaryPage++;
    drawSummaryRow();
  }
  lastB3 = b3;

  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // LCD 1 saniyede bir / LCD once a second
  if (nowMs - lastUiUpdateMs >= 1000) {
    lastUiUpdateMs = nowMs;
    char line[41];
    if (wifiOk()) snprintf(line, sizeof(line), "IP: %s", iotbot.wifiGetIPAddress().c_str());
    else snprintf(line, sizeof(line), "%s", L("WiFi YOK", "NO WiFi"));
    lcdRow(0, line);
    lcdRow(1, wifiOk() ? L("OTA hazır B3:sonraki", "OTA ready  B3:next") : L("OTA çalışmaz", "OTA not working"));
    String timeStr = iotbot.ntpGetDateTimeString(); // "2026-10-06 14:05:09" (19 karakter / chars)
    lcdRow(2, timeStr.length() ? timeStr.c_str() : L("NTP bekleniyor", "Waiting for NTP"));
  }

  // Her dakika (ya da istenince) özeti yenile / refresh the summary every minute (or on request)
  if (fetchNow || nowMs - lastRemoteFetchMs >= 60000) fetchSummary();

  delay(10);
}
