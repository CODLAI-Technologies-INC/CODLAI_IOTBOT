/*
 * TR: WİKİPEDİA'DAN BİLGİ - İnternetten konu özeti
 *  - IOTBOT WiFi'ye bağlanır ve Wikipedia'dan "Robot" konusunun özetini çeker.
 *    Özet Seri Monitör'e tam olarak, LCD'ye sayfa sayfa (3 satır x 20 karakter) yazılır.
 *  - B3 butonu: LCD'de özetin SONRAKİ sayfası.
 *  - Wikipedia dili kartın diliyle aynıdır (Türkçe -> tr.wikipedia, English -> en.wikipedia).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help                -> komut listesi
 *      ara <konu> / search <topic>  -> başka bir konu ara (ör. "ara Mustafa Kemal Atatürk")
 *      ozet   / summary             -> son özeti tekrar yazdır
 *      durum  / status              -> WiFi durumu, konu, dil
 *      dil    / lang                -> dili değiştir (Wikipedia dili de değişir, konu yeniden aranır)
 *  - Not: İstek birkaç saniye sürebilir; o sırada kart bekler.
 *
 * EN: INFO FROM WIKIPEDIA - Topic summary from the internet
 *  - The IOTBOT joins WiFi and fetches the summary of the "Robot" topic from Wikipedia.
 *    The full summary goes to the Serial Monitor, and to the LCD page by page (3 rows x
 *    20 characters).
 *  - B3 button: the NEXT page of the summary on the LCD.
 *  - The Wikipedia language follows the board's language (Turkish -> tr.wikipedia,
 *    English -> en.wikipedia).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim              -> command list
 *      search <topic> / ara <konu>  -> look up another topic (e.g. "search Ada Lovelace")
 *      summary / ozet               -> print the last summary again
 *      status / durum               -> WiFi state, topic, language
 *      lang   / dil                 -> switch language (the Wikipedia language too, topic searched again)
 *  - Note: a request can take a few seconds; the board waits meanwhile.
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. / NO extra module needed.
 */
#define USE_WIKIPEDIA
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// WiFi bilgileri / WiFi credentials
const char *ssid = "YOUR_SSID";
const char *password = "YOUR_PASSWORD";

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

String query = "Robot"; // Aranan konu / topic
String summary = "";    // Son özet / last summary
int page = 0;           // LCD sayfası (60 karakter) / LCD page (60 characters)
bool lastB3 = false;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
String rawLine; // Komutun orijinal hali (konu için) / original command text (for the topic)
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "ÖZET" -> "ozet"
// Lower-cases and simplifies Turkish letters: "ÖZET" -> "ozet"
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

// LCD'ye özetin bir sayfasını yazar (başlık + 3 satır) / writes one page of the summary to the LCD (title + 3 rows)
void showPage() {
  if (utf8Slice(summary, page * 60, 1).length() == 0) page = 0; // Sona gelince başa dön / back to the start at the end
  char title[41];
  snprintf(title, sizeof(title), L("Özet s.%d  B3:sonraki", "Summary p.%d B3:next"), page + 1);
  lcdRow(0, title);
  for (int r = 0; r < 3; r++) lcdRow(r + 1, utf8Slice(summary, page * 60 + r * 20, 20).c_str());
}

void printHelp() {
  iotbot.serialWrite(L("---- WİKİPEDİA - Komutlar ----", "---- WIKIPEDIA - Commands ----"));
  iotbot.serialWrite(L("  yardim     : bu liste", "  help       : this list"));
  iotbot.serialWrite(L("  ara <konu> : başka bir konu ara", "  search <topic> : look up another topic"));
  iotbot.serialWrite(L("  ozet       : özeti tekrar yazdır", "  summary    : print the summary again"));
  iotbot.serialWrite(L("  durum      : WiFi, konu, dil", "  status     : WiFi, topic, language"));
  iotbot.serialWrite(L("  dil        : English'e geç (en.wikipedia)", "  lang       : switch to Turkish (tr.wikipedia)"));
  iotbot.serialWrite(L("  B3 butonu  : LCD'de sonraki sayfa", "  B3 button  : next page on the LCD"));
}

void search() {
  if (!wifiOk()) {
    iotbot.serialWrite(L("WiFi bağlı değil, arama yapılamadı.", "WiFi not connected, cannot search."));
    return;
  }
  String shown = String(L("Aranıyor: ", "Searching: ")) + query;
  iotbot.lcdShowLoading(utf8Slice(shown, 0, 20)); // ~2,4 sn animasyon / ~2.4 s animation
  iotbot.serialWrite(shown + "  (" + L("tr", "en") + ".wikipedia.org)");

  // Wikipedia'dan özet al ("tr" Türkçe, "en" İngilizce) / get the summary ("tr" Turkish, "en" English)
  String result = iotbot.getWikipedia(query, L("tr", "en"));
  result.replace("\n", " ");
  if (result.length() == 0 || result == "No Summary Found" || result.startsWith("Error")) {
    summary = L("Bu konuda özet bulunamadı.", "No summary found for this topic.");
    iotbot.buzzerPlayTone(400, 200);
  } else {
    summary = result;
    iotbot.buzzerPlayTone(2000, 150);
  }
  iotbot.serialWrite(L("Özet:", "Summary:"));
  iotbot.serialWrite(summary);
  page = 0;
  showPage();
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
      query = topic;
      search();
    }
  } else if (word == "ozet" || word == "summary") {
    iotbot.serialWrite(summary);
  } else if (word == "durum" || word == "status") {
    iotbot.serialWrite(String("WiFi: ") + (wifiOk() ? String(L("bağlı, IP ", "connected, IP ")) + iotbot.wifiGetIPAddress() : String(L("YOK", "NONE"))));
    iotbot.serialWrite(String(L("Konu: ", "Topic: ")) + query + L("   Wikipedia dili: tr", "   Wikipedia language: en"));
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe (tr.wikipedia)", "Language: English (en.wikipedia)"));
    printHelp();
    search(); // Aynı konuyu yeni dilde ara / look up the same topic in the new language
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);

  iotbot.lcdShowLoading(L("WiFi bağlanıyor", "Connecting WiFi"));
  iotbot.buzzerPlayTone(1000, 200);

  iotbot.wifiStartAndConnect(ssid, password); // WiFi bağlantısı / WiFi connection

  if (wifiOk()) {
    iotbot.lcdShowStatus(L("WiFi Durumu", "WiFi Status"), L("Bağlandı", "Connected"), true);
    iotbot.buzzerPlayTone(1500, 200);
    delay(1000);
    search();
  } else {
    iotbot.lcdShowStatus(L("WiFi Durumu", "WiFi Status"), L("Bağlanamadı", "Not connected"), false);
    iotbot.serialWrite(L("WiFi bağlantısı yok! SSID/şifreyi kontrol edin.", "No WiFi connection! Check SSID/password."));
  }
  printHelp();
}

void loop() {
  // B3 = sonraki sayfa (sadece basıldığı an) / B3 = next page (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3 && summary.length() > 0) {
    page++;
    showPage();
  }
  lastB3 = b3;

  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
  delay(10);
}
