/*
 * TR: HAVA DURUMU - İnternetten anlık hava durumu
 *  - IOTBOT WiFi'ye bağlanır ve seçilen şehrin hava durumunu (sıcaklık + durum) LCD'de
 *    ve Seri Monitör'de gösterir. Her 5 dakikada bir kendiliğinden yenilenir.
 *  - API anahtarı GEREKMEZ: anahtar boşsa ücretsiz wttr.in servisi kullanılır. Bir
 *    OpenWeatherMap anahtarınız varsa apiKey'e yazabilirsiniz.
 *  - B3 butonu: hava durumunu hemen yenile.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help                -> komut listesi
 *      sehir Ankara / city Ankara   -> şehri değiştir ve hava durumunu getir
 *      guncelle / update            -> hemen yenile
 *      durum  / status              -> WiFi, şehir, son sonuç
 *      dil    / lang                -> dili değiştir (Türkçe <-> English)
 *  - Not: Servisten gelen durum metni (ör. "Sunny") İngilizcedir. İstek birkaç saniye
 *    sürebilir; o sırada kart bekler.
 *
 * EN: WEATHER - Current weather from the internet
 *  - The IOTBOT joins WiFi and shows the weather (temperature + condition) of the chosen
 *    city on the LCD and the Serial Monitor. It refreshes by itself every 5 minutes.
 *  - NO API key needed: with an empty key the free wttr.in service is used. If you have
 *    an OpenWeatherMap key you can put it into apiKey.
 *  - B3 button: refresh the weather now.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim              -> command list
 *      city Ankara / sehir Ankara   -> change the city and fetch its weather
 *      update / guncelle            -> refresh now
 *      status / durum               -> WiFi, city, last result
 *      lang   / dil                 -> switch language (Turkish <-> English)
 *  - Note: the condition text from the service (e.g. "Sunny") is in English. A request
 *    can take a few seconds; the board waits meanwhile.
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. / NO extra module needed.
 */
#define USE_WEATHER
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// WiFi bilgileri / WiFi credentials
const char *ssid = "YOUR_SSID";
const char *password = "YOUR_PASSWORD";

// OpenWeatherMap API anahtarı. Yoksa boş bırakın, ücretsiz wttr.in kullanılır.
// OpenWeatherMap API key. Leave it empty if you have none; the free wttr.in is used.
String apiKey = "";
String city = "Istanbul";

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

const uint32_t kRefreshMs = 5UL * 60UL * 1000UL; // 5 dakikada bir / every 5 minutes
uint32_t lastFetchMs = 0;
bool fetchNow = true;
String weatherData = "";
bool lastB3 = false;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
String rawLine; // Komutun orijinal hali (şehir adı için) / original command text (for the city name)
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "ŞEHİR" -> "sehir"
// Lower-cases and simplifies Turkish letters: "ŞEHİR" -> "sehir"
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
    if (cmdBuffer.length() < 60) cmdBuffer += c;
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

// Not: getWeather() şehir adındaki boşluk ve Türkçe harfleri kendisi adrese uygun
// hale getirir (%XX); şehri olduğu gibi (ör. "Kahramanmaraş") verin.
// Note: getWeather() makes spaces and Turkish letters in the city name URL-safe
// itself (%XX); pass the city as it is (e.g. "New York").

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

bool wifiOk() { return WiFi.status() == WL_CONNECTED; }

void drawScreen() {
  lcdRow(0, L("    HAVA DURUMU", "   WEATHER REPORT"));
  lcdRow(1, city.c_str());
  // LCD "°" karakterini bilmez; kendi derece işaretini (223) kullanıyoruz
  // The LCD does not know "°"; we use its own degree sign (223)
  String shown = weatherData;
  shown.replace("°", String((char)223));
  if (shown.length() > 20) shown = shown.substring(0, 20);
  lcdRow(2, shown.c_str());
  lcdRow(3, L("B3: yenile", "B3: refresh"));
}

void printHelp() {
  iotbot.serialWrite(L("---- HAVA DURUMU - Komutlar ----", "---- WEATHER - Commands ----"));
  iotbot.serialWrite(L("  yardim       : bu liste", "  help         : this list"));
  iotbot.serialWrite(L("  sehir Ankara : şehri değiştir", "  city Ankara  : change the city"));
  iotbot.serialWrite(L("  guncelle     : hemen yenile", "  update       : refresh now"));
  iotbot.serialWrite(L("  durum        : WiFi, şehir, son sonuç", "  status       : WiFi, city, last result"));
  iotbot.serialWrite(L("  dil          : English'e geç", "  lang         : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu    : hemen yenile", "  B3 button    : refresh now"));
}

void fetchWeather() {
  lastFetchMs = millis();
  fetchNow = false;
  if (!wifiOk()) {
    weatherData = L("WiFi yok", "No WiFi");
    drawScreen();
    return;
  }
  lcdRow(3, L("Alınıyor...", "Fetching..."));
  weatherData = iotbot.getWeather(city, apiKey); // Hava durumunu al / get the weather
  weatherData.trim();
  iotbot.serialWrite(String(L("Hava durumu (", "Weather in ")) + city + L("): ", ": ") + weatherData);
  drawScreen();
  iotbot.buzzerPlayTone(1500, 60);
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "sehir" || word == "city") {
    String name = argText();
    if (name.length() == 0) {
      iotbot.serialWrite(L("Kullanım: sehir Ankara", "Usage: city Ankara"));
    } else {
      city = name;
      fetchWeather();
    }
  } else if (word == "guncelle" || word == "update") {
    fetchWeather();
  } else if (word == "durum" || word == "status") {
    iotbot.serialWrite(String("WiFi: ") + (wifiOk() ? String(L("bağlı, IP ", "connected, IP ")) + iotbot.wifiGetIPAddress() : String(L("YOK", "NONE"))));
    iotbot.serialWrite(String(L("Şehir: ", "City: ")) + city + L("   Son sonuç: ", "   Last result: ") + weatherData +
                       L("   Servis: ", "   Service: ") + (apiKey.length() ? "OpenWeatherMap" : "wttr.in"));
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    drawScreen();
    printHelp();
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
    iotbot.buzzerPlayTone(2000, 300);
  } else {
    iotbot.lcdShowStatus(L("WiFi Durumu", "WiFi Status"), L("Bağlanamadı", "Not connected"), false);
    iotbot.serialWrite(L("WiFi bağlantısı yok! SSID/şifreyi kontrol edin.", "No WiFi connection! Check SSID/password."));
    iotbot.buzzerPlayTone(400, 400);
  }
  delay(1000);
  iotbot.lcdClear();
  printHelp();
}

void loop() {
  // B3 = hemen yenile (sadece basıldığı an) / B3 = refresh now (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) fetchNow = true;
  lastB3 = b3;

  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // Beklemesiz zamanlayıcı: delay(60000) yerine / non-blocking timer instead of delay(60000)
  if (fetchNow || millis() - lastFetchMs >= kRefreshMs) fetchWeather();
  delay(10);
}
