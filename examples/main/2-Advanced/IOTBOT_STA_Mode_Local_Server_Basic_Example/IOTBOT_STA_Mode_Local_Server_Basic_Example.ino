/*
 * TR: EV AĞINDA (STA MODU) WEB SUNUCU - bağlanamazsa kendi ağını (AP) kurar
 *  - IOTBOT aşağıda yazdığınız WiFi ağına bağlanır. Bağlanınca LCD'de IP adresini
 *    gösterir: aynı ağdaki telefon/bilgisayarda http://<IP>/demopage adresini açın.
 *  - 15 saniyede bağlanamazsa kendi ağını kurar: "CODLAI Server" (şifre 12345678),
 *    adres: http://192.168.4.1/demopage
 *  - Sayfadaki butona basınca IOTBOT bip sesi çıkarır ve LCD'ye "Web'den merhaba!" yazar.
 *  - Sayfanın dili kartın diliyle aynıdır (sayfa açılırken /lang adresinden öğrenir).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help    -> komut listesi
 *      durum  / status  -> mod (STA/AP), IP, sinyal gücü
 *      dil    / lang    -> dili değiştir (Türkçe <-> English, web sayfası da değişir)
 *
 * EN: WEB SERVER ON YOUR HOME NETWORK (STA MODE) - falls back to its own network (AP)
 *  - The IOTBOT joins the WiFi network you write below. Once connected the LCD shows
 *    its IP address: open http://<IP>/demopage on a phone/PC on the same network.
 *  - If it cannot connect within 15 seconds it creates its own network:
 *    "CODLAI Server" (password 12345678), address: http://192.168.4.1/demopage
 *  - Pressing the button on the page makes the IOTBOT beep and writes "Hello from
 *    web!" on the LCD.
 *  - The page uses the board's language (it asks /lang when it loads).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim  -> command list
 *      status / durum   -> mode (STA/AP), IP, signal strength
 *      lang   / dil     -> switch language (Turkish <-> English, the web page too)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. / NO extra module needed.
 */
#define USE_SERVER
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// WiFi ayarları: bağlanmak istediğiniz ağın adı ve şifresi
// WiFi settings: name and password of the network you want to join
#define WIFI_SSID "WIFI_SSID"
#define WIFI_PASS "WiFi_PASS"

// Bağlanamazsa kurulacak erişim noktası (AP) / Access point (AP) used as a fallback
#define AP_SSID "CODLAI Server" // AP adı / AP name
#define AP_PASS "12345678"      // AP şifresi (en az 8 karakter) / AP password (at least 8 characters)

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

bool apMode = false;               // true = kendi ağımızı kurduk / we created our own network
volatile int helloRequests = 0;    // Web'den gelen istek sayısı / requests from the web
int helloHandled = 0;

// ---------------------------------------------------------------------------
// Web sayfası (HTML, CSS, JavaScript) / Web page (HTML, CSS, JavaScript)
// serverCreateLocalPage() HTML içindeki iki "%s" yerine SIRAYLA önce Script'i,
// sonra CSS'i yerleştirir. HTML'de başka "%" karakteri kullanmayın.
// serverCreateLocalPage() puts the Script into the first "%s" and the CSS into
// the second "%s" of the HTML. Do not use any other "%" character in the HTML.
// ---------------------------------------------------------------------------
const char WEBPageScript[] PROGMEM = R"rawliteral(
<script>
  var texts = {
    tr: { title: "IOTBOT Web Sayfası", button: "Tıklayın", info: "Butona basınca IOTBOT bip sesi çıkarır." },
    en: { title: "IOTBOT Web Page", button: "Click me", info: "When you press the button the IOTBOT beeps." }
  };
  function applyLanguage() {
    fetch('/lang').then(r => r.text()).then(lang => {
      var t = texts[lang] || texts.tr;
      document.documentElement.lang = lang;
      document.getElementById('title').innerText = t.title;
      document.getElementById('hello').innerText = t.button;
      document.getElementById('info').innerText = t.info;
    });
  }
  function sayHello() {
    fetch('/hello').then(r => r.text()).then(reply => alert(reply));
  }
  window.onload = applyLanguage;
</script>
)rawliteral";

const char WEBPageCSS[] PROGMEM = R"rawliteral(
<style>
  body { text-align: center; font-family: Arial, sans-serif; }
  button { font-size: 20px; padding: 10px; margin: 20px; }
</style>
)rawliteral";

const char WEBPageHTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="tr">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>IOTBOT Web Server</title>
  %s <!-- JavaScript buraya / JavaScript goes here -->
  %s <!-- CSS buraya / CSS goes here -->
</head>
<body>
  <h1 id="title">IOTBOT</h1>
  <button id="hello" onclick="sayHello()">...</button>
  <p id="info"></p>
</body>
</html>
)rawliteral";

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "DURUM" -> "durum"
// Lower-cases and simplifies Turkish letters: "DURUM" -> "durum"
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

String currentIP() { return apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString(); }

void drawScreen() {
  char line[41];
  lcdRow(0, apMode ? L("AP Modu Aktif", "AP Mode Active") : L("Sunucu Hazır (STA)", "Server Ready (STA)"));
  snprintf(line, sizeof(line), "SSID: %s", apMode ? AP_SSID : WIFI_SSID);
  lcdRow(1, line);
  snprintf(line, sizeof(line), "IP: %s", currentIP().c_str());
  lcdRow(2, line);
  lcdRow(3, L("Adres: /demopage", "Go to /demopage"));
}

void printHelp() {
  iotbot.serialWrite(L("---- STA WEB SUNUCU - Komutlar ----", "---- STA WEB SERVER - Commands ----"));
  iotbot.serialWrite(L("  yardim   : bu liste", "  help     : this list"));
  iotbot.serialWrite(L("  durum    : mod, IP, sinyal gücü", "  status   : mode, IP, signal strength"));
  iotbot.serialWrite(L("  dil      : English'e geç (web sayfası da)", "  lang     : switch to Turkish (web page too)"));
  iotbot.serialWrite(String(L("  Tarayıcı : http://", "  Browser  : http://")) + currentIP() + "/demopage");
}

void printStatus() {
  if (apMode) {
    iotbot.serialWrite(String(L("Mod: AP (kendi ağı)  SSID: ", "Mode: AP (own network)  SSID: ")) + AP_SSID + L("  Şifre: ", "  Password: ") + AP_PASS);
    iotbot.serialWrite(String(L("Bağlı cihaz: ", "Connected devices: ")) + WiFi.softAPgetStationNum());
  } else {
    iotbot.serialWrite(String(L("Mod: STA (ev ağı)  SSID: ", "Mode: STA (home network)  SSID: ")) + WIFI_SSID);
    iotbot.serialWrite(String(L("Bağlantı: ", "Connection: ")) + (WiFi.status() == WL_CONNECTED ? L("VAR", "OK") : L("KOPTU", "LOST")) +
                       L("   Sinyal: ", "   Signal: ") + WiFi.RSSI() + " dBm");
  }
  iotbot.serialWrite(String("IP: ") + currentIP());
  iotbot.serialWrite(String(L("Web'den gelen merhaba: ", "Hellos from the web: ")) + helloHandled);
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "durum" || cmd == "status") {
    printStatus();
  } else if (cmd == "dil" || cmd == "lang" || cmd == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe (web sayfasını yenileyin)", "Language: English (refresh the web page)"));
    drawScreen();
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// Ev ağına en fazla 15 sn bağlanmayı dener / tries to join the home network for at most 15 s
bool tryConnectHome() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(250);
  }
  return WiFi.status() == WL_CONNECTED;
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();             // IoTBot başlatılıyor / Initialize IoTBot
  iotbot.serialStart(115200); // Seri haberleşme / Serial communication

  iotbot.lcdShowLoading(L("WiFi'ye bağlanıyor", "Connecting WiFi"));
  iotbot.buzzerPlayTone(1000, 200);
  iotbot.serialWrite(String(L("WiFi'ye bağlanılıyor: ", "Connecting to WiFi: ")) + WIFI_SSID);

  // NOT: serverStart() sunucuyu kurar; İKİ KEZ çağrılmamalı. Bu yüzden önce ağı
  // kendimiz deniyoruz, sonra serverStart()'ı doğru modla BİR KEZ çağırıyoruz.
  // NOTE: serverStart() sets up the server and must NOT be called twice. So we try
  // the network ourselves first, then call serverStart() ONCE with the right mode.
  if (tryConnectHome()) {
    apMode = false;
    iotbot.serverStart("STA", WIFI_SSID, WIFI_PASS); // Zaten bağlı, hemen devam eder / already connected, continues quickly
    iotbot.lcdShowStatus(L("WiFi Bağlandı", "WiFi Connected"), "IP: " + iotbot.wifiGetIPAddress(), true);
    iotbot.buzzerPlayTone(2000, 300);
  } else {
    apMode = true;
    iotbot.serialWrite(L("Ev ağına bağlanılamadı -> kendi ağımızı (AP) kuruyoruz.", "Could not join the home network -> creating our own (AP)."));
    WiFi.disconnect();
    WiFi.mode(WIFI_AP); // Saf AP modu: serverContinue() DNS yönlendirmesini yapsın / pure AP mode so serverContinue() handles DNS
    iotbot.serverStart("AP", AP_SSID, AP_PASS);
    iotbot.lcdShowStatus(L("WiFi Yok", "WiFi Failed"), L("AP modu açıldı", "AP mode started"), false);
    iotbot.buzzerPlayTone(500, 400);
  }
  delay(1500);

  // Web sayfasını yayınla: http://<IP>/demopage / Publish the page: http://<IP>/demopage
  iotbot.serverCreateLocalPage("demopage", WEBPageScript, WEBPageCSS, WEBPageHTML);
  iotbot.serverOnRequest("/lang", []() -> String { return turkish ? "tr" : "en"; });
  iotbot.serverOnRequest("/hello", []() -> String {
    helloRequests++; // Asıl iş loop() içinde / the real work is in loop()
    return turkish ? "Merhaba! IOTBOT mesajınızı aldı." : "Hello! The IOTBOT got your message.";
  });

  drawScreen();
  iotbot.serialWrite(String(L("Web sunucusu hazır: http://", "Web server ready: http://")) + currentIP() + "/demopage");
  printHelp();
}

void loop() {
  iotbot.serverContinue(); // AP modunda DNS yönlendirmeyi sürdür / keep DNS redirection running in AP mode

  if (helloHandled != helloRequests) {
    helloHandled = helloRequests;
    char line[41];
    snprintf(line, sizeof(line), L("Web'den merhaba! %d", "Hello from web! %d"), helloHandled);
    lcdRow(3, line);
    iotbot.serialWrite(line);
    iotbot.buzzerPlayTone(1500, 80);
  }

  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
}
