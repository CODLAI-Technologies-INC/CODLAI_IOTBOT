/*
 * TR: ERİŞİM NOKTASI (AP) MODUNDA WEB SUNUCU
 *  - IOTBOT kendi WiFi ağını kurar ("CODLAI Server", şifre 12345678). Telefonunuzu
 *    bu ağa bağlayın ve tarayıcıda 192.168.4.1/demopage adresini açın.
 *  - Sayfadaki butona basınca IOTBOT bip sesi çıkarır, LCD'ye "Web'den merhaba!"
 *    yazar ve sayfada bir mesaj kutusu açılır.
 *  - Sayfanın dili kartın diliyle aynıdır (sayfa açılırken /lang adresinden öğrenir).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help    -> komut listesi
 *      durum  / status  -> ağ adı, IP, bağlı cihaz sayısı
 *      dil    / lang    -> dili değiştir (Türkçe <-> English, web sayfası da değişir)
 *
 * EN: WEB SERVER IN ACCESS POINT (AP) MODE
 *  - The IOTBOT creates its own WiFi network ("CODLAI Server", password 12345678).
 *    Join it with your phone and open 192.168.4.1/demopage in the browser.
 *  - Pressing the button on the page makes the IOTBOT beep, writes "Hello from
 *    web!" on the LCD and shows a message box on the page.
 *  - The page uses the board's language (it asks /lang when it loads).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim  -> command list
 *      status / durum   -> network name, IP, number of connected devices
 *      lang   / dil     -> switch language (Turkish <-> English, the web page too)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. / NO extra module needed.
 * Not / Note: USE_SERVER tanımı kütüphaneden ÖNCE yazılmalıdır (aşağıda var).
 *             USE_SERVER must be defined BEFORE including the library (done below).
 */
#define USE_SERVER
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Erişim noktası (AP) bilgileri / Access Point (AP) settings
#define AP_SSID "CODLAI Server" // AP adı / AP name
#define AP_PASS "12345678"      // AP şifresi (en az 8 karakter) / AP password (at least 8 characters)

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Web sayfasından gelen "merhaba" istekleri (web sunucusu ayrı bir görevde çalışır,
// bu yüzden sayacı sadece artırıp asıl işi loop() içinde yapıyoruz).
// "Hello" requests from the web page (the web server runs in a separate task,
// so we only count them there and do the real work in loop()).
volatile int helloRequests = 0;
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
  // Kartın dilini öğren ve sayfa yazılarını ona göre değiştir / ask the board's language and update the texts
  function applyLanguage() {
    fetch('/lang').then(r => r.text()).then(lang => {
      var t = texts[lang] || texts.tr;
      document.documentElement.lang = lang;
      document.getElementById('title').innerText = t.title;
      document.getElementById('hello').innerText = t.button;
      document.getElementById('info').innerText = t.info;
    });
  }
  // Butona basınca IOTBOT'a haber ver ve cevabı göster / tell the IOTBOT and show its reply
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
// lcdWriteFixedTxt Türkçe harfleri LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void drawScreen() {
  lcdRow(0, L("AP Modu Aktif", "AP Mode Active"));
  lcdRow(1, "SSID: " AP_SSID);
  lcdRow(2, "IP: 192.168.4.1");
  lcdRow(3, L("Adres: /demopage", "Go to /demopage"));
}

void printHelp() {
  iotbot.serialWrite(L("---- AP WEB SUNUCU - Komutlar ----", "---- AP WEB SERVER - Commands ----"));
  iotbot.serialWrite(L("  yardim   : bu liste", "  help     : this list"));
  iotbot.serialWrite(L("  durum    : ağ adı, IP, bağlı cihaz sayısı", "  status   : network name, IP, connected devices"));
  iotbot.serialWrite(L("  dil      : English'e geç (web sayfası da)", "  lang     : switch to Turkish (web page too)"));
  iotbot.serialWrite(L("  Tarayıcı : http://192.168.4.1/demopage", "  Browser  : http://192.168.4.1/demopage"));
}

void printStatus() {
  iotbot.serialWrite(String(L("Ağ adı (SSID): ", "Network (SSID): ")) + AP_SSID + L("   Şifre: ", "   Password: ") + AP_PASS);
  iotbot.serialWrite(String("IP: ") + WiFi.softAPIP().toString());
  iotbot.serialWrite(String(L("Bağlı cihaz: ", "Connected devices: ")) + WiFi.softAPgetStationNum());
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

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();             // IoTBot başlatılıyor / Initialize IoTBot
  iotbot.serialStart(115200); // Seri haberleşme / Serial communication

  iotbot.lcdShowLoading(L("AP başlatılıyor", "Starting AP mode"));
  iotbot.buzzerPlayTone(1000, 200);

  // IOTBOT'u erişim noktası (AP) olarak başlat / Start IOTBOT as an access point (AP)
  iotbot.serverStart("AP", AP_SSID, AP_PASS);

  // Web sayfasını yayınla: http://192.168.4.1/demopage / Publish the page: http://192.168.4.1/demopage
  iotbot.serverCreateLocalPage("demopage", WEBPageScript, WEBPageCSS, WEBPageHTML);

  // Sayfa dili sorar: "tr" ya da "en" / The page asks for the language: "tr" or "en"
  iotbot.serverOnRequest("/lang", []() -> String { return turkish ? "tr" : "en"; });

  // Sayfadaki buton: sadece sayacı artır, bip/LCD işini loop() yapar.
  // Button on the page: only count it here, loop() does the beep/LCD work.
  iotbot.serverOnRequest("/hello", []() -> String {
    helloRequests++;
    return turkish ? "Merhaba! IOTBOT mesajınızı aldı." : "Hello! The IOTBOT got your message.";
  });

  iotbot.lcdShowStatus(L("AP Başladı", "AP Started"), "IP: 192.168.4.1", true);
  iotbot.buzzerPlayTone(2000, 300);
  delay(1500);
  drawScreen();

  iotbot.serialWrite(L("AP web sunucusu hazır: http://192.168.4.1/demopage", "AP web server ready: http://192.168.4.1/demopage"));
  printHelp();
}

void loop() {
  iotbot.serverContinue(); // AP modunda DNS yönlendirmeyi sürdür / keep DNS redirection running in AP mode

  // Web'den yeni "merhaba" geldi mi? / New "hello" from the web?
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
