/*
 * TR: WiFi WEB KONTROL PANELİ - Telefon/tarayıcıdan LED ve röle kontrolü
 *  - IOTBOT kendi WiFi ağını kurar. Telefonla bu ağa bağlanıp tarayıcıda panel
 *    sayfasını açın; LED'leri ve röleyi açıp kapatın, ışık/potansiyometre değerini
 *    canlı izleyin. Sayfanın dili kartın diliyle aynıdır.
 *  - serverOnRequest() fonksiyonu, serverCreateLocalPage()'in aksine, bir adrese
 *    (ör. "/led-on") istek gelince GERÇEKTEN kod çalıştırır (bir pini yakıp söndürür).
 *  - Açılışta OTOMATİK mod: 5 LED sırayla kayar (kara şimşek), röle kapalı.
 *  - B3 butonu OTOMATİK <-> MANUEL. Web'den ya da seri porttan bir LED/röle komutu
 *    gelince kart kendiliğinden MANUEL moda geçer.
 *  - Kurulum:
 *    1) Bu kodu IOTBOT'a yükleyin.
 *    2) Telefonun WiFi ayarlarından "CODLAI_IOTBOT" ağına bağlanın (şifre: 12345678).
 *    3) Tarayıcıda 192.168.4.1/panel adresini açın.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help                 -> komut listesi
 *      led ac / led on               -> LED'leri yak       (led kapat / led off: söndür)
 *      role ac / relay on            -> röleyi aç          (role kapat / relay off: kapat)
 *      oto    / auto                 -> otomatik mod
 *      manuel / manual               -> manuel mod
 *      durum  / status               -> mod, LED, röle, bağlı cihaz sayısı
 *      dil    / lang                 -> dili değiştir (web sayfası da değişir)
 *
 * EN: WiFi WEB CONTROL PANEL - control LEDs and the relay from a phone/browser
 *  - The IOTBOT creates its own WiFi network. Join it with your phone and open the
 *    panel page in the browser; switch the LEDs and the relay, and watch the light/
 *    potentiometer values live. The page uses the board's language.
 *  - Unlike serverCreateLocalPage(), serverOnRequest() really RUNS code when a URL
 *    (e.g. "/led-on") is requested (it switches a pin).
 *  - At startup AUTO mode: the 5 LEDs scan back and forth, the relay is off.
 *  - The B3 button toggles AUTO <-> MANUAL. An LED/relay command from the web or the
 *    serial port switches to MANUAL by itself.
 *  - Setup:
 *    1) Upload this code to the IOTBOT.
 *    2) On your phone, join the "CODLAI_IOTBOT" WiFi network (password: 12345678).
 *    3) Open 192.168.4.1/panel in the browser.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim               -> command list
 *      led on / led ac               -> LEDs on            (led off / led kapat: off)
 *      relay on / role ac            -> relay on           (relay off / role kapat: off)
 *      auto   / oto                  -> auto mode
 *      manual / manuel               -> manual mode
 *      status / durum                -> mode, LEDs, relay, connected devices
 *      lang   / dil                  -> switch language (the web page too)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ - LED'ler P1-P5 hatlarında (IO32, IO33, IO25,
 * IO26, IO27), röle kartın üzerindedir. / NO extra module needed - the LEDs are on the
 * P1-P5 lines (IO32, IO33, IO25, IO26, IO27) and the relay is on the board.
 * Not / Note: WiFi açıkken B1/B2 ve joystick X güvenilir değildir; bu örnek B3 kullanır.
 *             With WiFi on, B1/B2 and joystick X are unreliable; this example uses B3.
 */

#define USE_SERVER
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

#define AP_SSID "CODLAI_IOTBOT"
#define AP_PASS "12345678"

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Fiziksel yerleşime göre soldan sağa: 32, 33, 25, 26, 27 / left to right as on the board
const uint8_t kLedPins[] = {IO32, IO33, IO25, IO26, IO27};
const uint8_t kLedCount = 5;

// Web sunucusu ayrı bir görevde çalışır; bu değişkenleri o da değiştirir (volatile)
// The web server runs in a separate task and changes these variables too (volatile)
volatile bool manualMode = false; // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
volatile bool ledsOn = false;
volatile bool relayOn = false;
volatile bool webChanged = false; // Web'den bir değişiklik geldi (loop haber versin) / a change came from the web (loop reports it)
bool shownManual = false;         // loop'un en son bildirdiği mod / the mode loop reported last
int scanPos = 0, scanDir = 1;
uint32_t lastScanMs = 0, lastScreenMs = 0;
bool lastB3 = false;

void allLeds(bool state) {
  for (uint8_t i = 0; i < kLedCount; i++) iotbot.digitalWritePin(kLedPins[i], state);
  ledsOn = state;
}

void setRelay(bool on) {
  relayOn = on;
  iotbot.relayWrite(on);
}

// ---------------------------------------------------------------------------
// Web sayfası / Web page
// serverCreateLocalPage() HTML içindeki iki "%s" yerine SIRAYLA önce Script'i, sonra
// CSS'i yerleştirir. HTML'de başka "%" karakteri kullanmayın.
// serverCreateLocalPage() puts the Script into the first "%s" and the CSS into the
// second "%s" of the HTML. Do not use any other "%" character in the HTML.
// ---------------------------------------------------------------------------
const char WEBPageScript[] PROGMEM = R"rawliteral(
<script>
  var texts = {
    tr: { title: "IOTBOT Kontrol Paneli", ledOn: "LED AÇ", ledOff: "LED KAPAT", relayOn: "RÖLE AÇ", relayOff: "RÖLE KAPAT", auto: "OTOMATİK", manual: "MANUEL" },
    en: { title: "IOTBOT Control Panel", ledOn: "LED ON", ledOff: "LED OFF", relayOn: "RELAY ON", relayOff: "RELAY OFF", auto: "AUTO", manual: "MANUAL" }
  };
  function applyLanguage() {
    fetch('/lang').then(r => r.text()).then(lang => {
      var t = texts[lang] || texts.tr;
      document.documentElement.lang = lang;
      document.title = t.title;
      for (var id in t) { var e = document.getElementById(id); if (e) e.innerText = t[id]; }
    });
  }
  function callAction(url) {
    fetch(url).then(() => refreshStatus());
  }
  function refreshStatus() {
    fetch('/status').then(r => r.text()).then(text => {
      document.getElementById('status').innerText = text;
    });
  }
  setInterval(refreshStatus, 1000);
  window.onload = function () { applyLanguage(); refreshStatus(); };
</script>
)rawliteral";

const char WEBPageCSS[] PROGMEM = R"rawliteral(
<style>
  body { text-align: center; font-family: Arial, sans-serif; background: #101418; color: #eee; }
  h1 { color: #4fd1c5; }
  button { font-size: 18px; padding: 12px 20px; margin: 10px; border-radius: 8px; border: none; }
  .on { background: #38a169; color: white; }
  .off { background: #e53e3e; color: white; }
  .mode { background: #3182ce; color: white; }
  #status { white-space: pre-line; font-size: 16px; margin-top: 20px; }
</style>
)rawliteral";

const char WEBPageHTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="tr">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>IOTBOT</title>
  %s
  %s
</head>
<body>
  <h1 id="title">IOTBOT</h1>
  <div>
    <button id="ledOn" class="on" onclick="callAction('/led-on')">LED ON</button>
    <button id="ledOff" class="off" onclick="callAction('/led-off')">LED OFF</button>
  </div>
  <div>
    <button id="relayOn" class="on" onclick="callAction('/relay-on')">RELAY ON</button>
    <button id="relayOff" class="off" onclick="callAction('/relay-off')">RELAY OFF</button>
  </div>
  <div>
    <button id="auto" class="mode" onclick="callAction('/auto')">AUTO</button>
    <button id="manual" class="mode" onclick="callAction('/manual')">MANUAL</button>
  </div>
  <pre id="status">...</pre>
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

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "RÖLE AÇ" -> "role ac"
// Lower-cases and simplifies Turkish letters: "RÖLE AÇ" -> "role ac"
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

const char *onOff(bool on) { return on ? L("AÇIK", "ON") : L("KAPALI", "OFF"); }

String statusText() {
  String s;
  s += String(L("Mod: ", "Mode: ")) + (manualMode ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO")) + "\n";
  s += String("LED: ") + (manualMode ? onOff(ledsOn) : L("kayan ışık", "scanner")) + "\n";
  s += String(L("Röle: ", "Relay: ")) + onOff(relayOn) + "\n";
  s += String(L("Işık (LDR): ", "Light (LDR): ")) + iotbot.ldrRead() + "\n";
  s += String("Pot: ") + iotbot.potentiometerRead();
  return s;
}

void printHelp() {
  iotbot.serialWrite(L("---- WEB KONTROL PANELİ - Komutlar ----", "---- WEB CONTROL PANEL - Commands ----"));
  iotbot.serialWrite(L("  yardim             : bu liste", "  help               : this list"));
  iotbot.serialWrite(L("  led ac / led kapat : LED'ler", "  led on / led off   : LEDs"));
  iotbot.serialWrite(L("  role ac / kapat    : röle", "  relay on / off     : relay"));
  iotbot.serialWrite(L("  oto / manuel       : mod seç", "  auto / manual      : choose mode"));
  iotbot.serialWrite(L("  durum              : durum bilgisi", "  status             : status information"));
  iotbot.serialWrite(L("  dil                : English'e geç (web sayfası da)", "  lang               : switch to Turkish (web page too)"));
  iotbot.serialWrite(L("  Tarayıcı: http://192.168.4.1/panel   B3: OTOMATİK <-> MANUEL", "  Browser: http://192.168.4.1/panel   B3: AUTO <-> MANUAL"));
}

void drawStaticScreen() {
  lcdRow(0, "192.168.4.1/panel");
  lcdRow(3, manualMode ? L("B3: otomatik mod", "B3: auto mode") : L("B3: manuel kontrol", "B3: manual control"));
  lastScreenMs = 0;
}

// Modu değiştirir (web görevinden de çağrılabilir: bip/yazı işini loop yapar)
// Changes the mode (can be called from the web task too: loop does the beep/printing)
void applyMode(bool manual) {
  manualMode = manual;
  if (manual) allLeds(ledsOn);   // Gösteri bitti, LED'ler manuel durumuna / show over, LEDs to the manual state
  else setRelay(false);          // Otomatikte röle kapalı / relay off in auto
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "led ac" || cmd == "led on") {
    applyMode(true); allLeds(true); iotbot.serialWrite(L("LED'ler açıldı", "LEDs on"));
  } else if (cmd == "led kapat" || cmd == "led off") {
    applyMode(true); allLeds(false); iotbot.serialWrite(L("LED'ler kapatıldı", "LEDs off"));
  } else if (cmd == "role ac" || cmd == "relay on") {
    applyMode(true); setRelay(true); iotbot.serialWrite(L("Röle açıldı", "Relay on"));
  } else if (cmd == "role kapat" || cmd == "relay off") {
    applyMode(true); setRelay(false); iotbot.serialWrite(L("Röle kapatıldı", "Relay off"));
  } else if (cmd == "oto" || cmd == "otomatik" || cmd == "auto") {
    applyMode(false);
  } else if (cmd == "manuel" || cmd == "manual") {
    applyMode(true);
  } else if (cmd == "durum" || cmd == "status") {
    String s = statusText();
    s.replace("\n", "   ");
    iotbot.serialWrite(s);
    iotbot.serialWrite(String(L("Bağlı cihaz: ", "Connected devices: ")) + WiFi.softAPgetStationNum());
  } else if (cmd == "dil" || cmd == "lang" || cmd == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe (web sayfasını yenileyin)", "Language: English (refresh the web page)"));
    drawStaticScreen();
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.lcdShowLoading(L("AP başlatılıyor", "Starting AP mode"));
  iotbot.buzzerPlayTone(1000, 200);

  allLeds(false);
  setRelay(false);

  iotbot.serverStart("AP", AP_SSID, AP_PASS);
  // NOT: "/" adresini kütüphane kullanıyor ("CODLAI Server is Running!"), bu yüzden
  // panel "/panel" adresinde. / NOTE: the library already uses "/" ("CODLAI Server is
  // Running!"), so the panel lives at "/panel".
  iotbot.serverCreateLocalPage("panel", WEBPageScript, WEBPageCSS, WEBPageHTML);

  // Her butonun ARKASINDA gerçek kod çalışıyor / real code runs BEHIND every button
  iotbot.serverOnRequest("/led-on", []() -> String { applyMode(true); allLeds(true); webChanged = true; return "LED ON"; });
  iotbot.serverOnRequest("/led-off", []() -> String { applyMode(true); allLeds(false); webChanged = true; return "LED OFF"; });
  iotbot.serverOnRequest("/relay-on", []() -> String { applyMode(true); setRelay(true); webChanged = true; return "RELAY ON"; });
  iotbot.serverOnRequest("/relay-off", []() -> String { applyMode(true); setRelay(false); webChanged = true; return "RELAY OFF"; });
  iotbot.serverOnRequest("/auto", []() -> String { applyMode(false); webChanged = true; return "AUTO"; });
  iotbot.serverOnRequest("/manual", []() -> String { applyMode(true); webChanged = true; return "MANUAL"; });
  iotbot.serverOnRequest("/status", []() -> String { return statusText(); });
  iotbot.serverOnRequest("/lang", []() -> String { return turkish ? "tr" : "en"; });

  iotbot.buzzerPlayTone(2000, 300);
  iotbot.lcdClear();
  drawStaticScreen();
  iotbot.serialWrite(L("Kontrol paneli hazır: 'CODLAI_IOTBOT' ağına bağlanın (şifre 12345678), http://192.168.4.1/panel",
                       "Control panel ready: join 'CODLAI_IOTBOT' (password 12345678), http://192.168.4.1/panel"));
  printHelp();
}

void loop() {
  uint32_t now = millis();
  iotbot.serverContinue(); // AP modunda DNS yönlendirmesi / DNS redirection in AP mode

  // 1) B3 -> mod değiştir (sadece basıldığı an) / B3 -> toggle mode (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) applyMode(!manualMode);
  lastB3 = b3;

  // 2) Seri komutlar / serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Mod değiştiyse (B3, seri ya da web) bip + mesaj / if the mode changed (B3, serial or web): beep + message
  if (shownManual != manualMode) {
    shownManual = manualMode;
    iotbot.buzzerPlayTone(manualMode ? 1500 : 1000, 60);
    iotbot.serialWrite(manualMode ? L(">> MANUEL mod: web sayfası veya seri komutlarla kontrol edin.", ">> MANUAL mode: control from the web page or serial commands.")
                                  : L(">> OTOMATİK mod: LED gösterisi, röle kapalı.", ">> AUTO mode: LED show, relay off."));
    drawStaticScreen();
  }
  if (webChanged) {
    webChanged = false;
    iotbot.serialWrite(String(L("Web'den komut geldi -> ", "Command from the web -> ")) + "LED " + onOff(ledsOn) + L(", Röle ", ", Relay ") + onOff(relayOn));
    lastScreenMs = 0;
  }

  // 4) Otomatik gösteri: her 120 ms'de yanan LED bir kayar / auto show: the lit LED moves every 120 ms
  if (!manualMode && now - lastScanMs >= 120) {
    lastScanMs = now;
    for (uint8_t i = 0; i < kLedCount; i++) iotbot.digitalWritePin(kLedPins[i], i == scanPos);
    scanPos += scanDir;
    if (scanPos >= kLedCount - 1 || scanPos <= 0) scanDir = -scanDir;
  }

  // 5) LCD (300 ms'de bir, titremesiz) / LCD (every 300 ms, no flicker)
  if (now - lastScreenMs >= 300) {
    lastScreenMs = now;
    char text[41];
    if (manualMode) snprintf(text, sizeof(text), L("MANUEL   Röle:%s", "MANUAL  Relay:%s"), onOff(relayOn));
    else snprintf(text, sizeof(text), L("OTOMATİK Röle:%s", "AUTO    Relay:%s"), onOff(relayOn));
    lcdRow(1, text);
    snprintf(text, sizeof(text), "LED: %s", manualMode ? onOff(ledsOn) : L("kayan ışık", "scanner"));
    lcdRow(2, text);
  }
}
