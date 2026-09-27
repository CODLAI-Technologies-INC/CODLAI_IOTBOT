// TR: Telefon/tarayicidan (IOTBOT'un kendi WiFi agina baglanarak) LED'leri
// ve roleyi acip kapatabildiginiz, sicaklik/isik degerini canli izleyebildiginiz
// bir kontrol paneli. Yeni `serverOnRequest()` fonksiyonu kullanilir - bu
// fonksiyon, `serverCreateLocalPage()`'in aksine, bir adrese (ornegin
// "/led-on") istek geldiginde GERCEKTEN kod calistirmaniza (bir pini
// yakip sondurmenize) izin verir.
// EN: A control panel you open from your phone/browser (by joining
// IOTBOT's own WiFi network) to turn LEDs and the relay on/off, and watch
// the light/temperature value live. Uses the new `serverOnRequest()`
// function - unlike `serverCreateLocalPage()`, it lets code actually RUN
// (toggle a pin) when a URL (e.g. "/led-on") is requested.
//
// Kurulum / Setup:
// 1) Bu kodu IOTBOT'a yukleyin / Upload this to your IOTBOT.
// 2) Telefonunuzun WiFi ayarlarindan "CODLAI_IOTBOT" agina baglanin,
//    sifre: 12345678 / On your phone, join the "CODLAI_IOTBOT" WiFi
//    network, password: 12345678.
// 3) Tarayicida 192.168.4.1 adresini acin / Open 192.168.4.1 in a browser.

#define USE_SERVER
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define AP_SSID "CODLAI_IOTBOT"
#define AP_PASS "12345678"

namespace {
  // Fiziksel yerlesime gore soldan saga: 32, 33, 25, 26, 27 (bkz. egitim
  // uygulamasindaki ayni sira).
  constexpr uint8_t kLedPins[] = {IO32, IO33, IO25, IO26, IO27};
  constexpr uint8_t kLedCount = 5;
  bool ledsOn = false;
  bool relayOn = false;

  void allLeds(bool state) {
    for (uint8_t pin : kLedPins) {
      iotbot.digitalWritePin(pin, state);
    }
    ledsOn = state;
  }
}

// JS ve CSS parcalari, HTML sablonundaki iki "%s" yerine sirayla (once
// Script, sonra CSS) yerlestirilir - bkz. serverCreateLocalPage.
const char WEBPageScript[] PROGMEM = R"rawliteral(
<script>
  function callAction(url) {
    fetch(url).then(() => refreshStatus());
  }
  function refreshStatus() {
    fetch('/status').then(r => r.text()).then(text => {
      document.getElementById('status').innerText = text;
    });
  }
  setInterval(refreshStatus, 1000);
  window.onload = refreshStatus;
</script>
)rawliteral";

const char WEBPageCSS[] PROGMEM = R"rawliteral(
<style>
  body { text-align: center; font-family: Arial, sans-serif; background: #101418; color: #eee; }
  h1 { color: #4fd1c5; }
  button { font-size: 18px; padding: 12px 20px; margin: 10px; border-radius: 8px; border: none; }
  .on { background: #38a169; color: white; }
  .off { background: #e53e3e; color: white; }
  #status { white-space: pre-line; font-size: 16px; margin-top: 20px; }
</style>
)rawliteral";

const char WEBPageHTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>IOTBOT Kontrol Paneli</title>
  %s
  %s
</head>
<body>
  <h1>IOTBOT</h1>
  <div>
    <button class="on" onclick="callAction('/led-on')">LED ON / ACIK</button>
    <button class="off" onclick="callAction('/led-off')">LED OFF / KAPALI</button>
  </div>
  <div>
    <button class="on" onclick="callAction('/relay-on')">RELAY ON / ROLE AC</button>
    <button class="off" onclick="callAction('/relay-off')">RELAY OFF / ROLE KAPA</button>
  </div>
  <pre id="status">...</pre>
</body>
</html>
)rawliteral";

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.lcdShowLoading(turkish ? "AP baslatiliyor" : "Starting AP mode");
  iotbot.buzzerPlayTone(1000, 200);

  allLeds(false);
  iotbot.relayWrite(false);

  iotbot.serverStart("AP", AP_SSID, AP_PASS);
  iotbot.serverCreateLocalPage("/", WEBPageScript, WEBPageCSS, WEBPageHTML);

  // "Yeni" kisim: her butonun ARKASINDA gercek kod calisiyor.
  iotbot.serverOnRequest("/led-on", []() -> String {
    allLeds(true);
    return "LED ON";
  });
  iotbot.serverOnRequest("/led-off", []() -> String {
    allLeds(false);
    return "LED OFF";
  });
  iotbot.serverOnRequest("/relay-on", []() -> String {
    relayOn = true;
    iotbot.relayWrite(true);
    return "RELAY ON";
  });
  iotbot.serverOnRequest("/relay-off", []() -> String {
    relayOn = false;
    iotbot.relayWrite(false);
    return "RELAY OFF";
  });
  iotbot.serverOnRequest("/status", []() -> String {
    String s;
    s += turkish ? "LED: " : "LED: ";
    s += ledsOn ? (turkish ? "ACIK" : "ON") : (turkish ? "KAPALI" : "OFF");
    s += "\n";
    s += turkish ? "Role: " : "Relay: ";
    s += relayOn ? (turkish ? "ACIK" : "ON") : (turkish ? "KAPALI" : "OFF");
    s += "\n";
    s += turkish ? "Isik (LDR): " : "Light (LDR): ";
    s += String(iotbot.ldrRead());
    s += "\n";
    s += turkish ? "Pot: " : "Pot: ";
    s += String(iotbot.potentiometerRead());
    return s;
  });

  iotbot.buzzerPlayTone(2000, 400);
  iotbot.lcdWriteMid(turkish ? "AP AKTIF" : "AP ACTIVE",
                      "SSID: " AP_SSID,
                      "IP: 192.168.4.1",
                      turkish ? "Tarayicidan gir" : "Open in browser");
  iotbot.serialWrite(turkish ? "Kontrol paneli hazir: http://192.168.4.1"
                             : "Control panel ready: http://192.168.4.1");
}

void loop() {
  iotbot.serverContinue();
}
