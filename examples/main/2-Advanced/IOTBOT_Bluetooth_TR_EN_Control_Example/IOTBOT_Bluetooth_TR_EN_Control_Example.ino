/*
 * TR: BLUETOOTH UZAKTAN KUMANDA - LED'ler ve röle (Türkçe + İngilizce komutlar)
 *  - Telefonunuzda bir Bluetooth terminal uygulaması açın (ör. "Serial Bluetooth
 *    Terminal"), "IOTBOT_BT" cihazına bağlanın (PIN: 1234).
 *  - Açılışta OTOMATİK mod çalışır: 5 LED sırayla yanıp kayar (kara şimşek), röle kapalı.
 *  - B3 butonu OTOMATİK <-> MANUEL arasında geçiş yapar. Bir LED/röle komutu
 *    gönderince de kendiliğinden MANUEL moda geçer.
 *  - Komutlar telefondan VEYA Seri Monitör'den (115200 baud) yazılabilir, Türkçe
 *    veya İngilizce:
 *      led ac    / led on      -> tüm LED'leri yak          (kısa: 1)
 *      led kapat / led off     -> tüm LED'leri söndür       (kısa: 0)
 *      role ac   / relay on    -> röleyi aç                 (kısa: R)
 *      role kapat/ relay off   -> röleyi kapat              (kısa: r)
 *      isik      / light       -> ışık (LDR) değerini oku   (kısa: ?)
 *      oto       / auto        -> otomatik mod (LED gösterisi)
 *      manuel    / manual      -> manuel mod
 *      durum     / status      -> mod, LED, röle, ışık
 *      yardim    / help        -> komut listesi
 *      dil       / lang        -> dili değiştir (Türkçe <-> English)
 *
 * EN: BLUETOOTH REMOTE CONTROL - LEDs and relay (Turkish + English commands)
 *  - Open a Bluetooth terminal app on your phone (e.g. "Serial Bluetooth
 *    Terminal") and connect to "IOTBOT_BT" (PIN: 1234).
 *  - At startup AUTO mode runs: the 5 LEDs light up one after another (scanner),
 *    the relay is off.
 *  - The B3 button toggles AUTO <-> MANUAL. Sending an LED/relay command also
 *    switches to MANUAL by itself.
 *  - Commands can be typed on the phone OR in the Serial Monitor (115200 baud),
 *    in Turkish or English:
 *      led on    / led ac      -> turn all LEDs on          (short: 1)
 *      led off   / led kapat   -> turn all LEDs off         (short: 0)
 *      relay on  / role ac     -> turn the relay on         (short: R)
 *      relay off / role kapat  -> turn the relay off        (short: r)
 *      light     / isik        -> read the light (LDR) value (short: ?)
 *      auto      / oto         -> auto mode (LED show)
 *      manual    / manuel      -> manual mode
 *      status    / durum       -> mode, LEDs, relay, light
 *      help      / yardim      -> command list
 *      lang      / dil         -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ - LED'ler P1-P5 hatlarında (IO32, IO33, IO25,
 * IO26, IO27), röle kartın üzerindedir. Soketlere başka modül takılıysa çıkarın.
 * NO extra module needed - the LEDs are on the P1-P5 lines (IO32, IO33, IO25, IO26,
 * IO27) and the relay is on the board. Remove other modules from the sockets.
 */

#define USE_BLUETOOTH
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

#define BT_NAME "IOTBOT_BT" // Telefonda görünen ad / name shown on the phone
#define BT_PIN "1234"       // Eşleşme PIN'i / pairing PIN

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Fiziksel sıraya göre soldan sağa / left to right in physical order
const uint8_t kLedPins[] = {IO32, IO33, IO25, IO26, IO27};
const uint8_t kLedCount = 5;

bool manualMode = false;  // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
bool ledsOn = false;      // Manuel moddaki LED durumu / LED state in manual mode
bool relayOn = false;
int scanPos = 0, scanDir = 1;    // Otomatik gösteri: yanan LED ve yön / auto show: lit LED and direction
uint32_t lastScanMs = 0, lastScreenMs = 0;
bool lastB3 = false;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
String rawLine; // Komutun orijinal hali ("R" ile "r" farklı) / original text ("R" and "r" differ)
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
      rawLine = cmdBuffer; rawLine.trim();
      cmd = normalizeCommand(cmdBuffer);
      cmdBuffer = "";
      return true;
    }
    if (cmdBuffer.length() < 40) cmdBuffer += c;
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

// ---------------------------------------------------------------------------
// Bluetooth satır okuyucu (beklemesiz) / Bluetooth line reader (non-blocking)
// iotbot.bluetoothRead() mesajı tek parça okur (~40 ms bekler); bu okuyucu hiç
// beklemez ve satır satır çalışır.
// iotbot.bluetoothRead() reads a message in one piece (waits ~40 ms); this reader
// never waits and works line by line.
// ---------------------------------------------------------------------------
String btBuffer;
uint32_t lastBtCharMs = 0;

bool readBluetoothLine(String &line) {
  BluetoothSerial *bt = iotbot.getBluetoothObject();
  while (bt->available() > 0) {
    char c = bt->read();
    lastBtCharMs = millis();
    if (c == '\n' || c == '\r') {
      if (btBuffer.length() == 0) continue;
      line = btBuffer; line.trim();
      btBuffer = "";
      return line.length() > 0;
    }
    if (btBuffer.length() < 40) btBuffer += c;
  }
  if (btBuffer.length() > 0 && millis() - lastBtCharMs > 150) {
    line = btBuffer; line.trim();
    btBuffer = "";
    return line.length() > 0;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Çıkışlar / Outputs
// ---------------------------------------------------------------------------
void allLeds(bool state) {
  for (uint8_t i = 0; i < kLedCount; i++) iotbot.digitalWritePin(kLedPins[i], state);
}

void setRelay(bool on) {
  relayOn = on;
  iotbot.relayWrite(on);
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

bool fromPhone = false; // Şu anki komut telefondan mı geldi? / did the current command come from the phone?

// Cevabı Seri Monitör'e, komut telefondan geldiyse telefona da yazar.
// Writes the reply to the Serial Monitor, and to the phone if the command came from it.
void reply(const String &text) {
  iotbot.serialWrite(text);
  if (fromPhone) iotbot.bluetoothWrite(text);
}

void printHelp() {
  reply(L("---- BLUETOOTH KUMANDA - Komutlar ----", "---- BLUETOOTH CONTROL - Commands ----"));
  reply(L("  led ac / led kapat   (1 / 0)", "  led on / led off     (1 / 0)"));
  reply(L("  role ac / role kapat (R / r)", "  relay on / relay off (R / r)"));
  reply(L("  isik : ışık değeri   (?)", "  light : light value  (?)"));
  reply(L("  oto / manuel : mod seç", "  auto / manual : choose mode"));
  reply(L("  durum, yardim, dil", "  status, help, lang"));
  reply(L("  B3 butonu : OTOMATİK <-> MANUEL", "  B3 button : AUTO <-> MANUAL"));
}

void drawStaticScreen() {
  lcdRow(0, L("BLUETOOTH KUMANDA", "BLUETOOTH CONTROL"));
  lcdRow(3, manualMode ? L("B3: otomatik mod", "B3: auto mode") : L("B3: manuel kontrol", "B3: manual control"));
  lastScreenMs = 0;
}

void setMode(bool manual) {
  manualMode = manual;
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  if (manual) {
    allLeds(ledsOn); // Gösteri bitti, LED'ler manuel durumuna döner / show over, LEDs go back to the manual state
    reply(L(">> MANUEL mod: LED ve röleyi komutlarla kontrol edin.", ">> MANUAL mode: control the LEDs and relay with commands."));
  } else {
    setRelay(false); // Otomatikte röle kapalı / relay off in auto
    reply(L(">> OTOMATİK mod: LED gösterisi.", ">> AUTO mode: LED show."));
  }
  drawStaticScreen();
}

void printStatus() {
  reply(String(L("Mod: ", "Mode: ")) + (manualMode ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO")) +
        L("  LED: ", "  LED: ") + (manualMode ? (ledsOn ? L("AÇIK", "ON") : L("KAPALI", "OFF")) : L("gösteri", "show")) +
        L("  Röle: ", "  Relay: ") + (relayOn ? L("AÇIK", "ON") : L("KAPALI", "OFF")) +
        L("  Işık: ", "  Light: ") + iotbot.ldrRead());
}

// Telefon ve Seri Monitör AYNI komutları kullanır / the phone and Serial Monitor use the SAME commands
void handleCommand(const String &raw, bool phone) {
  fromPhone = phone;
  String cmd = normalizeCommand(raw);
  bool ledCmd = false, relayCmd = false, value = false;

  // Tek harfli kısa komutlar (büyük/küçük harf önemli: R = aç, r = kapat)
  // Single-letter short commands (case matters: R = on, r = off)
  if (raw == "1") { ledCmd = true; value = true; }
  else if (raw == "0") { ledCmd = true; value = false; }
  else if (raw == "R") { relayCmd = true; value = true; }
  else if (raw == "r") { relayCmd = true; value = false; }
  else if (cmd == "led ac" || cmd == "led on" || cmd == "ledler ac" || cmd == "leds on") { ledCmd = true; value = true; }
  else if (cmd == "led kapat" || cmd == "led off" || cmd == "ledler kapat" || cmd == "leds off") { ledCmd = true; value = false; }
  else if (cmd == "role ac" || cmd == "relay on") { relayCmd = true; value = true; }
  else if (cmd == "role kapat" || cmd == "relay off") { relayCmd = true; value = false; }

  if (ledCmd || relayCmd) {
    if (!manualMode) setMode(true); // Komut gelince manuele geç / a command switches to manual
    if (ledCmd) {
      ledsOn = value;
      allLeds(value);
      reply(value ? L("LED'ler açıldı", "LEDs turned on") : L("LED'ler kapatıldı", "LEDs turned off"));
    } else {
      setRelay(value);
      reply(value ? L("Röle açıldı", "Relay turned on") : L("Röle kapatıldı", "Relay turned off"));
    }
    lastScreenMs = 0;
  } else if (raw == "?" || cmd == "isik" || cmd == "light" || cmd == "ldr") {
    reply(String(L("Işık değeri: ", "Light value: ")) + iotbot.ldrRead());
  } else if (cmd == "oto" || cmd == "otomatik" || cmd == "auto") {
    setMode(false);
  } else if (cmd == "manuel" || cmd == "manual") {
    setMode(true);
  } else if (cmd == "durum" || cmd == "status") {
    printStatus();
  } else if (cmd == "yardim" || cmd == "help") {
    printHelp();
  } else if (cmd == "dil" || cmd == "lang" || cmd == "language") {
    turkish = !turkish;
    reply(L("Dil: Türkçe", "Language: English"));
    drawStaticScreen();
  } else {
    reply(String(L("Bilinmeyen komut: ", "Unknown command: ")) + raw + L("  (yardim yazın)", "  (type help)"));
  }
  fromPhone = false;
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  allLeds(false);
  setRelay(false);
  iotbot.lcdClear();
  drawStaticScreen();

  iotbot.bluetoothStart(BT_NAME, BT_PIN);
  iotbot.serialWrite(L("Bluetooth başlatıldı: IOTBOT_BT (PIN 1234)", "Bluetooth started: IOTBOT_BT (PIN 1234)"));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) B3 -> mod değiştir (sadece basıldığı an) / B3 -> toggle mode (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) setMode(!manualMode);
  lastB3 = b3;

  // 2) Telefondan ve Seri Monitör'den gelen komutlar / commands from the phone and the Serial Monitor
  String line;
  if (readBluetoothLine(line)) {
    iotbot.serialWrite(String(L("Telefondan: ", "From phone: ")) + line);
    handleCommand(line, true);
  }
  String cmd;
  if (readCommand(cmd)) handleCommand(rawLine, false);

  // 3) Otomatik gösteri: her 120 ms'de yanan LED bir kayar / auto show: the lit LED moves every 120 ms
  if (!manualMode && now - lastScanMs >= 120) {
    lastScanMs = now;
    for (uint8_t i = 0; i < kLedCount; i++) iotbot.digitalWritePin(kLedPins[i], i == scanPos);
    scanPos += scanDir;
    if (scanPos >= kLedCount - 1 || scanPos <= 0) scanDir = -scanDir;
  }

  // 4) LCD (300 ms'de bir, titremesiz) / LCD (every 300 ms, no flicker)
  if (now - lastScreenMs >= 300) {
    lastScreenMs = now;
    char text[41];
    // Satır 1: mod + röle, satır 2: LED'ler (en fazla 20 karakter) / row 1: mode + relay, row 2: LEDs (max 20 chars)
    if (manualMode) snprintf(text, sizeof(text), L("MANUEL   Röle:%s", "MANUAL  Relay:%s"), relayOn ? L("AÇIK", "ON") : L("KAPALI", "OFF"));
    else snprintf(text, sizeof(text), L("OTOMATİK Röle:%s", "AUTO    Relay:%s"), relayOn ? L("AÇIK", "ON") : L("KAPALI", "OFF"));
    lcdRow(1, text);
    snprintf(text, sizeof(text), "LED: %s", manualMode ? (ledsOn ? L("AÇIK", "ON") : L("KAPALI", "OFF")) : L("kayan ışık", "scanner"));
    lcdRow(2, text);
  }
}
