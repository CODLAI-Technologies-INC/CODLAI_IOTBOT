/*
 * TR: EĞLENCELİ KABLOSUZ ÖRNEK - MiniBot'un butonuyla IOTBOT'un akıllı LED'ini yönet
 *  - Bir MINIBOT'un butonuna her basışta, uzaktaki bu IOTBOT'un akıllı LED'inin
 *    (NeoPixel) efekti değişir. İki kart arasında kablo yok - sadece ESP-NOW.
 *  - Açılışta OTOMATİK mod: efektler 5 saniyede bir kendiliğinden değişir.
 *  - B3 butonu OTOMATİK <-> MANUEL. MANUEL modda efekti MiniBot'un butonu veya seri
 *    komutlar seçer. MiniBot'tan komut gelince kart kendiliğinden MANUEL'e geçer.
 *  - Efektler: 0 Gökkuşağı, 1 Gökkuşağı geçit, 2 Geçit (tek renk), 3 Renk dolgusu.
 *  - Önce bu kodu IOTBOT'a, sonra MINIBOT tarafını (CODLAI_MINIBOT kütüphanesindeki
 *    MINIBOT_IoTBot_SmartLED_Remote_Example.ino) bir MINIBOT'a yükleyin. MINIBOT
 *    kodundaki kPeerMac, bu IOTBOT'un MAC adresi olmalı ("durum" komutu gösterir).
 *  - Sadece deviceType 41 (MINIBOT, kütüphane örneklerinin kart kimliği) paketleri
 *    komut sayılır; bkz. COMMANDS_README "deviceType haritası".
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help                     -> komut listesi
 *      oto    / auto                     -> otomatik mod
 *      manuel / manual                   -> manuel mod
 *      efekt 2 / effect 2                -> efekti seç (0-3)
 *      renk 255 0 0 / color 255 0 0      -> tek renk yak
 *      parlaklik 80 / brightness 80      -> parlaklık (0-255)
 *      kapat  / off                      -> LED'leri söndür
 *      durum  / status                   -> MAC adresi, mod, efekt
 *      dil    / lang                     -> dili değiştir (Türkçe <-> English)
 *
 * EN: A FUN WIRELESS EXAMPLE - control the IOTBOT's smart LED with a MiniBot's button
 *  - Every press of a MINIBOT's button changes the effect of this remote IOTBOT's
 *    smart LED (NeoPixel). No wire between the boards - just ESP-NOW.
 *  - At startup AUTO mode: the effects change by themselves every 5 seconds.
 *  - The B3 button toggles AUTO <-> MANUAL. In MANUAL mode the MiniBot's button or the
 *    serial commands pick the effect. A command from the MiniBot switches to MANUAL.
 *  - Effects: 0 Rainbow, 1 Rainbow chase, 2 Chase (one color), 3 Color wipe.
 *  - Upload this to an IOTBOT, then the MINIBOT side (MINIBOT_IoTBot_SmartLED_Remote_
 *    Example.ino in the CODLAI_MINIBOT library) to a MINIBOT. kPeerMac in the MINIBOT
 *    sketch must be this IOTBOT's MAC address (the "status" command shows it).
 *  - Only packets with deviceType 41 (MINIBOT, the library example board ID) count as
 *    commands; see the COMMANDS_README "deviceType map".
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim                   -> command list
 *      auto   / oto                      -> auto mode
 *      manual / manuel                   -> manual mode
 *      effect 2 / efekt 2                -> pick the effect (0-3)
 *      color 255 0 0 / renk 255 0 0      -> light one color
 *      brightness 80 / parlaklik 80      -> brightness (0-255)
 *      off    / kapat                    -> turn the LEDs off
 *      status / durum                    -> MAC address, mode, effect
 *      lang   / dil                      -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Akıllı LED modülünü IO27 soketine takın (3 LED'li modül).
 * Plug the smart LED module into the IO27 socket (3-LED module).
 */

#define USE_ESPNOW
#define USE_NEOPIXEL
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

#define LED_PIN IO27 // Akıllı LED'in bağlı olduğu pin / pin the smart LED is connected to

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

const int kLedCount = 3; // moduleSmartLEDPrepare 3 LED hazırlar / prepares 3 LEDs
const char *namesTr[] = {"Gökkuşağı", "Gökkuşağı geçit", "Geçit", "Renk dolgusu", "Tek renk", "Kapalı"};
const char *namesEn[] = {"Rainbow", "Rainbow chase", "Chase", "Color wipe", "Solid color", "Off"};
const int EFFECT_SOLID = 4, EFFECT_OFF = 5;
const uint8_t kColors[4][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 170, 0}}; // Geçit/dolgu renkleri / chase/wipe colors

bool manualMode = false;   // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
int effect = 0;            // Çalan efekt / effect playing
int brightness = 150;
int solidR = 255, solidG = 0, solidB = 0;
int frame = 0;             // Efekt adımı / effect step
uint32_t lastFrameMs = 0, lastAutoChangeMs = 0, lastScreenMs = 0;
int remoteCount = 0;       // MiniBot'tan gelen komut sayısı / commands received from the MiniBot
bool lastB3 = false;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "PARLAKLIK" -> "parlaklik"
// Lower-cases and simplifies Turkish letters: "PARLAKLIK" -> "parlaklik"
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
// Efektler (beklemesiz: her çağrıda bir adım) / Effects (non-blocking: one step per call)
// ---------------------------------------------------------------------------
// Renk tekerleği: 0-359 derece -> kırmızı/yeşil/mavi / color wheel: 0-359 degrees -> red/green/blue
void hueToRgb(int hue, int &r, int &g, int &b) {
  hue %= 360;
  int x = (hue % 120) * 255 / 120;
  if (hue < 120)      { r = 255 - x; g = x;       b = 0; }
  else if (hue < 240) { r = 0;       g = 255 - x; b = x; }
  else                { r = x;       g = 0;       b = 255 - x; }
}

void effectStep() {
  int r, g, b;
  const uint8_t *c = kColors[(frame / 30) % 4]; // Geçit/dolgu rengi yavaşça değişir / chase/wipe color changes slowly
  switch (effect) {
    case 0: // Gökkuşağı: tüm LED'ler renk tekerleğinde döner / rainbow: all LEDs rotate on the color wheel
      for (int i = 0; i < kLedCount; i++) {
        hueToRgb(frame * 4 + i * 120, r, g, b);
        iotbot.moduleSmartLEDWrite(i, r, g, b);
      }
      break;
    case 1: // Gökkuşağı geçit: tek LED, renk değiştirerek kayar / rainbow chase: one LED moves, changing color
      hueToRgb(frame * 12, r, g, b);
      for (int i = 0; i < kLedCount; i++) {
        if (i == frame % kLedCount) iotbot.moduleSmartLEDWrite(i, r, g, b);
        else iotbot.moduleSmartLEDWrite(i, 0, 0, 0);
      }
      break;
    case 2: // Geçit: tek renk kayar / chase: one color moves
      for (int i = 0; i < kLedCount; i++) {
        if (i == frame % kLedCount) iotbot.moduleSmartLEDWrite(i, c[0], c[1], c[2]);
        else iotbot.moduleSmartLEDWrite(i, 0, 0, 0);
      }
      break;
    case 3: { // Renk dolgusu: LED'ler tek tek dolar, sonra söner / color wipe: LEDs fill one by one, then go dark
      int pos = frame % (kLedCount * 2);
      for (int i = 0; i < kLedCount; i++) {
        if (pos < kLedCount ? i <= pos : i > pos - kLedCount) iotbot.moduleSmartLEDWrite(i, c[0], c[1], c[2]);
        else iotbot.moduleSmartLEDWrite(i, 0, 0, 0);
      }
      break;
    }
    case EFFECT_SOLID:
      iotbot.moduleSmartLEDFill(solidR, solidG, solidB);
      break;
    default: // EFFECT_OFF
      iotbot.moduleSmartLEDClear();
      break;
  }
  frame++;
}

int frameDelayMs() {
  switch (effect) {
    case 0: return 30;
    case 1: return 120;
    case 2: return 150;
    case 3: return 200;
    default: return 500; // Tek renk/kapalı: arada bir tazele / solid/off: refresh now and then
  }
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

const char *effectName(int e) { return turkish ? namesTr[e] : namesEn[e]; }

void drawStaticScreen() {
  lcdRow(0, L("UZAKTAN KUMANDA LED", "REMOTE CONTROL LED"));
  lcdRow(3, manualMode ? L("B3: otomatik mod", "B3: auto mode") : L("B3: manuel kontrol", "B3: manual control"));
  lastScreenMs = 0;
}

void printHelp() {
  iotbot.serialWrite(L("---- UZAKTAN KUMANDALI LED - Komutlar ----", "---- REMOTE-CONTROLLED LED - Commands ----"));
  iotbot.serialWrite(L("  yardim          : bu liste", "  help            : this list"));
  iotbot.serialWrite(L("  oto / manuel    : mod seç", "  auto / manual   : choose mode"));
  iotbot.serialWrite(L("  efekt 0-3       : 0 Gökkuşağı, 1 Gökkuşağı geçit, 2 Geçit, 3 Renk dolgusu",
                       "  effect 0-3      : 0 Rainbow, 1 Rainbow chase, 2 Chase, 3 Color wipe"));
  iotbot.serialWrite(L("  renk 255 0 0    : tek renk", "  color 255 0 0   : one color"));
  iotbot.serialWrite(L("  parlaklik 0-255 : parlaklık", "  brightness 0-255: brightness"));
  iotbot.serialWrite(L("  kapat           : LED'leri söndür", "  off             : turn the LEDs off"));
  iotbot.serialWrite(L("  durum, dil", "  status, lang"));
  iotbot.serialWrite(L("  B3: OTOMATİK <-> MANUEL", "  B3: AUTO <-> MANUAL"));
}

void setEffect(int e) {
  effect = e;
  frame = 0;
  lastFrameMs = 0;
  lastScreenMs = 0;
  iotbot.serialWrite(String(L("Efekt: ", "Effect: ")) + effectName(e));
}

void setMode(bool manual) {
  manualMode = manual;
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  iotbot.serialWrite(manual ? L(">> MANUEL mod: efekti MiniBot'un butonu veya seri komutlar seçer.", ">> MANUAL mode: the MiniBot's button or serial commands pick the effect.")
                            : L(">> OTOMATİK mod: efektler 5 saniyede bir değişir.", ">> AUTO mode: the effects change every 5 seconds."));
  if (!manual) {
    lastAutoChangeMs = millis();
    if (effect > 3) setEffect(0);
  }
  drawStaticScreen();
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  String arg = (space < 0) ? "" : cmd.substring(space + 1);
  arg.trim();
  bool hasValue = arg.length() > 0;
  int value = arg.toInt();

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oto" || word == "otomatik" || word == "auto") {
    setMode(false);
  } else if (word == "manuel" || word == "manual") {
    setMode(true);
  } else if ((word == "efekt" || word == "effect") && hasValue) {
    if (value < 0 || value > 3) { iotbot.serialWrite(L("Efekt 0-3 olmalı.", "Effect must be 0-3.")); return; }
    if (!manualMode) setMode(true);
    setEffect(value);
  } else if ((word == "renk" || word == "color") && hasValue) {
    int r = 0, g = 0, b = 0;
    if (sscanf(arg.c_str(), "%d %d %d", &r, &g, &b) != 3) { iotbot.serialWrite(L("Kullanım: renk 255 0 0", "Usage: color 255 0 0")); return; }
    if (!manualMode) setMode(true);
    solidR = constrain(r, 0, 255); solidG = constrain(g, 0, 255); solidB = constrain(b, 0, 255);
    setEffect(EFFECT_SOLID);
  } else if ((word == "parlaklik" || word == "brightness") && hasValue) {
    brightness = constrain(value, 0, 255);
    iotbot.moduleSmartLEDSetBrightness(brightness);
    iotbot.serialWrite(String(L("Parlaklık: ", "Brightness: ")) + brightness);
  } else if (word == "kapat" || word == "off") {
    if (!manualMode) setMode(true);
    setEffect(EFFECT_OFF);
  } else if (word == "durum" || word == "status") {
    iotbot.serialWrite(String(L("Benim MAC adresim (MiniBot'taki kPeerMac): ", "My MAC address (kPeerMac on the MiniBot): ")) + WiFi.macAddress());
    iotbot.serialWrite(String(L("Mod: ", "Mode: ")) + (manualMode ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO")) +
                       L("   Efekt: ", "   Effect: ") + effectName(effect) + L("   Parlaklık: ", "   Brightness: ") + brightness +
                       L("   MiniBot komutu: ", "   MiniBot commands: ") + remoteCount);
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
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
  iotbot.moduleSmartLEDPrepare(LED_PIN);
  iotbot.moduleSmartLEDSetBrightness(brightness);

  iotbot.initESPNow();
  iotbot.startListening();

  iotbot.lcdClear();
  drawStaticScreen();
  iotbot.serialWrite(L("Hazır - MiniBot'tan komut bekleniyor.", "Ready - waiting for a command from the MiniBot."));
  iotbot.serialWrite(String(L("Benim MAC adresim: ", "My MAC address: ")) + WiFi.macAddress());
  printHelp();
  lastAutoChangeMs = millis();
}

void loop() {
  uint32_t now = millis();

  // 1) B3 -> mod değiştir (sadece basıldığı an) / B3 -> toggle mode (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) setMode(!manualMode);
  lastB3 = b3;

  // 2) MiniBot'tan komut: action alanı efekt numarasını taşır (paket yapısı değişmemeli)
  // 2) Command from the MiniBot: the action field carries the effect number (keep the packet layout)
  if (iotbot.newData) {
    iotbot.newData = false;
    if (iotbot.receivedData.deviceType == 41) { // 41 = MINIBOT (örnek kart kimliği / example board id)
      remoteCount++;
      if (!manualMode) setMode(true);
      iotbot.serialWrite(L("MiniBot'tan komut geldi!", "Command from the MiniBot!"));
      setEffect(iotbot.receivedData.action % 4);
    }
  }

  // 3) Seri komutlar / serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 4) Otomatik: 5 saniyede bir sonraki efekt / auto: next effect every 5 seconds
  if (!manualMode && now - lastAutoChangeMs >= 5000) {
    lastAutoChangeMs = now;
    setEffect((effect + 1) % 4);
  }

  // 5) Efektin bir adımını çiz / draw one step of the effect
  if (now - lastFrameMs >= (uint32_t)frameDelayMs()) {
    lastFrameMs = now;
    effectStep();
  }

  // 6) LCD (300 ms'de bir, titremesiz) / LCD (every 300 ms, no flicker)
  if (now - lastScreenMs >= 300) {
    lastScreenMs = now;
    char line[41];
    snprintf(line, sizeof(line), L("Mod: %s", "Mode: %s"), manualMode ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO"));
    lcdRow(1, line);
    lcdRow(2, effectName(effect));
  }
}
