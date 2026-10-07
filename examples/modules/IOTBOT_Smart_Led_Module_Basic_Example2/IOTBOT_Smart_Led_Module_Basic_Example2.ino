/*
 * TR: AKILLI LED (NeoPixel) MODÜLÜ - Örnek 2: Her LED'i AYRI kontrol etmek
 *  Modülde 3 LED var ve her birine ayrı renk verilebilir. LED numaraları 0'dan
 *  başlar: LED 0, LED 1, LED 2  ->  iotbot.moduleSmartLEDWrite(numara, R, G, B)
 *  - Açılışta OTOMATİK mod çalışır: LED'ler tek tek yanar (kırmızı, yeşil, mavi),
 *    sonra renkler LED'ler arasında döner, sonra beyaz bir ışık 0-1-2-1-0 gezer.
 *  - B3 butonuna basınca MANUEL moda geçer:
 *      B1 = sıradaki LED'i seç (LED 0 -> LED 1 -> LED 2 -> HEPSİ),
 *      potansiyometre = seçili LED'in rengi, joystick yukarı/aşağı = parlaklık.
 *    B3'e tekrar basınca otomatik moda döner.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim          / help           -> komut listesi
 *      oto             / auto           -> otomatik mod
 *      manuel          / manual         -> manuel mod
 *      led 1                            -> LED 1'i seç (0, 1 veya 2)
 *      hepsi           / all            -> tüm LED'leri seç
 *      renk 255 0 0    / color 255 0 0  -> seçili LED'in rengi (R G B, 0-255)
 *      kirmizi, yesil, mavi, sari, beyaz, mor, turuncu -> seçili LED'e hazır renk
 *      parlaklik 50    / brightness 50  -> parlaklık % (0-100)
 *      kapat           / off            -> tüm LED'leri söndür
 *      dil             / lang           -> dili değiştir (Türkçe <-> English)
 *    (Renk/LED komutları otomatik moddaysa manuel moda geçirir.)
 *
 * EN: SMART LED (NeoPixel) MODULE - Example 2: controlling EACH LED separately
 *  The module has 3 LEDs and each one can have its own color. LED numbers start
 *  at 0: LED 0, LED 1, LED 2  ->  iotbot.moduleSmartLEDWrite(number, R, G, B)
 *  - At startup AUTO mode runs: the LEDs light up one by one (red, green, blue),
 *    then the colors rotate between the LEDs, then a white light travels 0-1-2-1-0.
 *  - Press B3 to switch to MANUAL mode:
 *      B1 = select the next LED (LED 0 -> LED 1 -> LED 2 -> ALL),
 *      potentiometer = color of the selected LED, joystick up/down = brightness.
 *    Press B3 again to go back to auto mode.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help            / yardim         -> command list
 *      auto            / oto            -> auto mode
 *      manual          / manuel         -> manual mode
 *      led 1                            -> select LED 1 (0, 1 or 2)
 *      all             / hepsi          -> select all LEDs
 *      color 255 0 0   / renk 255 0 0   -> color of the selected LED (R G B, 0-255)
 *      red, green, blue, yellow, white, purple, orange -> preset color for the selected LED
 *      brightness 50   / parlaklik 50   -> brightness % (0-100)
 *      off             / kapat          -> all LEDs off
 *      lang            / dil            -> switch language (Turkish <-> English)
 *    (Color/LED commands switch to manual mode when in auto mode.)
 *
 * Bağlantı / Wiring: Akıllı LED modülünü (3 LED) IO27'ye bağlı sokete takın.
 * Desteklenen pinler / Supported pins: IO25 - IO26 - IO27 - IO32 - IO33
 * Not / Note: Akıllı LED fonksiyonları için sketch'in başında USE_NEOPIXEL tanımlı olmalı.
 *             USE_NEOPIXEL must be defined at the top of the sketch for the Smart LED functions.
 */

#define USE_NEOPIXEL
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

#define SMART_LED_PIN IO27 // Akıllı LED'in bağlı olduğu pin / Pin the Smart LED is connected to
const int LED_COUNT = 3;   // Modüldeki LED sayısı / number of LEDs on the module
const int ALL_LEDS = LED_COUNT; // "Seçili LED" = 3 ise hepsi seçili / "selected LED" = 3 means all

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Hazır renkler: komut (TR), ekranda görünen ad (TR), komut/ad (EN), R, G, B
// Preset colors: command (TR), shown name (TR), command/name (EN), R, G, B
struct NamedColor { const char *cmdTr; const char *nameTr; const char *en; uint8_t r, g, b; };
const NamedColor COLORS[] = {
  {"kirmizi", "kırmızı", "red",    255, 0,   0},
  {"yesil",   "yeşil",   "green",  0,   255, 0},
  {"mavi",    "mavi",    "blue",   0,   0,   255},
  {"sari",    "sarı",    "yellow", 255, 160, 0},
  {"beyaz",   "beyaz",   "white",  255, 255, 255},
  {"mor",     "mor",     "purple", 160, 0,   255},
  {"turuncu", "turuncu", "orange", 255, 70,  0},
};
const int COLOR_COUNT = sizeof(COLORS) / sizeof(COLORS[0]);

// Her LED'in rengi: ledColor[LED][0=R, 1=G, 2=B] / color of every LED
int ledColor[LED_COUNT][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}};

// Otomatik gösteri 500 ms'lik adımlardan oluşur / the auto show is made of 500 ms steps
const uint32_t AUTO_STEP_MS = 500;
const int AUTO_STEPS = 18; // 4 tek tek + 6 dönme + 8 gezinme / 4 one-by-one + 6 rotate + 8 scanner

bool manualMode = false;    // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
int selectedLed = 0;        // 0, 1, 2 veya ALL_LEDS / 0, 1, 2 or ALL_LEDS
int brightnessPct = 50;     // Parlaklık % / brightness %
int autoStep = 0;
uint32_t autoStepMs = 0;
bool dirty = true;          // LED'ler yeniden yazılmalı mı? / must the LEDs be rewritten?
uint32_t lastJoyMs = 0;
uint32_t lastScreenMs = 0;
bool lastB3 = false;
bool lastB1 = false;
uint32_t lastB1Ms = 0;      // B1 için basit sıçrama önleme / simple debounce for B1
int lastPotHue = -1;        // Potansiyometrenin son rengi (0-359) / last pot hue (0-359)

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "KIRMIZI" -> "kirmizi"
// Lower-cases and simplifies Turkish letters: "KIRMIZI" -> "kirmizi"
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
// LED yardımcıları / LED helpers
// ---------------------------------------------------------------------------
// Renk çemberi: 0° kırmızı, 120° yeşil, 240° mavi -> R, G, B (0-255)
// Color wheel: 0° red, 120° green, 240° blue -> R, G, B (0-255)
void hueToRgb(int hue, int &r, int &g, int &b) {
  hue = ((hue % 360) + 360) % 360;
  int x = (hue % 60) * 255 / 60; // Bölge içindeki ilerleme / progress inside the sector
  switch (hue / 60) {
    case 0:  r = 255;     g = x;       b = 0;       break;
    case 1:  r = 255 - x; g = 255;     b = 0;       break;
    case 2:  r = 0;       g = 255;     b = x;       break;
    case 3:  r = 0;       g = 255 - x; b = 255;     break;
    case 4:  r = x;       g = 0;       b = 255;     break;
    default: r = 255;     g = 0;       b = 255 - x; break;
  }
}

// Seçili LED'e (veya hepsine) renk ver / give a color to the selected LED (or all of them)
void setSelectedColor(int r, int g, int b) {
  for (int i = 0; i < LED_COUNT; i++) {
    if (selectedLed == ALL_LEDS || selectedLed == i) {
      ledColor[i][0] = constrain(r, 0, 255);
      ledColor[i][1] = constrain(g, 0, 255);
      ledColor[i][2] = constrain(b, 0, 255);
    }
  }
  dirty = true;
}

// Bütün LED'leri ledColor dizisinden yaz / write every LED from the ledColor array
void showLeds() {
  for (int i = 0; i < LED_COUNT; i++) {
    iotbot.moduleSmartLEDWrite(i, ledColor[i][0], ledColor[i][1], ledColor[i][2]);
  }
}

void applyBrightness() {
  iotbot.moduleSmartLEDSetBrightness(brightnessPct * 255 / 100); // Kütüphane 0-255 ister / library wants 0-255
  dirty = true; // Renkleri yeni parlaklıkla tekrar yaz / rewrite the colors with the new brightness
}

// Otomatik gösterinin bir adımını hazırla (bekleme yok).
// Prepare one step of the auto show (no waiting).
void autoShowStep(int stepNo) {
  const int base[LED_COUNT][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}}; // kırmızı, yeşil, mavi / red, green, blue
  for (int i = 0; i < LED_COUNT; i++) {
    int src = -1; // Bu LED hangi temel rengi alacak? (-1 = sönük) / which base color for this LED? (-1 = off)
    bool white = false;
    if (stepNo < 4) {
      // 1) Tek tek yan: adım 0 -> LED 0, adım 1 -> LED 0-1, adım 2 -> hepsi, adım 3 -> sönük
      // 1) One by one: step 0 -> LED 0, step 1 -> LEDs 0-1, step 2 -> all, step 3 -> off
      if (stepNo < 3 && i <= stepNo) src = i;
    } else if (stepNo < 10) {
      // 2) Renkler LED'ler arasında döner / colors rotate between the LEDs
      src = (i + stepNo - 4) % LED_COUNT;
    } else {
      // 3) Beyaz ışık gezer: 0,1,2,1,0,1,2,1 / a white light travels: 0,1,2,1,0,1,2,1
      const int scan[8] = {0, 1, 2, 1, 0, 1, 2, 1};
      white = (scan[stepNo - 10] == i);
    }
    for (int c = 0; c < 3; c++) {
      if (white) ledColor[i][c] = 255;
      else ledColor[i][c] = (src >= 0) ? base[src][c] : 0;
    }
  }
  dirty = true;
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
// lcdWriteFixedTxt Türkçe harfleri (ç, ğ, ı, ö, ş, ü) LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- AKILLI LED 2 - Komutlar ----", "---- SMART LED 2 - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oto           : otomatik mod", "  auto          : auto mode"));
  iotbot.serialWrite(L("  manuel        : manuel mod (B1 + pot + joystick)", "  manual        : manual mode (B1 + pot + joystick)"));
  iotbot.serialWrite(L("  led 0-2       : o LED'i seç", "  led 0-2       : select that LED"));
  iotbot.serialWrite(L("  hepsi         : tüm LED'leri seç", "  all           : select all LEDs"));
  iotbot.serialWrite(L("  renk R G B    : seçili LED'in rengi (ör: renk 0 0 255)", "  color R G B   : color of the selected LED (e.g. color 0 0 255)"));
  iotbot.serialWrite(L("  kirmizi yesil mavi sari beyaz mor turuncu", "  red green blue yellow white purple orange"));
  iotbot.serialWrite(L("  parlaklik 0-100: parlaklık %", "  brightness 0-100: brightness %"));
  iotbot.serialWrite(L("  kapat         : tüm LED'leri söndür", "  off           : all LEDs off"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : OTOMATİK <-> MANUEL", "  B3 button     : AUTO <-> MANUAL"));
  iotbot.serialWrite(L("  B1 butonu     : sıradaki LED (manuel)", "  B1 button     : next LED (manual)"));
}

void drawStaticScreen() {
  lcdRow(0, L("    AKILLI LED 2", "    SMART LED 2"));
  lcdRow(3, manualMode ? L("B1:LED Pot:renk Joy", "B1:LED Pot:color Joy") : L("B3: manuel kontrol", "B3: manual control"));
  lastScreenMs = 0; // Değerleri hemen çiz / draw the values right away
}

void printSelected() {
  if (selectedLed == ALL_LEDS) iotbot.serialWrite(L("Seçili: HEPSİ", "Selected: ALL"));
  else iotbot.serialWrite(String(L("Seçili: LED ", "Selected: LED ")) + selectedLed);
}

void setMode(bool manual) {
  manualMode = manual;
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  iotbot.serialWrite(manual ? L(">> MANUEL mod: B1 = LED seç, pot = renk, joystick = parlaklık.", ">> MANUAL mode: B1 = select LED, pot = color, joystick = brightness.")
                            : L(">> OTOMATİK mod: LED'ler sırayla gösteri yapıyor.", ">> AUTO mode: the LEDs run a show."));
  // Manuelde pot ancak çevrilince renk değiştirsin (gösterinin renkleri kalsın).
  // In manual the pot changes the color only when turned (the show's colors stay).
  lastPotHue = map(iotbot.potentiometerRead(), 0, 4095, 0, 359);
  if (!manual) {
    autoStep = 0;
    autoStepMs = millis();
    autoShowStep(autoStep);
  }
  drawStaticScreen();
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  // Hazır renk adı mı? / is it a preset color name?
  for (int i = 0; i < COLOR_COUNT; i++) {
    if (word == COLORS[i].cmdTr || word == COLORS[i].en) {
      if (!manualMode) setMode(true);
      setSelectedColor(COLORS[i].r, COLORS[i].g, COLORS[i].b);
      iotbot.serialWrite(String(L("Hazır renk: ", "Preset color: ")) + (turkish ? COLORS[i].nameTr : COLORS[i].en));
      return;
    }
  }

  int r, g, b;
  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oto" || word == "otomatik" || word == "auto") {
    setMode(false);
  } else if (word == "manuel" || word == "manual") {
    setMode(true);
  } else if (word == "hepsi" || word == "all" || cmd == "led hepsi" || cmd == "led all") {
    if (!manualMode) setMode(true);
    selectedLed = ALL_LEDS;
    printSelected();
    lastScreenMs = 0;
  } else if (word == "led" && hasValue && value >= 0 && value < LED_COUNT) {
    if (!manualMode) setMode(true);
    selectedLed = value;
    printSelected();
    lastScreenMs = 0;
  } else if ((word == "renk" || word == "color") && sscanf(cmd.c_str(), "%*s %d %d %d", &r, &g, &b) == 3) {
    if (!manualMode) setMode(true);
    setSelectedColor(r, g, b);
    iotbot.serialWrite(String(L("Renk: R=", "Color: R=")) + constrain(r, 0, 255) + " G=" + constrain(g, 0, 255) + " B=" + constrain(b, 0, 255));
  } else if ((word == "parlaklik" || word == "brightness") && hasValue) {
    brightnessPct = constrain(value, 0, 100);
    applyBrightness();
    iotbot.serialWrite(String(L("Parlaklık: %", "Brightness: ")) + brightnessPct + (turkish ? "" : "%"));
  } else if (word == "kapat" || word == "off") {
    if (!manualMode) setMode(true);
    int keep = selectedLed;
    selectedLed = ALL_LEDS; // "kapat" her zaman hepsini söndürür / "off" always turns all off
    setSelectedColor(0, 0, 0);
    selectedLed = keep;
    iotbot.serialWrite(L("Tüm LED'ler söndü.", "All LEDs off."));
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
  iotbot.begin();             // IoTBot başlatılıyor / Initialize IoTBot
  iotbot.serialStart(115200); // Seri haberleşme / Serial communication
  iotbot.moduleSmartLEDPrepare(SMART_LED_PIN); // LED'leri hazırla (hepsi sönük) / prepare the LEDs (all off)
  applyBrightness();
  iotbot.lcdClear();
  drawStaticScreen();
  iotbot.serialWrite(L("Akıllı LED testi 2 başladı.", "Smart LED test 2 started."));
  printHelp();
  autoStepMs = millis();
  autoShowStep(autoStep);
}

void loop() {
  uint32_t now = millis();

  // 1) B3 -> mod değiştir (sadece basıldığı an) / B3 -> toggle mode (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) setMode(!manualMode);
  lastB3 = b3;

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  if (manualMode) {
    // 3a) B1 -> sıradaki LED: 0 -> 1 -> 2 -> HEPSİ -> 0 ...
    bool b1 = iotbot.button1Read();
    if (b1 && !lastB1 && now - lastB1Ms > 200) {
      lastB1Ms = now;
      selectedLed = (selectedLed + 1) % (LED_COUNT + 1);
      printSelected();
      lastScreenMs = 0;
    }
    lastB1 = b1;

    // 3b) Pot -> seçili LED'in rengi (sadece gerçekten çevrilince, 4° eşik)
    // 3b) Pot -> color of the selected LED (only when really turned, 4° threshold)
    int potHue = map(iotbot.potentiometerRead(), 0, 4095, 0, 359);
    if (abs(potHue - lastPotHue) >= 4) {
      lastPotHue = potHue;
      int r, g, b;
      hueToRgb(potHue, r, g, b);
      setSelectedColor(r, g, b);
    }

    // 3c) Joystick Y -> parlaklık: itili tuttukça her 50 ms'de %2 değişir (ortası boşta).
    // 3c) Joystick Y -> brightness: changes 2% every 50 ms while pushed (center = idle).
    if (now - lastJoyMs >= 50) {
      lastJoyMs = now;
      int joy = iotbot.joystickYRead(); // 0-4095, ortası ~2000 / center ~2000
      int change = 0;
      if (joy > 3000) change = 2;
      else if (joy < 1000) change = -2;
      if (change != 0) {
        int newPct = constrain(brightnessPct + change, 0, 100);
        if (newPct != brightnessPct) {
          brightnessPct = newPct;
          applyBrightness();
        }
      }
    }
  } else if (now - autoStepMs >= AUTO_STEP_MS) {
    // 3d) Otomatik: her 500 ms'de gösterinin bir sonraki adımı / Auto: next show step every 500 ms
    autoStepMs = now;
    autoStep = (autoStep + 1) % AUTO_STEPS;
    autoShowStep(autoStep);
  }

  // 4) LED'leri sadece bir şey değişince yaz / write the LEDs only when something changed
  if (dirty) {
    dirty = false;
    showLeds();
    lastScreenMs = 0;
  }

  // 5) LCD (250 ms'de bir, titremesiz) / LCD (every 250 ms, no flicker)
  if (millis() - lastScreenMs >= 250) {
    lastScreenMs = millis();
    char line[41];
    snprintf(line, sizeof(line), L("Mod: %-8s %3d%%", "Mode: %-6s  %3d%%"), manualMode ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO"), brightnessPct);
    lcdRow(1, line);
    if (manualMode) {
      // Seçili LED ve rengi (hepsi seçiliyse LED 0'ın rengi) / selected LED and its color (LED 0's color if all)
      int show = (selectedLed == ALL_LEDS) ? 0 : selectedLed;
      if (selectedLed == ALL_LEDS) snprintf(line, sizeof(line), L("HEPSİ %3d,%3d,%3d", "ALL %3d,%3d,%3d"), ledColor[show][0], ledColor[show][1], ledColor[show][2]);
      else snprintf(line, sizeof(line), "LED%d: %3d,%3d,%3d", selectedLed, ledColor[show][0], ledColor[show][1], ledColor[show][2]);
    } else {
      // Her LED'in durumunu harfle göster: K/Y/M/B veya - (sönük) / show each LED as a letter
      char state[LED_COUNT + 1];
      for (int i = 0; i < LED_COUNT; i++) {
        int r = ledColor[i][0], g = ledColor[i][1], b = ledColor[i][2];
        if (r == 0 && g == 0 && b == 0) state[i] = '-';
        else if (r && g && b) state[i] = turkish ? 'B' : 'W';
        else if (r) state[i] = turkish ? 'K' : 'R';
        else if (g) state[i] = turkish ? 'Y' : 'G';
        else state[i] = turkish ? 'M' : 'B';
      }
      state[LED_COUNT] = '\0';
      snprintf(line, sizeof(line), L("LED 0-1-2: %c %c %c", "LED 0-1-2: %c %c %c"), state[0], state[1], state[2]);
    }
    lcdRow(2, line);
  }
}
