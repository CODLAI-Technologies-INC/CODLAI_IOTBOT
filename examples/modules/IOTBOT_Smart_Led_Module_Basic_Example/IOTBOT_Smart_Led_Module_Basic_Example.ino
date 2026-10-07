/*
 * TR: AKILLI LED (NeoPixel) MODÜLÜ - Işık efektleri + Manuel renk kontrolü
 *  - Açılışta OTOMATİK mod çalışır: gökkuşağı -> kayan ışık -> renk dolumu ->
 *    nefes alma efektleri sırayla oynar (hepsi loop'u bekletmeden çalışır).
 *  - B3 butonuna basınca MANUEL moda geçer: potansiyometre = renk (renk çemberi),
 *    B1 = parlaklık (%25 -> %50 -> %75 -> %100). B3'e tekrar basınca otomatiğe döner.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim          / help           -> komut listesi
 *      oto             / auto           -> otomatik mod (efektler)
 *      manuel          / manual         -> manuel mod (pot + B1)
 *      renk 255 0 0    / color 255 0 0  -> R G B rengi (0-255, manuel moda geçer)
 *      kirmizi, yesil, mavi, sari, beyaz, mor, turuncu
 *      red, green, blue, yellow, white, purple, orange -> hazır renkler
 *      parlaklik 50    / brightness 50  -> parlaklık % (0-100)
 *      kapat           / off            -> LED'leri söndür (manuel moda geçer)
 *      dil             / lang           -> dili değiştir (Türkçe <-> English)
 *
 * EN: SMART LED (NeoPixel) MODULE - Light effects + Manual color control
 *  - At startup AUTO mode runs: rainbow -> running light -> color wipe ->
 *    breathing effects play in turn (all without blocking the loop).
 *  - Press B3 to switch to MANUAL mode: potentiometer = color (color wheel),
 *    B1 = brightness (25% -> 50% -> 75% -> 100%). Press B3 again for auto mode.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help            / yardim         -> command list
 *      auto            / oto            -> auto mode (effects)
 *      manual          / manuel         -> manual mode (pot + B1)
 *      color 255 0 0   / renk 255 0 0   -> R G B color (0-255, switches to manual)
 *      red, green, blue, yellow, white, purple, orange
 *      kirmizi, yesil, mavi, sari, beyaz, mor, turuncu -> preset colors
 *      brightness 50   / parlaklik 50   -> brightness % (0-100)
 *      off             / kapat          -> LEDs off (switches to manual)
 *      lang            / dil            -> switch language (Turkish <-> English)
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

// Otomatik efektler / Auto effects
enum Effect { FX_RAINBOW, FX_RUNNING, FX_WIPE, FX_BREATHE, FX_COUNT };
const uint32_t EFFECT_MS = 6000; // Her efektin süresi / time of each effect

bool manualMode = false;    // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
int effect = FX_RAINBOW;
uint32_t effectStartMs = 0;
uint32_t lastFrameMs = 0;
int red = 255, green = 0, blue = 0; // Manuel renk / manual color
int brightnessPct = 50;     // Parlaklık % / brightness %
bool dirty = true;          // Manuel renk yeniden yazılmalı mı? / must the manual color be rewritten?
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
// Renk yardımcıları / Color helpers
// ---------------------------------------------------------------------------
// Renk çemberi: 0° kırmızı, 120° yeşil, 240° mavi -> R, G, B (0-255)
// Color wheel: 0° red, 120° green, 240° blue -> R, G, B (0-255)
void hueToRgb(int hue, int &r, int &g, int &b) {
  hue = ((hue % 360) + 360) % 360;
  int x = (hue % 60) * 255 / 60; // Bölge içindeki ilerleme / progress inside the sector
  switch (hue / 60) {
    case 0:  r = 255;     g = x;       b = 0;       break; // kırmızı -> sarı / red -> yellow
    case 1:  r = 255 - x; g = 255;     b = 0;       break; // sarı -> yeşil / yellow -> green
    case 2:  r = 0;       g = 255;     b = x;       break; // yeşil -> camgöbeği / green -> cyan
    case 3:  r = 0;       g = 255 - x; b = 255;     break; // camgöbeği -> mavi / cyan -> blue
    case 4:  r = x;       g = 0;       b = 255;     break; // mavi -> mor / blue -> magenta
    default: r = 255;     g = 0;       b = 255 - x; break; // mor -> kırmızı / magenta -> red
  }
}

void applyBrightness() {
  // Kütüphane parlaklığı 0-255 ister / the library wants brightness as 0-255
  iotbot.moduleSmartLEDSetBrightness(brightnessPct * 255 / 100);
  dirty = true; // Renkleri yeni parlaklıkla tekrar yaz / rewrite the colors with the new brightness
}

// ---------------------------------------------------------------------------
// Otomatik efektler: her çağrıda sadece BİR kare çizilir (bekleme yok).
// Auto effects: each call draws only ONE frame (no waiting).
// ---------------------------------------------------------------------------
const char *effectName(int fx) {
  switch (fx) {
    case FX_RAINBOW: return L("Gökkuşağı", "Rainbow");
    case FX_RUNNING: return L("Kayan ışık", "Running light");
    case FX_WIPE:    return L("Renk dolumu", "Color wipe");
    default:         return L("Nefes alma", "Breathing");
  }
}

void drawEffectFrame(uint32_t t) { // t = efektin başından beri geçen ms / ms since the effect started
  int r, g, b;
  if (effect == FX_RAINBOW) {
    // Her LED renk çemberinde 120° geride; renkler yavaşça döner.
    // Each LED is 120° behind on the color wheel; the colors turn slowly.
    for (int i = 0; i < LED_COUNT; i++) {
      hueToRgb(t / 10 + i * 120, r, g, b);
      iotbot.moduleSmartLEDWrite(i, r, g, b);
    }
  } else if (effect == FX_RUNNING) {
    // Tek bir ışık LED'den LED'e kayar, her turda renk değişir.
    // One light runs from LED to LED, the color changes every round.
    int pos = (t / 200) % LED_COUNT;
    hueToRgb((t / 600) * 60, r, g, b);
    for (int i = 0; i < LED_COUNT; i++) {
      if (i == pos) iotbot.moduleSmartLEDWrite(i, r, g, b);
      else iotbot.moduleSmartLEDWrite(i, 0, 0, 0);
    }
  } else if (effect == FX_WIPE) {
    // LED'ler tek tek yeni renge boyanır / LEDs are painted with a new color one by one
    int stepNo = t / 300;                // Her 300 ms'de bir LED / one LED every 300 ms
    int lit = stepNo % (LED_COUNT + 1);  // Kaç LED yeni renkte / how many LEDs have the new color
    int wipeRound = stepNo / (LED_COUNT + 1);
    int r2, g2, b2;
    hueToRgb(wipeRound * 120, r, g, b);    // Yeni renk / new color
    hueToRgb((wipeRound - 1) * 120, r2, g2, b2); // Önceki renk / previous color
    for (int i = 0; i < LED_COUNT; i++) {
      if (i < lit) iotbot.moduleSmartLEDWrite(i, r, g, b);
      else iotbot.moduleSmartLEDWrite(i, r2, g2, b2);
    }
  } else {
    // Mavi ışık 2 saniyede bir yavaşça yanıp söner (üçgen dalga).
    // Blue light slowly fades in and out every 2 seconds (triangle wave).
    int phase = t % 2000;
    int level = (phase < 1000) ? phase * 255 / 1000 : (2000 - phase) * 255 / 1000;
    iotbot.moduleSmartLEDFill(0, level / 3, level);
  }
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
// lcdWriteFixedTxt Türkçe harfleri (ç, ğ, ı, ö, ş, ü) LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- AKILLI LED - Komutlar ----", "---- SMART LED - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oto           : otomatik mod (efektler)", "  auto          : auto mode (effects)"));
  iotbot.serialWrite(L("  manuel        : manuel mod (pot + B1)", "  manual        : manual mode (pot + B1)"));
  iotbot.serialWrite(L("  renk R G B    : renk, her biri 0-255 (ör: renk 255 0 0)", "  color R G B   : color, each 0-255 (e.g. color 255 0 0)"));
  iotbot.serialWrite(L("  kirmizi yesil mavi sari beyaz mor turuncu", "  red green blue yellow white purple orange"));
  iotbot.serialWrite(L("  parlaklik 0-100: parlaklık %", "  brightness 0-100: brightness %"));
  iotbot.serialWrite(L("  kapat         : LED'leri söndür", "  off           : LEDs off"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : OTOMATİK <-> MANUEL", "  B3 button     : AUTO <-> MANUAL"));
  iotbot.serialWrite(L("  B1 butonu     : parlaklık (manuel)", "  B1 button     : brightness (manual)"));
}

void drawStaticScreen() {
  lcdRow(0, L("     AKILLI LED", "     SMART LED"));
  lcdRow(3, manualMode ? L("Pot:renk B1:parlak", "Pot:color B1:bright") : L("B3: manuel kontrol", "B3: manual control"));
  lastScreenMs = 0; // Değerleri hemen çiz / draw the values right away
}

void startEffect(int fx) {
  effect = fx;
  effectStartMs = millis();
  iotbot.serialWrite(String(L("Efekt: ", "Effect: ")) + effectName(effect));
}

void setMode(bool manual) {
  manualMode = manual;
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  iotbot.serialWrite(manual ? L(">> MANUEL mod: pot = renk, B1 = parlaklık.", ">> MANUAL mode: pot = color, B1 = brightness.")
                            : L(">> OTOMATİK mod: ışık efektleri oynuyor.", ">> AUTO mode: light effects are playing."));
  lastPotHue = -1; // Manuelde pot hemen geçerli olsun / pot takes effect immediately in manual
  dirty = true;
  if (!manual) startEffect(FX_RAINBOW);
  drawStaticScreen();
}

// Seri komutla gelen rengi ayarla; pot ancak çevrilince bu rengi değiştirir.
// Set a color from a serial command; the pot changes it only when turned.
void setColor(int r, int g, int b) {
  if (!manualMode) setMode(true);
  red = constrain(r, 0, 255);
  green = constrain(g, 0, 255);
  blue = constrain(b, 0, 255);
  lastPotHue = map(iotbot.potentiometerRead(), 0, 4095, 0, 359);
  dirty = true;
  iotbot.serialWrite(String(L("Renk: R=", "Color: R=")) + red + " G=" + green + " B=" + blue);
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  // Hazır renk adı mı? / is it a preset color name?
  for (int i = 0; i < COLOR_COUNT; i++) {
    if (word == COLORS[i].cmdTr || word == COLORS[i].en) {
      setColor(COLORS[i].r, COLORS[i].g, COLORS[i].b);
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
  } else if ((word == "renk" || word == "color") && sscanf(cmd.c_str(), "%*s %d %d %d", &r, &g, &b) == 3) {
    setColor(r, g, b);
  } else if ((word == "parlaklik" || word == "brightness") && hasValue) {
    brightnessPct = constrain(value, 0, 100);
    applyBrightness();
    iotbot.serialWrite(String(L("Parlaklık: %", "Brightness: ")) + brightnessPct + (turkish ? "" : "%"));
  } else if (word == "kapat" || word == "off") {
    setColor(0, 0, 0);
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
  iotbot.serialWrite(L("Akıllı LED testi başladı.", "Smart LED test started."));
  printHelp();
  startEffect(FX_RAINBOW);
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
    // 3a) Pot -> renk çemberi (titreşimi yok saymak için 4° eşik)
    // 3a) Pot -> color wheel (4° threshold ignores jitter)
    int potHue = map(iotbot.potentiometerRead(), 0, 4095, 0, 359);
    if (lastPotHue < 0 || abs(potHue - lastPotHue) >= 4) {
      lastPotHue = potHue;
      hueToRgb(potHue, red, green, blue);
      dirty = true;
    }

    // 3b) B1 -> parlaklık %25 -> %50 -> %75 -> %100 -> %25 ...
    bool b1 = iotbot.button1Read();
    if (b1 && !lastB1 && now - lastB1Ms > 200) {
      lastB1Ms = now;
      brightnessPct = (brightnessPct >= 100) ? 25 : (brightnessPct / 25 + 1) * 25;
      applyBrightness();
      iotbot.serialWrite(String(L("Parlaklık: %", "Brightness: ")) + brightnessPct + (turkish ? "" : "%"));
    }
    lastB1 = b1;

    // 3c) Renk sadece değişince yazılır / the color is written only when it changed
    if (dirty) {
      dirty = false;
      iotbot.moduleSmartLEDFill(red, green, blue);
      lastScreenMs = 0;
    }
  } else {
    // 3d) Otomatik: 6 sn'de bir sonraki efekt, her 20 ms'de bir kare.
    // 3d) Auto: next effect every 6 s, one frame every 20 ms.
    if (now - effectStartMs >= EFFECT_MS) startEffect((effect + 1) % FX_COUNT);
    if (now - lastFrameMs >= 20) {
      lastFrameMs = now;
      drawEffectFrame(millis() - effectStartMs);
    }
  }

  // 4) LCD (250 ms'de bir, titremesiz) / LCD (every 250 ms, no flicker)
  if (millis() - lastScreenMs >= 250) {
    lastScreenMs = millis();
    char line[41];
    snprintf(line, sizeof(line), L("Mod: %-8s %3d%%", "Mode: %-6s  %3d%%"), manualMode ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO"), brightnessPct);
    lcdRow(1, line);
    if (manualMode) snprintf(line, sizeof(line), "R:%3d G:%3d B:%3d", red, green, blue);
    else snprintf(line, sizeof(line), L("Efekt: %s", "FX: %s"), effectName(effect));
    lcdRow(2, line);
  }
}
