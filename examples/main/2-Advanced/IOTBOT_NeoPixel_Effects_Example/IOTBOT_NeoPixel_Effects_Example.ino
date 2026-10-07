/*
 * TR: NEOPİXEL (AKILLI LED) EFEKTLERİ - Otomatik gösteri + Manuel kontrol
 *  - Kolaylık fonksiyonlarını gösterir: tek renge boyama (Fill), söndürme (Clear),
 *    parlaklık (SetBrightness), yanıp sönme (Blink) ve "nefes alma" (Breathe).
 *  - Açılışta OTOMATİK mod: bu efektler sırayla, kendi kendine oynar.
 *  - B3 butonuna basınca MANUEL moda geçer: potansiyometre PARLAKLIĞI ayarlar, B1
 *    butonu SIRADAKİ RENGE geçer, B2 butonu 3 kez yakıp söndürür. B3'e tekrar basınca
 *    otomatik moda döner.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help                   -> komut listesi
 *      oto    / auto                   -> otomatik mod (gösteri)
 *      manuel / manual                 -> manuel mod
 *      kirmizi/red, yesil/green, mavi/blue, sari/yellow, mor/purple, beyaz/white
 *                                      -> o renge boya
 *      renk 255 0 0 / color 255 0 0    -> istediğiniz renk (kırmızı yeşil mavi, 0-255)
 *      parlaklik 80 / brightness 80    -> parlaklık (0-255)
 *      yanson / blink                  -> 3 kez yanıp sön
 *      nefes  / breathe                -> nefes alma efekti
 *      kapat  / off                    -> söndür
 *      dil    / lang                   -> dili değiştir (Türkçe <-> English)
 *    Bir renk/parlaklık komutu otomatik moddayken kartı MANUEL moda geçirir.
 *
 * EN: NEOPIXEL (SMART LED) EFFECTS - Automatic show + Manual control
 *  - Shows the convenience functions: fill with one color (Fill), turn off (Clear),
 *    brightness (SetBrightness), blink on/off (Blink) and "breathing" (Breathe).
 *  - At startup AUTO mode: these effects play one after another by themselves.
 *  - Press B3 to switch to MANUAL mode: the potentiometer sets the BRIGHTNESS, the B1
 *    button goes to the NEXT COLOR, the B2 button blinks 3 times. Press B3 again to go
 *    back to auto mode.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim                 -> command list
 *      auto   / oto                    -> auto mode (show)
 *      manual / manuel                 -> manual mode
 *      red/kirmizi, green/yesil, blue/mavi, yellow/sari, purple/mor, white/beyaz
 *                                      -> fill with that color
 *      color 255 0 0 / renk 255 0 0    -> any color (red green blue, 0-255)
 *      brightness 80 / parlaklik 80    -> brightness (0-255)
 *      blink  / yanson                 -> blink 3 times
 *      breathe / nefes                 -> breathing effect
 *      off    / kapat                  -> turn off
 *      lang   / dil                    -> switch language (Turkish <-> English)
 *    A color/brightness command in auto mode switches the board to MANUAL.
 *
 * Bağlantı / Wiring: Akıllı LED'i P1-P5 soketlerinden BİRİNE takın ve aşağıdaki LED_PIN
 * değerini o soketin sinyaline göre ayarlayın. / Plug the smart LED into ONE of the
 * P1-P5 sockets and set LED_PIN below to match that socket's signal.
 * Not / Note: Blink ve Breathe kütüphanede kısa süre (en fazla ~1 sn) bekler.
 *             Blink and Breathe wait briefly inside the library (at most ~1 s).
 */

#define USE_NEOPIXEL
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

#define LED_PIN IO27 // Akıllı LED'in bağlı olduğu pin / pin the smart LED is connected to
// Desteklenen pinler / Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Renk paleti (B1 sırayla geçer) / color palette (B1 steps through it)
struct NamedColor { const char *tr; const char *en; uint8_t r, g, b; };
const NamedColor kColors[] = {
  {"Kırmızı", "Red", 255, 0, 0},     {"Yeşil", "Green", 0, 255, 0},    {"Mavi", "Blue", 0, 0, 255},
  {"Sarı", "Yellow", 255, 200, 0},   {"Mor", "Purple", 150, 0, 255},   {"Turkuaz", "Cyan", 0, 200, 200},
  {"Beyaz", "White", 255, 255, 255}};
const int kColorCount = 7;

bool manualMode = false;  // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
int colorIndex = 0;       // -1 = özel renk / custom color
int red = 255, green = 0, blue = 0;
int brightness = 255;
bool ledOn = true;
int lastPotBright = -1;
int autoStep = 0;
uint32_t nextStepMs = 0, lastScreenMs = 0;
bool lastB1 = false, lastB2 = false, lastB3 = false;

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
// LED ve ekran / LED and screen
// ---------------------------------------------------------------------------
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

// Manuel moddaki rengi ve parlaklığı LED'e uygular / applies the manual color and brightness to the LED
void applyManual() {
  iotbot.moduleSmartLEDSetBrightness(brightness);
  if (ledOn) iotbot.moduleSmartLEDFill(red, green, blue);
  else iotbot.moduleSmartLEDClear();
  lastScreenMs = 0;
}

void selectColor(int index) {
  colorIndex = index;
  red = kColors[index].r; green = kColors[index].g; blue = kColors[index].b;
  ledOn = true;
  iotbot.serialWrite(String(L("Renk: ", "Color: ")) + (turkish ? kColors[index].tr : kColors[index].en));
  applyManual();
}

void drawStaticScreen() {
  lcdRow(0, L("  NEOPİXEL EFEKTLERİ", "  NEOPIXEL EFFECTS"));
  if (!manualMode) lcdRow(1, L("Mod: OTOMATİK", "Mode: AUTO")); // Manuelde satır 1 parlaklığı gösterir / in manual row 1 shows the brightness
  lcdRow(3, manualMode ? L("Pot:ışık B1:renk", "Pot:light B1:color") : L("B3: manuel kontrol", "B3: manual control"));
  lastScreenMs = 0;
}

void printHelp() {
  iotbot.serialWrite(L("---- NEOPİXEL EFEKTLERİ - Komutlar ----", "---- NEOPIXEL EFFECTS - Commands ----"));
  iotbot.serialWrite(L("  yardim          : bu liste", "  help            : this list"));
  iotbot.serialWrite(L("  oto / manuel    : mod seç", "  auto / manual   : choose mode"));
  iotbot.serialWrite(L("  kirmizi, yesil, mavi, sari, mor, beyaz : renk", "  red, green, blue, yellow, purple, white : color"));
  iotbot.serialWrite(L("  renk 255 0 0    : istediğiniz renk", "  color 255 0 0   : any color"));
  iotbot.serialWrite(L("  parlaklik 0-255 : parlaklık", "  brightness 0-255: brightness"));
  iotbot.serialWrite(L("  yanson / nefes  : yanıp sön / nefes al", "  blink / breathe : blink / breathe"));
  iotbot.serialWrite(L("  kapat           : söndür", "  off             : turn off"));
  iotbot.serialWrite(L("  dil             : English'e geç", "  lang            : switch to Turkish"));
  iotbot.serialWrite(L("  B3: OTOMATİK <-> MANUEL. Manuel: Pot=parlaklık, B1=renk, B2=yanıp sön",
                       "  B3: AUTO <-> MANUAL. Manual: Pot=brightness, B1=color, B2=blink"));
}

void setMode(bool manual) {
  manualMode = manual;
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  if (manual) {
    iotbot.serialWrite(L(">> MANUEL mod: Pot=parlaklık, B1=sıradaki renk, B2=yanıp sön.", ">> MANUAL mode: Pot=brightness, B1=next color, B2=blink."));
    lastPotBright = -1; // Pot hemen geçerli olsun / the pot takes effect right away
    applyManual();
  } else {
    iotbot.serialWrite(L(">> OTOMATİK mod: efekt gösterisi.", ">> AUTO mode: effect show."));
    autoStep = 0;
    nextStepMs = millis();
  }
  drawStaticScreen();
}

// Otomatik gösterinin bir adımı; her adım ne kadar sürecek onu döndürür (ms)
// One step of the auto show; returns how long the step lasts (ms)
uint32_t autoShowStep(int step, char *label, size_t size) {
  switch (step) {
    case 0:
      iotbot.moduleSmartLEDSetBrightness(255);
      iotbot.moduleSmartLEDFill(255, 0, 0);
      snprintf(label, size, "%s", L("Fill: Kırmızı", "Fill: Red"));
      return 1000;
    case 1:
      iotbot.moduleSmartLEDFill(0, 255, 0);
      snprintf(label, size, "%s", L("Fill: Yeşil", "Fill: Green"));
      return 1000;
    case 2:
      iotbot.moduleSmartLEDClear();
      snprintf(label, size, "%s", L("Clear: Söndü", "Clear: Off"));
      return 1000;
    case 3:
      iotbot.moduleSmartLEDSetBrightness(40);
      iotbot.moduleSmartLEDFill(0, 0, 255);
      snprintf(label, size, "%s", L("Parlaklık düşük", "Brightness low"));
      return 1000;
    case 4:
      iotbot.moduleSmartLEDSetBrightness(255);
      iotbot.moduleSmartLEDFill(0, 0, 255);
      snprintf(label, size, "%s", L("Parlaklık yüksek", "Brightness high"));
      return 1000;
    case 5: case 6: case 7:
      // Blink'i tek tek çağırıyoruz (her biri 0,4 sn) - B3 arada çalışır
      // We call Blink one at a time (0.4 s each) - B3 works in between
      iotbot.moduleSmartLEDBlink(255, 255, 0, 1, 200);
      snprintf(label, size, "%s", L("Blink: sarı", "Blink: yellow"));
      return 50;
    case 8:
      iotbot.moduleSmartLEDBreathe(150, 0, 255, 1000); // 1 sn sürer / takes 1 s
      snprintf(label, size, "%s", L("Breathe: mor", "Breathe: purple"));
      return 50;
    default:
      iotbot.moduleSmartLEDClear();
      snprintf(label, size, "%s", L("Clear (mola)", "Clear (pause)"));
      return 2000;
  }
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  String arg = (space < 0) ? "" : cmd.substring(space + 1);
  arg.trim();
  bool hasValue = arg.length() > 0;

  // İsimli renkler / named colors
  const char *trNames[] = {"kirmizi", "yesil", "mavi", "sari", "mor", "turkuaz", "beyaz"};
  const char *enNames[] = {"red", "green", "blue", "yellow", "purple", "cyan", "white"};
  for (int i = 0; i < kColorCount; i++) {
    if (word == trNames[i] || word == enNames[i]) {
      if (!manualMode) setMode(true);
      selectColor(i);
      return;
    }
  }

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oto" || word == "otomatik" || word == "auto") {
    setMode(false);
  } else if (word == "manuel" || word == "manual") {
    setMode(true);
  } else if ((word == "renk" || word == "color") && hasValue) {
    int r = 0, g = 0, b = 0;
    if (sscanf(arg.c_str(), "%d %d %d", &r, &g, &b) != 3) {
      iotbot.serialWrite(L("Kullanım: renk 255 0 0", "Usage: color 255 0 0"));
      return;
    }
    if (!manualMode) setMode(true);
    red = constrain(r, 0, 255); green = constrain(g, 0, 255); blue = constrain(b, 0, 255);
    colorIndex = -1;
    ledOn = true;
    applyManual();
    iotbot.serialWrite(String(L("Renk: ", "Color: ")) + red + " " + green + " " + blue);
  } else if ((word == "parlaklik" || word == "brightness") && hasValue) {
    if (!manualMode) setMode(true);
    brightness = constrain(arg.toInt(), 0, 255);
    // Seri komutla verilen değeri pot hemen ezmesin / so the pot does not override the serial value right away
    lastPotBright = map(iotbot.potentiometerRead(), 0, 4095, 0, 255);
    applyManual();
    iotbot.serialWrite(String(L("Parlaklık: ", "Brightness: ")) + brightness);
  } else if (word == "yanson" || word == "blink") {
    if (!manualMode) setMode(true);
    iotbot.moduleSmartLEDBlink(red, green, blue, 3, 200);
    applyManual();
  } else if (word == "nefes" || word == "breathe") {
    if (!manualMode) setMode(true);
    iotbot.moduleSmartLEDBreathe(red, green, blue, 2000);
    applyManual();
  } else if (word == "kapat" || word == "off") {
    if (!manualMode) setMode(true);
    ledOn = false;
    applyManual();
    iotbot.serialWrite(L("LED söndü.", "LED off."));
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
  iotbot.lcdClear();
  drawStaticScreen();
  iotbot.serialWrite(L("NeoPixel efektleri başladı.", "NeoPixel effects started."));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) B3 -> mod değiştir (sadece basıldığı an) / B3 -> toggle mode (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) setMode(!manualMode);
  lastB3 = b3;

  // 2) Seri komutlar / serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  if (!manualMode) {
    // 3a) OTOMATİK: sıradaki efekt adımı / AUTO: next effect step
    if ((int32_t)(now - nextStepMs) >= 0) {
      char label[41];
      uint32_t duration = autoShowStep(autoStep, label, sizeof(label));
      if (autoStep != 6 && autoStep != 7) iotbot.serialWrite(label); // Blink'i bir kez yaz / print Blink once
      lcdRow(2, label);
      autoStep = (autoStep + 1) % 10;
      nextStepMs = millis() + duration;
    }
  } else {
    // 3b) MANUEL: pot = parlaklık (8 birimden fazla değişince), B1 = renk, B2 = yanıp sön
    // 3b) MANUAL: pot = brightness (when it changes by more than 8), B1 = color, B2 = blink
    int potBright = map(iotbot.potentiometerRead(), 0, 4095, 0, 255);
    if (lastPotBright < 0 || abs(potBright - lastPotBright) > 8) {
      lastPotBright = potBright;
      brightness = potBright;
      applyManual();
    }
    bool b1 = iotbot.button1Read();
    if (b1 && !lastB1) selectColor((colorIndex + 1) % kColorCount);
    lastB1 = b1;
    bool b2 = iotbot.button2Read();
    if (b2 && !lastB2) {
      iotbot.moduleSmartLEDBlink(red, green, blue, 3, 150);
      applyManual();
    }
    lastB2 = b2;

    // LCD (300 ms'de bir) / LCD (every 300 ms)
    if (now - lastScreenMs >= 300) {
      lastScreenMs = now;
      char line[41];
      if (!ledOn) snprintf(line, sizeof(line), "%s", L("Renk: KAPALI", "Color: OFF"));
      else if (colorIndex >= 0) snprintf(line, sizeof(line), L("Renk: %s", "Color: %s"), turkish ? kColors[colorIndex].tr : kColors[colorIndex].en);
      else snprintf(line, sizeof(line), "RGB %d %d %d", red, green, blue);
      lcdRow(2, line);
      snprintf(line, sizeof(line), L("MANUEL Parlaklık:%d", "MANUAL Bright: %d"), brightness);
      lcdRow(1, line);
    }
  }
}
