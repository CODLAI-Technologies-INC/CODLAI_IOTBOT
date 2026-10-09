/*
 * TR: GERÇEK PROJE - Sese Duyarlı LED'ler. Mikrofon ortamdaki sesi dinler,
 * 3 akıllı LED de sesin şiddetine göre yanar - tıpkı müzik setlerindeki "VU
 * metre" gibi: hafif seste sadece YEŞİL, daha yüksekte SARI, çok yüksek seste
 * KIRMIZI LED de yanar. Işıklar sesle ANINDA yükselir ama YAVAŞÇA söner;
 * böylece titremez, göze hoş gelir. LCD'de 20 kutuluk bir ses çubuğu ve son 1
 * saniyenin en yüksek seviyesini gösteren "tepe" işareti (|) var.
 *  - B3 butonu modu sırayla değiştirir:
 *      VU METRE (otomatik) -> PARTİ (otomatik: sesin şiddeti LED renklerini
 *      değiştirir; müzik çalın ve izleyin!) -> MANUEL (rengi potansiyometre ile
 *      siz seçersiniz) -> VU METRE ...
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim         / help            -> komut listesi
 *      oto  (= vu)    / auto            -> VU metre modu
 *      parti          / party           -> parti modu
 *      manuel         / manual          -> manuel mod (pot = renk)
 *      renk 255 0 0   / color 255 0 0   -> LED rengi R G B (manuel moda geçer)
 *      parlaklik 80   / brightness 80   -> manuel parlaklık % (manuel moda geçer)
 *      kapat          / off             -> LED'leri söndür (manuel moda geçer)
 *      dil            / lang            -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Sound Reactive LEDs. The microphone listens to the
 * room and the 3 smart LEDs light up with the loudness - just like the "VU
 * meter" on a music system: a soft sound lights only the GREEN LED, louder
 * adds YELLOW, very loud adds RED too. The lights jump up INSTANTLY with the
 * sound but fade out SLOWLY, so they don't flicker and look nice. The LCD
 * shows a 20-cell level bar and a "peak" mark (|) holding the loudest level
 * of the last second.
 *  - Button B3 cycles the mode:
 *      VU METER (auto) -> PARTY (auto: loudness changes the LED colors; play
 *      some music and watch!) -> MANUAL (you pick the color with the
 *      potentiometer) -> VU METER ...
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help           / yardim          -> command list
 *      auto (= vu)    / oto             -> VU meter mode
 *      party          / parti           -> party mode
 *      manual         / manuel          -> manual mode (pot = color)
 *      color 255 0 0  / renk 255 0 0    -> LED color R G B (switches to manual)
 *      brightness 80  / parlaklik 80    -> manual brightness % (switches to manual)
 *      off            / kapat           -> LEDs off (switches to manual)
 *      lang           / dil             -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Mikrofon (ses sensörü) modülünü P4 soketine (IO32),
 * akıllı LED (3 LED'li NeoPixel) modülünü P1 soketine (IO25) takın. Pot ve B3
 * kart üzerindedir. / Plug the microphone (sound sensor) module into socket
 * P4 (IO32) and the smart LED (3-LED NeoPixel) module into socket P1 (IO25).
 * The pot and B3 are on the board.
 */

#define USE_NEOPIXEL
#include <IOTBOT.h>

IOTBOT iotbot;

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

#define MIC_PIN IO32 // P4 soketi (analog için ADC1 pini) / socket P4 (ADC1 pin for analog)
#define LED_PIN IO25 // P1 soketi / socket P1

namespace {
  constexpr uint32_t kSampleWindowMs = 10; // Her seviye ölçümü 10 ms / each level reading is 10 ms
  constexpr int kLoudLevel = 1200;         // Bu seviye = %100 (duyarlılık) / this level = 100% (sensitivity)
  constexpr float kAttack = 0.7f;          // Hızlı yükseliş (0-1) / fast attack (0-1)
  constexpr float kDecay = 0.92f;          // Yavaş sönüş: her adımda %8 azal / slow decay: -8% per step
  constexpr uint32_t kPeakHoldMs = 1000;   // Tepe işareti bekleme / peak mark hold time
  constexpr uint32_t kScreenMs = 200;      // LCD yenileme aralığı / LCD refresh interval
  constexpr int kMaxBrightness = 150;      // LED en fazla parlaklık (0-255) / LED max brightness
  constexpr int kPotMoveRaw = 80;          // Pot "gerçekten çevrildi" eşiği / pot "really turned" threshold

  enum Mode { VU_METER, PARTY, MANUAL };
  Mode mode = VU_METER;
  int baseline = 2048;   // Sessizken ortalama ham değer / mean raw value in silence
  int noiseFloor = 0;    // Sessiz odanın gürültü seviyesi / noise level of the quiet room
  float smoothPct = 0;   // Yumuşatılmış ses yüzdesi / smoothed loudness percent
  int peakPct = 0;
  uint32_t peakMs = 0;
  uint8_t partyHue = 0;  // Parti modunda renk çarkı konumu / color wheel position in party mode
  int manualR = 255, manualG = 0, manualB = 0; // Manuel renk / manual color
  int manualBrightness = 100;                  // Manuel parlaklık % / manual brightness %
  bool manualDirty = true;                     // LED'ler yeniden yazılsın mı? / rewrite the LEDs?
  int lastPotRaw = -1000;
  bool b3WasDown = false;
  uint32_t lastB3Ms = 0;
  uint32_t lastScreenMs = 0;
}

// Enum parametreli fonksiyonlarin prototipleri: Arduino IDE otomatik
// prototipleri enum tanimindan ONCE yazdigi icin "declared void" hatasi
// veriyordu. / Prototypes of functions taking an enum: the Arduino IDE
// writes its auto-prototypes BEFORE the enum ("declared void" error).
void setMode(Mode m);

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
// Mikrofon ve LED / Microphone and LEDs
// ---------------------------------------------------------------------------
// Mikrofon sesi, baseline etrafında salınan bir dalga olarak verir; 10 ms içinde dalganın
// baseline'dan en çok uzaklaştığı miktar = ses seviyesi. / The mic wave swings around the
// baseline; its biggest distance from the baseline within 10 ms = the sound level.
int readLevel() {
  int maxDev = 0;
  uint32_t start = millis();
  while (millis() - start < kSampleWindowMs) {
    int dev = abs(iotbot.moduleMicRead(MIC_PIN) - baseline);
    if (dev > maxDev) maxDev = dev;
  }
  return maxDev;
}

void calibrate() {
  iotbot.lcdWriteMid(L("SESE DUYARLI LED", "SOUND REACTIVE LED"), "", L("Sessiz olun...", "Please be quiet..."), "");
  long sum = 0;
  long count = 0;
  uint32_t start = millis();
  while (millis() - start < 500) { // Dalganın orta çizgisi / middle line of the wave
    sum += iotbot.moduleMicRead(MIC_PIN);
    count++;
    delay(1);
  }
  baseline = sum / count;
  start = millis();
  while (millis() - start < 500) { // Sessiz odanın gürültüsü / noise of the quiet room
    int level = readLevel();
    if (level > noiseFloor) noiseFloor = level;
  }
}

// Rengi (r,g,b) yüzde parlaklıkla yazar. / Writes color (r,g,b) at a percent brightness.
void setLed(int index, int r, int g, int b, int percent) {
  int scale = percent * kMaxBrightness / 100;
  iotbot.moduleSmartLEDWrite(index, r * scale / 255, g * scale / 255, b * scale / 255);
}

// Renk çarkı: 0-255 arası bir sayıyı kırmızı -> yeşil -> mavi -> kırmızı renge çevirir.
// Color wheel: turns a number 0-255 into red -> green -> blue -> red.
void wheel(uint8_t pos, int &r, int &g, int &b) {
  if (pos < 85) {
    r = 255 - pos * 3; g = pos * 3; b = 0;
  } else if (pos < 170) {
    pos -= 85;
    r = 0; g = 255 - pos * 3; b = pos * 3;
  } else {
    pos -= 170;
    r = pos * 3; g = 0; b = 255 - pos * 3;
  }
}

void showVuMeter(int pct) {
  // Her LED'in kendi bölgesi var: LED1 %0-33, LED2 %33-66, LED3 %66-100.
  // Each LED has its own zone: LED1 0-33%, LED2 33-66%, LED3 66-100%.
  setLed(0, 0, 255, 0, constrain(pct * 3, 0, 100));          // Yeşil / green
  setLed(1, 255, 160, 0, constrain((pct - 33) * 3, 0, 100)); // Sarı / yellow
  setLed(2, 255, 0, 0, constrain((pct - 66) * 3, 0, 100));   // Kırmızı / red
}

void showParty(int pct) {
  partyHue++; // Renkler yavaşça döner... / colors slowly rotate...
  for (int i = 0; i < 3; i++) {
    int r, g, b;
    // ...ve yüksek ses renk çarkında büyük bir sıçrama yapar. / ...and loud sound jumps far on the wheel.
    wheel((uint8_t)(partyHue + pct * 2 + i * 85), r, g, b);
    setLed(i, r, g, b, 15 + pct * 85 / 100);
  }
}

// Manuel renk: sadece değişince yazılır. / Manual color: written only when it changes.
void showManual() {
  if (!manualDirty) return;
  manualDirty = false;
  for (int i = 0; i < 3; i++) setLed(i, manualR, manualG, manualB, manualBrightness);
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
// lcdWriteFixedTxt Türkçe harfleri (ç, ğ, ı, ö, ş, ü) LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- SESE DUYARLI LED - Komutlar ----", "---- SOUND REACTIVE LED - Commands ----"));
  iotbot.serialWrite(L("  yardim          : bu liste", "  help            : this list"));
  iotbot.serialWrite(L("  oto (vu)        : VU metre modu", "  auto (vu)       : VU meter mode"));
  iotbot.serialWrite(L("  parti           : parti modu", "  party           : party mode"));
  iotbot.serialWrite(L("  manuel          : manuel mod (pot = renk)", "  manual          : manual mode (pot = color)"));
  iotbot.serialWrite(L("  renk R G B      : LED rengi (0-255)", "  color R G B     : LED color (0-255)"));
  iotbot.serialWrite(L("  parlaklik 0-100 : manuel parlaklık %", "  brightness 0-100: manual brightness %"));
  iotbot.serialWrite(L("  kapat           : LED'leri söndür", "  off             : LEDs off"));
  iotbot.serialWrite(L("  dil             : English'e geç", "  lang            : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu       : VU -> PARTİ -> MANUEL", "  B3 button       : VU -> PARTY -> MANUAL"));
}

void showModeScreen() {
  lcdRow(0, L("  SESE DUYARLI LED", " SOUND REACTIVE LED"));
  const char *text;
  if (mode == VU_METER) text = L("Mod: VU METRE  (B3)", "Mode: VU METER (B3)");
  else if (mode == PARTY) text = L("Mod: PARTİ     (B3)", "Mode: PARTY    (B3)");
  else text = L("Mod: MANUEL Pot:renk", "Mode: MANUAL Pot:clr");
  lcdRow(3, text);
  lastScreenMs = 0;
}

void setMode(Mode m) {
  mode = m;
  iotbot.moduleSmartLEDClear();
  manualDirty = true;
  lastPotRaw = iotbot.potentiometerRead(); // Pot ancak çevrilince rengi değiştirir / the pot changes the color only when turned
  iotbot.buzzerPlayTone(m == MANUAL ? 1500 : 1000, 60);
  if (m == VU_METER) iotbot.serialWrite(L(">> VU METRE modu (otomatik).", ">> VU METER mode (auto)."));
  else if (m == PARTY) iotbot.serialWrite(L(">> PARTİ modu (otomatik): müzik çalın!", ">> PARTY mode (auto): play some music!"));
  else iotbot.serialWrite(L(">> MANUEL mod: rengi potansiyometre ile seçin.", ">> MANUAL mode: pick the color with the potentiometer."));
  showModeScreen();
}

void drawScreen(int pct) {
  // Satır 1: 20 kutuluk ses çubuğu + tepe işareti. / Row 1: 20-cell level bar + peak mark.
  int cells = pct * 20 / 100;
  int peakCell = constrain(peakPct * 20 / 100, 0, 19);
  char line[41];
  for (int i = 0; i < 20; i++) {
    line[i] = (i < cells) ? '\xFF' : ((i == peakCell && peakPct > 0) ? '|' : ' '); // 0xFF = dolu kutu / full block
  }
  line[20] = '\0';
  lcdRow(1, line);
  if (mode == MANUAL) {
    snprintf(line, sizeof(line), "R%3d G%3d B%3d %%%3d", manualR, manualG, manualB, manualBrightness);
  } else {
    snprintf(line, sizeof(line), L("Ses:%3d%%  Tepe:%3d%%", "Level:%3d%% Peak:%3d%%"), pct, peakPct);
  }
  lcdRow(2, line);
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  String rest = hasValue ? cmd.substring(space + 1) : "";

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oto" || word == "otomatik" || word == "auto" || word == "vu") {
    setMode(VU_METER);
  } else if (word == "parti" || word == "party") {
    setMode(PARTY);
  } else if (word == "manuel" || word == "manual") {
    setMode(MANUAL);
  } else if ((word == "renk" || word == "color") && hasValue) {
    int r = 0, g = 0, b = 0;
    if (sscanf(rest.c_str(), "%d %d %d", &r, &g, &b) != 3) {
      iotbot.serialWrite(L("Kullanım: renk 255 0 0", "Usage: color 255 0 0"));
      return;
    }
    if (mode != MANUAL) setMode(MANUAL);
    manualR = constrain(r, 0, 255);
    manualG = constrain(g, 0, 255);
    manualB = constrain(b, 0, 255);
    manualDirty = true;
    iotbot.serialWrite(String(L("Renk: ", "Color: ")) + manualR + " " + manualG + " " + manualB);
  } else if ((word == "parlaklik" || word == "brightness") && hasValue) {
    if (mode != MANUAL) setMode(MANUAL);
    manualBrightness = constrain(rest.toInt(), 0, 100);
    manualDirty = true;
    iotbot.serialWrite(String(L("Parlaklık: %", "Brightness: ")) + manualBrightness + L("", "%"));
  } else if (word == "kapat" || word == "off") {
    if (mode != MANUAL) setMode(MANUAL);
    manualR = manualG = manualB = 0;
    manualDirty = true;
    iotbot.serialWrite(L("LED'ler söndü.", "LEDs off."));
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    showModeScreen();
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
  delay(500); // Açılış sesi bitsin / let the startup beep fade
  calibrate();
  iotbot.lcdClear();
  showModeScreen();
  iotbot.serialWrite(L("Sese duyarlı LED hazır. B3 = mod değiştir.", "Sound reactive LED ready. B3 = change mode."));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) B3: basıldığı an bir kez mod değiştir (200 ms sıçrama koruması): VU -> PARTİ -> MANUEL.
  // 1) B3: change mode once per press (200 ms debounce): VU -> PARTY -> MANUAL.
  bool b3Down = iotbot.button3Read();
  if (b3Down && !b3WasDown && now - lastB3Ms > 200) {
    lastB3Ms = now;
    setMode(mode == VU_METER ? PARTY : (mode == PARTY ? MANUAL : VU_METER));
  }
  b3WasDown = b3Down;

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Ses seviyesini gürültünün üstünde yüzdeye çevir. / Turn loudness above the noise into a percent.
  int level = readLevel();
  int pct = constrain(map(level - noiseFloor, 0, kLoudLevel, 0, 100), 0, 100);

  // Hızlı yükseliş / yavaş sönüş: yükselirken hedefe çabuk yaklaş, düşerken yavaşça azal.
  // Fast attack / slow decay: rise quickly towards louder values, fall slowly.
  if (pct > smoothPct) {
    smoothPct += (pct - smoothPct) * kAttack;
  } else {
    smoothPct *= kDecay;
  }
  int shown = (int)smoothPct;

  // Tepe tutma: en yüksek değer 1 sn bekler, sonra yavaşça iner.
  // Peak hold: the highest value waits 1 s, then slowly falls.
  if (shown >= peakPct) {
    peakPct = shown;
    peakMs = now;
  } else if (now - peakMs > kPeakHoldMs && peakPct > 0) {
    peakPct--;
  }

  if (mode == VU_METER) {
    showVuMeter(shown);
  } else if (mode == PARTY) {
    showParty(shown);
  } else {
    // MANUEL: pot çevrilince renk çarkından renk seç. / MANUAL: pick a wheel color when the pot is turned.
    int raw = iotbot.potentiometerRead();
    if (abs(raw - lastPotRaw) >= kPotMoveRaw) {
      lastPotRaw = raw;
      wheel((uint8_t)map(raw, 0, 4095, 0, 255), manualR, manualG, manualB);
      manualDirty = true;
    }
    showManual();
  }

  if (now - lastScreenMs >= kScreenMs) {
    lastScreenMs = now;
    drawScreen(shown);
  }
}
