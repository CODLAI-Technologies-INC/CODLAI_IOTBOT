// TR: GERCEK PROJE - Sese Duyarli LED'ler. Mikrofon ortamdaki sesi dinler,
// 3 akilli LED de sesin siddetine gore yanar - tipki muzik setlerindeki
// "VU metre" gibi: hafif seste sadece YESIL, daha yuksekte SARI, cok yuksek
// seste KIRMIZI LED de yanar. Isiklar sesle ANINDA yukselir ama YAVASCA
// soner; boylece titremez, goze hos gelir. LCD'de 20 kutuluk bir ses cubugu
// ve son 1 saniyenin en yuksek seviyesini gosteren "tepe" isareti (|) var.
// B3 butonu modu degistirir: VU metre <-> PARTI modu (sesin siddeti LED
// renklerini degistirir; muzik calin ve izleyin!).
// EN: A REAL PROJECT - Sound Reactive LEDs. The microphone listens to the
// room and the 3 smart LEDs light up with the loudness - just like the "VU
// meter" on a music system: a soft sound lights only the GREEN LED, louder
// adds YELLOW, very loud adds RED too. The lights jump up INSTANTLY with the
// sound but fade out SLOWLY, so they don't flicker and look nice. The LCD
// shows a 20-cell level bar and a "peak" mark (|) holding the loudest level
// of the last second. Button B3 switches the mode: VU meter <-> PARTY mode
// (loudness changes the LED colors; play some music and watch!).
//
// Baglanti / Wiring: Mikrofon (ses sensoru) modulunu P4 soketine (IO32),
// akilli LED (3 LED'li NeoPixel) modulunu P1 soketine (IO25) takin. / Plug
// the microphone (sound sensor) module into socket P4 (IO32) and the smart
// LED (3-LED NeoPixel) module into socket P1 (IO25).

#define USE_NEOPIXEL
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define MIC_PIN IO32 // P4 soketi (analog icin ADC1 pini) / socket P4 (ADC1 pin for analog)
#define LED_PIN IO25 // P1 soketi / socket P1

namespace {
  constexpr uint32_t kSampleWindowMs = 10; // Her seviye olcumu 10 ms / each level reading is 10 ms
  constexpr int kLoudLevel = 1200;         // Bu seviye = %100 (duyarlilik) / this level = 100% (sensitivity)
  constexpr float kAttack = 0.7f;          // Hizli yukselis (0-1) / fast attack (0-1)
  constexpr float kDecay = 0.92f;          // Yavas sonus: her adimda %8 azal / slow decay: -8% per step
  constexpr uint32_t kPeakHoldMs = 1000;   // Tepe isareti bekleme / peak mark hold time
  constexpr uint32_t kScreenMs = 200;      // LCD yenileme araligi / LCD refresh interval
  constexpr int kMaxBrightness = 150;      // LED en fazla parlaklik (0-255) / LED max brightness

  enum Mode { VU_METER, PARTY };
  Mode mode = VU_METER;
  int baseline = 2048;   // Sessizken ortalama ham deger / mean raw value in silence
  int noiseFloor = 0;    // Sessiz odanin gurultu seviyesi / noise level of the quiet room
  float smoothPct = 0;   // Yumusatilmis ses yuzdesi / smoothed loudness percent
  int peakPct = 0;
  uint32_t peakMs = 0;
  uint8_t partyHue = 0;  // Parti modunda renk carki konumu / color wheel position in party mode
  bool b3WasDown = false;
  uint32_t lastB3Ms = 0;
  uint32_t lastScreenMs = 0;
}

// Mikrofon sesi, baseline etrafinda salinan bir dalga olarak verir; 10 ms icinde dalganin
// baseline'dan en cok uzaklastigi miktar = ses seviyesi. / The mic wave swings around the
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
  iotbot.lcdWriteMid(turkish ? "SESE DUYARLI LED" : "SOUND REACTIVE LED", "",
                      turkish ? "Sessiz olun..." : "Please be quiet...", "");
  long sum = 0;
  long count = 0;
  uint32_t start = millis();
  while (millis() - start < 500) { // Dalganin orta cizgisi / middle line of the wave
    sum += iotbot.moduleMicRead(MIC_PIN);
    count++;
    delay(1);
  }
  baseline = sum / count;
  start = millis();
  while (millis() - start < 500) { // Sessiz odanin gurultusu / noise of the quiet room
    int level = readLevel();
    if (level > noiseFloor) noiseFloor = level;
  }
}

// Rengi (r,g,b) yuzde parlaklikla yazar. / Writes color (r,g,b) at a percent brightness.
void setLed(int index, int r, int g, int b, int percent) {
  int scale = percent * kMaxBrightness / 100;
  iotbot.moduleSmartLEDWrite(index, r * scale / 255, g * scale / 255, b * scale / 255);
}

// Renk carki: 0-255 arasi bir sayiyi kirmizi -> yesil -> mavi -> kirmizi renge cevirir.
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
  // Her LED'in kendi bolgesi var: LED1 %0-33, LED2 %33-66, LED3 %66-100.
  // Each LED has its own zone: LED1 0-33%, LED2 33-66%, LED3 66-100%.
  setLed(0, 0, 255, 0, constrain(pct * 3, 0, 100));          // Yesil / green
  setLed(1, 255, 160, 0, constrain((pct - 33) * 3, 0, 100)); // Sari / yellow
  setLed(2, 255, 0, 0, constrain((pct - 66) * 3, 0, 100));   // Kirmizi / red
}

void showParty(int pct) {
  partyHue++; // Renkler yavasca doner... / colors slowly rotate...
  for (int i = 0; i < 3; i++) {
    int r, g, b;
    // ...ve yuksek ses renk carkinda buyuk bir sicrama yapar. / ...and loud sound jumps far on the wheel.
    wheel((uint8_t)(partyHue + pct * 2 + i * 85), r, g, b);
    setLed(i, r, g, b, 15 + pct * 85 / 100);
  }
}

void showModeScreen() {
  iotbot.lcdWriteMid(turkish ? "SESE DUYARLI LED" : "SOUND REACTIVE LED", "", "",
                      mode == VU_METER ? (turkish ? "Mod: VU METRE  (B3)" : "Mode: VU METER (B3)")
                                       : (turkish ? "Mod: PARTI     (B3)" : "Mode: PARTY    (B3)"));
}

void drawScreen(int pct) {
  // Satir 1: 20 kutuluk ses cubugu + tepe isareti. / Row 1: 20-cell level bar + peak mark.
  int cells = pct * 20 / 100;
  int peakCell = constrain(peakPct * 20 / 100, 0, 19);
  char line[21];
  for (int i = 0; i < 20; i++) {
    line[i] = (i < cells) ? '\xFF' : ((i == peakCell && peakPct > 0) ? '|' : ' '); // 0xFF = dolu kutu / full block
  }
  line[20] = '\0';
  iotbot.lcdWriteFixed(1, line);
  char text[21];
  snprintf(text, sizeof(text), turkish ? "Ses:%3d%%  Tepe:%3d%%" : "Level:%3d%% Peak:%3d%%", pct, peakPct);
  snprintf(line, sizeof(line), "%-20s", text);
  iotbot.lcdWriteFixed(2, line);
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.moduleSmartLEDPrepare(LED_PIN);
  delay(500); // Acilis sesi bitsin / let the startup beep fade
  calibrate();
  showModeScreen();
  iotbot.serialWrite(turkish ? "Sese duyarli LED hazir. B3 = mod degistir." : "Sound reactive LED ready. B3 = change mode.");
}

void loop() {
  uint32_t now = millis();

  // B3: basildigi an bir kez mod degistir (200 ms sicrama korumasi).
  // B3: change mode once per press (200 ms debounce).
  bool b3Down = iotbot.button3Read();
  if (b3Down && !b3WasDown && now - lastB3Ms > 200) {
    lastB3Ms = now;
    mode = (mode == VU_METER) ? PARTY : VU_METER;
    iotbot.moduleSmartLEDClear();
    showModeScreen();
    iotbot.serialWrite(mode == VU_METER ? "VU" : (turkish ? "PARTI" : "PARTY"));
  }
  b3WasDown = b3Down;

  // Ses seviyesini gurultunun ustunde yuzdeye cevir. / Turn loudness above the noise into a percent.
  int level = readLevel();
  int pct = constrain(map(level - noiseFloor, 0, kLoudLevel, 0, 100), 0, 100);

  // Hizli yukselis / yavas sonus: yukselirken hedefe cabuk yaklas, duserken yavasca azal.
  // Fast attack / slow decay: rise quickly towards louder values, fall slowly.
  if (pct > smoothPct) {
    smoothPct += (pct - smoothPct) * kAttack;
  } else {
    smoothPct *= kDecay;
  }
  int shown = (int)smoothPct;

  // Tepe tutma: en yuksek deger 1 sn bekler, sonra yavasca iner.
  // Peak hold: the highest value waits 1 s, then slowly falls.
  if (shown >= peakPct) {
    peakPct = shown;
    peakMs = now;
  } else if (now - peakMs > kPeakHoldMs && peakPct > 0) {
    peakPct--;
  }

  if (mode == VU_METER) {
    showVuMeter(shown);
  } else {
    showParty(shown);
  }

  if (now - lastScreenMs >= kScreenMs) {
    lastScreenMs = now;
    drawScreen(shown);
  }
}
