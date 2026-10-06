// TR: GERCEK PROJE - Akilli Sera Catisi. Kart uzerindeki isik sensoru
// (LDR) gunesi olcer; hava aydinlaninca step motor seranin cati penceresini
// ACAR, kararinca KAPATIR. Motor hareket ederken LCD'de ilerleme cubugu
// gorunur. B3 butonu ile pencereyi elle acip kapatabilirsiniz (manuel mod
// 60 saniye surer, sonra otomatige doner).
// NEDEN IKI FARKLI ESIK (HISTEREZIS)? Tek bir esik olsaydi (ornegin %50),
// isik tam %49-%51 arasinda gidip gelirken cati durmadan acilip kapanirdi.
// Bu yuzden %60'in USTUNDE acar, %40'in ALTINDA kapatiriz; aradaki bolgede
// hicbir sey yapmayiz. Ayrica isik 2 saniye boyunca esigin otesinde kalmali;
// boylece sensorun uzerinden gecen bir el golgesi catiyi kapatmaz.
// EN: A REAL PROJECT - Smart Greenhouse Roof. The onboard light sensor
// (LDR) measures the sun; when it gets bright the step motor OPENS the
// greenhouse's roof window, when it gets dark it CLOSES it. A progress bar
// is shown on the LCD while the motor moves. Button B3 opens/closes the
// window by hand (manual mode lasts 60 seconds, then back to automatic).
// WHY TWO DIFFERENT LEVELS (HYSTERESIS)? With a single level (say 50%), the
// roof would open and close nonstop while the light wobbles between 49% and
// 51%. So we open ABOVE 60% and close BELOW 40%, and do nothing in between.
// The light must also stay past the level for 2 seconds, so a hand's shadow
// passing over the sensor does not close the roof.
//
// Baglanti / Wiring: Step motoru P6 motor soketine takin (IO26, IO33, IO32,
// IO27 pinlerini kullanir - bu yuzden P2-P5'e baska modul TAKMAYIN). LDR kart
// uzerindedir. Acmadan once pencereyi elle KAPALI konuma getirin: program
// catinin kapali basladigini varsayar. / Plug the step motor into the P6
// motor socket (it uses IO26, IO33, IO32, IO27 - so do NOT plug other modules
// into P2-P5). The LDR is on the board. Close the window by hand before
// powering on: the program assumes the roof starts closed.

#define USE_STEP_MOTOR
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

namespace {
  constexpr int kOpenLightPct = 60;          // Bunun ustunde ac / open above this
  constexpr int kCloseLightPct = 40;         // Bunun altinda kapat / close below this
  constexpr uint32_t kConfirmMs = 2000;      // Isik bu kadar sure esigi gecmeli / light must stay past the level
  constexpr uint32_t kManualHoldMs = 60000;  // Manuel mod suresi / manual mode duration
  constexpr int kStepsPerRev = 50;           // Kutuphane ornekleriyle ayni / same as the library examples
  constexpr int kRoofTotalSteps = 200;       // Tam acilma adimi (mekanizmaniza gore) / full-open steps (fit your build)
  constexpr int kChunkSteps = 8;             // Parca basina adim, 4'un kati olmali / steps per chunk, keep a multiple of 4
  constexpr int kMotorRpm = 90;              // Motor hizi / motor speed
  constexpr bool kOpenDirection = true;      // Ters yone aciliyorsa false yapin / set false if it opens the wrong way
  constexpr uint32_t kScreenMs = 300;        // LCD yenileme araligi / LCD refresh interval

  bool roofOpen = false; // Program catinin KAPALI basladigini varsayar / assumes the roof starts CLOSED
  bool manualMode = false;
  uint32_t manualStartMs = 0;
  uint32_t brightSinceMs = 0; // 0 = su an parlak degil / 0 = not bright right now
  uint32_t darkSinceMs = 0;   // 0 = su an karanlik degil / 0 = not dark right now
  bool b3WasDown = false;
  uint32_t lastScreenMs = 0;
}

// 8 okumanin ortalamasi -> 0-100 arasi yuzde. Bu kartta LDR degeri isik arttikca BUYUR.
// Average of 8 readings -> percent 0-100. On this board the LDR value GROWS with light.
int readLightPct() {
  long sum = 0;
  for (int i = 0; i < 8; i++) sum += iotbot.ldrRead();
  return constrain(map(sum / 8, 0, 4095, 0, 100), 0, 100);
}

void writeRow(int row, const char *text) {
  char line[21];
  snprintf(line, sizeof(line), "%-20s", text); // 20'ye bosluklarla tamamla / pad to 20 with spaces
  iotbot.lcdWriteFixed(row, line);
}

void showMainScreen() {
  iotbot.lcdWriteMid(turkish ? "AKILLI SERA CATISI" : "SMART GREENHOUSE", "", "", "");
  lastScreenMs = 0; // Degerleri hemen ciz / draw the values right away
}

// Hareketten sonra bobinlerdeki akimi keseriz: Stepper kutuphanesi son adimi enerjili birakir,
// motor ve surucu bosuna isinir. / After moving we cut the coil current: the Stepper library
// leaves the last step energized and the motor and driver heat up for nothing.
void releaseCoils() {
  const int coilPins[] = {IO26, IO33, IO32, IO27};
  for (int pin : coilPins) digitalWrite(pin, LOW);
}

void moveRoof(bool open) {
  if (open == roofOpen) return; // Zaten o durumda: asla iki kez acma! / already there: never open twice!
  iotbot.lcdWriteMid(turkish ? "AKILLI SERA CATISI" : "SMART GREENHOUSE", "",
                      open ? (turkish ? "Cati aciliyor..." : "Roof opening...") : (turkish ? "Cati kapaniyor..." : "Roof closing..."), "");
  iotbot.serialWrite(open ? (turkish ? "Cati aciliyor" : "Roof opening") : (turkish ? "Cati kapaniyor" : "Roof closing"));

  // moduleStepMotorMotion() hareket bitene kadar BEKLER (blocking). Tek seferde 200 adim yerine
  // kucuk parcalar halinde donup aralarda LCD'yi guncelliyoruz. / moduleStepMotorMotion() WAITS
  // until the move ends (blocking). Instead of 200 steps at once we turn in small chunks and
  // update the LCD in between.
  const int chunks = kRoofTotalSteps / kChunkSteps;
  for (int i = 1; i <= chunks; i++) {
    iotbot.moduleStepMotorMotion(kStepsPerRev, open ? kOpenDirection : !kOpenDirection, kChunkSteps, kMotorRpm);
    int percent = i * 100 / chunks;
    char bar[21];
    for (int c = 0; c < 14; c++) bar[c] = (c < percent * 14 / 100) ? '\xFF' : '.'; // 0xFF = dolu kutu / full block
    bar[14] = '\0';
    char line[21];
    snprintf(line, sizeof(line), "%s %3d%%", bar, percent);
    writeRow(3, line);
  }
  releaseCoils();
  roofOpen = open;
  iotbot.buzzerPlayTone(open ? 1500 : 900, 80);
  brightSinceMs = 0;
  darkSinceMs = 0;
  showMainScreen();
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  showMainScreen();
  iotbot.serialWrite(turkish ? "Akilli sera hazir. Cati kapali kabul edildi." : "Smart greenhouse ready. Roof assumed closed.");
}

void loop() {
  uint32_t now = millis();
  int lightPct = readLightPct();

  // B3 (true = basili): elle ac/kapat ve 60 sn manuel moda gec.
  // B3 (true = pressed): open/close by hand and switch to manual mode for 60 s.
  bool b3Down = iotbot.button3Read();
  if (b3Down && !b3WasDown) {
    manualMode = true;
    manualStartMs = now;
    moveRoof(!roofOpen);
    b3WasDown = true;
    return;
  }
  b3WasDown = b3Down;

  if (manualMode && now - manualStartMs >= kManualHoldMs) {
    manualMode = false;
    iotbot.serialWrite(turkish ? "Otomatik moda donuldu" : "Back to automatic mode");
  }

  if (!manualMode) {
    if (lightPct >= kOpenLightPct) {
      darkSinceMs = 0;
      if (brightSinceMs == 0) brightSinceMs = now; // Parlaklik yeni basladi / brightness just started
      if (!roofOpen && now - brightSinceMs >= kConfirmMs) moveRoof(true);
    } else if (lightPct <= kCloseLightPct) {
      brightSinceMs = 0;
      if (darkSinceMs == 0) darkSinceMs = now;
      if (roofOpen && now - darkSinceMs >= kConfirmMs) moveRoof(false);
    } else {
      brightSinceMs = 0; // Ara bolge: hicbir sey yapma / in-between zone: do nothing
      darkSinceMs = 0;
    }
  }

  if (now - lastScreenMs >= kScreenMs) {
    lastScreenMs = now;
    char line[21];
    snprintf(line, sizeof(line), turkish ? "  Isik: %3d%%" : "  Light: %3d%%", lightPct);
    writeRow(1, line);
    writeRow(2, roofOpen ? (turkish ? "  Cati: ACIK" : "  Roof: OPEN") : (turkish ? "  Cati: KAPALI" : "  Roof: CLOSED"));
    if (manualMode) {
      snprintf(line, sizeof(line), turkish ? "  Mod: MANUEL %lu sn" : "  Mode: MANUAL %lu s",
               (unsigned long)((kManualHoldMs - (now - manualStartMs) + 999) / 1000));
    } else {
      snprintf(line, sizeof(line), "%s", turkish ? "  Mod: OTOMATIK" : "  Mode: AUTO");
    }
    writeRow(3, line);
  }
  delay(20);
}
