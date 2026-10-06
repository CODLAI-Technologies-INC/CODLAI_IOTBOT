// TR: GERCEK PROJE - Alkisla Yanan Lamba. Iki kez pes pese alkis calinca
// karttaki role (ve ona baglayacaginiz lamba) acilir, tekrar iki alkista
// kapanir. Acilista 1 saniye boyunca odanin "sessizlik seviyesini" olcer ve
// alkis esigini buna gore kendisi ayarlar; boylece sessiz bir odada da
// gurultulu bir sinifta da calisir.
// NEDEN CIFT ALKIS? Tek bir yuksek ses (kapi carpmasi, dusen kalem, bir
// oksuruk) cok sik olur ve lambayi rastgele acip kapatirdi. Ama iki KISA
// sesin 0.15 ile 0.7 saniye arayla gelmesi tesadufen neredeyse hic olmaz.
// Ayrica uzun suren sesler (konusma, muzik) "kisa" olmadigi icin alkis
// sayilmaz. Bu iki kural yanlis tetiklenmeyi cok azaltir.
// EN: A REAL PROJECT - Clap Switch. Clap twice in a row and the board's
// relay (and the lamp you connect to it) turns on; two more claps turn it
// off. At startup it measures the room's "quiet level" for 1 second and
// sets the clap threshold from it, so it works in a quiet room and in a
// noisy classroom.
// WHY A DOUBLE CLAP? A single loud sound (a door slam, a dropped pen, a
// cough) happens all the time and would toggle the lamp randomly. But two
// SHORT sounds arriving 0.15 to 0.7 seconds apart almost never happens by
// accident. Long sounds (talking, music) are also not "short", so they are
// not counted as claps. These two rules cut false triggers a lot.
//
// Baglanti / Wiring: Mikrofon (ses sensoru) modulunu P4 soketine (IO32)
// takin. Role kart uzerindedir. / Plug the microphone (sound sensor)
// module into socket P4 (IO32). The relay is on the board.

#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define MIC_PIN IO32 // P4 soketi (analog icin ADC1 pini) / socket P4 (ADC1 pin for analog)

namespace {
  constexpr uint32_t kCalibrationMs = 1000; // Sessizlik olcum suresi / quiet calibration time
  constexpr int kMargin = 250;              // Sessizligin ne kadar ustu alkis / how far above quiet = clap
  constexpr uint32_t kSampleWindowMs = 10;  // Her seviye olcumu 10 ms / each level reading is 10 ms
  constexpr uint32_t kMaxClapMs = 120;      // Alkis KISA bir patlamadir / a clap is a SHORT burst
  constexpr uint32_t kMinGapMs = 150;       // Iki alkis arasi en az / min gap between the claps
  constexpr uint32_t kMaxGapMs = 700;       // Iki alkis arasi en fazla / max gap between the claps
  constexpr uint32_t kCooldownMs = 1000;    // Role "tik" sesi alkis sanilmasin / relay click is not a clap
  constexpr uint32_t kScreenMs = 200;       // LCD yenileme araligi / LCD refresh interval

  int baseline = 2048;       // Sessizken ortalama ham deger / mean raw value in silence
  int threshold = 400;       // Alkis esigi (seviye) / clap threshold (level)
  bool lampOn = false;
  bool inPeak = false;       // Su an yuksek ses suruyor mu? / is a loud sound going on now?
  uint32_t peakStartMs = 0;
  uint32_t firstClapMs = 0;  // 0 = bekleyen ilk alkis yok / 0 = no first clap waiting
  uint32_t lastToggleMs = 0;
  uint32_t lastScreenMs = 0;
  int shownPeak = 0;         // Iki ekran yenilemesi arasindaki en yuksek seviye / max level between refreshes
}

// Mikrofon sesi, baseline etrafinda salinan bir dalga olarak verir. 10 ms boyunca dalganin
// baseline'dan en cok ne kadar uzaklastigina bakariz: bu, o anki "ses seviyesi"dir.
// The mic gives sound as a wave swinging around the baseline. For 10 ms we look at how far
// the wave gets from the baseline: that is the current "sound level".
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
  iotbot.lcdWriteMid(turkish ? "ALKIS ANAHTARI" : "CLAP SWITCH", "", turkish ? "Sessiz olun..." : "Please be quiet...",
                      turkish ? "Olculuyor" : "Measuring");
  // 1) Yarim saniye: sessizken ortalama ham deger (dalganin orta cizgisi).
  // 1) Half a second: the mean raw value in silence (the middle line of the wave).
  long sum = 0;
  long count = 0;
  uint32_t start = millis();
  while (millis() - start < kCalibrationMs / 2) {
    sum += iotbot.moduleMicRead(MIC_PIN);
    count++;
    delay(1);
  }
  baseline = sum / count;
  // 2) Yarim saniye: sessiz odadaki en yuksek gurultu seviyesi. Esik = gurultu + pay.
  // 2) Half a second: the loudest noise level of the quiet room. Threshold = noise + margin.
  int quietLevel = 0;
  start = millis();
  while (millis() - start < kCalibrationMs / 2) {
    int level = readLevel();
    if (level > quietLevel) quietLevel = level;
  }
  threshold = quietLevel + kMargin;

  char msg[64];
  snprintf(msg, sizeof(msg), turkish ? "Orta deger: %d  Sessizlik: %d  Esik: %d" : "Baseline: %d  Quiet: %d  Threshold: %d",
           baseline, quietLevel, threshold);
  iotbot.serialWrite(msg);
}

void setLamp(bool on) {
  lampOn = on;
  iotbot.relayWrite(lampOn);
  iotbot.buzzerPlayTone(lampOn ? 1600 : 900, 60);
  iotbot.serialWrite(lampOn ? (turkish ? "Cift alkis: lamba ACIK" : "Double clap: lamp ON")
                            : (turkish ? "Cift alkis: lamba KAPALI" : "Double clap: lamp OFF"));
}

// Kisa bir ses patlamasi bitince cagrilir. clapMs = alkisin basladigi an.
// Called when a short burst of sound ends. clapMs = when the clap started.
void onClap(uint32_t clapMs) {
  if (firstClapMs == 0) {
    firstClapMs = clapMs; // Ilk alkis: ikincisini bekle / first clap: wait for the second one
    return;
  }
  uint32_t gap = clapMs - firstClapMs;
  if (gap < kMinGapMs) {
    return; // Ayni alkisin yankisi / an echo of the same clap
  }
  if (gap <= kMaxGapMs) {
    setLamp(!lampOn); // Cift alkis! / double clap!
    firstClapMs = 0;
    lastToggleMs = millis();
  } else {
    firstClapMs = clapMs; // Cok gec geldi: bunu yeni "ilk alkis" say / too late: new first clap
  }
}

void drawScreen() {
  // Satir 1: seviye cubugu. Esik tam ortada (8. kutu) '|' ile gosterilir.
  // Row 1: level bar. The threshold is marked with '|' in the middle (cell 8).
  int cells = constrain(shownPeak * 16 / (threshold * 2), 0, 16);
  char line[21];
  memcpy(line, turkish ? "Ses " : "Mic ", 4);
  for (int i = 0; i < 16; i++) {
    line[4 + i] = (i < cells) ? '\xFF' : (i == 8 ? '|' : ' '); // 0xFF = LCD'de dolu kutu / full block
  }
  line[20] = '\0';
  iotbot.lcdWriteFixed(1, line);

  snprintf(line, sizeof(line), "%-20s", lampOn ? (turkish ? "   Lamba: ACIK" : "   Lamp: ON")
                                               : (turkish ? "   Lamba: KAPALI" : "   Lamp: OFF"));
  iotbot.lcdWriteFixed(2, line);
  snprintf(line, sizeof(line), "%-20s", firstClapMs != 0 ? (turkish ? "  Bir daha alkisla!" : "   Clap once more!")
                                                         : (turkish ? "  2 kez alkislayin" : "    Clap twice"));
  iotbot.lcdWriteFixed(3, line);
  shownPeak = 0;
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.relayWrite(false);
  delay(500); // Acilis sesi bitsin, mikrofon onu duymasin / let the startup beep fade first
  calibrate();
  iotbot.lcdWriteMid(turkish ? "ALKIS ANAHTARI" : "CLAP SWITCH", "", "", "");
  iotbot.serialWrite(turkish ? "Alkis anahtari hazir." : "Clap switch ready.");
}

void loop() {
  int level = readLevel();
  uint32_t now = millis();
  if (level > shownPeak) shownPeak = level;

  // Ikinci alkis zamaninda gelmediyse ilk alkisi unut. / Forget the first clap if no second one came.
  if (firstClapMs != 0 && now - firstClapMs > kMaxGapMs) {
    firstClapMs = 0;
  }

  bool loud = level > threshold;
  if (loud && !inPeak) {
    inPeak = true; // Yuksek ses basladi / a loud sound started
    peakStartMs = now;
  } else if (!loud && inPeak) {
    inPeak = false; // Yuksek ses bitti: ne kadar surdu? / the loud sound ended: how long was it?
    bool shortBurst = (now - peakStartMs) <= kMaxClapMs;
    if (!shortBurst) {
      firstClapMs = 0; // Konusma/muzik gibi uzun ses: alkis degil / long sound (talk/music): not a clap
    } else if (now - lastToggleMs >= kCooldownMs) {
      onClap(peakStartMs);
    }
  }

  if (now - lastScreenMs >= kScreenMs) {
    lastScreenMs = now;
    drawScreen();
  }
}
