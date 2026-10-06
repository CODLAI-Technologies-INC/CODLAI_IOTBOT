// TR: GERCEK PROJE - Metronom. Muzisyenlerin ritim tutmak icin kullandigi
// aletin aynisi! Potansiyometreyi cevirerek tempoyu dakikada 40 ile 208
// vurus (BPM) arasinda ayarlayin. Buzzer her vurusta "tik" yapar; olcunun
// ILK vurusu (vurgu) daha ince bir sesle calar, boylece "1-2-3-4" sayabilirsiniz.
// B3 butonu olcuyu degistirir: 2/4 -> 3/4 -> 4/4. LCD'de tempo, tempo adi
// (Adagio, Allegro...), olcu ve hangi vurusta oldugunuz "[X] [ ] [ ] [ ]"
// seklinde gorunur. Istege bagli akilli LED vurusla birlikte yanip soner.
// EN: A REAL PROJECT - Metronome. The same tool musicians use to keep the
// beat! Turn the potentiometer to set the tempo between 40 and 208 beats
// per minute (BPM). The buzzer ticks on every beat; the FIRST beat of the
// bar (the accent) has a higher pitch so you can count "1-2-3-4". B3
// changes the time signature: 2/4 -> 3/4 -> 4/4. The LCD shows the tempo,
// the tempo name (Adagio, Allegro...), the signature and which beat you are
// on like "[X] [ ] [ ] [ ]". An optional smart LED flashes with the beat.
//
// Baglanti / Wiring: Potansiyometre, buzzer ve B3 kart uzerindedir.
// ISTEGE BAGLI: Akilli LED modulunu P1 soketine (IO25) takin - beyaz yanar,
// vurgu vurusunda kirmizi yanar. LED takili degilse ornek yine calisir.
// / The potentiometer, buzzer and B3 are on the board. OPTIONAL: plug the
// smart LED module into socket P1 (IO25) - it flashes white, red on the
// accent beat. The example still works without the LED.

#define USE_NEOPIXEL
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define LED_PIN IO25 // Istege bagli akilli LED: P1 / optional smart LED: P1

namespace {
  constexpr int kMinBpm = 40;               // En yavas tempo / slowest tempo
  constexpr int kMaxBpm = 208;              // En hizli tempo / fastest tempo
  constexpr int kAccentHz = 2000;           // Vurgu (1. vurus) sesi / accent (beat 1) pitch
  constexpr int kBeatHz = 1000;             // Normal vurus sesi / normal beat pitch
  constexpr int kClickMs = 30;              // Tik uzunlugu / click length
  constexpr uint32_t kLedOnMs = 80;         // LED'in yanik kalma suresi / LED on time
  constexpr uint32_t kUiIntervalMs = 200;   // LCD yenileme araligi / LCD refresh interval
  constexpr int kPotHysteresis = 12;        // Pot titremesini yok say / ignore pot jitter
  const int kBeatsPerBar[] = {2, 3, 4};     // 2/4, 3/4, 4/4

  int sigIndex = 2;          // Baslangic 4/4 / start with 4/4
  int bpm = 120;
  int beat = 0;              // Siradaki vurus (0 = vurgu) / next beat (0 = accent)
  float potSmooth = 0;       // Yumusatilmis pot degeri / smoothed pot value
  int potAccepted = -1000;   // BPM'e cevrilen son pot degeri / last pot value turned into BPM
  uint32_t nextBeatMs = 0;
  uint32_t ledOnSinceMs = 0;
  bool ledOn = false;
  bool uiDirty = true;       // Ust satirlar yenilensin mi? / redraw the top rows?
  uint32_t lastUiMs = 0;
  bool lastB3 = false;
  uint32_t lastB3Ms = 0;    // Buton parazitini onlemek icin / for button debounce
}

const char *tempoName(int b) {
  if (b < 60) return "Largo";
  if (b < 76) return "Adagio";
  if (b < 108) return "Andante";
  if (b < 120) return "Moderato";
  if (b < 168) return "Allegro";
  return "Presto";
}

// "[X] [ ] [ ] [ ]" gibi vurus gostergesini 4. satira yazar.
// Writes a beat indicator like "[X] [ ] [ ] [ ]" to row 4.
void drawBeatRow(int current) {
  char line[21];
  memset(line, ' ', 20);
  line[20] = '\0';
  int beats = kBeatsPerBar[sigIndex];
  for (int i = 0; i < beats; i++) {
    line[i * 4] = '[';
    line[i * 4 + 1] = (i == current) ? 'X' : ' ';
    line[i * 4 + 2] = ']';
  }
  iotbot.lcdWriteFixed(3, line);
}

void drawInfoRows() {
  char line[21];
  snprintf(line, sizeof(line), "BPM: %3d  %s", bpm, tempoName(bpm));
  iotbot.lcdWriteFixed(1, line);
  snprintf(line, sizeof(line), turkish ? "Olcu: %d/4  (B3)" : "Time: %d/4  (B3)", kBeatsPerBar[sigIndex]);
  iotbot.lcdWriteFixed(2, line);
}

void playBeat() {
  bool accent = (beat == 0);
  // Gorsel once: LED ve ses ayni anda baslasin / visual first: LED and sound start together
  if (accent) iotbot.moduleSmartLEDFill(200, 0, 0);   // Vurgu: kirmizi / accent: red
  else iotbot.moduleSmartLEDFill(80, 80, 80);         // Normal: beyaz / normal: white
  ledOn = true;
  ledOnSinceMs = millis();
  iotbot.buzzerPlayTone(accent ? kAccentHz : kBeatHz, kClickMs);  // 30 ms: kisa, sorun degil / short, fine
  drawBeatRow(beat);
  beat = (beat + 1) % kBeatsPerBar[sigIndex];
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.moduleSmartLEDPrepare(LED_PIN);
  iotbot.moduleSmartLEDClear();
  potSmooth = iotbot.potentiometerRead();
  iotbot.lcdClear();
  iotbot.lcdWriteFixed(0, turkish ? "      METRONOM" : "      METRONOME");
  iotbot.serialWrite(turkish ? "Metronom hazir." : "Metronome ready.");
  nextBeatMs = millis() + 300;
}

void loop() {
  uint32_t now = millis();

  // 1) Potansiyometre -> BPM. Yumusatma + kucuk esik: sayi ekranda titremesin.
  // 1) Potentiometer -> BPM. Smoothing + a small threshold so the number does not flicker.
  potSmooth = potSmooth * 0.9f + iotbot.potentiometerRead() * 0.1f;
  if (abs((int)potSmooth - potAccepted) > kPotHysteresis) {
    potAccepted = (int)potSmooth;
    int newBpm = constrain(map(potAccepted, 0, 4050, kMinBpm, kMaxBpm), kMinBpm, kMaxBpm);
    if (newBpm != bpm) { bpm = newBpm; uiDirty = true; }
  }

  // 2) B3 -> olcuyu degistir ve olcuyu bastan (vurguyla) baslat.
  // 2) B3 -> change the signature and restart the bar (with the accent).
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3 && now - lastB3Ms > 200) {
    lastB3Ms = now;
    sigIndex = (sigIndex + 1) % 3;
    beat = 0;
    nextBeatMs = now + 200;
    uiDirty = true;
    drawBeatRow(-1);
  }
  lastB3 = b3;

  // 3) Zamanlama: bir sonraki vurusun zamani "planlanir" (+= aralik). Boylece
  // kucuk gecikmeler birikip tempoyu kaydirmaz.
  // 3) Timing: the next beat time is "scheduled" (+= interval), so small
  // delays do not add up and drift the tempo.
  uint32_t interval = 60000UL / bpm;
  if ((int32_t)(now - nextBeatMs) >= 0) {
    playBeat();
    nextBeatMs += interval;
    if ((int32_t)(millis() - nextBeatMs) >= 0) nextBeatMs = millis() + interval;  // Cok geride kaldiysa yeniden hizala / resync if far behind
  }

  // 4) LED'i kisa bir sure sonra sondur / turn the LED off after a short time
  if (ledOn && now - ledOnSinceMs >= kLedOnMs) {
    iotbot.moduleSmartLEDClear();
    ledOn = false;
  }

  // 5) Ust satirlari sadece degisince ve en fazla 200 ms'de bir yaz.
  // 5) Redraw the top rows only when changed, at most every 200 ms.
  if (uiDirty && now - lastUiMs >= kUiIntervalMs) {
    lastUiMs = now;
    uiDirty = false;
    drawInfoRows();
  }
  delay(2);
}
