/*
 * TR: GERÇEK PROJE - Metronom. Müzisyenlerin ritim tutmak için kullandığı
 * aletin aynısı! Potansiyometreyi çevirerek tempoyu dakikada 40 ile 208 vuruş
 * (BPM) arasında ayarlayın. Buzzer her vuruşta "tık" yapar; ölçünün İLK vuruşu
 * (vurgu) daha ince bir sesle çalar, böylece "1-2-3-4" sayabilirsiniz. B3
 * butonu ölçüyü değiştirir: 2/4 -> 3/4 -> 4/4. LCD'de tempo, tempo adı
 * (Adagio, Allegro...), ölçü ve hangi vuruşta olduğunuz "[X] [ ] [ ] [ ]"
 * şeklinde görünür. İsteğe bağlı akıllı LED vuruşla birlikte yanıp söner.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim    / help       -> komut listesi
 *      bpm 120   / tempo 120  -> tempoyu ayarla (40-208; potu çevirince pot geçerli olur)
 *      olcu 3    / time 3     -> ölçü: 2, 3 ya da 4 (2/4, 3/4, 4/4)
 *      dur       / stop       -> metronomu durdur
 *      basla     / start      -> metronomu yeniden başlat
 *      dil       / lang       -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Metronome. The same tool musicians use to keep the
 * beat! Turn the potentiometer to set the tempo between 40 and 208 beats per
 * minute (BPM). The buzzer ticks on every beat; the FIRST beat of the bar
 * (the accent) has a higher pitch so you can count "1-2-3-4". B3 changes the
 * time signature: 2/4 -> 3/4 -> 4/4. The LCD shows the tempo, the tempo name
 * (Adagio, Allegro...), the signature and which beat you are on like
 * "[X] [ ] [ ] [ ]". An optional smart LED flashes with the beat.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help      / yardim     -> command list
 *      tempo 120 / bpm 120    -> set the tempo (40-208; turning the pot takes over again)
 *      time 3    / olcu 3     -> signature: 2, 3 or 4 (2/4, 3/4, 4/4)
 *      stop      / dur        -> stop the metronome
 *      start     / basla      -> start the metronome again
 *      lang      / dil        -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Potansiyometre, buzzer ve B3 kart üzerindedir.
 * İSTEĞE BAĞLI: Akıllı LED modülünü P1 soketine (IO25) takın - beyaz yanar,
 * vurgu vuruşunda kırmızı yanar. LED takılı değilse örnek yine çalışır.
 * / The potentiometer, buzzer and B3 are on the board. OPTIONAL: plug the
 * smart LED module into socket P1 (IO25) - it flashes white, red on the
 * accent beat. The example still works without the LED.
 */

#define USE_NEOPIXEL
#include <IOTBOT.h>

IOTBOT iotbot;

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

#define LED_PIN IO25 // İsteğe bağlı akıllı LED: P1 / optional smart LED: P1

namespace {
  constexpr int kMinBpm = 40;               // En yavaş tempo / slowest tempo
  constexpr int kMaxBpm = 208;              // En hızlı tempo / fastest tempo
  constexpr int kAccentHz = 2000;           // Vurgu (1. vuruş) sesi / accent (beat 1) pitch
  constexpr int kBeatHz = 1000;             // Normal vuruş sesi / normal beat pitch
  constexpr int kClickMs = 30;              // Tık uzunluğu / click length
  constexpr uint32_t kLedOnMs = 80;         // LED'in yanık kalma süresi / LED on time
  constexpr uint32_t kUiIntervalMs = 200;   // LCD yenileme aralığı / LCD refresh interval
  constexpr int kPotHysteresis = 12;        // Pot titremesini yok say / ignore pot jitter
  constexpr int kPotTakeOverRaw = 150;      // Seri komuttan sonra potun geri alma eşiği / pot take-over threshold after a serial command
  const int kBeatsPerBar[] = {2, 3, 4};     // 2/4, 3/4, 4/4

  int sigIndex = 2;          // Başlangıç 4/4 / start with 4/4
  int bpm = 120;
  int beat = 0;              // Sıradaki vuruş (0 = vurgu) / next beat (0 = accent)
  float potSmooth = 0;       // Yumuşatılmış pot değeri / smoothed pot value
  int potAccepted = -1000;   // BPM'e çevrilen son pot değeri / last pot value turned into BPM
  bool potLocked = false;    // true = tempo seri porttan verildi, pot çevrilene kadar bekle / tempo came from serial
  int potLockRaw = 0;
  bool running = true;       // Metronom çalışıyor mu? / is the metronome running?
  uint32_t nextBeatMs = 0;
  uint32_t ledOnSinceMs = 0;
  bool ledOn = false;
  bool uiDirty = true;       // Üst satırlar yenilensin mi? / redraw the top rows?
  uint32_t lastUiMs = 0;
  bool lastB3 = false;
  uint32_t lastB3Ms = 0;    // Buton parazitini önlemek için / for button debounce
}

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "ÖLÇÜ" -> "olcu"
// Lower-cases and simplifies Turkish letters: "ÖLÇÜ" -> "olcu"
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
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
// lcdWriteFixedTxt Türkçe harfleri (ç, ğ, ı, ö, ş, ü) LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- METRONOM - Komutlar ----", "---- METRONOME - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  bpm 40-208    : tempo", "  tempo 40-208  : tempo"));
  iotbot.serialWrite(L("  olcu 2/3/4    : ölçü (2/4, 3/4, 4/4)", "  time 2/3/4    : signature (2/4, 3/4, 4/4)"));
  iotbot.serialWrite(L("  dur / basla   : durdur / başlat", "  stop / start  : stop / start"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  Potansiyometre: tempo,  B3: ölçü", "  Potentiometer : tempo,  B3: signature"));
}

const char *tempoName(int b) {
  if (b < 60) return "Largo";
  if (b < 76) return "Adagio";
  if (b < 108) return "Andante";
  if (b < 120) return "Moderato";
  if (b < 168) return "Allegro";
  return "Presto";
}

// "[X] [ ] [ ] [ ]" gibi vuruş göstergesini 4. satıra yazar.
// Writes a beat indicator like "[X] [ ] [ ] [ ]" to row 4.
void drawBeatRow(int current) {
  if (!running) {
    lcdRow(3, L("DURDU  (basla yazın)", "STOPPED (type start)"));
    return;
  }
  char line[21];
  memset(line, ' ', 20);
  line[20] = '\0';
  int beats = kBeatsPerBar[sigIndex];
  for (int i = 0; i < beats; i++) {
    line[i * 4] = '[';
    line[i * 4 + 1] = (i == current) ? 'X' : ' ';
    line[i * 4 + 2] = ']';
  }
  lcdRow(3, line);
}

void drawInfoRows() {
  char line[41];
  snprintf(line, sizeof(line), "BPM: %3d  %s", bpm, tempoName(bpm));
  lcdRow(1, line);
  snprintf(line, sizeof(line), L("Ölçü: %d/4  (B3)", "Time: %d/4  (B3)"), kBeatsPerBar[sigIndex]);
  lcdRow(2, line);
}

void drawAll() {
  lcdRow(0, L("      METRONOM", "      METRONOME"));
  drawInfoRows();
  drawBeatRow(-1);
}

void playBeat() {
  bool accent = (beat == 0);
  // Görsel önce: LED ve ses aynı anda başlasın / visual first: LED and sound start together
  if (accent) iotbot.moduleSmartLEDFill(200, 0, 0);   // Vurgu: kırmızı / accent: red
  else iotbot.moduleSmartLEDFill(80, 80, 80);         // Normal: beyaz / normal: white
  ledOn = true;
  ledOnSinceMs = millis();
  iotbot.buzzerPlayTone(accent ? kAccentHz : kBeatHz, kClickMs);  // 30 ms: kısa, sorun değil / short, fine
  drawBeatRow(beat);
  beat = (beat + 1) % kBeatsPerBar[sigIndex];
}

void setSignature(int index) {
  sigIndex = index;
  beat = 0;
  nextBeatMs = millis() + 200;
  uiDirty = true;
  drawBeatRow(-1);
  iotbot.serialWrite(String(L("Ölçü: ", "Signature: ")) + kBeatsPerBar[sigIndex] + "/4");
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if ((word == "bpm" || word == "tempo") && hasValue) {
    bpm = constrain(value, kMinBpm, kMaxBpm);
    // Pot, ancak gerçekten çevrilince tempoyu yeniden alır. / The pot takes the tempo back only when really turned.
    potLocked = true;
    potLockRaw = (int)potSmooth;
    uiDirty = true;
    iotbot.serialWrite(String(L("Tempo: ", "Tempo: ")) + bpm + " BPM (" + tempoName(bpm) + ")");
  } else if ((word == "olcu" || word == "time") && hasValue) {
    if (value >= 2 && value <= 4) setSignature(value - 2);
    else iotbot.serialWrite(L("Ölçü 2, 3 ya da 4 olmalı.", "The signature must be 2, 3 or 4."));
  } else if (word == "dur" || word == "stop") {
    running = false;
    iotbot.moduleSmartLEDClear();
    ledOn = false;
    drawBeatRow(-1);
    iotbot.serialWrite(L("Metronom durdu.", "Metronome stopped."));
  } else if (word == "basla" || word == "start") {
    running = true;
    beat = 0;
    nextBeatMs = millis() + 200;
    drawBeatRow(-1);
    iotbot.serialWrite(L("Metronom başladı.", "Metronome started."));
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    drawAll();
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
  iotbot.moduleSmartLEDClear();
  potSmooth = iotbot.potentiometerRead();
  iotbot.lcdClear();
  drawAll();
  iotbot.serialWrite(L("Metronom hazır.", "Metronome ready."));
  printHelp();
  nextBeatMs = millis() + 300;
}

void loop() {
  uint32_t now = millis();

  // 1) Potansiyometre -> BPM. Yumuşatma + küçük eşik: sayı ekranda titremesin.
  // 1) Potentiometer -> BPM. Smoothing + a small threshold so the number does not flicker.
  potSmooth = potSmooth * 0.9f + iotbot.potentiometerRead() * 0.1f;
  if (potLocked && abs((int)potSmooth - potLockRaw) > kPotTakeOverRaw) {
    potLocked = false; // Pot çevrildi: tempo yeniden potta / pot turned: the pot owns the tempo again
  }
  if (!potLocked && abs((int)potSmooth - potAccepted) > kPotHysteresis) {
    potAccepted = (int)potSmooth;
    int newBpm = constrain(map(potAccepted, 0, 4050, kMinBpm, kMaxBpm), kMinBpm, kMaxBpm);
    if (newBpm != bpm) { bpm = newBpm; uiDirty = true; }
  }

  // 2) B3 -> ölçüyü değiştir ve ölçüyü baştan (vurguyla) başlat.
  // 2) B3 -> change the signature and restart the bar (with the accent).
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3 && now - lastB3Ms > 200) {
    lastB3Ms = now;
    setSignature((sigIndex + 1) % 3);
  }
  lastB3 = b3;

  // 3) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 4) Zamanlama: bir sonraki vuruşun zamanı "planlanır" (+= aralık). Böylece
  // küçük gecikmeler birikip tempoyu kaydırmaz.
  // 4) Timing: the next beat time is "scheduled" (+= interval), so small
  // delays do not add up and drift the tempo.
  uint32_t interval = 60000UL / bpm;
  if (running && (int32_t)(now - nextBeatMs) >= 0) {
    playBeat();
    nextBeatMs += interval;
    if ((int32_t)(millis() - nextBeatMs) >= 0) nextBeatMs = millis() + interval;  // Çok geride kaldıysa yeniden hizala / resync if far behind
  }

  // 5) LED'i kısa bir süre sonra söndür / turn the LED off after a short time
  if (ledOn && now - ledOnSinceMs >= kLedOnMs) {
    iotbot.moduleSmartLEDClear();
    ledOn = false;
  }

  // 6) Üst satırları sadece değişince ve en fazla 200 ms'de bir yaz.
  // 6) Redraw the top rows only when changed, at most every 200 ms.
  if (uiDirty && now - lastUiMs >= kUiIntervalMs) {
    lastUiMs = now;
    uiDirty = false;
    drawInfoRows();
  }
  delay(2);
}
