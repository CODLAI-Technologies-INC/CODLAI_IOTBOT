// TR: GERCEK PROJE - Bomba Imha Oyunu. Kart, rastgele 3 haneli gizli bir
// kod secer (her hane 0-9). 60 saniyeniz var! ENCODER'i cevirerek bir rakam
// secin, encoder'in dugmesine basarak onaylayin. LCD size ipucu verir:
// "Dogru!" ya da "Daha BUYUK / Daha KUCUK". Dogru rakami bulana kadar ayni
// haneyi tekrar deneyebilirsiniz. Sure akarken buzzer her saniye "bip"
// eder, son 10 saniyede hizlanir; trafik lambasinin sarisi yanip soner.
// Uc haneyi de bulursaniz yesil yanar ve zafer melodisi calar; sure biterse
// kirmizi yanar ve "BOOM!" Yeni oyun icin B3'e basin.
// IPUCU: hep kalan araligin ortasini deneyin ("ikili arama"), en fazla 4 deneme!
// EN: A REAL PROJECT - Bomb Defuse Game. The board picks a random secret
// 3-digit code (each digit 0-9). You have 60 seconds! Turn the ENCODER to
// pick a digit and press the encoder's button to confirm. The LCD gives a
// hint: "Correct!" or "Go HIGHER / Go LOWER". You retry the same digit until
// you find it. While time runs the buzzer beeps every second, faster in the
// last 10 seconds, and the traffic light's yellow blinks. Find all three
// digits and green lights up with a victory melody; run out of time and red
// lights up - "BOOM!" Press B3 for a new game.
// TIP: always try the middle of the remaining range ("binary search"): 4 tries max!
//
// Baglanti / Wiring: Sadece kart uzerindeki parcalar (encoder, B3, buzzer,
// LCD) kullanilir. Trafik lambasi modulu ISTEGE BAGLIDIR ve sabit pinler
// kullanir (KIRMIZI=IO32, SARI=IO26, YESIL=IO25); yoksa kUseTrafficLight'i
// false yapin. / Only onboard parts (encoder, B3, buzzer, LCD) are used. The
// traffic light module is OPTIONAL and uses fixed pins (RED=IO32,
// YELLOW=IO26, GREEN=IO25); if you don't have it, set kUseTrafficLight false.

#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

namespace {
  constexpr bool kUseTrafficLight = true;   // Trafik lambasi takili mi? / traffic light plugged in?
  constexpr uint32_t kGameTimeMs = 60000;   // Oyun suresi / game time
  constexpr uint32_t kHurryMs = 10000;      // Son 10 sn hizli bip / last 10 s fast beeps
  constexpr uint32_t kBeepSlowMs = 1000;    // Normal bip araligi / normal beep interval
  constexpr uint32_t kBeepFastMs = 250;     // Hizli bip araligi / fast beep interval
  constexpr uint32_t kEncoderSettleMs = 40; // Encoder durulma suresi / encoder settle time
  constexpr int kCodeLength = 3;            // Kod hane sayisi / number of code digits

  enum GameState { GAME_PLAYING, GAME_DEFUSED, GAME_EXPLODED };
  enum Hint { HINT_NONE, HINT_CORRECT, HINT_HIGHER, HINT_LOWER };
  GameState state = GAME_PLAYING;
  Hint hint = HINT_NONE;
  int code[kCodeLength];
  int digitIndex = 0;   // Su an aranan hane / digit being searched now
  int selected = 5;     // Encoder ile secilen rakam / digit picked with the encoder
  int lastGuess = 0;
  uint32_t startMs = 0;
  uint32_t nextBeepMs = 0;
  bool yellowOn = false;
  bool beepOn = false;
  uint32_t beepStartMs = 0;
  uint32_t beepLengthMs = 0;
  int confirmedCount = 0;
  int lastRaw = 0;
  uint32_t lastRawChangeMs = 0;
  bool encBtnWasDown = false;
  uint32_t lastEncBtnMs = 0;
  bool b3WasDown = false;
  unsigned long shownSeconds = 0; // Ekrandaki kalan saniye / seconds left shown on screen
  bool screenDirty = true;
}

void light(bool red, bool yellow, bool green) {
  if (kUseTrafficLight) iotbot.moduleTraficLightWrite(red, yellow, green);
}

// buzzerPlayTone() ses bitene kadar BEKLER ve encoder adimlari kacar; bu yuzden buzzerStart() +
// millis() ile kendimiz durduruyoruz. / buzzerPlayTone() WAITS and encoder steps get missed, so
// we use buzzerStart() and stop it ourselves with millis().
void startBeep(int freq, uint32_t lengthMs) {
  iotbot.buzzerStart(freq);
  beepOn = true;
  beepStartMs = millis();
  beepLengthMs = lengthMs;
}

void updateBeep() {
  if (beepOn && millis() - beepStartMs >= beepLengthMs) {
    iotbot.buzzerStop();
    beepOn = false;
  }
}

// encoderRead() "son cagridan beri kac adim" DEGIL, acilistan beri biriken (kumulatif) sayaci
// dondurur. Biz onceki degerle farkina (delta) bakariz. Tek bir "tik" bazen 2-4 adim sayar; bu
// yuzden sayac 40 ms sabit kalinca degisimi TEK tik kabul ederiz: +1 ya da -1.
// encoderRead() returns a cumulative counter since startup, NOT "steps since last call". We
// look at the difference (delta) to the previous value. One "click" may count 2-4 steps, so
// once the counter stays still for 40 ms we treat the change as ONE click: +1 or -1.
int readEncoderClick() {
  int raw = iotbot.encoderRead();
  uint32_t now = millis();
  if (raw != lastRaw) {
    lastRaw = raw;
    lastRawChangeMs = now;
    return 0;
  }
  if (raw != confirmedCount && now - lastRawChangeMs >= kEncoderSettleMs) {
    int delta = raw - confirmedCount;
    confirmedCount = raw;
    return (delta > 0) ? 1 : -1;
  }
  return 0;
}

void writeRow(int row, const char *text) {
  char line[21];
  snprintf(line, sizeof(line), "%-20s", text); // 20'ye bosluklarla tamamla / pad to 20 with spaces
  iotbot.lcdWriteFixed(row, line);
}

// LCD yazmak yavastir (o sirada encoder okunmaz): sadece DEGISEN satirlari yaziyoruz.
// Writing to the LCD is slow (the encoder isn't read meanwhile): we only write CHANGED rows.
void drawTimer(unsigned long secondsLeft) {
  char line[21];
  snprintf(line, sizeof(line), turkish ? "BOMBA IMHA  Sure:%3lu" : "DEFUSE IT   Time:%3lu", secondsLeft);
  writeRow(0, line);
}

void drawCodeAndHint() {
  char line[21];
  // Bulunan haneler, [secili rakam], henuz bakilmayanlar "_". / Found digits, [picked digit], "_".
  char cells[kCodeLength * 3 + 1];
  for (int i = 0; i < kCodeLength; i++) {
    if (i < digitIndex) snprintf(cells + i * 3, 4, " %d ", code[i]);
    else if (i == digitIndex) snprintf(cells + i * 3, 4, "[%d]", selected);
    else snprintf(cells + i * 3, 4, " _ ");
  }
  snprintf(line, sizeof(line), turkish ? "   KOD: %s" : "  CODE: %s", cells);
  writeRow(1, line);

  switch (hint) {
    case HINT_CORRECT: snprintf(line, sizeof(line), "%s", turkish ? "   Dogru! Devam..." : "   Correct! Next..."); break;
    case HINT_HIGHER:  snprintf(line, sizeof(line), turkish ? "  Daha BUYUK! (>%d)" : "  Go HIGHER! (>%d)", lastGuess); break;
    case HINT_LOWER:   snprintf(line, sizeof(line), turkish ? "  Daha KUCUK! (<%d)" : "  Go LOWER! (<%d)", lastGuess); break;
    default:           snprintf(line, sizeof(line), turkish ? "  %d. rakami bulun" : "  Find digit %d", digitIndex + 1); break;
  }
  writeRow(2, line);
}

void newGame() {
  for (int i = 0; i < kCodeLength; i++) code[i] = random(0, 10); // ESP32: donanim rastgele sayi / hardware RNG
  digitIndex = 0;
  selected = 5;
  hint = HINT_NONE;
  state = GAME_PLAYING;
  startMs = millis();
  nextBeepMs = startMs;
  confirmedCount = lastRaw = iotbot.encoderRead(); // Eski donusleri sayma / ignore old turns
  iotbot.buzzerStop();
  light(false, false, false);
  iotbot.lcdClear();
  writeRow(3, turkish ? "Cevir:sec  Bas:onay" : "Turn:pick  Push:ok");
  shownSeconds = 0; // Sure satirini hemen ciz / draw the time row right away
  screenDirty = true;
  char msg[48];
  snprintf(msg, sizeof(msg), turkish ? "Yeni oyun. Gizli kod (ogretmen icin): %d%d%d" : "New game. Secret code (for the teacher): %d%d%d",
           code[0], code[1], code[2]);
  iotbot.serialWrite(msg);
}

void defused() {
  state = GAME_DEFUSED;
  iotbot.buzzerStop();
  light(false, false, true); // Yesil / green
  char line[21];
  snprintf(line, sizeof(line), turkish ? "Kalan sure: %lu sn" : "Time left: %lu s",
           (unsigned long)((kGameTimeMs - (millis() - startMs)) / 1000));
  iotbot.lcdWriteMid(turkish ? "BOMBA IMHA EDILDI!" : "BOMB DEFUSED!", turkish ? "Tebrikler!" : "Well done!", line,
                      turkish ? "B3: Yeni oyun" : "B3: New game");
  iotbot.serialWrite(turkish ? "Bomba imha edildi!" : "Bomb defused!");
  iotbot.buzzerPlayMelody(4); // Zafer melodisi / victory melody
}

void exploded() {
  state = GAME_EXPLODED;
  light(true, false, false); // Kirmizi / red
  char line[21];
  snprintf(line, sizeof(line), turkish ? "Kod: %d %d %d" : "Code: %d %d %d", code[0], code[1], code[2]);
  iotbot.lcdWriteMid("!!! BOOM !!!", turkish ? "Sure doldu..." : "Time is up...", line, turkish ? "B3: Yeni oyun" : "B3: New game");
  iotbot.serialWrite("BOOM!");
  for (int f = 900; f > 100; f -= 25) iotbot.buzzerPlayTone(f, 12); // Dusen ses / falling sound
  for (int i = 0; i < 25; i++) iotbot.buzzerPlayTone(random(60, 200), 25); // Patlama gurultusu / explosion noise
}

void confirmDigit() {
  lastGuess = selected;
  if (selected == code[digitIndex]) {
    digitIndex++;
    hint = HINT_CORRECT;
    startBeep(2000, 120);
    if (digitIndex == kCodeLength) {
      defused();
      return;
    }
    selected = 5;
  } else {
    hint = (selected < code[digitIndex]) ? HINT_HIGHER : HINT_LOWER;
    startBeep(300, 200);
  }
  screenDirty = true;
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  delay(1000); // Acilis ekrani gorunsun / show the startup screen briefly
  newGame();
}

void loop() {
  uint32_t now = millis();
  updateBeep();

  // B3 (true = basili): her an yeni oyun. / B3 (true = pressed): new game at any time.
  bool b3Down = iotbot.button3Read();
  if (b3Down && !b3WasDown) newGame();
  b3WasDown = b3Down;
  if (state != GAME_PLAYING) {
    delay(10);
    return;
  }

  uint32_t elapsed = now - startMs;
  if (elapsed >= kGameTimeMs) {
    exploded();
    return;
  }
  bool hurry = (kGameTimeMs - elapsed) <= kHurryMs;

  int click = readEncoderClick();
  if (click != 0) {
    selected = (selected + click + 10) % 10; // 9'dan sonra 0, 0'dan once 9 / wraps 9->0 and 0->9
    startBeep(3000, 5);                      // Minik "tik" sesi / tiny click sound
    screenDirty = true;
  }

  // Encoder dugmesi INPUT_PULLUP: basiliyken false (LOW) doner. 250 ms sicrama korumasi, yoksa
  // tek basis iki onay sayilabilir. / Encoder button is INPUT_PULLUP: false when pressed. 250 ms
  // debounce, otherwise one press could count as two confirms.
  bool encBtnDown = !iotbot.encoderButtonRead();
  if (encBtnDown && !encBtnWasDown && now - lastEncBtnMs >= 250) {
    lastEncBtnMs = now;
    confirmDigit();
  }
  encBtnWasDown = encBtnDown;
  if (state != GAME_PLAYING) return; // Son hane bulunduysa / if the last digit was found

  // Geri sayim sesi ve sari lamba / countdown beep and yellow light
  if ((int32_t)(now - nextBeepMs) >= 0) {
    startBeep(hurry ? 2200 : 1500, 40);
    yellowOn = !yellowOn;
    light(false, yellowOn, false);
    nextBeepMs += hurry ? kBeepFastMs : kBeepSlowMs;
  }

  unsigned long secondsLeft = (kGameTimeMs - elapsed + 999) / 1000;
  if (secondsLeft != shownSeconds) {
    shownSeconds = secondsLeft;
    drawTimer(secondsLeft);
  }
  if (screenDirty) {
    screenDirty = false;
    drawCodeAndHint();
  }
  delay(2); // Encoder'i sik okumak icin cok kisa / very short, to read the encoder often
}
