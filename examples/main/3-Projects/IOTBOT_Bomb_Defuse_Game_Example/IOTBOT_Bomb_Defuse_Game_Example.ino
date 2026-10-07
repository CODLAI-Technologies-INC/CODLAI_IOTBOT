/*
 * TR: GERÇEK PROJE - Bomba İmha Oyunu. Kart, rastgele 3 haneli gizli bir kod
 * seçer (her hane 0-9). 60 saniyeniz var! ENCODER'ı çevirerek bir rakam seçin,
 * encoder'ın düğmesine basarak onaylayın. LCD size ipucu verir: "Doğru!" ya da
 * "Daha BÜYÜK / Daha KÜÇÜK". Doğru rakamı bulana kadar aynı haneyi tekrar
 * deneyebilirsiniz. Süre akarken buzzer her saniye "bip" eder, son 10 saniyede
 * hızlanır; trafik lambasının sarısı yanıp söner. Üç haneyi de bulursanız
 * yeşil yanar ve zafer melodisi çalar; süre biterse kırmızı yanar ve "BOOM!"
 * Yeni oyun için B3'e basın.
 * İPUCU: hep kalan aralığın ortasını deneyin ("ikili arama"), en fazla 4 deneme!
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim   / help      -> komut listesi
 *      yeni     / new       -> yeni oyun (B3 gibi)
 *      sure 90  / time 90   -> oyun süresi (20-300 sn, yeni oyunda geçerli)
 *      dil      / lang      -> dili değiştir (Türkçe <-> English)
 *    Gizli kod her yeni oyunda Seri Monitör'e yazılır (öğretmen için).
 *
 * EN: A REAL PROJECT - Bomb Defuse Game. The board picks a random secret
 * 3-digit code (each digit 0-9). You have 60 seconds! Turn the ENCODER to pick
 * a digit and press the encoder's button to confirm. The LCD gives a hint:
 * "Correct!" or "Go HIGHER / Go LOWER". You retry the same digit until you
 * find it. While time runs the buzzer beeps every second, faster in the last
 * 10 seconds, and the traffic light's yellow blinks. Find all three digits
 * and green lights up with a victory melody; run out of time and red lights
 * up - "BOOM!" Press B3 for a new game.
 * TIP: always try the middle of the remaining range ("binary search"): 4 tries max!
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help     / yardim    -> command list
 *      new      / yeni      -> new game (like B3)
 *      time 90  / sure 90   -> game time (20-300 s, used from the next game)
 *      lang     / dil       -> switch language (Turkish <-> English)
 *    The secret code is printed to the Serial Monitor at every new game (for the teacher).
 *
 * Bağlantı / Wiring: Sadece kart üzerindeki parçalar (encoder, B3, buzzer,
 * LCD) kullanılır. Trafik lambası modülü İSTEĞE BAĞLIDIR ve sabit pinler
 * kullanır (KIRMIZI=IO32, SARI=IO26, YEŞİL=IO25); yoksa kUseTrafficLight'ı
 * false yapın. / Only onboard parts (encoder, B3, buzzer, LCD) are used. The
 * traffic light module is OPTIONAL and uses fixed pins (RED=IO32,
 * YELLOW=IO26, GREEN=IO25); if you don't have it, set kUseTrafficLight false.
 */

#include <IOTBOT.h>

IOTBOT iotbot;

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

namespace {
  constexpr bool kUseTrafficLight = true;   // Trafik lambası takılı mı? / traffic light plugged in?
  constexpr uint32_t kHurryMs = 10000;      // Son 10 sn hızlı bip / last 10 s fast beeps
  constexpr uint32_t kBeepSlowMs = 1000;    // Normal bip aralığı / normal beep interval
  constexpr uint32_t kBeepFastMs = 250;     // Hızlı bip aralığı / fast beep interval
  constexpr uint32_t kEncoderSettleMs = 40; // Encoder durulma süresi / encoder settle time
  constexpr int kCodeLength = 3;            // Kod hane sayısı / number of code digits

  enum GameState { GAME_PLAYING, GAME_DEFUSED, GAME_EXPLODED };
  enum Hint { HINT_NONE, HINT_CORRECT, HINT_HIGHER, HINT_LOWER };
  uint32_t gameTimeMs = 60000;  // Oyun süresi / game time
  GameState state = GAME_PLAYING;
  Hint hint = HINT_NONE;
  int code[kCodeLength];
  int digitIndex = 0;   // Şu an aranan hane / digit being searched now
  int selected = 5;     // Encoder ile seçilen rakam / digit picked with the encoder
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
  unsigned long defusedLeftSec = 0;
  bool screenDirty = true;
}

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "SÜRE" -> "sure"
// Lower-cases and simplifies Turkish letters: "SÜRE" -> "sure"
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
// Donanım yardımcıları / Hardware helpers
// ---------------------------------------------------------------------------
void light(bool red, bool yellow, bool green) {
  if (kUseTrafficLight) iotbot.moduleTraficLightWrite(red, yellow, green);
}

// buzzerPlayTone() ses bitene kadar BEKLER ve encoder adımları kaçar; bu yüzden buzzerStart() +
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

// encoderRead() "son çağrıdan beri kaç adım" DEĞİL, açılıştan beri biriken (kümülatif) sayacı
// döndürür. Biz önceki değerle farkına (delta) bakarız. Tek bir "tık" bazen 2-4 adım sayar; bu
// yüzden sayaç 40 ms sabit kalınca değişimi TEK tık kabul ederiz: +1 ya da -1.
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

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
// lcdWriteFixedTxt Türkçe harfleri (ç, ğ, ı, ö, ş, ü) LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- BOMBA İMHA OYUNU - Komutlar ----", "---- BOMB DEFUSE GAME - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  yeni          : yeni oyun", "  new           : new game"));
  iotbot.serialWrite(L("  sure 20-300   : oyun süresi (sn)", "  time 20-300   : game time (s)"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  Encoder: çevir = rakam seç, bas = onayla.  B3: yeni oyun", "  Encoder: turn = pick a digit, push = confirm.  B3: new game"));
}

// LCD yazmak yavaştır (o sırada encoder okunmaz): sadece DEĞİŞEN satırları yazıyoruz.
// Writing to the LCD is slow (the encoder isn't read meanwhile): we only write CHANGED rows.
void drawTimer(unsigned long secondsLeft) {
  char line[41];
  snprintf(line, sizeof(line), L("BOMBA İMHA  Süre:%3lu", "DEFUSE IT   Time:%3lu"), secondsLeft);
  lcdRow(0, line);
}

void drawCodeAndHint() {
  char line[41];
  // Bulunan haneler, [seçili rakam], henüz bakılmayanlar "_". / Found digits, [picked digit], "_".
  char cells[kCodeLength * 3 + 1];
  for (int i = 0; i < kCodeLength; i++) {
    if (i < digitIndex) snprintf(cells + i * 3, 4, " %d ", code[i]);
    else if (i == digitIndex) snprintf(cells + i * 3, 4, "[%d]", selected);
    else snprintf(cells + i * 3, 4, " _ ");
  }
  snprintf(line, sizeof(line), L("   KOD: %s", "  CODE: %s"), cells);
  lcdRow(1, line);

  switch (hint) {
    case HINT_CORRECT: snprintf(line, sizeof(line), "%s", L("   Doğru! Devam...", "   Correct! Next...")); break;
    case HINT_HIGHER:  snprintf(line, sizeof(line), L("  Daha BÜYÜK! (>%d)", "  Go HIGHER! (>%d)"), lastGuess); break;
    case HINT_LOWER:   snprintf(line, sizeof(line), L("  Daha KÜÇÜK! (<%d)", "  Go LOWER! (<%d)"), lastGuess); break;
    default:           snprintf(line, sizeof(line), L("  %d. rakamı bulun", "  Find digit %d"), digitIndex + 1); break;
  }
  lcdRow(2, line);
}

void showDefusedScreen() {
  char line[41];
  snprintf(line, sizeof(line), L("Kalan süre: %lu sn", "Time left: %lu s"), defusedLeftSec);
  iotbot.lcdWriteMid(L("BOMBA İMHA EDİLDİ!", "BOMB DEFUSED!"), L("Tebrikler!", "Well done!"), line, L("B3: Yeni oyun", "B3: New game"));
}

void showExplodedScreen() {
  char line[41];
  snprintf(line, sizeof(line), L("Kod: %d %d %d", "Code: %d %d %d"), code[0], code[1], code[2]);
  iotbot.lcdWriteMid("!!! BOOM !!!", L("Süre doldu...", "Time is up..."), line, L("B3: Yeni oyun", "B3: New game"));
}

// Oyun ekranını baştan çizer (yeni oyunda ve dil değişince).
// Redraws the game screen from scratch (new game and language change).
void drawPlayScreen() {
  iotbot.lcdClear();
  lcdRow(3, L("Çevir:seç  Bas:onay", "Turn:pick  Push:ok"));
  shownSeconds = 0; // Süre satırını hemen çiz / draw the time row right away
  screenDirty = true;
}

void newGame() {
  for (int i = 0; i < kCodeLength; i++) code[i] = random(0, 10); // ESP32: donanım rastgele sayı / hardware RNG
  digitIndex = 0;
  selected = 5;
  hint = HINT_NONE;
  state = GAME_PLAYING;
  startMs = millis();
  nextBeepMs = startMs;
  confirmedCount = lastRaw = iotbot.encoderRead(); // Eski dönüşleri sayma / ignore old turns
  iotbot.buzzerStop();
  light(false, false, false);
  drawPlayScreen();
  char msg[80];
  snprintf(msg, sizeof(msg), L("Yeni oyun (%lu sn). Gizli kod (öğretmen için): %d%d%d", "New game (%lu s). Secret code (for the teacher): %d%d%d"),
           (unsigned long)(gameTimeMs / 1000), code[0], code[1], code[2]);
  iotbot.serialWrite(msg);
}

void defused() {
  state = GAME_DEFUSED;
  iotbot.buzzerStop();
  light(false, false, true); // Yeşil / green
  defusedLeftSec = (gameTimeMs - (millis() - startMs)) / 1000;
  showDefusedScreen();
  iotbot.serialWrite(String(L("Bomba imha edildi! Kalan süre: ", "Bomb defused! Time left: ")) + defusedLeftSec + L(" sn", " s"));
  iotbot.buzzerPlayMelody(4); // Zafer melodisi / victory melody
}

void exploded() {
  state = GAME_EXPLODED;
  light(true, false, false); // Kırmızı / red
  showExplodedScreen();
  iotbot.serialWrite("BOOM!");
  for (int f = 900; f > 100; f -= 25) iotbot.buzzerPlayTone(f, 12); // Düşen ses / falling sound
  for (int i = 0; i < 25; i++) iotbot.buzzerPlayTone(random(60, 200), 25); // Patlama gürültüsü / explosion noise
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

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "yeni" || word == "new") {
    newGame();
  } else if ((word == "sure" || word == "time") && hasValue) {
    gameTimeMs = (uint32_t)constrain(value, 20, 300) * 1000UL;
    iotbot.serialWrite(String(L("Oyun süresi: ", "Game time: ")) + (gameTimeMs / 1000) +
                       L(" sn (yeni oyunda geçerli: yeni yazın)", " s (used from the next game: type new)"));
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    if (state == GAME_PLAYING) drawPlayScreen();
    else if (state == GAME_DEFUSED) showDefusedScreen();
    else showExplodedScreen();
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  printHelp();
  delay(1000); // Açılış ekranı görünsün / show the startup screen briefly
  newGame();
}

void loop() {
  uint32_t now = millis();
  updateBeep();

  // B3 (true = basılı): her an yeni oyun. / B3 (true = pressed): new game at any time.
  bool b3Down = iotbot.button3Read();
  if (b3Down && !b3WasDown) newGame();
  b3WasDown = b3Down;

  // Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  if (state != GAME_PLAYING) {
    delay(10);
    return;
  }

  uint32_t elapsed = now - startMs;
  if (elapsed >= gameTimeMs) {
    exploded();
    return;
  }
  bool hurry = (gameTimeMs - elapsed) <= kHurryMs;

  int click = readEncoderClick();
  if (click != 0) {
    selected = (selected + click + 10) % 10; // 9'dan sonra 0, 0'dan önce 9 / wraps 9->0 and 0->9
    startBeep(3000, 5);                      // Minik "tık" sesi / tiny click sound
    screenDirty = true;
  }

  // Encoder düğmesi INPUT_PULLUP: basılıyken false (LOW) döner. 250 ms sıçrama koruması, yoksa
  // tek basış iki onay sayılabilir. / Encoder button is INPUT_PULLUP: false when pressed. 250 ms
  // debounce, otherwise one press could count as two confirms.
  bool encBtnDown = !iotbot.encoderButtonRead();
  if (encBtnDown && !encBtnWasDown && now - lastEncBtnMs >= 250) {
    lastEncBtnMs = now;
    confirmDigit();
  }
  encBtnWasDown = encBtnDown;
  if (state != GAME_PLAYING) return; // Son hane bulunduysa / if the last digit was found

  // Geri sayım sesi ve sarı lamba / countdown beep and yellow light
  if ((int32_t)(now - nextBeepMs) >= 0) {
    startBeep(hurry ? 2200 : 1500, 40);
    yellowOn = !yellowOn;
    light(false, yellowOn, false);
    nextBeepMs += hurry ? kBeepFastMs : kBeepSlowMs;
  }

  unsigned long secondsLeft = (gameTimeMs - elapsed + 999) / 1000;
  if (secondsLeft != shownSeconds) {
    shownSeconds = secondsLeft;
    drawTimer(secondsLeft);
  }
  if (screenDirty) {
    screenDirty = false;
    drawCodeAndHint();
  }
  delay(2); // Encoder'ı sık okumak için çok kısa / very short, to read the encoder often
}
