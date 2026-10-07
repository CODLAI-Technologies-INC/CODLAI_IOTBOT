/*
 * TR: BUZZER TESTİ - Otomatik melodi + Manuel çalma
 *  - Açılışta OTOMATİK mod çalışır: buzzer do-re-mi gamını çıkar/iner, sonra
 *    "Daha Dün Annemizin" melodisinin başını çalar ve tekrar eder. Çalma loop'u
 *    bekletmez (millis ile), bu yüzden butonlar ve seri port hemen tepki verir.
 *  - B3 butonuna basınca MANUEL moda geçer: potansiyometre = ses perdesi (Hz),
 *    B1 basılı tuttukça ses çalar. LCD frekansı ve en yakın notayı gösterir.
 *    B3'e tekrar basınca otomatik moda döner.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim     / help        -> komut listesi
 *      oto        / auto        -> otomatik mod (melodi)
 *      manuel     / manual      -> manuel mod (pot + B1)
 *      nota C4    / note C4     -> notayı çal (C D E F G A B, # / b, oktav 2-7;
 *                                  do re mi fa sol la si da olur: nota la4)
 *      ton 440    / tone 440    -> 440 Hz çal (yarım saniye)
 *      sus        / mute        -> sesi hemen kes (manuel moda geçer)
 *      dil        / lang        -> dili değiştir (Türkçe <-> English)
 *    (nota/ton komutları otomatik moddaysa manuel moda geçirir.)
 *
 * EN: BUZZER TEST - Automatic melody + Manual playing
 *  - At startup AUTO mode runs: the buzzer plays the do-re-mi scale up and down,
 *    then the start of the "Twinkle Twinkle Little Star" tune, and repeats. Playing
 *    does not block the loop (millis), so the buttons and serial react at once.
 *  - Press B3 to switch to MANUAL mode: potentiometer = pitch (Hz), the sound
 *    plays while you hold B1. The LCD shows the frequency and the nearest note.
 *    Press B3 again to go back to auto mode.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help       / yardim      -> command list
 *      auto       / oto         -> auto mode (melody)
 *      manual     / manuel      -> manual mode (pot + B1)
 *      note C4    / nota C4     -> play that note (C D E F G A B, # / b, octave 2-7;
 *                                  do re mi fa sol la si also work: note la4)
 *      tone 440   / ton 440     -> play 440 Hz (half a second)
 *      mute       / sus         -> stop the sound now (switches to manual)
 *      lang       / dil         -> switch language (Turkish <-> English)
 *    (note/tone commands switch to manual mode when in auto mode.)
 *
 * Bağlantı / Wiring: Ek bağlantı gerekmez; buzzer IoTBot kartının üzerindedir (GPIO12).
 *                    No extra wiring; the buzzer is on the IoTBot board (GPIO12).
 */

#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Otomatik melodi: frekans (Hz, 0 = sessiz mola) ve süre (ms)
// Auto melody: frequency (Hz, 0 = silent rest) and duration (ms)
// Do-re-mi gamı çıkış/iniş, sonra "Daha Dün Annemizin" (Twinkle Twinkle) başı.
// Do-re-mi scale up/down, then the start of "Twinkle Twinkle Little Star".
const int MELODY_HZ[] = {
  262, 294, 330, 349, 392, 440, 494, 523,     // C4 D4 E4 F4 G4 A4 B4 C5
  494, 440, 392, 349, 330, 294, 262, 0,       // B4 ... C4, mola / rest
  262, 262, 392, 392, 440, 440, 392,          // C C G G A A G
  349, 349, 330, 330, 294, 294, 262, 0        // F F E E D D C, mola / rest
};
const int MELODY_MS[] = {
  200, 200, 200, 200, 200, 200, 200, 400,
  200, 200, 200, 200, 200, 200, 400, 800,
  350, 350, 350, 350, 350, 350, 700,
  350, 350, 350, 350, 350, 350, 700, 1500
};
const int MELODY_LEN = sizeof(MELODY_HZ) / sizeof(MELODY_HZ[0]);

const int MIN_HZ = 200;     // Pot ile en pes ses / lowest pitch with the pot
const int MAX_HZ = 2000;    // Pot ile en tiz ses / highest pitch with the pot

bool manualMode = false;    // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
int noteIndex = 0;          // Melodide sıradaki nota / current note in the melody
uint32_t noteStartMs = 0;   // Notanın başladığı an / when the note started
bool noteSounding = false;  // Nota şu an çalıyor mu? / is the note sounding now?
int currentHz = 0;          // Şu an çalan frekans (0 = sessiz) / frequency playing now (0 = silent)
uint32_t oneShotUntilMs = 0; // Seri komutla çalan sesin bitiş anı / end of a sound started by serial
bool oneShot = false;
uint32_t lastScreenMs = 0;
bool lastB3 = false;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "SUS" -> "sus"
// Lower-cases and simplifies Turkish letters: "SUS" -> "sus"
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
// Ses yardımcıları / Sound helpers
// ---------------------------------------------------------------------------
// buzzerPlayTone() ses bitene kadar BEKLER (blocking). Bu yüzden buzzerStart() ile
// sesi başlatıp buzzerStop() ile millis zamanında kendimiz durduruyoruz.
// buzzerPlayTone() WAITS until the sound ends (blocking). So we start the sound with
// buzzerStart() and stop it ourselves at the right millis time with buzzerStop().
void soundOn(int hz) {
  if (hz == currentHz) return; // Aynı ses zaten çalıyor / the same sound is already playing
  currentHz = hz;
  if (hz > 0) iotbot.buzzerStart(hz);
  else iotbot.buzzerStop();
}

void soundOff() { soundOn(0); }

// Nota adını frekansa çevirir: "c4", "f#5", "bb3", "la4", "sol5" ... (tanınmazsa 0)
// Converts a note name to a frequency: "c4", "f#5", "bb3", "la4", "sol5" ... (0 if unknown)
int noteToHz(String s) {
  int semitone = -1; // C'den itibaren yarım ton / semitones from C
  int pos = 0;
  // Önce solfej adları (do re mi fa sol la si) / solfège names first
  const char *solfege[] = {"sol", "do", "re", "mi", "fa", "la", "si"};
  const int solfegeSemi[] = {7, 0, 2, 4, 5, 9, 11};
  for (int i = 0; i < 7; i++) {
    if (s.startsWith(solfege[i])) {
      semitone = solfegeSemi[i];
      pos = strlen(solfege[i]);
      break;
    }
  }
  // Sonra harf adları (C D E F G A B) / then letter names (C D E F G A B)
  if (semitone < 0 && s.length() > 0) {
    const char *letters = "cdefgab";
    const int letterSemi[] = {0, 2, 4, 5, 7, 9, 11};
    const char *found = strchr(letters, s[0]);
    if (found == nullptr || s[0] == '\0') return 0;
    semitone = letterSemi[found - letters];
    pos = 1;
  }
  if (semitone < 0) return 0;
  // Diyez (#) veya bemol (b) / sharp (#) or flat (b)
  if (pos < (int)s.length() && s[pos] == '#') { semitone++; pos++; }
  else if (pos < (int)s.length() && s[pos] == 'b') { semitone--; pos++; }
  int octave = (pos < (int)s.length()) ? s.substring(pos).toInt() : 4; // Oktav yoksa 4 / octave 4 if missing
  if (octave < 2 || octave > 7) return 0;
  // A4 = 440 Hz; her yarım ton 2^(1/12) kat / each semitone is 2^(1/12) times
  int fromA4 = (octave - 4) * 12 + (semitone - 9);
  return (int)round(440.0 * pow(2.0, fromA4 / 12.0));
}

// Frekansa en yakın notanın adı: 440 -> "A4" / name of the nearest note: 440 -> "A4"
void hzToNoteName(int hz, char *out, size_t size) {
  if (hz <= 0) { snprintf(out, size, "-"); return; }
  const char *names[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
  int n = (int)lround(12.0 * log(hz / 440.0) / log(2.0)) + 57; // 57 = A4 (C0'dan beri / since C0)
  if (n < 0) n = 0;
  snprintf(out, size, "%s%d", names[n % 12], n / 12);
}

void setMode(bool manual); // Aşağıda tanımlı / defined below

// Seri komutla bir sesi belirli süre çal (bekletmeden) / play a sound for a time from serial (no blocking)
void playOneShot(int hz, uint32_t ms) {
  if (!manualMode) setMode(true);
  soundOn(hz);
  oneShot = true;
  oneShotUntilMs = millis() + ms;
  lastScreenMs = 0;
}

int potToHz() { return map(iotbot.potentiometerRead(), 0, 4095, MIN_HZ, MAX_HZ); }

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
// lcdWriteFixedTxt Türkçe harfleri (ç, ğ, ı, ö, ş, ü) LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- BUZZER - Komutlar ----", "---- BUZZER - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oto           : otomatik mod (melodi)", "  auto          : auto mode (melody)"));
  iotbot.serialWrite(L("  manuel        : manuel mod (pot + B1)", "  manual        : manual mode (pot + B1)"));
  iotbot.serialWrite(L("  nota C4       : notayı çal (C4, F#5, Bb3, la4, sol5 ...)", "  note C4       : play the note (C4, F#5, Bb3, la4, sol5 ...)"));
  iotbot.serialWrite(L("  ton 440       : 440 Hz çal (50-5000)", "  tone 440      : play 440 Hz (50-5000)"));
  iotbot.serialWrite(L("  sus           : sesi kes", "  mute          : stop the sound"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : OTOMATİK <-> MANUEL", "  B3 button     : AUTO <-> MANUAL"));
  iotbot.serialWrite(L("  B1 basılı     : pot frekansında çal (manuel)", "  hold B1       : play at the pot pitch (manual)"));
}

void drawStaticScreen() {
  lcdRow(0, L("    BUZZER TESTİ", "    BUZZER TEST"));
  lcdRow(3, manualMode ? L("Pot:ton  B1:çal", "Pot:pitch  B1:play") : L("B3: manuel kontrol", "B3: manual control"));
  lastScreenMs = 0; // Değerleri hemen çiz / draw the values right away
}

void setMode(bool manual) {
  manualMode = manual;
  soundOff();
  oneShot = false;
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  currentHz = 0; // Bip bitti, buzzer sessiz / the beep is over, the buzzer is silent
  iotbot.serialWrite(manual ? L(">> MANUEL mod: potla sesi ayarlayın, B1 basılıyken çalar.", ">> MANUAL mode: set the pitch with the pot, it plays while B1 is held.")
                            : L(">> OTOMATİK mod: gam ve melodi çalıyor.", ">> AUTO mode: playing the scale and the tune."));
  if (!manual) {
    noteIndex = 0;
    noteStartMs = millis();
    noteSounding = false;
  }
  drawStaticScreen();
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  String arg = hasValue ? cmd.substring(space + 1) : "";
  arg.trim();

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oto" || word == "otomatik" || word == "auto") {
    setMode(false);
  } else if (word == "manuel" || word == "manual") {
    setMode(true);
  } else if ((word == "nota" || word == "note") && hasValue) {
    int hz = noteToHz(arg);
    if (hz > 0) {
      playOneShot(hz, 500);
      iotbot.serialWrite(String(L("Nota: ", "Note: ")) + arg + " = " + hz + " Hz");
    } else {
      iotbot.serialWrite(L("Nota anlaşılmadı. Örnek: nota C4, nota F#5, nota la4", "Note not understood. Example: note C4, note F#5, note la4"));
    }
  } else if ((word == "ton" || word == "tone") && hasValue) {
    int hz = constrain(arg.toInt(), 50, 5000);
    playOneShot(hz, 500);
    iotbot.serialWrite(String(L("Ton: ", "Tone: ")) + hz + " Hz");
  } else if (word == "sus" || word == "mute") {
    if (!manualMode) setMode(true);
    soundOff();
    oneShot = false;
    iotbot.serialWrite(L("Ses kesildi.", "Sound stopped."));
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
  iotbot.lcdClear();
  drawStaticScreen();
  iotbot.serialWrite(L("Buzzer testi başladı.", "Buzzer test started."));
  printHelp();
  noteStartMs = millis();
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
    if (oneShot) {
      // 3a) Seri komutla başlayan ses süresi dolunca durur / a serial sound stops when its time is up
      if ((int32_t)(millis() - oneShotUntilMs) >= 0) {
        oneShot = false;
        soundOff();
      }
    } else if (iotbot.button1Read()) {
      // 3b) B1 basılı: pot frekansında çal (10 Hz'den küçük oynamaları yok say)
      // 3b) B1 held: play at the pot pitch (ignore changes smaller than 10 Hz)
      int hz = potToHz();
      if (currentHz == 0 || abs(hz - currentHz) >= 10) soundOn(hz);
    } else {
      soundOff();
    }
  } else {
    // 3c) Otomatik melodi: notanın %90'ı ses, %10'u sessizlik (notalar birbirinden ayrılsın)
    // 3c) Auto melody: 90% of the note is sound, 10% silence (so notes are separated)
    uint32_t elapsed = millis() - noteStartMs;
    if (elapsed >= (uint32_t)MELODY_MS[noteIndex]) {
      noteIndex = (noteIndex + 1) % MELODY_LEN;
      noteStartMs = millis();
      noteSounding = false;
    } else if (!noteSounding && elapsed < (uint32_t)MELODY_MS[noteIndex] * 9 / 10) {
      noteSounding = true;
      soundOn(MELODY_HZ[noteIndex]);
      lastScreenMs = 0;
    } else if (noteSounding && elapsed >= (uint32_t)MELODY_MS[noteIndex] * 9 / 10) {
      soundOff();
    }
  }

  // 4) LCD (250 ms'de bir, titremesiz) / LCD (every 250 ms, no flicker)
  if (millis() - lastScreenMs >= 250) {
    lastScreenMs = millis();
    char line[41];
    char note[8];
    snprintf(line, sizeof(line), L("Mod: %s", "Mode: %s"), manualMode ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO"));
    lcdRow(1, line);
    int shownHz = currentHz;
    if (manualMode && currentHz == 0) shownHz = potToHz(); // Sessizken potun ayarı / pot setting while silent
    hzToNoteName(shownHz, note, sizeof(note));
    if (shownHz > 0) snprintf(line, sizeof(line), L("Ton:%5d Hz  %s", "Tone:%5d Hz  %s"), shownHz, note);
    else snprintf(line, sizeof(line), "%s", L("Ton: sessiz", "Tone: silent"));
    lcdRow(2, line);
  }
}
