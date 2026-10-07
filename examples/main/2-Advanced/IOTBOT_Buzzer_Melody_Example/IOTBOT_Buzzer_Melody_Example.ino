/*
 * TR: MÜZİK / MELODİ ÖZELLİKLERİ - Otomatik müzik kutusu + Manuel piyano
 *  - Açılışta OTOMATİK mod çalışır: kart üzerindeki buzzer, nota adlarıyla ("C4",
 *    "D#5" gibi) yazılmış 3 şarkıyı sırayla çalar (müzik kutusu).
 *  - B3 butonuna basınca MANUEL moda geçer: PİYANO. Potansiyometre ile notayı seçin
 *    (Do4 ... Do6), B1 butonuna (veya joystick butonuna) basınca nota çalar.
 *    B3'e tekrar basınca otomatik moda döner.
 *  - Kütüphanedeki hazır melodiler (1 Doğum Günü, 2 Twinkle Twinkle, 3 Jingle Bells,
 *    4 Başlangıç, 5 Daha Dün Annemizin) "hazir 1-5" komutuyla çalınır. NOT: hazır
 *    melodi çalarken kart başka işe bakmaz (buzzerPlayMelody bitene kadar bekler).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help          -> komut listesi
 *      oto    / auto          -> otomatik mod (müzik kutusu)
 *      manuel / manual        -> manuel mod (piyano)
 *      cal 2  / play 2        -> bu örnekteki 2. şarkıyı çal (1-3)
 *      hazir 5 / preset 5     -> kütüphanedeki hazır melodiyi çal (1-5)
 *      nota C4 / note C4      -> tek nota çal (ör. "nota D#5 500" = 500 ms)
 *      tempo 150              -> tempo (BPM, 40-300)
 *      dur    / stop          -> çalan şarkıyı durdur
 *      dil    / lang          -> dili değiştir (Türkçe <-> English)
 *
 * EN: MUSIC / MELODY FEATURES - Automatic music box + Manual piano
 *  - At startup AUTO mode runs: the onboard buzzer plays 3 songs written with note
 *    names (like "C4", "D#5") one after another (music box).
 *  - Press B3 to switch to MANUAL mode: PIANO. Pick the note with the potentiometer
 *    (C4 ... C6) and press B1 (or the joystick button) to play it. Press B3 again to
 *    go back to auto mode.
 *  - The library's preset melodies (1 Happy Birthday, 2 Twinkle Twinkle, 3 Jingle
 *    Bells, 4 Startup, 5 Daha Dun Annemizin) are played with "preset 1-5". NOTE:
 *    while a preset plays the board does nothing else (buzzerPlayMelody waits).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim        -> command list
 *      auto   / oto           -> auto mode (music box)
 *      manual / manuel        -> manual mode (piano)
 *      play 2 / cal 2         -> play song 2 of this example (1-3)
 *      preset 5 / hazir 5     -> play the library's preset melody (1-5)
 *      note C4 / nota C4      -> play one note (e.g. "note D#5 500" = 500 ms)
 *      tempo 150              -> tempo (BPM, 40-300)
 *      stop   / dur           -> stop the song
 *      lang   / dil           -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ - buzzer, potansiyometre ve butonlar kartın
 * üzerindedir. / NO extra module needed - buzzer, potentiometer and buttons are onboard.
 */

#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// ---------------------------------------------------------------------------
// Şarkılar: nota adı + vuruş (1 = dörtlük nota). "" = sessizlik.
// Songs: note name + beats (1 = quarter note). "" = rest.
// ---------------------------------------------------------------------------
struct Note { const char *name; float beats; };

const Note songScale[] = { // Do-Re-Mi: gam yukarı ve aşağı / scale up and down
  {"C4", 0.5}, {"D4", 0.5}, {"E4", 0.5}, {"F4", 0.5}, {"G4", 0.5}, {"A4", 0.5}, {"B4", 0.5}, {"C5", 1},
  {"B4", 0.5}, {"A4", 0.5}, {"G4", 0.5}, {"F4", 0.5}, {"E4", 0.5}, {"D4", 0.5}, {"C4", 1}};

const Note songDahaDun[] = { // Daha Dün Annemizin (ilk iki satır / first two lines)
  {"C4", 1}, {"C4", 1}, {"G4", 1}, {"G4", 1}, {"A4", 1}, {"A4", 1}, {"G4", 2},
  {"F4", 1}, {"F4", 1}, {"E4", 1}, {"E4", 1}, {"D4", 1}, {"D4", 1}, {"C4", 2}};

const Note songJingle[] = { // Jingle Bells (nakarat başı / start of the chorus)
  {"E4", 1}, {"E4", 1}, {"E4", 2}, {"E4", 1}, {"E4", 1}, {"E4", 2},
  {"E4", 1}, {"G4", 1}, {"C4", 1.5}, {"D4", 0.5}, {"E4", 4}};

struct Song { const char *nameTr; const char *nameEn; const Note *notes; int count; };
const Song songs[] = {
  {"Do-Re-Mi gamı", "Do-Re-Mi scale", songScale, sizeof(songScale) / sizeof(Note)},
  {"Daha Dün Annemizin", "Daha Dun Annemizin", songDahaDun, sizeof(songDahaDun) / sizeof(Note)},
  {"Jingle Bells", "Jingle Bells", songJingle, sizeof(songJingle) / sizeof(Note)}};
const int kSongCount = 3;

const char *presetNamesTr[] = {"", "Doğum Günü", "Twinkle Twinkle", "Jingle Bells", "Başlangıç", "Daha Dün Annemizin"};
const char *presetNamesEn[] = {"", "Happy Birthday", "Twinkle Twinkle", "Jingle Bells", "Startup", "Daha Dun Annemizin"};

// Piyano notaları (potansiyometre ile seçilir) / piano notes (picked with the potentiometer)
const char *pianoNotes[] = {"C4", "D4", "E4", "F4", "G4", "A4", "B4", "C5", "D5", "E5", "F5", "G5", "A5", "B5", "C6"};
const char *solfege[] = {"Do4", "Re4", "Mi4", "Fa4", "Sol4", "La4", "Si4", "Do5", "Re5", "Mi5", "Fa5", "Sol5", "La5", "Si5", "Do6"};
const int kPianoCount = 15;

bool manualMode = false;   // false = OTOMATİK (müzik kutusu), true = MANUEL (piyano)
int bpm = 120;             // Tempo (dakikada vuruş) / tempo (beats per minute)
int currentSong = -1;      // Çalan şarkı (-1 = yok) / song playing (-1 = none)
int noteIndex = 0;
int nextAutoSong = 0;      // Otomatikte sıradaki şarkı / next song in auto mode
uint32_t nextNoteMs = 0;   // Sonraki notanın zamanı / time of the next note
uint32_t lastScreenMs = 0;
int pianoIndex = -1;       // Pot ile seçili nota / note picked with the pot
bool lastB3 = false, lastB1 = false, lastJoy = false;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "ÇAL 2" -> "cal 2"
// Lower-cases and simplifies Turkish letters: "ÇAL 2" -> "cal 2"
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
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- BUZZER MELODİ - Komutlar ----", "---- BUZZER MELODY - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oto / manuel  : müzik kutusu / piyano", "  auto / manual : music box / piano"));
  iotbot.serialWrite(L("  cal 1-3       : bu örnekteki şarkıyı çal", "  play 1-3      : play a song of this example"));
  iotbot.serialWrite(L("  hazir 1-5     : kütüphane melodisini çal (bekletir)", "  preset 1-5    : play a library melody (blocks)"));
  iotbot.serialWrite(L("  nota C4 [ms]  : tek nota çal", "  note C4 [ms]  : play one note"));
  iotbot.serialWrite(L("  tempo 40-300  : hız (BPM)", "  tempo 40-300  : speed (BPM)"));
  iotbot.serialWrite(L("  dur           : şarkıyı durdur", "  stop          : stop the song"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3: OTOMATİK <-> MANUEL,  B1: piyano notası", "  B3: AUTO <-> MANUAL,  B1: piano note"));
}

void drawStaticScreen() {
  lcdRow(0, L("  BUZZER MELODİ", "  BUZZER MELODY"));
  lcdRow(3, manualMode ? L("Pot:nota  B1:çal", "Pot:note  B1:play") : L("B3: manuel (piyano)", "B3: manual (piano)"));
  lastScreenMs = 0;
}

void startSong(int index) {
  currentSong = index;
  noteIndex = 0;
  nextNoteMs = millis();
  iotbot.serialWrite(String(L("Çalıyor: ", "Playing: ")) + (turkish ? songs[index].nameTr : songs[index].nameEn) +
                     "  (" + bpm + " BPM)");
  lastScreenMs = 0;
}

void stopSong() {
  currentSong = -1;
  iotbot.buzzerStop();
  lastScreenMs = 0;
}

void setMode(bool manual) {
  manualMode = manual;
  stopSong();
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  iotbot.serialWrite(manual ? L(">> MANUEL mod: PİYANO. Pot ile nota seçin, B1 ile çalın.", ">> MANUAL mode: PIANO. Pick a note with the pot, play it with B1.")
                            : L(">> OTOMATİK mod: müzik kutusu şarkıları sırayla çalar.", ">> AUTO mode: the music box plays the songs in turn."));
  nextNoteMs = millis() + 800; // Otomatikte kısa bir moladan sonra başla / start after a short pause in auto
  pianoIndex = -1;
  drawStaticScreen();
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  String arg = (space < 0) ? "" : cmd.substring(space + 1);
  arg.trim();
  bool hasValue = arg.length() > 0;
  int value = arg.toInt();

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oto" || word == "otomatik" || word == "auto") {
    setMode(false);
  } else if (word == "manuel" || word == "manual") {
    setMode(true);
  } else if ((word == "cal" || word == "play") && hasValue) {
    if (value < 1 || value > kSongCount) {
      iotbot.serialWrite(L("Şarkı numarası 1-3 olmalı.", "Song number must be 1-3."));
      return;
    }
    if (!manualMode) setMode(true);
    startSong(value - 1);
  } else if ((word == "hazir" || word == "preset") && hasValue) {
    if (value < 1 || value > 5) {
      iotbot.serialWrite(L("Hazır melodi 1-5 olmalı.", "Preset must be 1-5."));
      return;
    }
    if (!manualMode) setMode(true);
    stopSong();
    char line[41];
    snprintf(line, sizeof(line), L("Hazır %d: %s", "Preset %d: %s"), value, turkish ? presetNamesTr[value] : presetNamesEn[value]);
    lcdRow(2, line);
    iotbot.serialWrite(String(line) + L("  (bitene kadar bekleyin)", "  (wait until it ends)"));
    iotbot.buzzerSetTempo(bpm);
    iotbot.buzzerPlayMelody(value); // Bitene kadar bekler / waits until it ends
    iotbot.serialWrite(L("Hazır melodi bitti.", "Preset melody finished."));
    lastScreenMs = 0;
  } else if ((word == "nota" || word == "note") && hasValue) {
    if (!manualMode) setMode(true);
    stopSong();
    int sp = arg.indexOf(' ');
    String noteName = (sp < 0) ? arg : arg.substring(0, sp);
    int ms = (sp < 0) ? 400 : constrain(arg.substring(sp + 1).toInt(), 50, 3000);
    noteName.setCharAt(0, toupper(noteName.charAt(0))); // "c4" -> "C4", "bb3" -> "Bb3"
    iotbot.serialWrite(String(L("Nota: ", "Note: ")) + noteName + " " + ms + " ms");
    iotbot.buzzerPlayNote(noteName.c_str(), ms);
  } else if (word == "tempo" && hasValue) {
    bpm = constrain(value, 40, 300);
    iotbot.buzzerSetTempo(bpm);
    iotbot.serialWrite(String("Tempo: ") + bpm + " BPM");
    lastScreenMs = 0;
  } else if (word == "dur" || word == "stop") {
    if (!manualMode) setMode(true);
    stopSong();
    iotbot.serialWrite(L("Durduruldu.", "Stopped."));
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
  iotbot.buzzerSetTempo(bpm);
  drawStaticScreen();
  iotbot.serialWrite(L("Buzzer melodi örneği başladı.", "Buzzer melody example started."));
  printHelp();
  nextNoteMs = millis() + 800;
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

  // 3) Otomatik: şarkı bitince 1,5 sn mola, sonra sıradaki şarkı
  // 3) Auto: after a song, a 1.5 s pause, then the next song
  if (!manualMode && currentSong < 0 && (int32_t)(now - nextNoteMs) >= 0) {
    startSong(nextAutoSong);
    nextAutoSong = (nextAutoSong + 1) % kSongCount;
  }

  // 4) Şarkıyı nota nota çal: her loop'ta en fazla BİR nota (B3 ve seri port arada çalışır)
  // 4) Play the song note by note: at most ONE note per loop (B3 and serial work in between)
  if (currentSong >= 0 && (int32_t)(now - nextNoteMs) >= 0) {
    const Song &s = songs[currentSong];
    if (noteIndex < s.count) {
      int beatMs = (int)(s.notes[noteIndex].beats * 60000.0f / bpm);
      iotbot.buzzerPlayNote(s.notes[noteIndex].name, beatMs * 9 / 10); // Notanın %90'ı ses / 90% sound
      nextNoteMs = millis() + beatMs / 10;                              // %10 boşluk / 10% gap
      noteIndex++;
    } else {
      currentSong = -1;
      nextNoteMs = millis() + 1500;
      lastScreenMs = 0;
    }
  }

  // 5) Manuel: potansiyometre notayı seçer, B1 veya joystick butonu çalar
  // 5) Manual: the potentiometer picks the note, B1 or the joystick button plays it
  if (manualMode) {
    int idx = map(iotbot.potentiometerRead(), 0, 4095, 0, kPianoCount - 1);
    if (idx != pianoIndex) {
      pianoIndex = idx;
      lastScreenMs = 0;
    }
    bool b1 = iotbot.button1Read();
    bool joy = !iotbot.joystickButtonRead(); // LOW = basılı / LOW = pressed
    if ((b1 && !lastB1) || (joy && !lastJoy)) {
      stopSong();
      iotbot.buzzerPlayNote(pianoNotes[pianoIndex], 300);
    }
    lastB1 = b1;
    lastJoy = joy;
  }

  // 6) LCD (300 ms'de bir, titremesiz) / LCD (every 300 ms, no flicker)
  if (now - lastScreenMs >= 300) {
    lastScreenMs = now;
    char line[41];
    snprintf(line, sizeof(line), L("%s  %d BPM", "%s  %d BPM"), manualMode ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO"), bpm);
    lcdRow(1, line);
    if (currentSong >= 0) {
      snprintf(line, sizeof(line), "%s", turkish ? songs[currentSong].nameTr : songs[currentSong].nameEn);
    } else if (manualMode && pianoIndex >= 0) {
      snprintf(line, sizeof(line), L("Nota: %s (%s)", "Note: %s (%s)"), turkish ? solfege[pianoIndex] : pianoNotes[pianoIndex],
               turkish ? pianoNotes[pianoIndex] : solfege[pianoIndex]);
    } else {
      snprintf(line, sizeof(line), "%s", L("Sıradaki şarkı...", "Next song..."));
    }
    lcdRow(2, line);
  }
}
