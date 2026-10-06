// TR: MUZIK/MELODI OZELLIKLERI. Bu ornek onboard buzzer ile: 1) tek tek
// nota adlariyla ("C4", "D#5" gibi) ozel bir melodi calmayi, 2) hazir
// melodilerden (Dogum Gunu, Twinkle Twinkle, Jingle Bells, Baslangic
// Melodisi, Daha Dun Annemizin) birini calmayi, 3) tempoyu (BPM)
// degistirmeyi gosterir.
// EN: MUSIC/MELODY FEATURES. This example shows, using the onboard
// buzzer: 1) playing a custom melody note-by-note using note names
// (like "C4", "D#5"), 2) playing one of the preset melodies (Happy
// Birthday, Twinkle Twinkle, Jingle Bells, Startup Jingle, Daha Dun
// Annemizin), 3) changing the tempo (BPM).
//
// NOT / NOTE: Melodi 5 ("Daha Dun Annemizin") sarkinin tam halidir (kita +
// nakarat); ezgisi "Ah! Vous dirai-je, Maman" oldugu icin ilk iki satiri
// Melodi 2 (Twinkle Twinkle) ile aynidir. / Melody 5 ("Daha Dun Annemizin")
// is the full song (verse + chorus); it uses the "Ah! Vous dirai-je,
// Maman" tune, so its first two lines match Melody 2 (Twinkle Twinkle).

#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.lcdWriteMid(turkish ? "MELODI OZELLIKLERI" : "MELODY FEATURES",
                      turkish ? "Baslatiliyor..." : "Starting...",
                      "", "");

  // 1) Ozel nota nota melodi / custom note-by-note melody
  iotbot.serialWrite(turkish ? "Ozel melodi: C4-E4-G4-C5" : "Custom melody: C4-E4-G4-C5");
  iotbot.buzzerPlayNote("C4", 300);
  iotbot.buzzerPlayNote("E4", 300);
  iotbot.buzzerPlayNote("G4", 300);
  iotbot.buzzerPlayNote("C5", 500);
  delay(500);

  // 2) Hazir melodiler / preset melodies
  iotbot.buzzerSetTempo(120);
  iotbot.serialWrite(turkish ? "Melodi 1: Dogum Gunu" : "Melody 1: Happy Birthday");
  iotbot.lcdWriteMid(turkish ? "Calıyor:" : "Playing:", turkish ? "Dogum Gunu" : "Happy Birthday", "", "");
  iotbot.buzzerPlayMelody(1);
  delay(500);

  iotbot.serialWrite(turkish ? "Melodi 2: Twinkle Twinkle" : "Melody 2: Twinkle Twinkle");
  iotbot.lcdWriteMid(turkish ? "Calıyor:" : "Playing:", "Twinkle Twinkle", "", "");
  iotbot.buzzerPlayMelody(2);
  delay(500);

  iotbot.serialWrite(turkish ? "Melodi 3: Jingle Bells" : "Melody 3: Jingle Bells");
  iotbot.lcdWriteMid(turkish ? "Calıyor:" : "Playing:", "Jingle Bells", "", "");
  iotbot.buzzerPlayMelody(3);
  delay(500);

  // 3) Tempoyu degistirip baslangic melodisini tekrar cal / change tempo, replay the startup jingle
  iotbot.buzzerSetTempo(200); // Daha hizli / faster
  iotbot.serialWrite(turkish ? "Melodi 4 (hizli tempo): Baslangic Melodisi" : "Melody 4 (fast tempo): Startup Jingle");
  iotbot.lcdWriteMid(turkish ? "Calıyor:" : "Playing:", turkish ? "Baslangic (hizli)" : "Startup (fast)", "", "");
  iotbot.buzzerPlayMelody(4);

  iotbot.serialWrite(turkish ? "Bitti!" : "Done!");
  iotbot.lcdWriteMid(turkish ? "Melodi ornekleri" : "Melody examples", turkish ? "tamamlandi." : "completed.", "", "");
}

void loop() {
}
