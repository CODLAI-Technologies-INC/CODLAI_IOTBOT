/*
 * TR: GERÇEK PROJE - Alkışla Yanan Lamba.
 *  - OTOMATİK mod (açılışta): iki kez peş peşe alkış çalınca karttaki röle (ve
 *    ona bağlayacağınız lamba) açılır, tekrar iki alkışta kapanır. Açılışta 1
 *    saniye boyunca odanın "sessizlik seviyesini" ölçer ve alkış eşiğini buna
 *    göre kendisi ayarlar; böylece sessiz bir odada da gürültülü bir sınıfta da
 *    çalışır.
 *  - B3 butonu MANUEL moda geçer: alkışlar yok sayılır, lambayı B1 (veya B2)
 *    butonu ile siz açıp kapatırsınız (örneğin gürültülü bir partide). B3'e
 *    tekrar basınca otomatik moda döner.
 *  - NEDEN ÇİFT ALKIŞ? Tek bir yüksek ses (kapı çarpması, düşen kalem, bir
 *    öksürük) çok sık olur ve lambayı rastgele açıp kapatırdı. Ama iki KISA
 *    sesin 0.15 ile 0.7 saniye arayla gelmesi tesadüfen neredeyse hiç olmaz.
 *    Ayrıca uzun süren sesler (konuşma, müzik) "kısa" olmadığı için alkış
 *    sayılmaz. Bu iki kural yanlış tetiklenmeyi çok azaltır.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim     / help           -> komut listesi
 *      oto        / auto           -> otomatik mod (alkış)
 *      manuel     / manual         -> manuel mod (B1 ile aç/kapat)
 *      ac         / on             -> lambayı aç (manuel moda geçer)
 *      kapat      / off            -> lambayı kapat (manuel moda geçer)
 *      kalibre    / calibrate      -> sessizliği yeniden ölç (sessiz olun!)
 *      esik 250   / threshold 250  -> sessizliğin ne kadar üstü alkış sayılır
 *      oku        / read           -> ses seviyesi ve eşiği yaz
 *      dil        / lang           -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Clap Switch.
 *  - AUTO mode (at startup): clap twice in a row and the board's relay (and
 *    the lamp you connect to it) turns on; two more claps turn it off. At
 *    startup it measures the room's "quiet level" for 1 second and sets the
 *    clap threshold from it, so it works in a quiet room and in a noisy
 *    classroom.
 *  - Button B3 switches to MANUAL mode: claps are ignored and you switch the
 *    lamp with button B1 (or B2) (e.g. at a noisy party). Press B3 again to go
 *    back to auto mode.
 *  - WHY A DOUBLE CLAP? A single loud sound (a door slam, a dropped pen, a
 *    cough) happens all the time and would toggle the lamp randomly. But two
 *    SHORT sounds arriving 0.15 to 0.7 seconds apart almost never happens by
 *    accident. Long sounds (talking, music) are also not "short", so they are
 *    not counted as claps. These two rules cut false triggers a lot.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help          / yardim     -> command list
 *      auto          / oto        -> auto mode (claps)
 *      manual        / manuel     -> manual mode (switch with B1)
 *      on            / ac         -> lamp on (switches to manual)
 *      off           / kapat      -> lamp off (switches to manual)
 *      calibrate     / kalibre    -> measure the quiet level again (be quiet!)
 *      threshold 250 / esik 250   -> how far above quiet counts as a clap
 *      read          / oku        -> print the sound level and threshold
 *      lang          / dil        -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Mikrofon (ses sensörü) modülünü P4 soketine (IO32)
 * takın. Röle, B1 ve B3 kart üzerindedir. / Plug the microphone (sound
 * sensor) module into socket P4 (IO32). The relay, B1 and B3 are on the board.
 */

#include <IOTBOT.h>

IOTBOT iotbot;

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

#define MIC_PIN IO32 // P4 soketi (analog için ADC1 pini) / socket P4 (ADC1 pin for analog)

namespace {
  constexpr uint32_t kCalibrationMs = 1000; // Sessizlik ölçüm süresi / quiet calibration time
  constexpr uint32_t kSampleWindowMs = 10;  // Her seviye ölçümü 10 ms / each level reading is 10 ms
  constexpr uint32_t kMaxClapMs = 120;      // Alkış KISA bir patlamadır / a clap is a SHORT burst
  constexpr uint32_t kMinGapMs = 150;       // İki alkış arası en az / min gap between the claps
  constexpr uint32_t kMaxGapMs = 700;       // İki alkış arası en fazla / max gap between the claps
  constexpr uint32_t kCooldownMs = 1000;    // Röle "tık" sesi alkış sanılmasın / relay click is not a clap
  constexpr uint32_t kScreenMs = 200;       // LCD yenileme aralığı / LCD refresh interval

  int margin = 250;          // Sessizliğin ne kadar üstü alkış / how far above quiet = clap
  int baseline = 2048;       // Sessizken ortalama ham değer / mean raw value in silence
  int quietLevel = 0;        // Sessiz odadaki en yüksek gürültü / loudest noise of the quiet room
  int threshold = 400;       // Alkış eşiği (seviye) / clap threshold (level)
  bool manualMode = false;   // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
  bool lampOn = false;
  bool inPeak = false;       // Şu an yüksek ses sürüyor mu? / is a loud sound going on now?
  uint32_t peakStartMs = 0;
  uint32_t firstClapMs = 0;  // 0 = bekleyen ilk alkış yok / 0 = no first clap waiting
  uint32_t lastToggleMs = 0;
  uint32_t lastScreenMs = 0;
  int shownPeak = 0;         // İki ekran yenilemesi arasındaki en yüksek seviye / max level between refreshes
  bool lastB3 = false;
  bool lastB1 = false;
  uint32_t lastButtonMs = 0;
}

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "AÇ" -> "ac"
// Lower-cases and simplifies Turkish letters: "AÇ" -> "ac"
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
// Ekran / Screen
// ---------------------------------------------------------------------------
// lcdWriteFixedTxt Türkçe harfleri (ç, ğ, ı, ö, ş, ü) LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

// ---------------------------------------------------------------------------
// Mikrofon / Microphone
// ---------------------------------------------------------------------------
// Mikrofon sesi, baseline etrafında salınan bir dalga olarak verir. 10 ms boyunca dalganın
// baseline'dan en çok ne kadar uzaklaştığına bakarız: bu, o anki "ses seviyesi"dir.
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
  iotbot.lcdWriteMid(L("ALKIŞ ANAHTARI", "CLAP SWITCH"), "", L("Sessiz olun...", "Please be quiet..."), L("Ölçülüyor", "Measuring"));
  // 1) Yarım saniye: sessizken ortalama ham değer (dalganın orta çizgisi).
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
  // 2) Yarım saniye: sessiz odadaki en yüksek gürültü seviyesi. Eşik = gürültü + pay.
  // 2) Half a second: the loudest noise level of the quiet room. Threshold = noise + margin.
  quietLevel = 0;
  start = millis();
  while (millis() - start < kCalibrationMs / 2) {
    int level = readLevel();
    if (level > quietLevel) quietLevel = level;
  }
  threshold = quietLevel + margin;

  char msg[80];
  snprintf(msg, sizeof(msg), L("Orta değer: %d  Sessizlik: %d  Eşik: %d", "Baseline: %d  Quiet: %d  Threshold: %d"),
           baseline, quietLevel, threshold);
  iotbot.serialWrite(msg);
  iotbot.lcdClear();
  lcdRow(0, L("   ALKIŞ ANAHTARI", "    CLAP SWITCH"));
}

void printHelp() {
  iotbot.serialWrite(L("---- ALKIŞ ANAHTARI - Komutlar ----", "---- CLAP SWITCH - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oto           : otomatik mod (alkış)", "  auto          : auto mode (claps)"));
  iotbot.serialWrite(L("  manuel        : manuel mod (B1 ile aç/kapat)", "  manual        : manual mode (switch with B1)"));
  iotbot.serialWrite(L("  ac / kapat    : lambayı aç / kapat", "  on / off      : lamp on / off"));
  iotbot.serialWrite(L("  kalibre       : sessizliği yeniden ölç", "  calibrate     : measure the quiet level again"));
  iotbot.serialWrite(L("  esik 50-2000  : alkış payı", "  threshold 50-2000: clap margin"));
  iotbot.serialWrite(L("  oku           : ses seviyesi ve eşik", "  read          : sound level and threshold"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : OTOMATİK <-> MANUEL", "  B3 button     : AUTO <-> MANUAL"));
}

void setLamp(bool on, const char *reason) {
  lampOn = on;
  iotbot.relayWrite(lampOn);
  iotbot.buzzerPlayTone(lampOn ? 1600 : 900, 60);
  lastToggleMs = millis(); // Röle tıkı ve bip alkış sanılmasın / relay click and beep are not claps
  iotbot.serialWrite(String(reason) + (lampOn ? L(": lamba AÇIK", ": lamp ON") : L(": lamba KAPALI", ": lamp OFF")));
}

void setMode(bool manual) {
  manualMode = manual;
  firstClapMs = 0;
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  lastToggleMs = millis();
  iotbot.serialWrite(manual ? L(">> MANUEL mod: alkışlar yok sayılır, lamba B1 (veya B2) ile.", ">> MANUAL mode: claps are ignored, lamp with B1 (or B2).")
                            : L(">> OTOMATİK mod: lambayı iki alkışla açıp kapatın.", ">> AUTO mode: switch the lamp with two claps."));
  lcdRow(0, L("   ALKIŞ ANAHTARI", "    CLAP SWITCH"));
  lastScreenMs = 0;
}

// Kısa bir ses patlaması bitince çağrılır. clapMs = alkışın başladığı an.
// Called when a short burst of sound ends. clapMs = when the clap started.
void onClap(uint32_t clapMs) {
  if (firstClapMs == 0) {
    firstClapMs = clapMs; // İlk alkış: ikincisini bekle / first clap: wait for the second one
    return;
  }
  uint32_t gap = clapMs - firstClapMs;
  if (gap < kMinGapMs) {
    return; // Aynı alkışın yankısı / an echo of the same clap
  }
  if (gap <= kMaxGapMs) {
    setLamp(!lampOn, L("Çift alkış", "Double clap")); // Çift alkış! / double clap!
    firstClapMs = 0;
  } else {
    firstClapMs = clapMs; // Çok geç geldi: bunu yeni "ilk alkış" say / too late: new first clap
  }
}

void drawScreen() {
  // Satır 1: seviye çubuğu. Eşik tam ortada (8. kutu) '|' ile gösterilir.
  // Row 1: level bar. The threshold is marked with '|' in the middle (cell 8).
  int cells = constrain(shownPeak * 16 / (threshold * 2), 0, 16);
  char line[41];
  memcpy(line, turkish ? "Ses " : "Mic ", 4);
  for (int i = 0; i < 16; i++) {
    line[4 + i] = (i < cells) ? '\xFF' : (i == 8 ? '|' : ' '); // 0xFF = LCD'de dolu kutu / full block
  }
  line[20] = '\0';
  lcdRow(1, line);

  snprintf(line, sizeof(line), L("Lamba: %-6s %s", "Lamp: %-4s  %s"), lampOn ? L("AÇIK", "ON") : L("KAPALI", "OFF"),
           manualMode ? L("MANUEL", "MANUAL") : L("OTO", "AUTO"));
  lcdRow(2, line);
  if (manualMode) lcdRow(3, L("B1:lamba  B3:oto", "B1:lamp  B3:auto"));
  else lcdRow(3, firstClapMs != 0 ? L("  Bir daha alkışla!", "   Clap once more!") : L("  2 kez alkışlayın", "    Clap twice"));
  shownPeak = 0;
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oto" || word == "otomatik" || word == "auto") {
    setMode(false);
  } else if (word == "manuel" || word == "manual") {
    setMode(true);
  } else if (word == "ac" || word == "on") {
    if (!manualMode) setMode(true);
    setLamp(true, L("Seri komut", "Serial command"));
  } else if (word == "kapat" || word == "off") {
    if (!manualMode) setMode(true);
    setLamp(false, L("Seri komut", "Serial command"));
  } else if (word == "kalibre" || word == "calibrate") {
    calibrate();
    lastToggleMs = millis();
    lastScreenMs = 0;
  } else if ((word == "esik" || word == "threshold") && hasValue) {
    margin = constrain(value, 50, 2000);
    threshold = quietLevel + margin;
    iotbot.serialWrite(String(L("Alkış payı: ", "Clap margin: ")) + margin + L("  -> eşik: ", "  -> threshold: ") + threshold);
  } else if (word == "oku" || word == "read") {
    char msg[80];
    snprintf(msg, sizeof(msg), L("Ses seviyesi: %d  Eşik: %d  Sessizlik: %d", "Sound level: %d  Threshold: %d  Quiet: %d"),
             readLevel(), threshold, quietLevel);
    iotbot.serialWrite(msg);
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    lcdRow(0, L("   ALKIŞ ANAHTARI", "    CLAP SWITCH"));
    lastScreenMs = 0;
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.relayWrite(false);
  delay(500); // Açılış sesi bitsin, mikrofon onu duymasın / let the startup beep fade first
  calibrate();
  iotbot.serialWrite(L("Alkış anahtarı hazır.", "Clap switch ready."));
  printHelp();
}

void loop() {
  int level = readLevel();
  uint32_t now = millis();
  if (level > shownPeak) shownPeak = level;

  // 1) B3 -> mod değiştir (sadece basıldığı an) / B3 -> toggle mode (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3 && now - lastButtonMs > 200) {
    lastButtonMs = now;
    setMode(!manualMode);
  }
  lastB3 = b3;

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  if (manualMode) {
    // MANUEL: B1 (veya B2) her basışta lambayı açar/kapatır, alkışlar yok sayılır.
    // MANUAL: B1 (or B2) switches the lamp on every press, claps are ignored.
    bool b1 = iotbot.button1Read() || iotbot.button2Read();
    if (b1 && !lastB1 && now - lastButtonMs > 200) {
      lastButtonMs = now;
      setLamp(!lampOn, L("B1 butonu", "B1 button"));
    }
    lastB1 = b1;
    inPeak = false;
  } else {
    // İkinci alkış zamanında gelmediyse ilk alkışı unut. / Forget the first clap if no second one came.
    if (firstClapMs != 0 && now - firstClapMs > kMaxGapMs) {
      firstClapMs = 0;
    }

    bool loud = level > threshold;
    if (loud && !inPeak) {
      inPeak = true; // Yüksek ses başladı / a loud sound started
      peakStartMs = now;
    } else if (!loud && inPeak) {
      inPeak = false; // Yüksek ses bitti: ne kadar sürdü? / the loud sound ended: how long was it?
      bool shortBurst = (now - peakStartMs) <= kMaxClapMs;
      if (!shortBurst) {
        firstClapMs = 0; // Konuşma/müzik gibi uzun ses: alkış değil / long sound (talk/music): not a clap
      } else if (now - lastToggleMs >= kCooldownMs) {
        onClap(peakStartMs);
      }
    }
  }

  if (now - lastScreenMs >= kScreenMs) {
    lastScreenMs = now;
    drawScreen();
  }
}
