/*
 * TR: GERÇEK PROJE - Refleks Testi Oyunu. B3'e basıp oyunu başlatın: trafik
 * lambası KIRMIZI yanar ve 1.5 - 4.5 saniye arası RASTGELE bir süre bekler.
 * Lamba YEŞİL olur olmaz B3'e basın! Kart, yeşilden basışa kadar geçen süreyi
 * milisaniye (ms) olarak ölçer ve bir not verir (S, A, B, C). Kırmızıda
 * basarsanız "ERKEN BASTIN!" der. En iyi skorunuz EEPROM'a kaydedilir, yani
 * kart kapanıp açılsa bile rekor kaybolmaz. Sonuç ekranındayken B3'ü 2 saniye
 * basılı tutarsanız rekor silinir.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help     -> komut listesi
 *      basla   / start    -> yeni tur başlat (B3'e kısa basmak gibi)
 *      rekor   / record   -> rekoru yaz
 *      sil     / reset    -> rekoru sil
 *      dil     / lang     -> dili değiştir (Türkçe <-> English)
 *    (Tepki süresi her zaman B3 ile ölçülür: seri port bunun için çok yavaştır.)
 *
 * EN: A REAL PROJECT - Reflex Tester Game. Press B3 to start: the traffic
 * light turns RED and waits a RANDOM time between 1.5 and 4.5 seconds. As
 * soon as it turns GREEN, press B3! The board measures the time from green to
 * your press in milliseconds (ms) and gives a grade (S, A, B, C). If you
 * press during red it says "TOO EARLY!". Your best score is stored in EEPROM,
 * so the record survives even when the board is switched off. Hold B3 for 2
 * seconds on the result screen to reset the record.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help    / yardim   -> command list
 *      start   / basla    -> start a new round (like a short B3 press)
 *      record  / rekor    -> print the record
 *      reset   / sil      -> reset the record
 *      lang    / dil      -> switch language (Turkish <-> English)
 *    (The reaction is always measured with B3: the serial port is too slow for it.)
 *
 * Bağlantı / Wiring: Trafik lambası modülü sabit pinler kullanır
 * (KIRMIZI=IO32, SARI=IO26, YEŞİL=IO25), P1-P5 soketinden seçim yapmanıza
 * gerek yok. B3 kart üzerindedir. / The traffic light module uses fixed pins
 * (RED=IO32, YELLOW=IO26, GREEN=IO25), no socket choice needed. B3 is on the
 * board.
 */

#include <IOTBOT.h>

IOTBOT iotbot;

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

namespace {
  constexpr int kBestAddress = 0;             // Rekorun EEPROM adresi / EEPROM address of the record
  constexpr uint32_t kMinWaitMs = 1500;       // En kısa kırmızı bekleme / shortest red wait
  constexpr uint32_t kMaxWaitMs = 4500;       // En uzun kırmızı bekleme / longest red wait
  constexpr uint32_t kHumanLimitMs = 100;     // Bundan hızlısı tahmindir / faster than this = a guess
  constexpr uint32_t kTooSlowMs = 3000;       // Bu sürede basılmazsa / no press within this time
  constexpr uint32_t kResetHoldMs = 2000;     // Rekor silmek için basılı tutma / hold time to reset
  constexpr uint32_t kDebounceMs = 30;        // Buton parazit süresi / button bounce time

  enum State { IDLE, WAIT_RED, GO_GREEN };
  State state = IDLE;
  uint32_t stateStartMs = 0;
  uint32_t waitMs = 0;          // Bu turun rastgele kırmızı süresi / this round's random red time
  int32_t bestMs = 0;           // 0 = henüz rekor yok / 0 = no record yet
  bool lastDown = false;
  uint32_t lastEdgeMs = 0;
  bool pressActive = false;     // IDLE'da basılı tutma ölçümü / hold measurement in IDLE
  bool resetDone = false;
  uint32_t pressStartMs = 0;
  uint32_t buzzerOffMs = 0;     // Buzzer'ın susacağı an (0 = çalmıyor) / when to stop the buzzer
}

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "BAŞLA" -> "basla"
// Lower-cases and simplifies Turkish letters: "BAŞLA" -> "basla"
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
// Ses ve ekran / Sound and screen
// ---------------------------------------------------------------------------
// Beklemeden (non-blocking) ses: buzzer'ı başlat, loop() zamanı gelince sustursun.
// Non-blocking sound: start the buzzer, loop() stops it when the time comes.
void beep(int freq, uint32_t ms) {
  iotbot.buzzerStart(freq);
  buzzerOffMs = millis() + ms;
}

char gradeFor(uint32_t ms) {
  if (ms < 200) return 'S';
  if (ms < 300) return 'A';
  if (ms < 450) return 'B';
  return 'C';
}

void recordLine(char *line, size_t size) {
  if (bestMs > 0) snprintf(line, size, L("Rekor: %ld ms", "Record: %ld ms"), (long)bestMs);
  else snprintf(line, size, "%s", L("Rekor: yok", "Record: none"));
}

void printHelp() {
  iotbot.serialWrite(L("---- REFLEKS TESTİ - Komutlar ----", "---- REFLEX TESTER - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  basla         : yeni tur", "  start         : new round"));
  iotbot.serialWrite(L("  rekor         : rekoru yaz", "  record        : print the record"));
  iotbot.serialWrite(L("  sil           : rekoru sil", "  reset         : reset the record"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : başla / tepki / 2 sn basılı: rekoru sil", "  B3 button     : start / react / hold 2 s: reset record"));
}

void showStartScreen() {
  char rec[41];
  recordLine(rec, sizeof(rec));
  iotbot.lcdWriteMid(L("REFLEKS TESTİ", "REFLEX TESTER"), rec, L("B3: Başla", "B3: Start"), L("B3 2sn: rekoru sil", "Hold B3 2s: reset"));
}

void showReadyScreen() {
  iotbot.lcdWriteMid(L("HAZIR OL...", "GET READY..."), L("KIRMIZI: bekle", "RED: wait"), L("YEŞİL yanınca", "When it turns GREEN"),
                     L("hemen B3'e bas!", "press B3 at once!"));
}

void resetRecord() {
  bestMs = 0;
  iotbot.eepromWriteInt32(kBestAddress, 0);
  beep(600, 300);
  if (state == IDLE) showStartScreen();
  iotbot.serialWrite(L("Rekor silindi.", "Record reset."));
}

void startRound() {
  state = WAIT_RED;
  stateStartMs = millis();
  waitMs = random((long)kMinWaitMs, (long)kMaxWaitMs + 1);  // ESP32: donanım rastgele sayı / hardware RNG
  iotbot.moduleTraficLightWrite(true, false, false);
  showReadyScreen();
  iotbot.serialWrite(L("Tur başladı: YEŞİL yanınca B3'e bas!", "Round started: press B3 when it turns GREEN!"));
}

// Tur bitti: sonucu göster, IDLE'a dön. ms = 0 ise geçersiz tur.
// Round over: show the result, go back to IDLE. ms = 0 means invalid round.
void finishRound(const char *title, uint32_t ms) {
  state = IDLE;
  iotbot.moduleTraficLightWrite(false, false, false);
  char line1[41], line2[41], line3[41];
  if (ms == 0) {
    snprintf(line1, sizeof(line1), "%s", L("Tekrar dene!", "Try again!"));
    recordLine(line2, sizeof(line2));
    beep(200, 400);  // Kalın "bzzt" sesi / low "bzzt" sound
  } else {
    snprintf(line1, sizeof(line1), L("%lu ms   Not: %c", "%lu ms   Grade: %c"), (unsigned long)ms, gradeFor(ms));
    if (bestMs == 0 || (int32_t)ms < bestMs) {
      bestMs = ms;
      // eepromWriteInt32 değeri yazar ve kendi içinde commit eder (kalıcı olur).
      // eepromWriteInt32 writes the value and commits it internally (persistent).
      iotbot.eepromWriteInt32(kBestAddress, bestMs);
      snprintf(line2, sizeof(line2), "%s", L("*** YENİ REKOR! ***", "*** NEW RECORD! ***"));
      beep(2000, 300);
    } else {
      recordLine(line2, sizeof(line2));
      beep(1400, 80);
    }
  }
  snprintf(line3, sizeof(line3), "%s", L("B3:tekrar  2sn:sil", "B3:again  hold:reset"));
  iotbot.lcdWriteMid(title, line1, line2, line3);
  if (ms > 0) {
    char msg[64];
    snprintf(msg, sizeof(msg), L("%s: %lu ms, not %c", "%s: %lu ms, grade %c"), title, (unsigned long)ms, gradeFor(ms));
    iotbot.serialWrite(msg);
  } else {
    iotbot.serialWrite(title);
  }
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "basla" || cmd == "start") {
    if (state == IDLE) startRound();
    else iotbot.serialWrite(L("Tur zaten sürüyor.", "A round is already running."));
  } else if (cmd == "rekor" || cmd == "record") {
    char rec[41];
    recordLine(rec, sizeof(rec));
    iotbot.serialWrite(rec);
  } else if (cmd == "sil" || cmd == "reset") {
    resetRecord();
  } else if (cmd == "dil" || cmd == "lang" || cmd == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    if (state == IDLE) showStartScreen();
    else if (state == WAIT_RED) showReadyScreen();
    // GO_GREEN: ölçüm sürerken ekrana dokunmuyoruz / GO_GREEN: we don't touch the screen while timing
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.eepromBegin(64);
  bestMs = iotbot.eepromReadInt32(kBestAddress, 0);
  // Boş (hiç yazılmamış) EEPROM 0xFFFFFFFF = -1 okur; mantıksız değerleri "rekor yok" say.
  // Blank EEPROM reads 0xFFFFFFFF = -1; treat nonsense values as "no record".
  if (bestMs < (int32_t)kHumanLimitMs || bestMs > 60000) bestMs = 0;
  iotbot.moduleTraficLightWrite(false, false, false);
  showStartScreen();
  iotbot.serialWrite(L("Refleks testi hazır.", "Reflex tester ready."));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // Buton okuma + parazit (bounce) önleme: değişimi HEMEN kabul et (ölçüm
  // gecikmesin), sonra 30 ms boyunca yeni değişimleri yok say.
  // Button read + debounce: accept a change AT ONCE (no timing delay), then
  // ignore further changes for 30 ms.
  bool pressed = false, released = false;
  bool raw = iotbot.button3Read();
  if (raw != lastDown && now - lastEdgeMs >= kDebounceMs) {
    lastDown = raw;
    lastEdgeMs = now;
    pressed = raw;
    released = !raw;
  }
  bool down = lastDown;

  if (buzzerOffMs != 0 && (int32_t)(now - buzzerOffMs) >= 0) {
    iotbot.buzzerStop();
    buzzerOffMs = 0;
  }

  // Seri komutlar (yeşil yanarken okunmaz: ölçüm hassas kalsın).
  // Serial commands (not read while green: keeps the timing precise).
  if (state != GO_GREEN) {
    String cmd;
    if (readCommand(cmd)) handleCommand(cmd);
  }

  switch (state) {
    case IDLE:
      // Kısa bas-bırak = yeni tur, 2 sn basılı tut = rekoru sil.
      // Short press-release = new round, hold 2 s = reset the record.
      if (pressed) { pressActive = true; resetDone = false; pressStartMs = now; }
      if (pressActive && down && !resetDone && now - pressStartMs >= kResetHoldMs) {
        resetDone = true;
        resetRecord();
      }
      if (pressActive && released) {
        pressActive = false;
        if (!resetDone) startRound();
      }
      break;

    case WAIT_RED:
      if (pressed) {
        finishRound(L("ERKEN BASTIN!", "TOO EARLY!"), 0);
      } else if (now - stateStartMs >= waitMs) {
        state = GO_GREEN;
        iotbot.moduleTraficLightWrite(false, false, true);
        stateStartMs = millis();  // Kronometre şimdi başlıyor / the stopwatch starts now
        // LCD yazmak ~20 ms sürer; insan 100 ms'den hızlı basamaz, ölçüm bozulmaz.
        // Writing the LCD takes ~20 ms; no human reacts under 100 ms, so timing stays fair.
        iotbot.lcdWriteMid("", L(">>> BAS! <<<", ">>> PRESS! <<<"), "", "");
      }
      break;

    case GO_GREEN: {
      uint32_t reaction = now - stateStartMs;
      if (pressed) {
        if (reaction < kHumanLimitMs) finishRound(L("TAHMİN ETTİN!", "YOU GUESSED!"), 0);
        else finishRound(L("SONUÇ", "RESULT"), reaction);
      } else if (reaction >= kTooSlowMs) {
        finishRound(L("ÇOK YAVAŞ!", "TOO SLOW!"), 0);
      }
      break;
    }
  }
  delay(1);  // Kısa bekleme: ölçüm hassas kalsın / tiny delay keeps timing precise
}
