// TR: GERCEK PROJE - Refleks Testi Oyunu. B3'e basip oyunu baslatin:
// trafik lambasi KIRMIZI yanar ve 1.5 - 4.5 saniye arasi RASTGELE bir sure
// bekler. Lamba YESIL olur olmaz B3'e basin! Kart, yesilden basisa kadar
// gecen sureyi milisaniye (ms) olarak olcer ve bir not verir (S, A, B, C).
// Kirmizida basarsaniz "ERKEN BASTIN!" der. En iyi skorunuz EEPROM'a
// kaydedilir, yani kart kapanip acilsa bile rekor kaybolmaz. Sonuc
// ekranindayken B3'u 2 saniye basili tutarsaniz rekor silinir.
// EN: A REAL PROJECT - Reflex Tester Game. Press B3 to start: the traffic
// light turns RED and waits a RANDOM time between 1.5 and 4.5 seconds. As
// soon as it turns GREEN, press B3! The board measures the time from green
// to your press in milliseconds (ms) and gives a grade (S, A, B, C). If you
// press during red it says "TOO EARLY!". Your best score is stored in
// EEPROM, so the record survives even when the board is switched off.
// Hold B3 for 2 seconds on the result screen to reset the record.
//
// Baglanti / Wiring: Trafik lambasi modulu sabit pinler kullanir
// (KIRMIZI=IO32, SARI=IO26, YESIL=IO25), P1-P5 soketinden secim yapmaniza
// gerek yok. B3 kart uzerindedir. / The traffic light module uses fixed
// pins (RED=IO32, YELLOW=IO26, GREEN=IO25), no socket choice needed. B3 is
// on the board.

#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

namespace {
  constexpr int kBestAddress = 0;             // Rekorun EEPROM adresi / EEPROM address of the record
  constexpr uint32_t kMinWaitMs = 1500;       // En kisa kirmizi bekleme / shortest red wait
  constexpr uint32_t kMaxWaitMs = 4500;       // En uzun kirmizi bekleme / longest red wait
  constexpr uint32_t kHumanLimitMs = 100;     // Bundan hizlisi tahmindir / faster than this = a guess
  constexpr uint32_t kTooSlowMs = 3000;       // Bu surede basilmazsa / no press within this time
  constexpr uint32_t kResetHoldMs = 2000;     // Rekor silmek icin basili tutma / hold time to reset
  constexpr uint32_t kDebounceMs = 30;        // Buton parazit suresi / button bounce time

  enum State { IDLE, WAIT_RED, GO_GREEN };
  State state = IDLE;
  uint32_t stateStartMs = 0;
  uint32_t waitMs = 0;          // Bu turun rastgele kirmizi suresi / this round's random red time
  int32_t bestMs = 0;           // 0 = henuz rekor yok / 0 = no record yet
  bool lastDown = false;
  uint32_t lastEdgeMs = 0;
  bool pressActive = false;     // IDLE'da basili tutma olcumu / hold measurement in IDLE
  bool resetDone = false;
  uint32_t pressStartMs = 0;
  uint32_t buzzerOffMs = 0;     // Buzzer'in susacagi an (0 = calmiyor) / when to stop the buzzer
}

// Beklemeden (non-blocking) ses: buzzer'i baslat, loop() zamani gelince sustursun.
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
  if (bestMs > 0) snprintf(line, size, turkish ? "Rekor: %ld ms" : "Record: %ld ms", (long)bestMs);
  else snprintf(line, size, turkish ? "Rekor: yok" : "Record: none");
}

void showStartScreen() {
  char rec[21];
  recordLine(rec, sizeof(rec));
  iotbot.lcdWriteMid(turkish ? "REFLEKS TESTI" : "REFLEX TESTER", rec,
                     turkish ? "B3: Basla" : "B3: Start",
                     turkish ? "B3 2sn: rekoru sil" : "Hold B3 2s: reset");
}

// Tur bitti: sonucu goster, IDLE'a don. ms = 0 ise gecersiz tur.
// Round over: show the result, go back to IDLE. ms = 0 means invalid round.
void finishRound(const char *title, uint32_t ms) {
  state = IDLE;
  iotbot.moduleTraficLightWrite(false, false, false);
  char line1[21], line2[21], line3[21];
  if (ms == 0) {
    snprintf(line1, sizeof(line1), "%s", turkish ? "Tekrar dene!" : "Try again!");
    recordLine(line2, sizeof(line2));
    beep(200, 400);  // Kalin "bzzt" sesi / low "bzzt" sound
  } else {
    snprintf(line1, sizeof(line1), turkish ? "%lu ms   Not: %c" : "%lu ms   Grade: %c",
             (unsigned long)ms, gradeFor(ms));
    if (bestMs == 0 || (int32_t)ms < bestMs) {
      bestMs = ms;
      // eepromWriteInt32 degeri yazar ve kendi icinde commit eder (kalici olur).
      // eepromWriteInt32 writes the value and commits it internally (persistent).
      iotbot.eepromWriteInt32(kBestAddress, bestMs);
      snprintf(line2, sizeof(line2), "%s", turkish ? "*** YENI REKOR! ***" : "*** NEW RECORD! ***");
      beep(2000, 300);
    } else {
      recordLine(line2, sizeof(line2));
      beep(1400, 80);
    }
  }
  snprintf(line3, sizeof(line3), "%s", turkish ? "B3:tekrar  2sn:sil" : "B3:again  hold:reset");
  iotbot.lcdWriteMid(title, line1, line2, line3);
  iotbot.serialWrite(ms > 0 ? String(title) + ": " + String(ms) + " ms" : String(title));
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.eepromBegin(64);
  bestMs = iotbot.eepromReadInt32(kBestAddress, 0);
  // Bos (hic yazilmamis) EEPROM 0xFFFFFFFF = -1 okur; mantiksiz degerleri "rekor yok" say.
  // Blank EEPROM reads 0xFFFFFFFF = -1; treat nonsense values as "no record".
  if (bestMs < (int32_t)kHumanLimitMs || bestMs > 60000) bestMs = 0;
  iotbot.moduleTraficLightWrite(false, false, false);
  showStartScreen();
}

void loop() {
  uint32_t now = millis();

  // Buton okuma + parazit (bounce) onleme: degisimi HEMEN kabul et (olcum
  // gecikmesin), sonra 30 ms boyunca yeni degisimleri yok say.
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

  switch (state) {
    case IDLE:
      // Kisa bas-birak = yeni tur, 2 sn basili tut = rekoru sil.
      // Short press-release = new round, hold 2 s = reset the record.
      if (pressed) { pressActive = true; resetDone = false; pressStartMs = now; }
      if (pressActive && down && !resetDone && now - pressStartMs >= kResetHoldMs) {
        resetDone = true;
        bestMs = 0;
        iotbot.eepromWriteInt32(kBestAddress, 0);
        beep(600, 300);
        showStartScreen();
        iotbot.serialWrite(turkish ? "Rekor silindi." : "Record reset.");
      }
      if (pressActive && released) {
        pressActive = false;
        if (!resetDone) {
          state = WAIT_RED;
          stateStartMs = now;
          waitMs = random((long)kMinWaitMs, (long)kMaxWaitMs + 1);  // ESP32: donanim rastgele sayi / hardware RNG
          iotbot.moduleTraficLightWrite(true, false, false);
          iotbot.lcdWriteMid(turkish ? "HAZIR OL..." : "GET READY...",
                             turkish ? "KIRMIZI: bekle" : "RED: wait",
                             turkish ? "YESIL yaninca" : "When it turns GREEN",
                             turkish ? "hemen B3'e bas!" : "press B3 at once!");
        }
      }
      break;

    case WAIT_RED:
      if (pressed) {
        finishRound(turkish ? "ERKEN BASTIN!" : "TOO EARLY!", 0);
      } else if (now - stateStartMs >= waitMs) {
        state = GO_GREEN;
        iotbot.moduleTraficLightWrite(false, false, true);
        stateStartMs = millis();  // Kronometre simdi basliyor / the stopwatch starts now
        // LCD yazmak ~20 ms surer; insan 100 ms'den hizli basamaz, olcum bozulmaz.
        // Writing the LCD takes ~20 ms; no human reacts under 100 ms, so timing stays fair.
        iotbot.lcdWriteMid("", turkish ? ">>> BAS! <<<" : ">>> PRESS! <<<", "", "");
      }
      break;

    case GO_GREEN: {
      uint32_t reaction = now - stateStartMs;
      if (pressed) {
        if (reaction < kHumanLimitMs) finishRound(turkish ? "TAHMIN ETTIN!" : "YOU GUESSED!", 0);
        else finishRound(turkish ? "SONUC" : "RESULT", reaction);
      } else if (reaction >= kTooSlowMs) {
        finishRound(turkish ? "COK YAVAS!" : "TOO SLOW!", 0);
      }
      break;
    }
  }
  delay(1);  // Kisa bekleme: olcum hassas kalsin / tiny delay keeps timing precise
}
