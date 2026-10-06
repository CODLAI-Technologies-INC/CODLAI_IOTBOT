// TR: GERCEK PROJE - Yaya Gecidi. Arabalar icin trafik lambasi normalde
// YESIL yanar. Bir yaya B3 (ya da B1) butonuna basinca LCD "Istek alindi"
// yazar; yaklasik 1 saniye sonra lamba SARI, ardindan KIRMIZI olur ve
// yayalar 6 saniye boyunca gecer. Bu sirada LCD'de BUYUK bir geri sayim
// gorunur, buzzer once yavas, son 2 saniyede hizli "tik" sesi cikarir -
// tipki gorme engelliler icin sesli yaya gecitleri gibi. Sonra lamba tekrar
// yesile doner ve arabalara en az 5 saniye yesil verilir; bu surede gelen
// istekler unutulmaz, sure dolunca sirayla karsilanir.
// EN: A REAL PROJECT - Pedestrian Crossing. The car traffic light is
// normally GREEN. When a pedestrian presses B3 (or B1) the LCD shows
// "Request received"; about 1 second later the light turns YELLOW, then
// RED, and pedestrians cross for 6 seconds. Meanwhile the LCD shows a BIG
// countdown and the buzzer ticks slowly, then fast in the last 2 seconds -
// just like accessible crossings for visually impaired people. Then the
// light goes back to green and cars get at least 5 seconds of green; a
// request made during that time is remembered and served afterwards.
//
// Baglanti / Wiring: Trafik lambasi modulu sabit pinler kullanir
// (KIRMIZI=IO32, SARI=IO26, YESIL=IO25), P1-P5 soketinden secim yapmaniza
// gerek yok. B1 ve B3 kart uzerindedir. / The traffic light module uses
// fixed pins (RED=IO32, YELLOW=IO26, GREEN=IO25), no socket choice needed.
// B1 and B3 are on the board.

#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

namespace {
  constexpr uint32_t kRequestDelayMs = 1000;  // Istekten sonra sariya gecis / delay before yellow
  constexpr uint32_t kYellowMs = 2000;        // Sari lamba suresi / yellow time
  constexpr uint32_t kWalkMs = 6000;          // Yayalarin gecis suresi / pedestrian walk time
  constexpr uint32_t kHurryMs = 2000;         // Son 2 sn hizli tik / last 2 s: fast ticks
  constexpr uint32_t kMinGreenMs = 5000;      // Arabalara en az yesil / minimum car green
  constexpr uint32_t kSlowTickGapMs = 1000;   // Yavas tik araligi / slow tick gap
  constexpr uint32_t kFastTickGapMs = 250;    // Hizli tik araligi / fast tick gap

  enum State { CAR_GREEN, CAR_YELLOW, WALK };
  State state = CAR_GREEN;
  uint32_t stateStartMs = 0;
  bool requestPending = false;   // Bekleyen yaya istegi var mi? / is a request waiting?
  uint32_t requestMs = 0;
  uint32_t lastTickMs = 0;
  int shownSecond = -1;          // LCD'de su an yazan saniye / second currently on the LCD
  bool buttonWasDown = false;

  // 3 satirlik "7 segment" rakamlar: LCD'de buyuk sayi cizmek icin.
  // 3-row "7-segment" digits: used to draw a big number on the LCD.
  const char *const kBigDigit[3][10] = {
    {" _ ", "   ", " _ ", " _ ", "   ", " _ ", " _ ", " _ ", " _ ", " _ "},
    {"| |", "  |", " _|", " _|", "|_|", "|_ ", "|_ ", "  |", "|_|", "|_|"},
    {"|_|", "  |", "|_ ", " _|", "  |", " _|", "|_|", "  |", "|_|", " _|"},
  };
}

// Metni 20 karakterlik satirin ortasina yazar (ekrani silmeden).
// Writes text centered on a 20-char row (without clearing the screen).
void writeCentered(int row, const char *text) {
  char line[21];
  int len = strlen(text);
  if (len > 20) len = 20;
  memset(line, ' ', 20);
  memcpy(line + (20 - len) / 2, text, len);
  line[20] = '\0';
  iotbot.lcdWriteFixed(row, line);
}

void showGreenScreen() {
  bool waiting = requestPending && (millis() - stateStartMs < kMinGreenMs);
  iotbot.lcdWriteMid(turkish ? "YAYA GECIDI" : "PEDESTRIAN CROSSING",
                     turkish ? "Arabalar: YESIL" : "Cars: GREEN",
                     requestPending ? (turkish ? "Istek alindi" : "Request received")
                                    : (turkish ? "Gecmek icin B3'e bas" : "Press B3 to cross"),
                     waiting ? (turkish ? "Lutfen bekleyin..." : "Please wait...") : "");
}

// Buyuk geri sayimi LCD'nin 2-4. satirlarina cizer.
// Draws the big countdown on LCD rows 2-4.
void drawCountdown(int sec, bool hurry) {
  for (int r = 0; r < 3; r++) {
    char line[21];
    memset(line, ' ', 20);
    line[20] = '\0';
    memcpy(line + 9, kBigDigit[r][sec], 3);
    if (r == 1) {
      const char *left = turkish ? "Kalan" : "Left";
      const char *right = turkish ? "saniye" : "sec";
      memcpy(line + 1, left, strlen(left));
      memcpy(line + 14, right, strlen(right));
    }
    if (r == 2 && hurry) {
      const char *msg = turkish ? "ACELE!" : "HURRY!";
      memcpy(line + 1, msg, strlen(msg));
    }
    iotbot.lcdWriteFixed(r + 1, line);
  }
}

void enterState(State s) {
  state = s;
  stateStartMs = millis();
  if (s == CAR_GREEN) {
    requestPending = false;  // Istek karsilandi / request has been served
    iotbot.moduleTraficLightWrite(false, false, true);
    showGreenScreen();
    iotbot.serialWrite(turkish ? "Arabalar: YESIL" : "Cars: GREEN");
  } else if (s == CAR_YELLOW) {
    iotbot.moduleTraficLightWrite(false, true, false);
    iotbot.lcdWriteMid(turkish ? "YAYA GECIDI" : "PEDESTRIAN CROSSING",
                       turkish ? "Arabalar: SARI" : "Cars: YELLOW",
                       turkish ? "Arabalar yavasliyor" : "Cars slowing down",
                       turkish ? "Hazir olun..." : "Get ready...");
    iotbot.serialWrite(turkish ? "Arabalar: SARI" : "Cars: YELLOW");
  } else {
    iotbot.moduleTraficLightWrite(true, false, false);
    iotbot.lcdClear();
    writeCentered(0, turkish ? "YAYALAR GECEBILIR" : "PEDESTRIANS: WALK");
    lastTickMs = 0;     // Ilk tik hemen calsin / first tick plays immediately
    shownSecond = -1;   // Rakam yeniden cizilsin / force redraw of the digit
    iotbot.serialWrite(turkish ? "Arabalar: KIRMIZI - yayalar geciyor" : "Cars: RED - pedestrians walking");
  }
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  enterState(CAR_GREEN);
}

void loop() {
  uint32_t now = millis();

  // Buton: sadece "basildigi an" sayilir (basili tutmak tekrar istek yapmaz).
  // Button: only the moment of pressing counts (holding does not repeat).
  bool down = iotbot.button3Read() || iotbot.button1Read();
  bool pressed = down && !buttonWasDown;
  buttonWasDown = down;

  switch (state) {
    case CAR_GREEN:
      if (pressed && !requestPending) {
        requestPending = true;
        requestMs = now;
        iotbot.buzzerPlayTone(1200, 40);
        showGreenScreen();
        iotbot.serialWrite(turkish ? "Yaya istegi alindi." : "Pedestrian request received.");
      }
      // Hem 1 sn gecmeli hem de arabalar en az 5 sn yesil gormus olmali.
      // At least 1 s must pass AND cars must have had 5 s of green.
      if (requestPending && now - requestMs >= kRequestDelayMs &&
          now - stateStartMs >= kMinGreenMs) {
        enterState(CAR_YELLOW);
      }
      break;

    case CAR_YELLOW:
      if (now - stateStartMs >= kYellowMs) enterState(WALK);
      break;

    case WALK: {
      uint32_t elapsed = now - stateStartMs;
      if (elapsed >= kWalkMs) {
        enterState(CAR_GREEN);
        break;
      }
      uint32_t remaining = kWalkMs - elapsed;
      bool hurry = remaining <= kHurryMs;
      int sec = (remaining + 999) / 1000;  // 6, 5, ... 1 (yukari yuvarla / round up)
      if (sec != shownSecond) {
        shownSecond = sec;
        drawCountdown(sec, hurry);
      }
      // Sesli yaya gecidi: yavas tik, son 2 sn'de hizli ve ince tik.
      // Accessible crossing: slow ticks, fast high ticks in the last 2 s.
      if (now - lastTickMs >= (hurry ? kFastTickGapMs : kSlowTickGapMs)) {
        lastTickMs = now;
        iotbot.buzzerPlayTone(hurry ? 1600 : 900, 30);
      }
      break;
    }
  }
  delay(10);
}
