/*
 * TR: GERÇEK PROJE - Yaya Geçidi.
 *  - OTOMATİK mod (açılışta): Arabalar için trafik lambası normalde YEŞİL yanar.
 *    Bir yaya B1 (veya B2) butonuna basınca LCD "İstek alındı" yazar; yaklaşık
 *    1 saniye sonra lamba SARI, ardından KIRMIZI olur ve yayalar 6 saniye
 *    boyunca geçer. Bu sırada LCD'de BÜYÜK bir geri sayım görünür, buzzer önce
 *    yavaş, son 2 saniyede hızlı "tik" sesi çıkarır - tıpkı görme engelliler
 *    için sesli yaya geçitleri gibi. Sonra lamba tekrar yeşile döner ve
 *    arabalara en az 5 saniye yeşil verilir; bu sürede gelen istekler
 *    unutulmaz, süre dolunca sırayla karşılanır.
 *  - B3 butonu MANUEL moda geçer (trafik polisi gibi): lambayı potansiyometre
 *    ile siz seçersiniz (sol = KIRMIZI, orta = SARI, sağ = YEŞİL). B3'e tekrar
 *    basınca otomatik moda döner.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim   / help      -> komut listesi
 *      oto      / auto      -> otomatik mod
 *      manuel   / manual    -> manuel mod (potansiyometre)
 *      istek    / request   -> yaya butonuna basmak gibi (otomatik modda)
 *      kirmizi  / red       -> kırmızı yak (manuel moda geçer)
 *      sari     / yellow    -> sarı yak (manuel moda geçer)
 *      yesil    / green     -> yeşil yak (manuel moda geçer)
 *      kapat    / off       -> tüm lambaları söndür (manuel moda geçer)
 *      dil      / lang      -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Pedestrian Crossing.
 *  - AUTO mode (at startup): The car traffic light is normally GREEN. When a
 *    pedestrian presses B1 (or B2) the LCD shows "Request received"; about 1
 *    second later the light turns YELLOW, then RED, and pedestrians cross for
 *    6 seconds. Meanwhile the LCD shows a BIG countdown and the buzzer ticks
 *    slowly, then fast in the last 2 seconds - just like accessible crossings
 *    for visually impaired people. Then the light goes back to green and cars
 *    get at least 5 seconds of green; a request made during that time is
 *    remembered and served afterwards.
 *  - Button B3 switches to MANUAL mode (like a traffic officer): you pick the
 *    light with the potentiometer (left = RED, middle = YELLOW, right = GREEN).
 *    Press B3 again to go back to auto mode.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help     / yardim    -> command list
 *      auto     / oto       -> auto mode
 *      manual   / manuel    -> manual mode (potentiometer)
 *      request  / istek     -> like pressing the pedestrian button (auto mode)
 *      red      / kirmizi   -> red on (switches to manual)
 *      yellow   / sari      -> yellow on (switches to manual)
 *      green    / yesil     -> green on (switches to manual)
 *      off      / kapat     -> all lights off (switches to manual)
 *      lang     / dil       -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Trafik lambası modülü sabit pinler kullanır
 * (KIRMIZI=IO32, SARI=IO26, YEŞİL=IO25), P1-P5 soketinden seçim yapmanıza
 * gerek yok. B1, B3 ve potansiyometre kart üzerindedir. / The traffic light
 * module uses fixed pins (RED=IO32, YELLOW=IO26, GREEN=IO25), no socket
 * choice needed. B1, B3 and the potentiometer are on the board.
 */

#include <IOTBOT.h>

IOTBOT iotbot;

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

namespace {
  constexpr uint32_t kRequestDelayMs = 1000;  // İstekten sonra sarıya geçiş / delay before yellow
  constexpr uint32_t kYellowMs = 2000;        // Sarı lamba süresi / yellow time
  constexpr uint32_t kWalkMs = 6000;          // Yayaların geçiş süresi / pedestrian walk time
  constexpr uint32_t kHurryMs = 2000;         // Son 2 sn hızlı tik / last 2 s: fast ticks
  constexpr uint32_t kMinGreenMs = 5000;      // Arabalara en az yeşil / minimum car green
  constexpr uint32_t kSlowTickGapMs = 1000;   // Yavaş tik aralığı / slow tick gap
  constexpr uint32_t kFastTickGapMs = 250;    // Hızlı tik aralığı / fast tick gap

  enum State { CAR_GREEN, CAR_YELLOW, WALK, MANUAL };
  enum Light { LIGHT_OFF, LIGHT_RED, LIGHT_YELLOW, LIGHT_GREEN };
  State state = CAR_GREEN;
  uint32_t stateStartMs = 0;
  bool requestPending = false;   // Bekleyen yaya isteği var mı? / is a request waiting?
  uint32_t requestMs = 0;
  uint32_t lastTickMs = 0;
  int shownSecond = -1;          // LCD'de şu an yazan saniye / second currently on the LCD
  bool buttonWasDown = false;
  bool lastB3 = false;
  uint32_t lastB3Ms = 0;
  Light manualLight = LIGHT_GREEN;
  int lastPotZone = -1;          // Potun son bölgesi (0-2) / last pot zone (0-2)

  // 3 satırlık "7 segment" rakamlar: LCD'de büyük sayı çizmek için.
  // 3-row "7-segment" digits: used to draw a big number on the LCD.
  const char *const kBigDigit[3][10] = {
    {" _ ", "   ", " _ ", " _ ", "   ", " _ ", " _ ", " _ ", " _ ", " _ "},
    {"| |", "  |", " _|", " _|", "|_|", "|_ ", "|_ ", "  |", "|_|", "|_|"},
    {"|_|", "  |", "|_ ", " _|", "  |", " _|", "|_|", "  |", "|_|", " _|"},
  };
}

// Enum parametreli fonksiyonlarin prototipleri: Arduino IDE otomatik
// prototipleri enum tanimindan ONCE yazdigi icin "declared void" hatasi
// veriyordu. / Prototypes of functions taking an enum: the Arduino IDE
// writes its auto-prototypes BEFORE the enum ("declared void" error).
const char * lightName(Light l);
void writeLight(Light l);
void enterState(State s);
void setManualLight(Light l);

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "KIRMIZI" -> "kirmizi"
// Lower-cases and simplifies Turkish letters: "KIRMIZI" -> "kirmizi"
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

// Metni 20 karakterlik satırın ortasına yazar (ekranı silmeden). Türkçe harfler 2 bayt
// olduğu için baytları değil, görünen harfleri sayarız.
// Writes text centered on a 20-char row (without clearing the screen). Turkish letters are
// 2 bytes, so we count visible letters, not bytes.
void writeCentered(int row, const char *text) {
  int visible = 0;
  for (const char *p = text; *p; p++) {
    if ((*p & 0xC0) != 0x80) visible++; // UTF-8 devam baytlarını sayma / skip UTF-8 continuation bytes
  }
  int pad = visible < 20 ? (20 - visible) / 2 : 0;
  char line[48];
  snprintf(line, sizeof(line), "%*s%s", pad, "", text);
  lcdRow(row, line);
}

void printHelp() {
  iotbot.serialWrite(L("---- YAYA GEÇİDİ - Komutlar ----", "---- PEDESTRIAN CROSSING - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oto           : otomatik mod", "  auto          : auto mode"));
  iotbot.serialWrite(L("  manuel        : manuel mod (potansiyometre)", "  manual        : manual mode (potentiometer)"));
  iotbot.serialWrite(L("  istek         : yaya butonu (otomatik mod)", "  request       : pedestrian button (auto mode)"));
  iotbot.serialWrite(L("  kirmizi / sari / yesil : o lambayı yak", "  red / yellow / green   : turn that light on"));
  iotbot.serialWrite(L("  kapat         : lambaları söndür", "  off           : all lights off"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B1 butonu     : yaya butonu", "  B1 button     : pedestrian button"));
  iotbot.serialWrite(L("  B3 butonu     : OTOMATİK <-> MANUEL", "  B3 button     : AUTO <-> MANUAL"));
}

const char *lightName(Light l) {
  switch (l) {
    case LIGHT_RED:    return L("KIRMIZI", "RED");
    case LIGHT_YELLOW: return L("SARI", "YELLOW");
    case LIGHT_GREEN:  return L("YEŞİL", "GREEN");
    default:           return L("KAPALI", "OFF");
  }
}

// Pot değerini 3 bölgeye ayırır (0 = sol, 1 = orta, 2 = sağ). Sınırda titremesin diye
// bölgeden çıkmak için sınırı 100 birim geçmek gerekir (histerezis).
// Splits the pot value into 3 zones (0 = left, 1 = middle, 2 = right). To avoid flicker at a
// border, leaving a zone needs going 100 units past the border (hysteresis).
int potZone(int raw, int current) {
  constexpr int kZoneWidth = 4096 / 3;
  constexpr int kMargin = 100;
  if (current < 0) return constrain(raw / kZoneWidth, 0, 2);
  if (current > 0 && raw < current * kZoneWidth - kMargin) return current - 1;
  if (current < 2 && raw > (current + 1) * kZoneWidth + kMargin) return current + 1;
  return current;
}

void writeLight(Light l) {
  iotbot.moduleTraficLightWrite(l == LIGHT_RED, l == LIGHT_YELLOW, l == LIGHT_GREEN);
}

void showGreenScreen() {
  bool waiting = requestPending && (millis() - stateStartMs < kMinGreenMs);
  iotbot.lcdWriteMid(L("YAYA GEÇİDİ", "PEDESTRIAN CROSSING"),
                     L("Arabalar: YEŞİL", "Cars: GREEN"),
                     requestPending ? L("İstek alındı", "Request received") : L("Geçmek için B1'e bas", "Press B1 to cross"),
                     waiting ? L("Lütfen bekleyin...", "Please wait...") : L("B3: manuel kontrol", "B3: manual control"));
}

void showYellowScreen() {
  iotbot.lcdWriteMid(L("YAYA GEÇİDİ", "PEDESTRIAN CROSSING"),
                     L("Arabalar: SARI", "Cars: YELLOW"),
                     L("Arabalar yavaşlıyor", "Cars slowing down"),
                     L("Hazır olun...", "Get ready..."));
}

void showWalkTitle() {
  iotbot.lcdClear();
  writeCentered(0, L("YAYALAR GEÇEBİLİR", "PEDESTRIANS: WALK"));
  shownSecond = -1;   // Rakam yeniden çizilsin / force redraw of the digit
}

void showManualScreen() {
  char line[41];
  snprintf(line, sizeof(line), L("Lamba: %s", "Light: %s"), lightName(manualLight));
  iotbot.lcdWriteMid(L("YAYA GEÇİDİ", "PEDESTRIAN CROSSING"), L("MANUEL MOD", "MANUAL MODE"), line,
                     L("Pot:lamba  B3:oto", "Pot:light  B3:auto"));
}

// O anki durumun ekranını yeniden çizer (dil değişince de kullanılır).
// Redraws the screen of the current state (also used after a language change).
void redrawScreen() {
  switch (state) {
    case CAR_GREEN:  showGreenScreen(); break;
    case CAR_YELLOW: showYellowScreen(); break;
    case WALK:       showWalkTitle(); break;
    case MANUAL:     showManualScreen(); break;
  }
}

// Büyük geri sayımı LCD'nin 2-4. satırlarına çizer. (Bu satırlar sadece ASCII karakter içerir,
// bu yüzden bayt konumlarıyla güvenle yerleştirebiliriz.)
// Draws the big countdown on LCD rows 2-4. (These rows hold ASCII characters only, so we can
// safely place them by byte position.)
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
    lcdRow(r + 1, line);
  }
}

void enterState(State s) {
  state = s;
  stateStartMs = millis();
  if (s == CAR_GREEN) {
    requestPending = false;  // İstek karşılandı / request has been served
    writeLight(LIGHT_GREEN);
    showGreenScreen();
    iotbot.serialWrite(L("Arabalar: YEŞİL", "Cars: GREEN"));
  } else if (s == CAR_YELLOW) {
    writeLight(LIGHT_YELLOW);
    showYellowScreen();
    iotbot.serialWrite(L("Arabalar: SARI", "Cars: YELLOW"));
  } else if (s == WALK) {
    writeLight(LIGHT_RED);
    showWalkTitle();
    lastTickMs = 0;     // İlk tik hemen çalsın / first tick plays immediately
    iotbot.serialWrite(L("Arabalar: KIRMIZI - yayalar geçiyor", "Cars: RED - pedestrians walking"));
  } else {
    writeLight(manualLight);
    showManualScreen();
  }
}

void setManualLight(Light l) {
  manualLight = l;
  writeLight(l);
  showManualScreen();
  iotbot.serialWrite(String(L("Manuel lamba: ", "Manual light: ")) + lightName(l));
}

void setMode(bool manual) {
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  if (manual) {
    // Manuele geçerken o anki lamba yanık kalır / the current light stays on when entering manual
    if (state == CAR_GREEN) manualLight = LIGHT_GREEN;
    else if (state == CAR_YELLOW) manualLight = LIGHT_YELLOW;
    else if (state == WALK) manualLight = LIGHT_RED;
    // Potun şu anki bölgesini kaydet: pot ancak çevrilince lambayı değiştirir.
    // Remember the pot's current zone: the pot changes the light only once it is turned.
    lastPotZone = potZone(iotbot.potentiometerRead(), -1);
    iotbot.serialWrite(L(">> MANUEL mod: lambayı potansiyometre ile seçin.", ">> MANUAL mode: pick the light with the potentiometer."));
    enterState(MANUAL);
  } else {
    iotbot.serialWrite(L(">> OTOMATİK mod: yaya geçidi çalışıyor.", ">> AUTO mode: the crossing is running."));
    enterState(CAR_GREEN);
  }
}

void pedestrianRequest() {
  if (state == MANUAL) {
    iotbot.serialWrite(L("Manuel moddayken yaya isteği alınmaz (oto yazın).", "No pedestrian requests in manual mode (type auto)."));
    return;
  }
  if (state != CAR_GREEN || requestPending) return; // Zaten sırada / already queued
  requestPending = true;
  requestMs = millis();
  iotbot.buzzerPlayTone(1200, 40);
  showGreenScreen();
  iotbot.serialWrite(L("Yaya isteği alındı.", "Pedestrian request received."));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "oto" || cmd == "otomatik" || cmd == "auto") {
    if (state == MANUAL) setMode(false);
  } else if (cmd == "manuel" || cmd == "manual") {
    if (state != MANUAL) setMode(true);
  } else if (cmd == "istek" || cmd == "request") {
    pedestrianRequest();
  } else if (cmd == "kirmizi" || cmd == "red" || cmd == "sari" || cmd == "yellow" || cmd == "yesil" || cmd == "green" ||
             cmd == "kapat" || cmd == "off") {
    if (state != MANUAL) setMode(true);
    Light l = LIGHT_OFF;
    if (cmd == "kirmizi" || cmd == "red") l = LIGHT_RED;
    else if (cmd == "sari" || cmd == "yellow") l = LIGHT_YELLOW;
    else if (cmd == "yesil" || cmd == "green") l = LIGHT_GREEN;
    setManualLight(l);
  } else if (cmd == "dil" || cmd == "lang" || cmd == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    redrawScreen();
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.serialWrite(L("Yaya geçidi hazır.", "Pedestrian crossing ready."));
  printHelp();
  enterState(CAR_GREEN);
}

void loop() {
  uint32_t now = millis();

  // 1) B3 -> mod değiştir (sadece basıldığı an) / B3 -> toggle mode (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3 && now - lastB3Ms > 200) {
    lastB3Ms = now;
    setMode(state != MANUAL);
  }
  lastB3 = b3;

  // 2) Yaya butonu B1/B2: sadece "basıldığı an" sayılır (basılı tutmak tekrar istek yapmaz).
  // 2) Pedestrian button B1/B2: only the moment of pressing counts (holding does not repeat).
  bool down = iotbot.button1Read() || iotbot.button2Read();
  if (down && !buttonWasDown) pedestrianRequest();
  buttonWasDown = down;

  // 3) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  switch (state) {
    case CAR_GREEN:
      // Hem 1 sn geçmeli hem de arabalar en az 5 sn yeşil görmüş olmalı.
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
      int sec = (remaining + 999) / 1000;  // 6, 5, ... 1 (yukarı yuvarla / round up)
      if (sec != shownSecond) {
        shownSecond = sec;
        drawCountdown(sec, hurry);
      }
      // Sesli yaya geçidi: yavaş tik, son 2 sn'de hızlı ve ince tik.
      // Accessible crossing: slow ticks, fast high ticks in the last 2 s.
      if (now - lastTickMs >= (hurry ? kFastTickGapMs : kSlowTickGapMs)) {
        lastTickMs = now;
        iotbot.buzzerPlayTone(hurry ? 1600 : 900, 30);
      }
      break;
    }

    case MANUAL: {
      // Pot 3 bölge: sol = kırmızı, orta = sarı, sağ = yeşil. Sadece bölge değişince uygulanır,
      // böylece seri porttan verilen lamba hemen ezilmez.
      // Pot has 3 zones: left = red, middle = yellow, right = green. Applied only when the zone
      // changes, so a light set from the serial port is not overridden right away.
      int zone = potZone(iotbot.potentiometerRead(), lastPotZone);
      if (zone != lastPotZone) {
        lastPotZone = zone;
        setManualLight(zone == 0 ? LIGHT_RED : (zone == 1 ? LIGHT_YELLOW : LIGHT_GREEN));
      }
      break;
    }
  }
  delay(10);
}
