/*
 * TR: GERÇEK PROJE - Akıllı Sera Çatısı. Kart üzerindeki ışık sensörü (LDR)
 * güneşi ölçer; hava aydınlanınca step motor seranın çatı penceresini AÇAR,
 * kararınca KAPATIR. Motor hareket ederken LCD'de ilerleme çubuğu görünür.
 *  - OTOMATİK mod (açılışta): çatıyı ışık sensörü yönetir.
 *  - B3 butonu: pencereyi elle açar/kapatır ve MANUEL moda geçer. Manuel mod
 *    son elle yapılan işlemden 60 saniye sonra kendiliğinden otomatiğe döner.
 *    Çatı hareket ederken B3'e basarsanız çatı geri döner.
 *  - NEDEN İKİ FARKLI EŞİK (HİSTEREZİS)? Tek bir eşik olsaydı (örneğin %50),
 *    ışık tam %49-%51 arasında gidip gelirken çatı durmadan açılıp kapanırdı.
 *    Bu yüzden %60'ın ÜSTÜNDE açar, %40'ın ALTINDA kapatırız; aradaki bölgede
 *    hiçbir şey yapmayız. Ayrıca ışık 2 saniye boyunca eşiğin ötesinde
 *    kalmalı; böylece sensörün üzerinden geçen bir el gölgesi çatıyı kapatmaz.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help     -> komut listesi
 *      oto     / auto     -> otomatik mod
 *      manuel  / manual   -> manuel mod (60 sn, çatı olduğu yerde kalır)
 *      ac      / open     -> çatıyı aç (manuel moda geçer)
 *      kapat   / close    -> çatıyı kapat (manuel moda geçer)
 *      oku     / read     -> ışık ve çatı durumunu yaz
 *      dil     / lang     -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Smart Greenhouse Roof. The onboard light sensor (LDR)
 * measures the sun; when it gets bright the step motor OPENS the
 * greenhouse's roof window, when it gets dark it CLOSES it. A progress bar is
 * shown on the LCD while the motor moves.
 *  - AUTO mode (at startup): the light sensor runs the roof.
 *  - Button B3: opens/closes the window by hand and switches to MANUAL mode.
 *    Manual mode returns to auto by itself 60 seconds after the last manual
 *    action. Pressing B3 while the roof is moving sends it back.
 *  - WHY TWO DIFFERENT LEVELS (HYSTERESIS)? With a single level (say 50%),
 *    the roof would open and close nonstop while the light wobbles between
 *    49% and 51%. So we open ABOVE 60% and close BELOW 40%, and do nothing in
 *    between. The light must also stay past the level for 2 seconds, so a
 *    hand's shadow passing over the sensor does not close the roof.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help    / yardim   -> command list
 *      auto    / oto      -> auto mode
 *      manual  / manuel   -> manual mode (60 s, the roof stays where it is)
 *      open    / ac       -> open the roof (switches to manual)
 *      close   / kapat    -> close the roof (switches to manual)
 *      read    / oku      -> print the light level and roof state
 *      lang    / dil      -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Step motoru P6 motor soketine takın (IO26, IO33, IO32,
 * IO27 pinlerini kullanır - bu yüzden P2-P5'e başka modül TAKMAYIN). LDR kart
 * üzerindedir. Açmadan önce pencereyi elle KAPALI konuma getirin: program
 * çatının kapalı başladığını varsayar. / Plug the step motor into the P6
 * motor socket (it uses IO26, IO33, IO32, IO27 - so do NOT plug other modules
 * into P2-P5). The LDR is on the board. Close the window by hand before
 * powering on: the program assumes the roof starts closed.
 */

#define USE_STEP_MOTOR
#include <IOTBOT.h>

IOTBOT iotbot;

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

namespace {
  constexpr int kOpenLightPct = 60;          // Bunun üstünde aç / open above this
  constexpr int kCloseLightPct = 40;         // Bunun altında kapat / close below this
  constexpr uint32_t kConfirmMs = 2000;      // Işık bu kadar süre eşiği geçmeli / light must stay past the level
  constexpr uint32_t kManualHoldMs = 60000;  // Manuel mod süresi / manual mode duration
  // Tur başına adım 4'ün KATI olmalı (48): kütüphane her çağrıda bobin sırasını baştan
  // başlatır; 50 gibi bir değerde yön değişince ilk parça ~2 adım ters gider.
  // Steps per revolution must be a MULTIPLE of 4 (48): the library restarts the coil
  // sequence on every call; with a value like 50 the first chunk after a direction
  // change goes ~2 steps the wrong way.
  constexpr int kStepsPerRev = 48;
  constexpr int kRoofTotalSteps = 200;       // Tam açılma adımı (mekanizmanıza göre) / full-open steps (fit your build)
  constexpr int kChunkSteps = 8;             // Parça başına adım, 4'ün katı olmalı / steps per chunk, keep a multiple of 4
  constexpr int kChunks = kRoofTotalSteps / kChunkSteps; // Tam yol kaç parça / chunks for the full travel
  constexpr int kMotorRpm = 90;              // Motor hızı / motor speed
  constexpr bool kOpenDirection = true;      // Ters yöne açılıyorsa false yapın / set false if it opens the wrong way
  constexpr uint32_t kScreenMs = 300;        // LCD yenileme aralığı / LCD refresh interval

  // Çatının konumu "parça" cinsinden: 0 = tam kapalı, kChunks = tam açık. Program çatının
  // KAPALI başladığını varsayar. / Roof position in chunks: 0 = fully closed, kChunks = fully
  // open. The program assumes the roof starts CLOSED.
  int roofPos = 0;
  int roofTarget = 0;
  bool manualMode = false;
  uint32_t manualStartMs = 0;
  uint32_t brightSinceMs = 0; // 0 = şu an parlak değil / 0 = not bright right now
  uint32_t darkSinceMs = 0;   // 0 = şu an karanlık değil / 0 = not dark right now
  bool b3WasDown = false;
  uint32_t lastScreenMs = 0;
  int lightPct = 0;
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
// Sensör ve motor / Sensor and motor
// ---------------------------------------------------------------------------
// 8 okumanın ortalaması -> 0-100 arası yüzde. Bu kartta LDR değeri ışık arttıkça BÜYÜR.
// Average of 8 readings -> percent 0-100. On this board the LDR value GROWS with light.
int readLightPct() {
  long sum = 0;
  for (int i = 0; i < 8; i++) sum += iotbot.ldrRead();
  return constrain(map(sum / 8, 0, 4095, 0, 100), 0, 100);
}

// Hareketten sonra bobinlerdeki akımı keseriz: Stepper kütüphanesi son adımı enerjili bırakır,
// motor ve sürücü boşuna ısınır. / After moving we cut the coil current: the Stepper library
// leaves the last step energized and the motor and driver heat up for nothing.
void releaseCoils() {
  const int coilPins[] = {IO26, IO33, IO32, IO27};
  for (int pin : coilPins) digitalWrite(pin, LOW);
}

bool roofMoving() { return roofPos != roofTarget; }
bool roofIsOpen() { return roofTarget == kChunks; } // Hedef = açık / target = open

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
// lcdWriteFixedTxt Türkçe harfleri (ç, ğ, ı, ö, ş, ü) LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- AKILLI SERA ÇATISI - Komutlar ----", "---- SMART GREENHOUSE ROOF - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oto           : otomatik mod (ışık sensörü)", "  auto          : auto mode (light sensor)"));
  iotbot.serialWrite(L("  manuel        : manuel mod (60 sn)", "  manual        : manual mode (60 s)"));
  iotbot.serialWrite(L("  ac / kapat    : çatıyı aç / kapat", "  open / close  : open / close the roof"));
  iotbot.serialWrite(L("  oku           : ışık ve çatı durumunu yaz", "  read          : print light level and roof state"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : çatıyı elle aç/kapat (manuel)", "  B3 button     : open/close the roof by hand (manual)"));
}

void showMainScreen() {
  lcdRow(0, L(" AKILLI SERA ÇATISI", "  SMART GREENHOUSE"));
  lastScreenMs = 0; // Değerleri hemen çiz / draw the values right away
}

void printStatus() {
  char msg[96];
  snprintf(msg, sizeof(msg), L("Işık: %%%d  Çatı: %s  Mod: %s", "Light: %d%%  Roof: %s  Mode: %s"), lightPct,
           roofMoving() ? (roofIsOpen() ? L("AÇILIYOR", "OPENING") : L("KAPANIYOR", "CLOSING"))
                        : (roofIsOpen() ? L("AÇIK", "OPEN") : L("KAPALI", "CLOSED")),
           manualMode ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO"));
  iotbot.serialWrite(msg);
}

// Çatıya yeni hedef verir. Motor loop() içinde parça parça hareket eder (bloklamaz).
// Gives the roof a new target. The motor moves chunk by chunk inside loop() (non-blocking).
void setRoof(bool open) {
  int target = open ? kChunks : 0;
  if (target == roofTarget) return; // Zaten oraya gidiyor: asla iki kez açma! / already going there: never open twice!
  roofTarget = target;
  iotbot.serialWrite(open ? L("Çatı açılıyor", "Roof opening") : L("Çatı kapanıyor", "Roof closing"));
  lastScreenMs = 0;
}

void startManual() {
  manualStartMs = millis(); // Her elle işlem 60 sn'yi yeniden başlatır / every manual action restarts the 60 s
  if (!manualMode) {
    manualMode = true;
    iotbot.buzzerPlayTone(1500, 60);
    iotbot.serialWrite(L(">> MANUEL mod (60 sn): B3 veya ac/kapat komutları.", ">> MANUAL mode (60 s): B3 or open/close commands."));
  }
  lastScreenMs = 0;
}

void startAuto() {
  manualMode = false;
  brightSinceMs = 0;
  darkSinceMs = 0;
  iotbot.buzzerPlayTone(1000, 60);
  iotbot.serialWrite(L(">> OTOMATİK mod: çatıyı ışık sensörü yönetiyor.", ">> AUTO mode: the light sensor runs the roof."));
  lastScreenMs = 0;
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "oto" || cmd == "otomatik" || cmd == "auto") {
    startAuto();
  } else if (cmd == "manuel" || cmd == "manual") {
    startManual();
  } else if (cmd == "ac" || cmd == "open") {
    startManual();
    setRoof(true);
  } else if (cmd == "kapat" || cmd == "close") {
    startManual();
    setRoof(false);
  } else if (cmd == "oku" || cmd == "read") {
    printStatus();
  } else if (cmd == "dil" || cmd == "lang" || cmd == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    showMainScreen();
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.lcdClear();
  showMainScreen();
  iotbot.serialWrite(L("Akıllı sera hazır. Çatı kapalı kabul edildi.", "Smart greenhouse ready. Roof assumed closed."));
  printHelp();
}

void loop() {
  uint32_t now = millis();
  lightPct = readLightPct();

  // 1) B3 (true = basılı): elle aç/kapat ve 60 sn manuel moda geç. Hareket sırasında
  // basılırsa çatı geri döner. / B3 (true = pressed): open/close by hand and switch to
  // manual mode for 60 s. Pressed while moving: the roof turns back.
  bool b3Down = iotbot.button3Read();
  if (b3Down && !b3WasDown) {
    startManual();
    setRoof(!roofIsOpen());
  }
  b3WasDown = b3Down;

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  if (manualMode && now - manualStartMs >= kManualHoldMs) {
    startAuto(); // 60 sn doldu / 60 s are up
  }

  // 3) Otomatik karar (histerezis + 2 sn onay) / Auto decision (hysteresis + 2 s confirm)
  if (!manualMode) {
    if (lightPct >= kOpenLightPct) {
      darkSinceMs = 0;
      if (brightSinceMs == 0) brightSinceMs = now; // Parlaklık yeni başladı / brightness just started
      if (!roofIsOpen() && now - brightSinceMs >= kConfirmMs) setRoof(true);
    } else if (lightPct <= kCloseLightPct) {
      brightSinceMs = 0;
      if (darkSinceMs == 0) darkSinceMs = now;
      if (roofIsOpen() && now - darkSinceMs >= kConfirmMs) setRoof(false);
    } else {
      brightSinceMs = 0; // Ara bölge: hiçbir şey yapma / in-between zone: do nothing
      darkSinceMs = 0;
    }
  }

  // 4) Motor: her loop'ta sadece BİR küçük parça (~0.1 sn). moduleStepMotorMotion() hareket
  // bitene kadar BEKLER; parçalara bölünce B3 ve seri komutlar hep cevap verir.
  // 4) Motor: only ONE small chunk per loop (~0.1 s). moduleStepMotorMotion() WAITS until the
  // move ends; splitting it into chunks keeps B3 and serial commands responsive.
  if (roofMoving()) {
    bool opening = roofTarget > roofPos;
    iotbot.moduleStepMotorMotion(kStepsPerRev, opening ? kOpenDirection : !kOpenDirection, kChunkSteps, kMotorRpm);
    roofPos += opening ? 1 : -1;

    int percent = roofPos * 100 / kChunks;
    char bar[41];
    for (int c = 0; c < 14; c++) bar[c] = (c < percent * 14 / 100) ? '\xFF' : '.'; // 0xFF = dolu kutu / full block
    bar[14] = '\0';
    char line[41];
    snprintf(line, sizeof(line), "%s %3d%%", bar, percent);
    lcdRow(3, line);
    lcdRow(2, opening ? L("  Çatı açılıyor...", "  Roof opening...") : L("  Çatı kapanıyor...", "  Roof closing..."));

    if (!roofMoving()) {
      releaseCoils();
      iotbot.buzzerPlayTone(opening ? 1500 : 900, 80);
      iotbot.serialWrite(opening ? L("Çatı AÇIK", "Roof OPEN") : L("Çatı KAPALI", "Roof CLOSED"));
      brightSinceMs = 0;
      darkSinceMs = 0;
      lastScreenMs = 0;
    }
    return; // Hareket varken ekranın geri kalanını yazma / skip the rest of the screen while moving
  }

  // 5) LCD ve seri durum / LCD and serial status
  if (now - lastScreenMs >= kScreenMs) {
    lastScreenMs = now;
    char line[41];
    snprintf(line, sizeof(line), L("  Işık: %%%d", "  Light: %d%%"), lightPct);
    lcdRow(1, line);
    lcdRow(2, roofIsOpen() ? L("  Çatı: AÇIK", "  Roof: OPEN") : L("  Çatı: KAPALI", "  Roof: CLOSED"));
    if (manualMode) {
      snprintf(line, sizeof(line), L("  Mod: MANUEL %lu sn", "  Mode: MANUAL %lu s"),
               (unsigned long)((kManualHoldMs - (now - manualStartMs) + 999) / 1000));
    } else {
      snprintf(line, sizeof(line), "%s", L("  Mod: OTOMATİK", "  Mode: AUTO"));
    }
    lcdRow(3, line);
  }
  delay(20);
}
