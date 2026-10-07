/*
 * TR: GERÇEK PROJE - Temassız Çöp Kutusu.
 *  - OTOMATİK mod (açılışta): Elinizi çöp kutusunun üzerindeki ultrasonik
 *    sensöre 20 cm'den fazla yaklaştırın: kapak (servo motor) kendi kendine
 *    açılır ve kısa bir "bip" sesi gelir. Eliniz yakındayken kapak açık kalır;
 *    elinizi çektikten 3 saniye sonra yavaşça kapanır. Kapağa hiç
 *    dokunmadığınız için mikrop bulaşmaz! LCD kutunun kaç kez kullanıldığını sayar.
 *  - B3 butonu MANUEL moda geçer: sensör yok sayılır, kapağı B1 (veya B2)
 *    butonu ile siz açıp kapatırsınız (örneğin kutuyu boşaltırken). B3'e tekrar
 *    basınca otomatik moda döner.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help     -> komut listesi
 *      oto     / auto     -> otomatik mod (sensör)
 *      manuel  / manual   -> manuel mod (B1 ile aç/kapat)
 *      ac      / open     -> kapağı aç (manuel moda geçer)
 *      kapat   / close    -> kapağı kapat (manuel moda geçer)
 *      oku     / read     -> mesafeyi ve sayacı yaz
 *      sifirla / reset    -> kullanım sayacını sıfırla
 *      dil     / lang     -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Touchless Trash Can.
 *  - AUTO mode (at startup): Bring your hand closer than 20 cm to the
 *    ultrasonic sensor on top of the bin: the lid (servo motor) opens by
 *    itself with a short "beep". The lid stays open while your hand is near
 *    and closes gently 3 seconds after you pull your hand away. You never
 *    touch the lid, so no germs are spread! The LCD counts how many times the
 *    bin was used.
 *  - Button B3 switches to MANUAL mode: the sensor is ignored and you open
 *    and close the lid with button B1 (or B2) (e.g. while emptying the bin).
 *    Press B3 again to go back to auto mode.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help    / yardim   -> command list
 *      auto    / oto      -> auto mode (sensor)
 *      manual  / manuel   -> manual mode (open/close with B1)
 *      open    / ac       -> open the lid (switches to manual)
 *      close   / kapat    -> close the lid (switches to manual)
 *      read    / oku      -> print the distance and the counter
 *      reset   / sifirla  -> reset the use counter
 *      lang    / dil      -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ultrasonik sensör sabit pinler kullanır (TRIG=IO27,
 * ECHO=IO32), soket seçmenize gerek yok. Kapak servosunu P2 soketine (IO26)
 * takın. Trafik lambasını bu örnekte KULLANMAYIN (IO32'yi paylaşır). B1, B3
 * kart üzerindedir. / The ultrasonic sensor uses fixed pins (TRIG=IO27,
 * ECHO=IO32), no socket choice needed. Plug the lid servo into socket P2
 * (IO26). Do NOT use the traffic light in this example (it shares IO32). B1
 * and B3 are on the board.
 */

#define USE_SERVO
#include <IOTBOT.h>

IOTBOT iotbot;

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

#define SERVO_PIN IO26 // Kapak servosu: P2 / lid servo: P2

namespace {
  constexpr int kNearCm = 20;                   // Bu mesafenin altı "el var" / closer than this = hand
  constexpr uint32_t kConfirmMs = 150;          // El bu kadar süre kalmalı / hand must stay this long
  constexpr uint32_t kCloseDelayMs = 3000;      // El gidince kapanma gecikmesi / close delay after hand leaves
  constexpr uint32_t kMeasureIntervalMs = 60;   // Sensör en az 60 ms arayla ölçmeli / sensor needs 60 ms between pings
  constexpr uint32_t kUiIntervalMs = 300;       // LCD yenileme aralığı / LCD refresh interval
  constexpr int kOpenAngle = 90;                // Kapak açık açı / lid open angle
  constexpr int kClosedAngle = 0;               // Kapak kapalı açı / lid closed angle
  constexpr int kOpenMsPerDeg = 3;              // Hızlı açılış (~0.3 sn) / fast opening (~0.3 s)
  constexpr int kCloseMsPerDeg = 6;             // Yavaş kapanış (~0.5 sn) / gentle closing (~0.5 s)

  bool manualMode = false;      // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
  bool lidOpen = false;         // Kapak açık mı olmalı? / should the lid be open?
  int lidAngle = kClosedAngle;  // Servonun şu anki açısı / current servo angle
  uint32_t lastServoMs = 0;
  bool handSeen = false;        // Şu an el algılanıyor mu? / is a hand detected right now?
  uint32_t handSinceMs = 0;     // El ne zamandan beri var / since when the hand is there
  uint32_t lastHandMs = 0;      // Eli en son ne zaman gördük / last time we saw the hand
  uint32_t lastMeasureMs = 0;
  uint32_t lastUiMs = 0;
  unsigned long useCount = 0;
  int distanceCm = 0;
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
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
// lcdWriteFixedTxt Türkçe harfleri (ç, ğ, ı, ö, ş, ü) LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- TEMASSIZ ÇÖP KUTUSU - Komutlar ----", "---- TOUCHLESS TRASH CAN - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oto           : otomatik mod (sensör)", "  auto          : auto mode (sensor)"));
  iotbot.serialWrite(L("  manuel        : manuel mod (B1 ile aç/kapat)", "  manual        : manual mode (open/close with B1)"));
  iotbot.serialWrite(L("  ac / kapat    : kapağı aç / kapat", "  open / close  : open / close the lid"));
  iotbot.serialWrite(L("  oku           : mesafe ve sayaç", "  read          : distance and counter"));
  iotbot.serialWrite(L("  sifirla       : sayacı sıfırla", "  reset         : reset the counter"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : OTOMATİK <-> MANUEL", "  B3 button     : AUTO <-> MANUAL"));
}

void showMainScreen() {
  lcdRow(0, L("TEMASSIZ ÇÖP KUTUSU", "TOUCHLESS TRASH CAN"));
  char line[41];
  snprintf(line, sizeof(line), L("%-6s Kapak: %s", "%-6s Lid: %s"), manualMode ? L("MANUEL", "MANUAL") : L("OTO", "AUTO"),
           lidOpen ? L("AÇIK", "OPEN") : L("KAPALI", "CLOSED"));
  lcdRow(1, line);
  snprintf(line, sizeof(line), L("%lu kez  B3:%s", "%lu uses  B3:%s"), useCount,
           manualMode ? L("oto", "auto") : L("manuel", "manual"));
  lcdRow(3, line);
  lastUiMs = 0; // Canlı satırı hemen çiz / draw the live row right away
}

// Kapağı aç/kapat. Servo loop() içinde yavaş yavaş hareket eder (bloklamaz).
// Open/close the lid. The servo moves bit by bit inside loop() (non-blocking).
void setLid(bool open) {
  if (open == lidOpen) return;
  lidOpen = open;
  if (open) {
    useCount++;
    iotbot.buzzerPlayTone(1500, 40);
  }
  lastServoMs = millis();
  showMainScreen();
  iotbot.serialWrite(open ? L("Kapak açıldı.", "Lid opened.") : L("Kapak kapandı.", "Lid closed."));
}

void setMode(bool manual) {
  manualMode = manual;
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  // Otomatiğe dönünce kapak açıksa: 3 sn sonra kapanır (el yoksa).
  // Back to auto with the lid open: it closes after 3 s (if no hand).
  lastHandMs = millis();
  handSeen = false;
  iotbot.serialWrite(manual ? L(">> MANUEL mod: kapağı B1 (veya B2) ile açıp kapatın.", ">> MANUAL mode: open/close the lid with B1 (or B2).")
                            : L(">> OTOMATİK mod: kapak elinizi görünce açılır.", ">> AUTO mode: the lid opens when it sees your hand."));
  showMainScreen();
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "oto" || cmd == "otomatik" || cmd == "auto") {
    setMode(false);
  } else if (cmd == "manuel" || cmd == "manual") {
    setMode(true);
  } else if (cmd == "ac" || cmd == "open") {
    if (!manualMode) setMode(true);
    setLid(true);
  } else if (cmd == "kapat" || cmd == "close") {
    if (!manualMode) setMode(true);
    setLid(false);
  } else if (cmd == "oku" || cmd == "read") {
    char msg[64];
    snprintf(msg, sizeof(msg), L("Mesafe: %d cm (0 = yankı yok)  Kullanım: %lu", "Distance: %d cm (0 = no echo)  Uses: %lu"),
             distanceCm, useCount);
    iotbot.serialWrite(msg);
  } else if (cmd == "sifirla" || cmd == "reset") {
    useCount = 0;
    showMainScreen();
    iotbot.serialWrite(L("Sayaç sıfırlandı.", "Counter reset."));
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
  iotbot.moduleServoGoAngle(SERVO_PIN, kClosedAngle, 1);  // Başlangıçta kapalı / start closed
  iotbot.lcdClear();
  showMainScreen();
  iotbot.serialWrite(L("Temassız çöp kutusu hazır.", "Touchless trash can ready."));
  printHelp();
}

void loop() {
  uint32_t now = millis();

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

  // 3) Ölçüm: 0 = "yankı yok / menzil dışı" demektir, el yok sayılır.
  // 3) Measure: 0 means "no echo / out of range", treated as no hand.
  if (now - lastMeasureMs >= kMeasureIntervalMs) {
    lastMeasureMs = now;
    distanceCm = iotbot.moduleUltrasonicDistanceRead();
    bool near = distanceCm > 0 && distanceCm < kNearCm;
    if (near) {
      if (!handSeen) { handSeen = true; handSinceMs = now; }
      lastHandMs = now;
    } else {
      handSeen = false;
    }
  }

  if (manualMode) {
    // MANUEL: B1 (veya B2) her basışta kapağı açar/kapatır.
    // MANUAL: B1 (or B2) opens/closes the lid on every press.
    bool b1 = iotbot.button1Read() || iotbot.button2Read();
    if (b1 && !lastB1 && now - lastButtonMs > 200) {
      lastButtonMs = now;
      setLid(!lidOpen);
    }
    lastB1 = b1;
  } else {
    // 4) Açma: el 150 ms boyunca kesintisiz yakın olmalı (tek hatalı ölçüm kapağı açmasın).
    // 4) Open: the hand must stay near for 150 ms (one bad reading must not open the lid).
    if (!lidOpen && handSeen && now - handSinceMs >= kConfirmMs) setLid(true);

    // 5) Kapama: el 3 sn boyunca hiç görünmediyse. El geri gelirse süre sıfırlanır.
    // 5) Close: when no hand was seen for 3 s. If the hand comes back the timer restarts.
    if (lidOpen && millis() - lastHandMs >= kCloseDelayMs) setLid(false);
  }

  // 6) Servo: hedefe doğru adım adım (açılış hızlı, kapanış yavaş). Geçen süreye göre
  // birkaç derece birden gidebilir, böylece ölçüm beklemeleri hareketi yavaşlatmaz.
  // 6) Servo: step toward the target (fast opening, gentle closing). It may move several
  // degrees at once based on the elapsed time, so sensor waits do not slow the motion.
  int lidTarget = lidOpen ? kOpenAngle : kClosedAngle;
  if (lidAngle != lidTarget) {
    int msPerDeg = lidOpen ? kOpenMsPerDeg : kCloseMsPerDeg;
    int due = (millis() - lastServoMs) / msPerDeg;
    if (due > 0) {
      lastServoMs = millis();
      int step = min(due, abs(lidTarget - lidAngle));
      lidAngle += (lidTarget > lidAngle) ? step : -step;
      iotbot.moduleServoGoAngle(SERVO_PIN, lidAngle, 1);
    }
  }

  // 7) Canlı satır: kapanmaya kalan süre, mesafe ya da ipucu.
  // 7) Live row: time left until closing, distance or a hint.
  if (now - lastUiMs >= kUiIntervalMs) {
    lastUiMs = now;
    char line[41];
    if (manualMode) {
      snprintf(line, sizeof(line), "%s", L("B1: kapağı aç/kapat", "B1: open/close lid"));
    } else if (lidOpen && !handSeen) {
      uint32_t gone = millis() - lastHandMs;
      uint32_t left = gone < kCloseDelayMs ? (kCloseDelayMs - gone + 999) / 1000 : 0;
      snprintf(line, sizeof(line), L("Kapanıyor: %lu sn", "Closing in: %lu s"), (unsigned long)left);
    } else if (distanceCm > 0) {
      snprintf(line, sizeof(line), L("Mesafe: %d cm", "Distance: %d cm"), distanceCm);
    } else {
      snprintf(line, sizeof(line), "%s", L("Elini yaklaştır", "Bring your hand"));
    }
    lcdRow(2, line);
  }
  delay(5);
}
