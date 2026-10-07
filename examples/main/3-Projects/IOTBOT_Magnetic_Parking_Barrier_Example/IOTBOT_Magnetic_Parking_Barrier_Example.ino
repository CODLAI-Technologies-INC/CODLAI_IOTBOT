/*
 * TR: GERÇEK PROJE - Manyetik Otopark Bariyeri. Gerçek otoparklarda yolun
 * altında arabanın metal gövdesini algılayan manyetik sensörler vardır. Bu
 * projede manyetik sensör "araba dedektörü" olur.
 *  - OTOMATİK mod (açılışta): sensöre bir mıknatıs (= araba) yaklaştırın ve
 *    0.3 saniye bekletin. Bariyer (servo motor) bir "bip" sesiyle 0'dan 90
 *    dereceye kalkar, araba oradayken açık kalır ve araba gittikten 2 saniye
 *    sonra iner. LCD bariyerin durumunu yazar ve geçen araçları sayar.
 *    Bariyer açıkken kart üzerindeki röle de açılır - buraya bir ikaz lambası
 *    bağlayabilirsiniz.
 *  - B3 butonu MANUEL moda geçer (görevli kulübesi): sensör yok sayılır,
 *    bariyeri B1 (veya B2) butonu ile siz kaldırıp indirirsiniz. B3'e tekrar
 *    basınca otomatik moda döner.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help     -> komut listesi
 *      oto     / auto     -> otomatik mod (sensör)
 *      manuel  / manual   -> manuel mod (B1 ile aç/kapat)
 *      ac      / open     -> bariyeri kaldır (manuel moda geçer)
 *      kapat   / close    -> bariyeri indir (manuel moda geçer)
 *      oku     / read     -> sensör ve sayaç durumunu yaz
 *      sifirla / reset    -> araç sayacını sıfırla
 *      dil     / lang     -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Magnetic Parking Barrier. Real car parks have magnetic
 * sensors under the road that detect a car's metal body. In this project the
 * magnetic sensor is the "car detector".
 *  - AUTO mode (at startup): bring a magnet (= a car) near the sensor and hold
 *    it for 0.3 seconds. The barrier (servo motor) rises from 0 to 90 degrees
 *    with a "beep", stays open while the car is there and goes down 2 seconds
 *    after the car leaves. The LCD shows the barrier state and counts the
 *    cars. While the barrier is open the onboard relay is also on - you can
 *    wire a warning lamp to it.
 *  - Button B3 switches to MANUAL mode (the attendant's booth): the sensor is
 *    ignored and you raise/lower the barrier with button B1 (or B2). Press B3
 *    again to go back to auto mode.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help    / yardim   -> command list
 *      auto    / oto      -> auto mode (sensor)
 *      manual  / manuel   -> manual mode (open/close with B1)
 *      open    / ac       -> raise the barrier (switches to manual)
 *      close   / kapat    -> lower the barrier (switches to manual)
 *      read    / oku      -> print the sensor and counter state
 *      reset   / sifirla  -> reset the car counter
 *      lang    / dil      -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Manyetik sensörü P1 soketine (IO25), bariyer servosunu
 * P2 soketine (IO26) takın. Röle, B1 ve B3 kart üzerindedir (röle istenirse
 * ikaz lambası için). / Plug the magnetic sensor into socket P1 (IO25) and
 * the barrier servo into socket P2 (IO26). The relay, B1 and B3 are on the
 * board (the relay is for an optional warning lamp).
 */

#define USE_SERVO
#include <IOTBOT.h>

IOTBOT iotbot;

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

#define MAGNET_PIN IO25 // Manyetik sensör: P1 / magnetic sensor: P1
#define SERVO_PIN IO26  // Bariyer servosu: P2 / barrier servo: P2

namespace {
  constexpr uint32_t kConfirmMs = 300;      // Araba bu kadar süre algılanmalı / car must be seen this long
  constexpr uint32_t kCloseDelayMs = 2000;  // Araba gidince kapanma gecikmesi / close delay after car leaves
  constexpr uint32_t kUiIntervalMs = 250;   // LCD yenileme aralığı / LCD refresh interval
  constexpr int kOpenAngle = 90;            // Bariyer yukarıda / barrier up
  constexpr int kClosedAngle = 0;           // Bariyer aşağıda / barrier down
  constexpr int kMsPerDeg = 5;              // Yavaş, gerçekçi hareket (~0.45 sn) / slow, realistic motion

  bool manualMode = false;      // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
  bool barrierOpen = false;     // Bariyer açık mı olmalı? / should the barrier be open?
  int barrierAngle = kClosedAngle;
  uint32_t lastServoMs = 0;
  bool rawSeen = false;         // Ham sensör okuması true mu? / raw sensor reading true?
  uint32_t rawSinceMs = 0;
  bool carPresent = false;
  uint32_t lastCarMs = 0;       // Arabayı en son ne zaman gördük / last time the car was seen
  uint32_t lastUiMs = 0;
  unsigned long carCount = 0;
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
  iotbot.serialWrite(L("---- OTOPARK BARİYERİ - Komutlar ----", "---- PARKING BARRIER - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oto           : otomatik mod (sensör)", "  auto          : auto mode (sensor)"));
  iotbot.serialWrite(L("  manuel        : manuel mod (B1 ile aç/kapat)", "  manual        : manual mode (open/close with B1)"));
  iotbot.serialWrite(L("  ac / kapat    : bariyeri kaldır / indir", "  open / close  : raise / lower the barrier"));
  iotbot.serialWrite(L("  oku           : sensör ve sayaç", "  read          : sensor and counter"));
  iotbot.serialWrite(L("  sifirla       : sayacı sıfırla", "  reset         : reset the counter"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : OTOMATİK <-> MANUEL", "  B3 button     : AUTO <-> MANUAL"));
}

void showMainScreen() {
  lcdRow(0, L("  OTOPARK BARİYERİ", "  PARKING BARRIER"));
  lcdRow(1, barrierOpen ? L("  BARİYER AÇIK", "  BARRIER OPEN") : L("  BARİYER KAPALI", "  BARRIER CLOSED"));
  char line[41];
  snprintf(line, sizeof(line), L("%-8s Araç: %lu", "%-8s Cars: %lu"), manualMode ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO"),
           carCount);
  lcdRow(3, line);
  lastUiMs = 0; // Canlı satırı hemen çiz / draw the live row right away
}

// Bariyeri kaldır/indir. Servo loop() içinde yavaşça hareket eder (bloklamaz).
// Raise/lower the barrier. The servo moves slowly inside loop() (non-blocking).
void setBarrier(bool open) {
  if (open == barrierOpen) return;
  barrierOpen = open;
  if (open) {
    carCount++;
    iotbot.relayWrite(true); // İkaz lambası hemen yanar / warning lamp on at once
    iotbot.buzzerPlayTone(1400, 50);
  } else {
    iotbot.buzzerPlayTone(800, 50);
    // Röle, bariyer tamamen inince loop() içinde kapanır / the relay turns off in loop() once fully down
  }
  lastServoMs = millis();
  showMainScreen();
  iotbot.serialWrite(open ? L("Bariyer açılıyor.", "Barrier opening.") : L("Bariyer kapanıyor.", "Barrier closing."));
}

void setMode(bool manual) {
  manualMode = manual;
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  lastCarMs = millis(); // Otomatiğe dönünce açık bariyer 2 sn sonra iner / back in auto an open barrier closes after 2 s
  iotbot.serialWrite(manual ? L(">> MANUEL mod: bariyeri B1 (veya B2) ile kaldırıp indirin.", ">> MANUAL mode: raise/lower the barrier with B1 (or B2).")
                            : L(">> OTOMATİK mod: bariyer aracı görünce açılır.", ">> AUTO mode: the barrier opens when it sees a car."));
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
    setBarrier(true);
  } else if (cmd == "kapat" || cmd == "close") {
    if (!manualMode) setMode(true);
    setBarrier(false);
  } else if (cmd == "oku" || cmd == "read") {
    char msg[80];
    snprintf(msg, sizeof(msg), L("Sensör: %s  Bariyer: %s  Araç sayısı: %lu", "Sensor: %s  Barrier: %s  Cars: %lu"),
             rawSeen ? L("mıknatıs VAR", "magnet YES") : L("mıknatıs yok", "magnet no"),
             barrierOpen ? L("AÇIK", "OPEN") : L("KAPALI", "CLOSED"), carCount);
    iotbot.serialWrite(msg);
  } else if (cmd == "sifirla" || cmd == "reset") {
    carCount = 0;
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
  iotbot.relayWrite(false);
  iotbot.moduleServoGoAngle(SERVO_PIN, kClosedAngle, 1);  // Başlangıçta kapalı / start closed
  iotbot.lcdClear();
  showMainScreen();
  iotbot.serialWrite(L("Otopark bariyeri hazır.", "Parking barrier ready."));
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

  // 3) Araba algılama: sensör girişi boşta "yüzebilir" (rastgele okuyabilir),
  // bu yüzden sinyal 300 ms KESİNTİSİZ true kalmadan araba sayılmaz.
  // 3) Car detection: the input can "float" (read random values), so the
  // signal must stay true for 300 ms WITHOUT a break before it counts.
  bool raw = iotbot.moduleMagneticRead(MAGNET_PIN);
  if (raw && !rawSeen) rawSinceMs = now;
  rawSeen = raw;
  carPresent = raw && now - rawSinceMs >= kConfirmMs;
  if (carPresent) lastCarMs = now;

  if (manualMode) {
    // MANUEL: B1 (veya B2) her basışta bariyeri kaldırır/indirir.
    // MANUAL: B1 (or B2) raises/lowers the barrier on every press.
    bool b1 = iotbot.button1Read() || iotbot.button2Read();
    if (b1 && !lastB1 && now - lastButtonMs > 200) {
      lastButtonMs = now;
      setBarrier(!barrierOpen);
    }
    lastB1 = b1;
  } else {
    // 4) Araba geldi -> bariyeri kaldır. / Car arrived -> raise the barrier.
    if (!barrierOpen && carPresent) {
      setBarrier(true);
      iotbot.serialWrite(L("Araç geldi.", "Car arrived."));
    }
    // 5) Araba 2 sn'dir yok -> bariyeri indir. Araba geri gelirse süre sıfırlanır.
    // 5) No car for 2 s -> lower the barrier. If the car comes back the timer restarts.
    if (barrierOpen && millis() - lastCarMs >= kCloseDelayMs) setBarrier(false);
  }

  // 6) Servo: her 5 ms'de 1 derece (geçen süreye göre). Tamamen inince ikaz lambası söner.
  // 6) Servo: 1 degree every 5 ms (based on elapsed time). The lamp goes off once fully down.
  int target = barrierOpen ? kOpenAngle : kClosedAngle;
  if (barrierAngle != target) {
    int due = (millis() - lastServoMs) / kMsPerDeg;
    if (due > 0) {
      lastServoMs = millis();
      int step = min(due, abs(target - barrierAngle));
      barrierAngle += (target > barrierAngle) ? step : -step;
      iotbot.moduleServoGoAngle(SERVO_PIN, barrierAngle, 1);
      if (!barrierOpen && barrierAngle == kClosedAngle) {
        iotbot.relayWrite(false); // Bariyer tamamen inince lamba söner / lamp off once fully down
        iotbot.serialWrite(L("Bariyer kapandı.", "Barrier closed."));
      }
    }
  }

  // 7) Canlı durum satırı / Live status row
  if (now - lastUiMs >= kUiIntervalMs) {
    lastUiMs = now;
    char line[41];
    if (manualMode) {
      snprintf(line, sizeof(line), "%s", L("B1: aç / kapat", "B1: open / close"));
    } else if (barrierOpen && carPresent) {
      snprintf(line, sizeof(line), "%s", L("Araç geçiyor...", "Car passing..."));
    } else if (barrierOpen) {
      uint32_t gone = millis() - lastCarMs;
      uint32_t left = gone < kCloseDelayMs ? (kCloseDelayMs - gone + 999) / 1000 : 0;
      snprintf(line, sizeof(line), L("Kapanıyor: %lu sn", "Closing in: %lu s"), (unsigned long)left);
    } else {
      snprintf(line, sizeof(line), "%s", L("Araç bekleniyor", "Waiting for a car"));
    }
    lcdRow(2, line);
  }
  delay(10);
}
