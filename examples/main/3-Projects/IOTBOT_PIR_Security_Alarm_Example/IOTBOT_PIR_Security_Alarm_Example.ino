/*
 * TR: GERÇEK PROJE - Güvenlik Alarmı. Sistem KURULU iken PIR hareket sensörü
 * bir hareket algılarsa: buzzer siren sesi çalar, LCD "ALARM!" yazar ve kart
 * üzerindeki röle tetiklenir (örneğin bir siren ya da ışık bağlayabilirsiniz).
 *  - Açılışta sistem 10 saniyelik "çıkış süresi" sayar, sonra KURULUR
 *    (OTOMATİK izleme). Bu süre hem odadan çıkmanız hem de PIR sensörünün
 *    ısınması içindir.
 *  - B3 butonu: alarm çalarken SUSTURUR (tıpkı gerçek bir alarm sisteminin
 *    "iptal" tuşu gibi); sistem kurulu kalır ve hareket bitince yeniden izlemeye
 *    başlar. Alarm çalmıyorken B3 sistemi ÇÖZER (kapatır) ya da yeniden KURAR.
 *  - Sistem ÇÖZÜLMÜŞKEN (MANUEL) röleyi seri porttan elle açıp kapatabilirsiniz
 *    (siren/lamba testi için).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help             -> komut listesi
 *      kur     / arm   (= oto)    -> sistemi kur (10 sn çıkış süresi)
 *      coz     / disarm (= manuel)-> sistemi çöz (alarm kapalı, röle elle)
 *      sustur  / mute             -> çalan alarmı sustur (sistem kurulu kalır)
 *      ac      / on               -> röleyi aç (sadece çözülmüşken)
 *      kapat   / off              -> röleyi kapat
 *      test                       -> 1 saniyelik siren testi
 *      oku     / read             -> sistem ve sensör durumunu yaz
 *      dil     / lang             -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Security Alarm. While the system is ARMED, if the PIR
 * motion sensor detects movement: the buzzer sounds a siren, the LCD shows
 * "ALARM!" and the board's relay is triggered (you can wire a siren or a
 * light to it).
 *  - At startup the system counts a 10-second "exit delay", then it is ARMED
 *    (AUTO watching). This time is for you to leave the room and for the PIR
 *    sensor to warm up.
 *  - Button B3: while the alarm sounds it SILENCES it (just like the "cancel"
 *    button on a real alarm system); the system stays armed and starts
 *    watching again once the motion is over. When no alarm is sounding, B3
 *    DISARMS the system or ARMS it again.
 *  - While DISARMED (MANUAL) you can switch the relay by hand from the serial
 *    port (to test the siren/lamp).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help    / yardim             -> command list
 *      arm     / kur    (= auto)    -> arm the system (10 s exit delay)
 *      disarm  / coz    (= manual)  -> disarm the system (alarm off, relay by hand)
 *      mute    / sustur             -> silence a sounding alarm (stays armed)
 *      on      / ac                 -> relay on (only while disarmed)
 *      off     / kapat              -> relay off
 *      test                         -> 1-second siren test
 *      read    / oku                -> print the system and sensor state
 *      lang    / dil                -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: PIR sensörünü P1-P5 soketlerinden BİRİNE takın ve
 * aşağıdaki PIR_PIN değerini o soketin sinyaline göre ayarlayın. Röle ve B3
 * kart üzerindedir. / Plug the PIR sensor into ONE of the P1-P5 sockets and
 * set PIR_PIN below to match that socket's signal. The relay and B3 are on
 * the board.
 */

#include <IOTBOT.h>

IOTBOT iotbot;

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

#define PIR_PIN IO27 // PIR sensörünün bağlı olduğu pin / Pin the PIR sensor is connected to
// Desteklenen pinler: IO25 - IO26 - IO27 - IO32 - IO33
// Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

namespace {
  constexpr uint32_t kExitDelayMs = 10000;   // Kurulmadan önce bekleme / delay before arming
  constexpr uint32_t kRearmQuietMs = 5000;   // Susturunca: bu kadar hareketsizlikten sonra yeniden izle / after mute: re-watch after this much stillness
  constexpr uint32_t kSirenStepMs = 300;     // Siren tonu değişim aralığı / siren tone change interval
  constexpr uint32_t kUiIntervalMs = 250;    // LCD yenileme aralığı / LCD refresh interval

  // DISARMED = çözülmüş (MANUEL), ARMING = çıkış süresi, ARMED = kurulu (OTOMATİK izleme),
  // ALARM = siren çalıyor, MUTED = susturuldu, hareketin bitmesi bekleniyor.
  // DISARMED = off (MANUAL), ARMING = exit delay, ARMED = watching (AUTO), ALARM = siren on,
  // MUTED = silenced, waiting for the motion to end.
  enum State { DISARMED, ARMING, ARMED, ALARM, MUTED };
  State state = ARMING;
  uint32_t stateStartMs = 0;
  uint32_t lastMotionMs = 0;
  bool motion = false;
  bool relayOn = false;
  bool sirenHigh = false;
  uint32_t lastSirenMs = 0;
  uint32_t testUntilMs = 0;     // Siren testi bitiş anı (0 = test yok) / siren test end (0 = no test)
  unsigned long alarmCount = 0;
  bool lastB3 = false;
  uint32_t lastB3Ms = 0;
  uint32_t lastUiMs = 0;
}

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "ÇÖZ" -> "coz"
// Lower-cases and simplifies Turkish letters: "ÇÖZ" -> "coz"
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
  iotbot.serialWrite(L("---- GÜVENLİK ALARMI - Komutlar ----", "---- SECURITY ALARM - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  kur (oto)     : sistemi kur (10 sn çıkış süresi)", "  arm (auto)    : arm the system (10 s exit delay)"));
  iotbot.serialWrite(L("  coz (manuel)  : sistemi çöz, röle elle", "  disarm (manual): disarm, relay by hand"));
  iotbot.serialWrite(L("  sustur        : çalan alarmı sustur", "  mute          : silence a sounding alarm"));
  iotbot.serialWrite(L("  ac / kapat    : röle aç / kapat (çözülmüşken)", "  on / off      : relay on / off (while disarmed)"));
  iotbot.serialWrite(L("  test          : 1 sn siren testi", "  test          : 1 s siren test"));
  iotbot.serialWrite(L("  oku           : durum", "  read          : status"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : alarmda sustur, yoksa kur/çöz", "  B3 button     : mute during alarm, else arm/disarm"));
}

const char *stateName() {
  switch (state) {
    case DISARMED: return L("ÇÖZÜLDÜ (MANUEL)", "DISARMED (MANUAL)");
    case ARMING:   return L("KURULUYOR", "ARMING");
    case ARMED:    return L("KURULU (OTOMATİK)", "ARMED (AUTO)");
    case ALARM:    return "!!! ALARM !!!";
    default:       return L("SUSTURULDU", "SILENCED");
  }
}

void drawScreen() {
  lcdRow(0, state == ALARM ? "   !!! ALARM !!!" : L("  GÜVENLİK ALARMI", "   SECURITY ALARM"));
  lcdRow(1, stateName());
  switch (state) {
    case DISARMED: lcdRow(3, L("B3: sistemi kur", "B3: arm system")); break;
    case ALARM:    lcdRow(3, L("B3: sustur", "B3: silence")); break;
    default:       lcdRow(3, L("B3: sistemi çöz", "B3: disarm system")); break;
  }
  lastUiMs = 0; // Canlı satırı hemen çiz / draw the live row right away
}

void setRelay(bool on) {
  relayOn = on;
  iotbot.relayWrite(on);
}

void sirenOff() {
  iotbot.buzzerStop();
  testUntilMs = 0;
}

void enterState(State s) {
  state = s;
  stateStartMs = millis();
  switch (s) {
    case DISARMED:
      sirenOff();
      setRelay(false);
      iotbot.buzzerPlayTone(800, 120);
      iotbot.serialWrite(L(">> Sistem ÇÖZÜLDÜ (MANUEL): alarm kapalı, röle ac/kapat ile.", ">> System DISARMED (MANUAL): alarm off, relay with on/off."));
      break;
    case ARMING:
      sirenOff();
      setRelay(false);
      iotbot.buzzerPlayTone(1500, 60);
      iotbot.serialWrite(L(">> Sistem 10 saniye içinde KURULACAK. Odadan çıkın!", ">> System will be ARMED in 10 seconds. Leave the room!"));
      break;
    case ARMED:
      iotbot.buzzerPlayTone(2000, 60);
      iotbot.serialWrite(L(">> Sistem KURULU (OTOMATİK): hareket izleniyor.", ">> System ARMED (AUTO): watching for motion."));
      break;
    case ALARM:
      alarmCount++;
      setRelay(true);
      lastSirenMs = 0;
      iotbot.serialWrite(L("ALARM: hareket algılandı!", "ALARM: motion detected!"));
      break;
    case MUTED:
      sirenOff();
      setRelay(false);
      iotbot.serialWrite(L("Alarm susturuldu. Hareket bitince sistem yeniden izler.", "Alarm silenced. The system watches again once the motion ends."));
      break;
  }
  drawScreen();
}

void printStatus() {
  char msg[96];
  snprintf(msg, sizeof(msg), L("Durum: %s  PIR: %s  Röle: %s  Alarm sayısı: %lu", "State: %s  PIR: %s  Relay: %s  Alarms: %lu"),
           stateName(), motion ? L("HAREKET", "MOTION") : L("sakin", "quiet"), relayOn ? L("AÇIK", "ON") : L("KAPALI", "OFF"),
           alarmCount);
  iotbot.serialWrite(msg);
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "kur" || cmd == "arm" || cmd == "oto" || cmd == "otomatik" || cmd == "auto") {
    if (state == DISARMED) enterState(ARMING);
    else iotbot.serialWrite(L("Sistem zaten kurulu.", "The system is already armed."));
  } else if (cmd == "coz" || cmd == "disarm" || cmd == "manuel" || cmd == "manual") {
    if (state != DISARMED) enterState(DISARMED);
  } else if (cmd == "sustur" || cmd == "mute") {
    if (state == ALARM) enterState(MUTED);
    else iotbot.serialWrite(L("Şu an çalan bir alarm yok.", "No alarm is sounding right now."));
  } else if (cmd == "ac" || cmd == "on") {
    if (state == DISARMED) {
      setRelay(true);
      iotbot.serialWrite(L("Röle AÇIK.", "Relay ON."));
    } else {
      iotbot.serialWrite(L("Röle sadece sistem çözülmüşken elle açılır (coz yazın).", "The relay can only be switched while disarmed (type disarm)."));
    }
  } else if (cmd == "kapat" || cmd == "off") {
    if (state == ALARM) {
      enterState(MUTED);
    } else {
      setRelay(false);
      iotbot.serialWrite(L("Röle KAPALI.", "Relay OFF."));
    }
  } else if (cmd == "test") {
    if (state != ALARM) {
      testUntilMs = millis() + 1000;
      lastSirenMs = 0;
      iotbot.serialWrite(L("Siren testi (1 sn)...", "Siren test (1 s)..."));
    }
  } else if (cmd == "oku" || cmd == "read") {
    printStatus();
  } else if (cmd == "dil" || cmd == "lang" || cmd == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    drawScreen();
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
  iotbot.lcdClear();
  iotbot.serialWrite(L("Güvenlik sistemi başladı.", "Security system started."));
  printHelp();
  enterState(ARMING);
}

void loop() {
  uint32_t now = millis();
  motion = iotbot.moduleMotionRead(PIR_PIN);
  if (motion) lastMotionMs = now;

  // 1) B3: alarmda sustur, çözülmüşken kur, diğer durumlarda çöz (sadece basıldığı an).
  // 1) B3: mute during alarm, arm when disarmed, otherwise disarm (on press only).
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3 && now - lastB3Ms > 300) {
    lastB3Ms = now;
    if (state == ALARM) enterState(MUTED);
    else if (state == DISARMED) enterState(ARMING);
    else enterState(DISARMED);
  }
  lastB3 = b3;

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Durum makinesi / State machine
  switch (state) {
    case ARMING:
      if (now - stateStartMs >= kExitDelayMs) enterState(ARMED);
      break;
    case ARMED:
      if (motion) enterState(ALARM);
      break;
    case MUTED:
      // PIR çıkışı hareketten sonra birkaç saniye HIGH kalır; hemen yeniden izlersek alarm
      // susturulur susturulmaz tekrar çalar. Bu yüzden 5 sn hareketsizlik bekleriz.
      // The PIR output stays HIGH for a few seconds after motion; if we watched again at once
      // the alarm would ring right after muting. So we wait for 5 s of stillness.
      if (now - lastMotionMs >= kRearmQuietMs && now - stateStartMs >= kRearmQuietMs) enterState(ARMED);
      break;
    default:
      break;
  }

  // 4) Siren: iki ton arasında gidip gelir, loop'u bloklamaz.
  // 4) Siren: switches between two tones, does not block the loop.
  bool sirenWanted = (state == ALARM) || (testUntilMs != 0);
  if (testUntilMs != 0 && (int32_t)(now - testUntilMs) >= 0) {
    sirenOff();
    sirenWanted = (state == ALARM);
  }
  if (sirenWanted && now - lastSirenMs >= kSirenStepMs) {
    lastSirenMs = now;
    sirenHigh = !sirenHigh;
    iotbot.buzzerStart(sirenHigh ? 2000 : 1500);
  }

  // 5) Canlı satır / Live row
  if (now - lastUiMs >= kUiIntervalMs) {
    lastUiMs = now;
    char line[41];
    if (state == ARMING) {
      snprintf(line, sizeof(line), L("Kurulmaya: %lu sn", "Arming in: %lu s"),
               (unsigned long)((kExitDelayMs - (now - stateStartMs) + 999) / 1000));
    } else if (state == ALARM) {
      snprintf(line, sizeof(line), "%s", L("Hareket algılandı!", "Motion detected!"));
    } else {
      snprintf(line, sizeof(line), L("PIR:%s  Röle:%s", "PIR:%s  Relay:%s"), motion ? L("VAR", "YES") : L("yok", "no"),
               relayOn ? L("AÇIK", "ON") : L("KAPALI", "OFF"));
    }
    lcdRow(2, line);
  }
  delay(20);
}
