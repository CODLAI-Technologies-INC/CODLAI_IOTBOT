/*
 * TR: GERÇEK PROJE - Duman/Gaz Alarmı. Duman sensörü havadaki gaz yoğunluğunu
 * sürekli ölçer. Değer, dinlenme (temiz hava) değerinin belirgin şekilde
 * üstüne çıktığında: buzzer alarm çalar, LCD "DUMAN/GAZ ALGILANDI!" yazar ve
 * kart üzerindeki röle tetiklenir (örneğin bir egzoz fanı ya da uyarı lambası
 * bağlayabilirsiniz).
 *  - OTOMATİK mod (açılışta): röle (fan) alarmla birlikte kendiliğinden açılır
 *    ve hava temizlenince kapanır.
 *  - B3 butonu: alarm çalarken sesi SUSTURUR; alarm yokken OTOMATİK <-> MANUEL
 *    geçişi yapar. MANUEL modda röleyi (fanı) B1 (veya B2) ile siz açıp
 *    kapatırsınız. GÜVENLİK İÇİN alarm sesi manuel modda da çalar.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim     / help           -> komut listesi
 *      oto        / auto           -> otomatik mod (röle alarmı izler)
 *      manuel     / manual         -> manuel mod (röle elle)
 *      ac         / on             -> röleyi aç (manuel moda geçer)
 *      kapat      / off            -> röleyi kapat (manuel moda geçer)
 *      sustur     / mute           -> çalan alarmı sustur
 *      kalibre    / calibrate      -> şu anki havayı "temiz hava" kabul et
 *      esik 400   / threshold 400  -> alarm eşiği (temiz havadan fark)
 *      oku        / read           -> sensör değerini şimdi yaz
 *      dil        / lang           -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Smoke/Gas Alarm. The smoke sensor continuously
 * measures the gas concentration in the air. When the value rises noticeably
 * above the resting (clean air) baseline: the buzzer sounds an alarm, the LCD
 * shows "SMOKE/GAS DETECTED!" and the board's relay is triggered (you can
 * wire an exhaust fan or a warning light to it).
 *  - AUTO mode (at startup): the relay (fan) turns on with the alarm by
 *    itself and turns off when the air is clean again.
 *  - Button B3: while the alarm sounds it SILENCES it; with no alarm it
 *    switches AUTO <-> MANUAL. In MANUAL mode you switch the relay (fan) with
 *    B1 (or B2). FOR SAFETY the alarm still sounds in manual mode.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help          / yardim     -> command list
 *      auto          / oto        -> auto mode (relay follows the alarm)
 *      manual        / manuel     -> manual mode (relay by hand)
 *      on            / ac         -> relay on (switches to manual)
 *      off           / kapat      -> relay off (switches to manual)
 *      mute          / sustur     -> silence a sounding alarm
 *      calibrate     / kalibre    -> take the current air as "clean air"
 *      threshold 400 / esik 400   -> alarm threshold (difference from clean air)
 *      read          / oku        -> print the sensor value now
 *      lang          / dil        -> switch language (Turkish <-> English)
 *
 * GÜVENLİK NOTU / SAFETY NOTE: Bu bir OYUNCAK/EĞİTİM projesidir, gerçek bir
 * yangın alarmı YERİNE KULLANILMAMALIDIR. Gaz sensörleri ilk çalıştırmada
 * birkaç dakika ısınmaya ihtiyaç duyar; ısındıktan sonra "kalibre" yazın.
 * / This is a TOY/EDUCATIONAL project and must NOT be used as a substitute for
 * a real fire alarm. Gas sensors need a few minutes to warm up when first
 * powered; type "calibrate" once warmed up.
 *
 * Bağlantı / Wiring: Duman sensörünü P1-P5 soketlerinden BİRİNE takın ve
 * aşağıdaki SMOKE_PIN değerini o soketin sinyaline göre ayarlayın. Röle, B1
 * ve B3 kart üzerindedir. / Plug the smoke sensor into ONE of the P1-P5
 * sockets and set SMOKE_PIN below to match that socket's signal. The relay,
 * B1 and B3 are on the board.
 */

#include <IOTBOT.h>

IOTBOT iotbot;

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

#define SMOKE_PIN IO27 // Duman sensörünün bağlı olduğu pin / Pin the smoke sensor is connected to
// Desteklenen pinler: IO25 - IO26 - IO27 - IO32 - IO33
// Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

namespace {
  int cleanAirBaseline = -1;
  // Bu değerin ÜZERİNE çıkıldığında alarm çalar - gerçek donanımla test edip ayarlayın.
  // Alarm triggers ABOVE this margin - tune by testing with real hardware.
  int alarmMargin = 400;
  constexpr uint32_t kReadIntervalMs = 150;   // Okuma aralığı / reading interval
  constexpr uint32_t kUiIntervalMs = 300;     // LCD yenileme aralığı / LCD refresh interval

  bool manualMode = false;   // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
  bool alarmActive = false;
  bool alarmMuted = false;
  bool relayOn = false;
  int value = 0;
  bool beepOn = false;
  uint32_t lastBeepMs = 0;
  uint32_t lastReadMs = 0;
  uint32_t lastUiMs = 0;
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

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "EŞİK" -> "esik"
// Lower-cases and simplifies Turkish letters: "EŞİK" -> "esik"
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
// Sensör ve röle / Sensor and relay
// ---------------------------------------------------------------------------
// 10 okumanın ortalaması / average of 10 readings
int readSmoke() {
  long sum = 0;
  for (int i = 0; i < 10; i++) sum += iotbot.moduleSmokeRead(SMOKE_PIN);
  return sum / 10;
}

// Temiz hava değeri: 1 saniye boyunca ortalama al (tek okuma çok gürültülü olabilir).
// Clean air value: average over 1 second (a single reading can be very noisy).
int measureBaseline() {
  long sum = 0;
  for (int i = 0; i < 20; i++) {
    sum += readSmoke();
    delay(50);
  }
  return sum / 20;
}

void setRelay(bool on) {
  if (on == relayOn) return;
  relayOn = on;
  iotbot.relayWrite(on);
  iotbot.serialWrite(on ? L("Röle (fan) AÇIK.", "Relay (fan) ON.") : L("Röle (fan) KAPALI.", "Relay (fan) OFF."));
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
// lcdWriteFixedTxt Türkçe harfleri (ç, ğ, ı, ö, ş, ü) LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- DUMAN/GAZ ALARMI - Komutlar ----", "---- SMOKE/GAS ALARM - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oto           : otomatik mod (röle alarmı izler)", "  auto          : auto mode (relay follows the alarm)"));
  iotbot.serialWrite(L("  manuel        : manuel mod (röle B1 ile)", "  manual        : manual mode (relay with B1)"));
  iotbot.serialWrite(L("  ac / kapat    : röleyi aç / kapat", "  on / off      : relay on / off"));
  iotbot.serialWrite(L("  sustur        : çalan alarmı sustur", "  mute          : silence a sounding alarm"));
  iotbot.serialWrite(L("  kalibre       : şu anki hava = temiz hava", "  calibrate     : current air = clean air"));
  iotbot.serialWrite(L("  esik 50-3000  : alarm eşiği (fark)", "  threshold 50-3000: alarm threshold (difference)"));
  iotbot.serialWrite(L("  oku           : sensör değerini yaz", "  read          : print the sensor value"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : alarmda sustur, yoksa OTOMATİK <-> MANUEL", "  B3 button     : mute during alarm, else AUTO <-> MANUAL"));
}

void drawScreen() {
  char line[41];
  lcdRow(0, alarmActive ? L("  !! DUMAN/GAZ !!", "  !! SMOKE/GAS !!") : L("  DUMAN/GAZ ALARMI", "   SMOKE/GAS ALARM"));
  snprintf(line, sizeof(line), L("Değer:%4d  Baz:%4d", "Value:%4d Base:%4d"), value, cleanAirBaseline);
  lcdRow(1, line);
  if (alarmActive) lcdRow(2, alarmMuted ? L("ALGILANDI! (sessiz)", "DETECTED! (muted)") : L("ALGILANDI! B3:sustur", "DETECTED! B3:mute"));
  else if (manualMode) lcdRow(2, L("B1:röle  B3:oto", "B1:relay  B3:auto"));
  else lcdRow(2, L("Hava temiz B3:manuel", "Air clean B3:manual"));
  snprintf(line, sizeof(line), L("%-6s Röle: %s", "%-6s Relay: %s"), manualMode ? L("MANUEL", "MANUAL") : L("OTO", "AUTO"),
           relayOn ? L("AÇIK", "ON") : L("KAPALI", "OFF"));
  lcdRow(3, line);
}

void printReading() {
  char msg[96];
  snprintf(msg, sizeof(msg), L("Duman değeri: %d  (temiz hava %d, fark %d, eşik %d)", "Smoke value: %d  (clean air %d, diff %d, threshold %d)"),
           value, cleanAirBaseline, value - cleanAirBaseline, alarmMargin);
  iotbot.serialWrite(msg);
}

void stopBeep() {
  iotbot.buzzerStop();
  beepOn = false;
}

void muteAlarm() {
  if (!alarmActive) {
    iotbot.serialWrite(L("Şu an çalan bir alarm yok.", "No alarm is sounding right now."));
    return;
  }
  alarmMuted = true;
  stopBeep();
  iotbot.serialWrite(L("Alarm sesi susturuldu (alarm sürüyor).", "Alarm sound silenced (the alarm is still on)."));
  drawScreen();
}

void setMode(bool manual) {
  manualMode = manual;
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  if (!manual) setRelay(alarmActive); // Otomatikte röle alarmı izler / in auto the relay follows the alarm
  iotbot.serialWrite(manual ? L(">> MANUEL mod: röleyi B1 (veya B2) ile açıp kapatın. Alarm sesi yine çalar.",
                                ">> MANUAL mode: switch the relay with B1 (or B2). The alarm still sounds.")
                            : L(">> OTOMATİK mod: röle alarmla birlikte açılır.", ">> AUTO mode: the relay turns on with the alarm."));
  drawScreen();
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int number = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oto" || word == "otomatik" || word == "auto") {
    setMode(false);
  } else if (word == "manuel" || word == "manual") {
    setMode(true);
  } else if (word == "ac" || word == "on") {
    if (!manualMode) setMode(true);
    setRelay(true);
    drawScreen();
  } else if (word == "kapat" || word == "off") {
    if (!manualMode) setMode(true);
    setRelay(false);
    drawScreen();
  } else if (word == "sustur" || word == "mute") {
    muteAlarm();
  } else if (word == "kalibre" || word == "calibrate") {
    iotbot.serialWrite(L("Temiz hava ölçülüyor (1 sn)...", "Measuring clean air (1 s)..."));
    cleanAirBaseline = measureBaseline();
    iotbot.serialWrite(String(L("Yeni temiz hava değeri: ", "New clean air value: ")) + cleanAirBaseline);
  } else if ((word == "esik" || word == "threshold") && hasValue) {
    alarmMargin = constrain(number, 50, 3000);
    iotbot.serialWrite(String(L("Alarm eşiği: ", "Alarm threshold: ")) + alarmMargin);
  } else if (word == "oku" || word == "read") {
    printReading();
  } else if (word == "dil" || word == "lang" || word == "language") {
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
  iotbot.lcdWriteMid(L("DUMAN/GAZ ALARMI", "SMOKE/GAS ALARM"), "", L("Temiz hava", "Measuring"), L("ölçülüyor...", "clean air..."));
  cleanAirBaseline = measureBaseline();
  value = cleanAirBaseline;
  iotbot.lcdClear();
  iotbot.serialWrite(L("Duman/gaz alarmı hazır.", "Smoke/gas alarm ready."));
  iotbot.serialWrite(String(L("Temiz hava değeri: ", "Clean air value: ")) + cleanAirBaseline);
  printHelp();
  drawScreen();
}

void loop() {
  uint32_t now = millis();

  // 1) B3: alarm çalarken sustur, yoksa mod değiştir (sadece basıldığı an).
  // 1) B3: mute while the alarm sounds, otherwise toggle mode (on press only).
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3 && now - lastButtonMs > 200) {
    lastButtonMs = now;
    if (alarmActive && !alarmMuted) muteAlarm();
    else setMode(!manualMode);
  }
  lastB3 = b3;

  // 2) MANUEL: B1 (veya B2) röleyi açar/kapatır / MANUAL: B1 (or B2) toggles the relay
  if (manualMode) {
    bool b1 = iotbot.button1Read() || iotbot.button2Read();
    if (b1 && !lastB1 && now - lastButtonMs > 200) {
      lastButtonMs = now;
      setRelay(!relayOn);
      drawScreen();
    }
    lastB1 = b1;
  }

  // 3) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 4) Ölçüm ve karar. Histerezis: alarm eşiğin %70'inin altına inmeden kapanmaz, böylece
  // eşiğin etrafında gidip gelirken röle titremez.
  // 4) Measurement and decision. Hysteresis: the alarm does not end until below 70% of the
  // threshold, so the relay does not chatter around the threshold.
  if (now - lastReadMs >= kReadIntervalMs) {
    lastReadMs = now;
    value = readSmoke();
    int diff = value - cleanAirBaseline;
    if (!alarmActive && diff >= alarmMargin) {
      alarmActive = true;
      alarmMuted = false;
      iotbot.serialWrite(L("ALARM: duman/gaz algılandı!", "ALARM: smoke/gas detected!"));
      if (!manualMode) setRelay(true);
      drawScreen();
    } else if (alarmActive && diff < alarmMargin * 7 / 10) {
      alarmActive = false;
      stopBeep();
      iotbot.serialWrite(L("Hava yeniden temiz.", "The air is clean again."));
      if (!manualMode) setRelay(false);
      drawScreen();
    }
  }

  // 5) Alarm sesi (bloklamaz): 100 ms bip, 100 ms sessiz.
  // 5) Alarm sound (non-blocking): 100 ms beep, 100 ms silence.
  if (alarmActive && !alarmMuted && now - lastBeepMs >= 100) {
    lastBeepMs = now;
    if (beepOn) stopBeep();
    else { iotbot.buzzerStart(2200); beepOn = true; }
  }

  if (now - lastUiMs >= kUiIntervalMs) {
    lastUiMs = now;
    drawScreen();
  }
  delay(10);
}
