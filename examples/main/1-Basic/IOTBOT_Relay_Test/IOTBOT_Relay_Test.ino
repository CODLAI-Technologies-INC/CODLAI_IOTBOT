/*
 * TR: KART ÜZERİNDEKİ RÖLE - Otomatik demo + Manuel kontrol
 *  - Açılışta röle KAPALI başlar, sonra OTOMATİK mod çalışır: röle belirli
 *    aralıklarla (varsayılan 2 sn) açılıp kapanır.
 *  - B3 butonuna basınca MANUEL moda geçer: röleyi B1 butonu ile açıp
 *    kapatırsınız (otomatik moddayken B1'e basmak da manuel moda geçirir).
 *    B3'e tekrar basınca otomatik moda döner.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim      / help          -> komut listesi
 *      oto         / auto          -> otomatik mod
 *      manuel      / manual        -> manuel mod (B1 butonu)
 *      ac          / on            -> röleyi aç (manuel moda geçer)
 *      kapat       / off           -> röleyi kapat (manuel moda geçer)
 *      degistir    / toggle        -> röleyi tersine çevir (manuel moda geçer)
 *      sure 2000   / interval 2000 -> otomatik moddaki aç/kapa süresi (ms)
 *      dil         / lang          -> dili değiştir (Türkçe <-> English)
 *
 * EN: ONBOARD RELAY - Automatic demo + Manual control
 *  - At startup the relay is OFF, then AUTO mode runs: the relay switches on
 *    and off at a fixed interval (2 s by default).
 *  - Press B3 to switch to MANUAL mode: switch the relay with button B1
 *    (pressing B1 in auto mode also switches to manual).
 *    Press B3 again to go back to auto mode.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help          / yardim      -> command list
 *      auto          / oto         -> auto mode
 *      manual        / manuel      -> manual mode (button B1)
 *      on            / ac          -> relay on (switches to manual)
 *      off           / kapat       -> relay off (switches to manual)
 *      toggle        / degistir    -> flip the relay (switches to manual)
 *      interval 2000 / sure 2000   -> on/off time in auto mode (ms)
 *      lang          / dil         -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek bağlantı gerekmez; röle IoTBot kartının üzerindedir
 * (GPIO14, iotbot.relayWrite). / No extra wiring; the relay is on the IoTBot
 * board (GPIO14, iotbot.relayWrite). Röle her değiştiğinde "tık" sesini duyarsınız.
 * / You hear a "click" every time the relay switches.
 * DİKKAT / WARNING: Şebeke gerilimini (220V) sadece bir yetişkin bağlamalıdır!
 *                   Only an adult should connect mains voltage (220V)!
 */

#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Röle mekanik bir anahtardır; çok hızlı aç/kapa onu yıpratır. En kısa süre 500 ms.
// A relay is a mechanical switch; switching too fast wears it out. Shortest time is 500 ms.
const uint32_t MIN_INTERVAL_MS = 500;
const uint32_t MAX_INTERVAL_MS = 60000;

bool manualMode = false;        // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
bool relayOn = false;           // Rölenin durumu / relay state
uint32_t intervalMs = 2000;     // Otomatik aç/kapa süresi / auto on/off time
uint32_t lastToggleMs = 0;      // Son değişim zamanı / time of the last switch
uint32_t lastScreenMs = 0;
bool lastB3 = false;
bool lastB1 = false;
uint32_t lastB1Ms = 0;          // B1 için basit sıçrama önleme / simple debounce for B1

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
  iotbot.serialWrite(L("---- RÖLE TESTİ - Komutlar ----", "---- RELAY TEST - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oto           : otomatik mod", "  auto          : auto mode"));
  iotbot.serialWrite(L("  manuel        : manuel mod (B1 butonu)", "  manual        : manual mode (button B1)"));
  iotbot.serialWrite(L("  ac / kapat    : röleyi aç / kapat", "  on / off      : relay on / off"));
  iotbot.serialWrite(L("  degistir      : röleyi tersine çevir", "  toggle        : flip the relay"));
  iotbot.serialWrite(L("  sure 500-60000: otomatik aç/kapa süresi (ms)", "  interval 500-60000: auto on/off time (ms)"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : OTOMATİK <-> MANUEL", "  B3 button     : AUTO <-> MANUAL"));
  iotbot.serialWrite(L("  B1 butonu     : röleyi aç/kapat", "  B1 button     : switch the relay"));
}

void drawStaticScreen() {
  lcdRow(0, L("     RÖLE TESTİ", "     RELAY TEST"));
  lcdRow(3, manualMode ? L("B1:aç/kapat B3:oto", "B1:on/off  B3:auto") : L("B3: manuel kontrol", "B3: manual control"));
  lastScreenMs = 0; // Değerleri hemen çiz / draw the values right away
}

void setRelay(bool on) {
  relayOn = on;
  iotbot.relayWrite(relayOn); // Kart üzerindeki röle / onboard relay
  lastToggleMs = millis();
  iotbot.serialWrite(relayOn ? L("Röle: AÇIK", "Relay: ON") : L("Röle: KAPALI", "Relay: OFF"));
  lastScreenMs = 0;
}

void setMode(bool manual) {
  manualMode = manual;
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  iotbot.serialWrite(manual ? L(">> MANUEL mod: röleyi B1 butonu veya seri komutlarla kontrol edin.", ">> MANUAL mode: control the relay with button B1 or serial commands.")
                            : L(">> OTOMATİK mod: röle kendi kendine açılıp kapanıyor.", ">> AUTO mode: the relay switches on and off by itself."));
  lastToggleMs = millis(); // Otomatik sayaç baştan / auto timer restarts
  drawStaticScreen();
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  long value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oto" || word == "otomatik" || word == "auto") {
    setMode(false);
  } else if (word == "manuel" || word == "manual") {
    setMode(true);
  } else if (word == "ac" || word == "on") {
    if (!manualMode) setMode(true);
    setRelay(true);
  } else if (word == "kapat" || word == "off") {
    if (!manualMode) setMode(true);
    setRelay(false);
  } else if (word == "degistir" || word == "toggle") {
    if (!manualMode) setMode(true);
    setRelay(!relayOn);
  } else if ((word == "sure" || word == "interval") && hasValue) {
    intervalMs = constrain(value, (long)MIN_INTERVAL_MS, (long)MAX_INTERVAL_MS);
    iotbot.serialWrite(String(L("Otomatik süre: ", "Auto interval: ")) + intervalMs + " ms");
    lastScreenMs = 0;
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    drawStaticScreen();
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();             // IoTBot başlatılıyor / Initialize IoTBot
  iotbot.serialStart(115200); // Seri haberleşme / Serial communication
  iotbot.lcdClear();
  drawStaticScreen();
  iotbot.serialWrite(L("Röle testi başladı.", "Relay test started."));
  printHelp();
  setRelay(false);            // Güvenlik: röle kapalı başlar / safety: relay starts off
}

void loop() {
  uint32_t now = millis();

  // 1) B3 -> mod değiştir (sadece basıldığı an) / B3 -> toggle mode (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) setMode(!manualMode);
  lastB3 = b3;

  // 2) B1 -> röleyi aç/kapat (otomatikteyse önce manuele geçer)
  // 2) B1 -> switch the relay (switches to manual first if in auto)
  bool b1 = iotbot.button1Read();
  if (b1 && !lastB1 && now - lastB1Ms > 200) {
    lastB1Ms = now;
    if (!manualMode) setMode(true);
    setRelay(!relayOn);
  }
  lastB1 = b1;

  // 3) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 4) Otomatik mod: süre dolunca röleyi çevir / Auto mode: flip the relay when the time is up
  if (!manualMode && millis() - lastToggleMs >= intervalMs) setRelay(!relayOn);

  // 5) LCD (200 ms'de bir, titremesiz) / LCD (every 200 ms, no flicker)
  if (millis() - lastScreenMs >= 200) {
    lastScreenMs = millis();
    char line[41];
    snprintf(line, sizeof(line), L("Mod: %s", "Mode: %s"), manualMode ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO"));
    lcdRow(1, line);
    if (manualMode) {
      snprintf(line, sizeof(line), "%s", relayOn ? L("Röle: AÇIK", "Relay: ON") : L("Röle: KAPALI", "Relay: OFF"));
    } else {
      // Durum yazısı aynı genişlikte olsun diye boşlukla doldurulur / padded so the width stays the same
      snprintf(line, sizeof(line), L("Röle: %s %lu.%lu sn", "Relay: %s  %lu.%lu s"),
               relayOn ? L("AÇIK  ", "ON ") : L("KAPALI", "OFF"), (unsigned long)(intervalMs / 1000), (unsigned long)(intervalMs % 1000 / 100));
    }
    lcdRow(2, line);
  }
}
