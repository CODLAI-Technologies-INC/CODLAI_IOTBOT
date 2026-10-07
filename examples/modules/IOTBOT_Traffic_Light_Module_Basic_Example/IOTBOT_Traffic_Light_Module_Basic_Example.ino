/*
 * TR: TRAFİK IŞIĞI MODÜLÜ - Otomatik demo + Manuel kontrol
 *  - Açılışta OTOMATİK mod gerçek bir trafik lambası gibi çalışır:
 *      KIRMIZI (5 sn) -> KIRMIZI+SARI (1,5 sn, "hazırlan") -> YEŞİL (5 sn)
 *      -> YEŞİL yanıp söner (3 sn) -> SARI (2 sn) -> tekrar KIRMIZI
 *  - B3 butonuna basınca MANUEL moda geçer: B1 her basışta sıradaki ışığı yakar
 *    (kırmızı -> sarı -> yeşil -> kapalı), potansiyometreyi çevirerek de ışık
 *    seçebilirsiniz. B3'e tekrar basınca otomatik moda döner.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim   / help     -> komut listesi
 *      oto      / auto     -> otomatik mod
 *      manuel   / manual   -> manuel mod (B1 / potansiyometre)
 *      kirmizi  / red      -> kırmızıyı yak (manuel moda geçer)
 *      sari     / yellow   -> sarıyı yak (manuel moda geçer)
 *      yesil    / green    -> yeşili yak (manuel moda geçer)
 *      kapat    / off      -> tüm ışıkları söndür (manuel moda geçer)
 *      dil      / lang     -> dili değiştir (Türkçe <-> English)
 *
 * EN: TRAFFIC LIGHT MODULE - Automatic demo + Manual control
 *  - At startup AUTO mode works like a real traffic light:
 *      RED (5 s) -> RED+YELLOW (1.5 s, "get ready") -> GREEN (5 s)
 *      -> GREEN blinking (3 s) -> YELLOW (2 s) -> RED again
 *  - Press B3 to switch to MANUAL mode: each B1 press lights the next lamp
 *    (red -> yellow -> green -> off), you can also pick a lamp by turning the
 *    potentiometer. Press B3 again to go back to auto mode.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help     / yardim   -> command list
 *      auto     / oto      -> auto mode
 *      manual   / manuel   -> manual mode (B1 / potentiometer)
 *      red      / kirmizi  -> red on (switches to manual)
 *      yellow   / sari     -> yellow on (switches to manual)
 *      green    / yesil    -> green on (switches to manual)
 *      off      / kapat    -> all lights off (switches to manual)
 *      lang     / dil      -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Trafik ışığı modülünü kendi soketine takın. Kırmızı = IO32,
 * Sarı = IO26, Yeşil = IO25 kullanılır; bu pinlere başka modül takmayın.
 * / Plug the traffic light module into its socket. Red = IO32, Yellow = IO26,
 * Green = IO25 are used; do not plug other modules into these pins.
 */

#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Otomatik mod adımları / Auto mode phases
enum Phase { PH_RED, PH_RED_YELLOW, PH_GREEN, PH_GREEN_BLINK, PH_YELLOW, PH_COUNT };
const uint32_t PHASE_MS[PH_COUNT] = {5000, 1500, 5000, 3000, 2000}; // Süreler / durations

// Manuel mod ışıkları / Manual mode lights
enum Light { LIGHT_RED, LIGHT_YELLOW, LIGHT_GREEN, LIGHT_OFF, LIGHT_COUNT };

bool manualMode = false;   // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
int phase = PH_RED;        // Otomatik moddaki adım / phase in auto mode
uint32_t phaseStartMs = 0; // Adımın başladığı an / when the phase started
int manualLight = LIGHT_OFF;
bool redOn = false, yellowOn = false, greenOn = false; // Şu an yanan ışıklar / lights on right now
uint32_t lastScreenMs = 0;
bool lastB3 = false;
bool lastB1 = false;
uint32_t lastB1Ms = 0;     // B1 için basit sıçrama önleme / simple debounce for B1
int lastPotLight = -1;     // Potansiyometrenin son seçtiği ışık / light last picked by the pot

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
// Işık yardımcıları / Light helpers
// ---------------------------------------------------------------------------
// Sadece değişiklik varsa modüle yazar / writes to the module only when something changed
void writeLights(bool red, bool yellow, bool green) {
  if (red == redOn && yellow == yellowOn && green == greenOn) return;
  redOn = red; yellowOn = yellow; greenOn = green;
  iotbot.moduleTraficLightWrite(red, yellow, green);
  lastScreenMs = 0; // Ekranı hemen güncelle / update the screen right away
}

const char *phaseName(int p) {
  switch (p) {
    case PH_RED:         return L("KIRMIZI", "RED");
    case PH_RED_YELLOW:  return L("KIRMIZI+SARI", "RED+YELLOW");
    case PH_GREEN:       return L("YEŞİL", "GREEN");
    case PH_GREEN_BLINK: return L("YEŞİL yanıp söner", "GREEN blinking");
    default:             return L("SARI", "YELLOW");
  }
}

const char *lightName(int light) {
  switch (light) {
    case LIGHT_RED:    return L("KIRMIZI", "RED");
    case LIGHT_YELLOW: return L("SARI", "YELLOW");
    case LIGHT_GREEN:  return L("YEŞİL", "GREEN");
    default:           return L("KAPALI", "OFF");
  }
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
// lcdWriteFixedTxt Türkçe harfleri (ç, ğ, ı, ö, ş, ü) LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- TRAFİK IŞIĞI - Komutlar ----", "---- TRAFFIC LIGHT - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oto           : otomatik mod", "  auto          : auto mode"));
  iotbot.serialWrite(L("  manuel        : manuel mod (B1 / pot)", "  manual        : manual mode (B1 / pot)"));
  iotbot.serialWrite(L("  kirmizi       : kırmızıyı yak", "  red           : red on"));
  iotbot.serialWrite(L("  sari          : sarıyı yak", "  yellow        : yellow on"));
  iotbot.serialWrite(L("  yesil         : yeşili yak", "  green         : green on"));
  iotbot.serialWrite(L("  kapat         : hepsini söndür", "  off           : all off"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : OTOMATİK <-> MANUEL", "  B3 button     : AUTO <-> MANUAL"));
  iotbot.serialWrite(L("  B1 butonu     : sıradaki ışık (manuel)", "  B1 button     : next light (manual)"));
}

void drawStaticScreen() {
  lcdRow(0, L("    TRAFİK IŞIĞI", "   TRAFFIC LIGHT"));
  lcdRow(3, manualMode ? L("B1/Pot:ışık B3:oto", "B1/Pot:light B3:auto") : L("B3: manuel kontrol", "B3: manual control"));
  lastScreenMs = 0; // Değerleri hemen çiz / draw the values right away
}

void startPhase(int p) {
  phase = p;
  phaseStartMs = millis();
  iotbot.serialWrite(String(L("Otomatik: ", "Auto: ")) + phaseName(phase));
}

void setManualLight(int light) {
  manualLight = light;
  writeLights(light == LIGHT_RED, light == LIGHT_YELLOW, light == LIGHT_GREEN);
  iotbot.serialWrite(String(L("Işık: ", "Light: ")) + lightName(light));
}

void setMode(bool manual) {
  manualMode = manual;
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  iotbot.serialWrite(manual ? L(">> MANUEL mod: B1 = sıradaki ışık, pot = ışık seç.", ">> MANUAL mode: B1 = next light, pot = pick a light.")
                            : L(">> OTOMATİK mod: gerçek trafik lambası sırası.", ">> AUTO mode: real traffic light sequence."));
  if (manual) {
    lastPotLight = map(iotbot.potentiometerRead(), 0, 4096, 0, LIGHT_COUNT); // Pot çevrilince devreye girer / pot acts when turned
    setManualLight(LIGHT_OFF);
  } else {
    startPhase(PH_RED); // Güvenli başlangıç: kırmızı / safe start: red
  }
  drawStaticScreen();
}

// Işık komutu: otomatikteyse önce manuele geç / light command: switch to manual first if in auto
void lightCommand(int light) {
  if (!manualMode) setMode(true);
  setManualLight(light);
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "oto" || cmd == "otomatik" || cmd == "auto") {
    setMode(false);
  } else if (cmd == "manuel" || cmd == "manual") {
    setMode(true);
  } else if (cmd == "kirmizi" || cmd == "red") {
    lightCommand(LIGHT_RED);
  } else if (cmd == "sari" || cmd == "yellow") {
    lightCommand(LIGHT_YELLOW);
  } else if (cmd == "yesil" || cmd == "green") {
    lightCommand(LIGHT_GREEN);
  } else if (cmd == "kapat" || cmd == "off") {
    lightCommand(LIGHT_OFF);
  } else if (cmd == "dil" || cmd == "lang" || cmd == "language") {
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
  iotbot.moduleTraficLightWrite(false, false, false); // Hepsi sönük başla / start all off
  iotbot.lcdClear();
  drawStaticScreen();
  iotbot.serialWrite(L("Trafik ışığı testi başladı.", "Traffic light test started."));
  printHelp();
  startPhase(PH_RED);
}

void loop() {
  uint32_t now = millis();

  // 1) B3 -> mod değiştir (sadece basıldığı an) / B3 -> toggle mode (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) setMode(!manualMode);
  lastB3 = b3;

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  if (manualMode) {
    // 3a) B1 -> sıradaki ışık / B1 -> next light
    bool b1 = iotbot.button1Read();
    if (b1 && !lastB1 && now - lastB1Ms > 200) {
      lastB1Ms = now;
      setManualLight((manualLight + 1) % LIGHT_COUNT);
    }
    lastB1 = b1;

    // 3b) Pot 4 bölgeye ayrılır; sadece bölge değişince ışık değişir.
    // 3b) The pot is split into 4 zones; the light changes only when the zone changes.
    int potLight = map(iotbot.potentiometerRead(), 0, 4096, 0, LIGHT_COUNT);
    if (potLight != lastPotLight) {
      lastPotLight = potLight;
      setManualLight(potLight);
    }
  } else {
    // 3c) Otomatik: süre dolunca sonraki adıma geç / Auto: next phase when the time is up
    now = millis();
    if (now - phaseStartMs >= PHASE_MS[phase]) startPhase((phase + 1) % PH_COUNT);
    uint32_t elapsed = millis() - phaseStartMs;
    switch (phase) {
      case PH_RED:         writeLights(true, false, false); break;
      case PH_RED_YELLOW:  writeLights(true, true, false); break;
      case PH_GREEN:       writeLights(false, false, true); break;
      case PH_GREEN_BLINK: writeLights(false, false, (elapsed / 500) % 2 == 0); break; // 0,5 sn yan / 0,5 sn sön
      default:             writeLights(false, true, false); break;
    }
  }

  // 4) LCD (200 ms'de bir, titremesiz) / LCD (every 200 ms, no flicker)
  if (millis() - lastScreenMs >= 200) {
    lastScreenMs = millis();
    char line[41];
    snprintf(line, sizeof(line), L("Mod: %s", "Mode: %s"), manualMode ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO"));
    lcdRow(1, line);
    if (manualMode) {
      snprintf(line, sizeof(line), L("Işık: %s", "Light: %s"), lightName(manualLight));
    } else {
      // Adım adı ve kalan saniye / phase name and seconds left
      uint32_t elapsed = millis() - phaseStartMs;
      unsigned long left = (elapsed >= PHASE_MS[phase]) ? 0 : (PHASE_MS[phase] - elapsed + 999) / 1000;
      snprintf(line, sizeof(line), "%s %lus", phase == PH_GREEN_BLINK ? L("YEŞİL (yanıp)", "GREEN (blink)") : phaseName(phase), left);
    }
    lcdRow(2, line);
  }
}
