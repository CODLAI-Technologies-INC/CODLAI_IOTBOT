/*
 * TR: IoTBot'A HOŞ GELDİNİZ - İlk örnek: "IoTBot Disko!"
 *  Bu örnek kartın temel parçalarını birlikte tanıtır: LCD, buzzer, röle,
 *  LED'ler, B1/B3 butonları ve potansiyometre.
 *  - Açılışta OTOMATİK mod: disko şovu! IO25-IO26-IO27-IO32-IO33 hatlarındaki
 *    LED'ler sırayla yanar, buzzer bir ritim çalar, röle her 8 adımda bir "tık"
 *    eder. LCD adımı, LED'i ve notayı gösterir.
 *  - B3 butonuna basınca MANUEL moda geçer: potansiyometreyi çevirerek LED'i
 *    ve sesin inceliğini seçin, B1'e basılı tuttukça buzzer çalar.
 *    B3'e tekrar basınca otomatik moda döner.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim   / help      -> komut listesi
 *      oto      / auto      -> otomatik mod (disko şovu)
 *      manuel   / manual    -> manuel mod (potansiyometre + B1)
 *      led 1-5              -> o LED'i yak (manuel moda geçer)
 *      nota 440 / tone 440  -> 440 Hz'lik kısa bir ses çal (manuel moda geçer)
 *      role     / relay     -> röleyi bir kez "tık"lat
 *      dil      / lang      -> dili değiştir (Türkçe <-> English)
 *
 * EN: WELCOME TO IoTBot - First example: "IoTBot Disco!"
 *  This example shows the board's basic parts together: LCD, buzzer, relay,
 *  LEDs, buttons B1/B3 and the potentiometer.
 *  - At startup AUTO mode: a disco show! The LEDs on the IO25-IO26-IO27-IO32-
 *    IO33 lines light up in turn, the buzzer plays a rhythm and the relay
 *    "clicks" every 8 steps. The LCD shows the step, the LED and the note.
 *  - Press B3 to switch to MANUAL mode: turn the potentiometer to pick the LED
 *    and the pitch, hold B1 to play the buzzer. Press B3 again to go back to auto.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help     / yardim    -> command list
 *      auto     / oto       -> auto mode (disco show)
 *      manual   / manuel    -> manual mode (potentiometer + B1)
 *      led 1-5              -> light that LED (switches to manual)
 *      tone 440 / nota 440  -> play a short 440 Hz sound (switches to manual)
 *      relay    / role      -> make the relay "click" once
 *      lang     / dil       -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül gerekmez. Bu örnek IO25, IO26, IO27, IO32, IO33
 *   hatlarını çıkış yapar: modül soketlerine (P1-P5) modül TAKMAYIN.
 *   No extra module needed. This example drives the IO25, IO26, IO27, IO32,
 *   IO33 lines as outputs: do NOT plug modules into the module sockets (P1-P5).
 */

#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Disko şovu: notalar (Hz) ve süreleri (ms) / Disco show: notes (Hz) and durations (ms)
const int NOTES[16] = {600, 800, 600, 700, 900, 700, 800, 600, 700, 600, 500, 700, 800, 600, 500, 600};
const int TIMES[16] = {150, 150, 150, 200, 150, 200, 150, 300, 200, 150, 150, 200, 250, 150, 100, 400};
const int LED_PINS[5] = {IO25, IO26, IO27, IO32, IO33}; // LED'li hatlar / lines with LEDs

bool manualMode = false;   // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
int step = 0;              // Şovdaki adım (0-15) / step in the show (0-15)
uint32_t stepStartMs = 0;  // Adımın başladığı an / when the step started
bool stepStarted = false;
uint32_t pauseUntilMs = 0; // Şov sonunda kısa mola / short rest at the end of the show
int ledIndex = -1;         // Yanan LED (0-4), -1 = hiçbiri / LED that is on (0-4), -1 = none
int buzzFreq = 0;          // Çalan ses (Hz), 0 = sessiz / sound playing (Hz), 0 = silent
uint32_t toneOffMs = 0;    // "nota" komutunun bitiş zamanı / end of the "tone" command sound
uint32_t relayOffMs = 0;   // Rölenin bırakılacağı an (0 = kapalı) / when to release the relay (0 = off)
int lastPotLed = -1;       // Potansiyometrenin son seçtiği LED / LED last picked by the pot
bool lastB3 = false;
uint32_t lastManualMs = 0;
uint32_t lastScreenMs = 0;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "AÇI" -> "aci"
// Lower-cases and simplifies Turkish letters: "AÇI" -> "aci"
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
// Donanım yardımcıları / Hardware helpers
// ---------------------------------------------------------------------------
// Sadece verilen LED'i yakar (-1 = hepsini söndür). / Lights only the given LED (-1 = all off).
void setLed(int index) {
  if (index == ledIndex) return;
  if (ledIndex >= 0) iotbot.digitalWritePin(LED_PINS[ledIndex], LOW);
  ledIndex = index;
  if (ledIndex >= 0) iotbot.digitalWritePin(LED_PINS[ledIndex], HIGH);
}

// Buzzer'ı bekletmeden çalar / susturur (0 = sus). / Plays / silences the buzzer without waiting (0 = silent).
void buzz(int freq) {
  if (freq == buzzFreq) return;
  buzzFreq = freq;
  if (freq > 0) iotbot.buzzerStart(freq);
  else iotbot.buzzerStop();
}

void relayClick() {
  iotbot.relayWrite(true);
  relayOffMs = millis() + 60; // 60 ms sonra bırak / release after 60 ms
}

void allOff() {
  setLed(-1);
  buzz(0);
  toneOffMs = 0;
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
// lcdWriteFixedTxt Türkçe harfleri (ç, ğ, ı, ö, ş, ü) LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- IoTBot DİSKO - Komutlar ----", "---- IoTBot DISCO - Commands ----"));
  iotbot.serialWrite(L("  yardim     : bu liste", "  help       : this list"));
  iotbot.serialWrite(L("  oto        : otomatik mod (disko şovu)", "  auto       : auto mode (disco show)"));
  iotbot.serialWrite(L("  manuel     : manuel mod (pot + B1)", "  manual     : manual mode (pot + B1)"));
  iotbot.serialWrite(L("  led 1-5    : o LED'i yak", "  led 1-5    : light that LED"));
  iotbot.serialWrite(L("  nota 440   : 440 Hz ses çal", "  tone 440   : play a 440 Hz sound"));
  iotbot.serialWrite(L("  role       : röleyi tıklat", "  relay      : click the relay"));
  iotbot.serialWrite(L("  dil        : English'e geç", "  lang       : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu  : OTOMATİK <-> MANUEL", "  B3 button  : AUTO <-> MANUAL"));
}

void drawStaticScreen() {
  lcdRow(0, L("   IoTBot DİSKO!", "   IoTBot DISCO!"));
  lcdRow(3, manualMode ? L("Pot+B1:çal  B3:oto", "Pot+B1:play B3:auto") : L("B3: manuel kontrol", "B3: manual control"));
  lastScreenMs = 0; // Değerleri hemen çiz / draw the values right away
}

void setMode(bool manual) {
  manualMode = manual;
  allOff();
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  iotbot.serialWrite(manual ? L(">> MANUEL mod: potansiyometre ile LED'i seçin, B1 ile çalın.", ">> MANUAL mode: pick the LED with the potentiometer, play with B1.")
                            : L(">> OTOMATİK mod: disko şovu başlıyor!", ">> AUTO mode: the disco show starts!"));
  step = 0;
  stepStarted = false;
  pauseUntilMs = 0;
  lastPotLed = -1; // Manuelde pot hemen geçerli olsun / pot takes effect immediately in manual
  drawStaticScreen();
}

int potLed() { return constrain(map(iotbot.potentiometerRead(), 0, 4095, 0, 5), 0, 4); }
int potFreq() { return map(iotbot.potentiometerRead(), 0, 4095, 300, 1500); }

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oto" || word == "otomatik" || word == "auto") {
    setMode(false);
  } else if (word == "manuel" || word == "manual") {
    setMode(true);
  } else if (word == "led" && hasValue) {
    if (!manualMode) setMode(true);
    setLed(constrain(value, 1, 5) - 1);
    lastPotLed = potLed(); // Pot hemen ezmesin / so the pot does not override it right away
    iotbot.serialWrite(String("LED ") + (ledIndex + 1) + " (IO" + LED_PINS[ledIndex] + L(") yandı.", ") is on."));
  } else if ((word == "nota" || word == "tone") && hasValue) {
    if (!manualMode) setMode(true);
    buzz(constrain(value, 100, 5000));
    toneOffMs = millis() + 300;
    iotbot.serialWrite(String(L("Çalan nota: ", "Playing: ")) + buzzFreq + " Hz");
  } else if (word == "role" || word == "relay") {
    relayClick();
    iotbot.serialWrite(L("Röle tık!", "Relay click!"));
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
// OTOMATİK: disko şovu (delay() yok, B3 ve seri port hep hemen tepki verir)
// AUTO: disco show (no delay(), B3 and the serial port always react instantly)
// Her adım: LED yanar + nota çalar (TIMES[step] ms), sonra aynı süre sessiz, LED söner.
// Each step: LED on + note plays (TIMES[step] ms), then silent for the same time, LED off.
// ---------------------------------------------------------------------------
void runDisco(uint32_t now) {
  if (now < pauseUntilMs) return;

  if (!stepStarted) {
    if (step == 0) iotbot.serialWrite(L("Ritim başlıyor!", "The beat starts!"));
    stepStarted = true;
    stepStartMs = now;
    setLed(step % 5);
    buzz(NOTES[step]);
    if (step % 8 == 0) relayClick(); // Her 8 adımda röle tık / relay click every 8 steps
    char msg[64];
    snprintf(msg, sizeof(msg), L("Adım %2d: LED IO%d, nota %d Hz", "Step %2d: LED IO%d, note %d Hz"), step + 1, LED_PINS[step % 5], NOTES[step]);
    iotbot.serialWrite(msg);
    lastScreenMs = 0;
  }

  uint32_t elapsed = now - stepStartMs;
  if (elapsed >= (uint32_t)TIMES[step]) buzz(0);  // Nota bitti / the note ended
  if (elapsed >= 2UL * TIMES[step]) {             // Adım bitti / the step ended
    setLed(-1);
    stepStarted = false;
    if (++step >= 16) {
      step = 0;
      pauseUntilMs = now + 2000; // Şov bitti, 2 sn mola / show over, 2 s rest
      iotbot.serialWrite(L("Şov bitti! 2 saniye sonra tekrar...", "Show over! Again in 2 seconds..."));
    }
  }
}

// MANUEL: pot LED'i ve sesin inceliğini seçer, B1 basılıyken buzzer çalar.
// MANUAL: the pot picks the LED and the pitch, the buzzer plays while B1 is held.
void runManual(uint32_t now) {
  if (now - lastManualMs < 30) return;
  lastManualMs = now;

  // Pot sadece gerçekten çevrilince LED'i değiştirir (seri "led" komutu hemen ezilmesin).
  // The pot changes the LED only when really turned (so a serial "led" command is not overridden).
  int p = potLed();
  if (p != lastPotLed) {
    lastPotLed = p;
    setLed(p);
  }

  if (iotbot.button1Read()) buzz(potFreq()); // B1 basılı: çal / B1 held: play
  else if (now >= toneOffMs) buzz(0);         // Bırakıldı ve "nota" bitti: sus / released and "tone" ended: silent
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();             // IoTBot başlatılıyor / Initialize IoTBot
  iotbot.serialStart(115200); // Seri haberleşme / Serial communication
  iotbot.lcdClear();
  drawStaticScreen();
  iotbot.serialWrite(L("Merhaba! IoTBot Disko başladı.", "Hello! IoTBot Disco started."));
  printHelp();
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

  // 3) Seçili modu çalıştır / Run the selected mode
  if (manualMode) runManual(now);
  else runDisco(now);

  // 4) Röle "tık"ını bitir / Finish the relay "click"
  if (relayOffMs != 0 && now >= relayOffMs) {
    iotbot.relayWrite(false);
    relayOffMs = 0;
  }

  // 5) LCD (200 ms'de bir, titremesiz) / LCD (every 200 ms, no flicker)
  if (now - lastScreenMs >= 200) {
    lastScreenMs = now;
    char line[41];
    snprintf(line, sizeof(line), L("Mod: %s", "Mode: %s"), manualMode ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO"));
    lcdRow(1, line);
    if (manualMode) {
      if (ledIndex >= 0) snprintf(line, sizeof(line), L("LED:IO%d  Ses:%4dHz", "LED:IO%d Tone:%4dHz"), LED_PINS[ledIndex], potFreq());
      else snprintf(line, sizeof(line), L("Ses:%4dHz", "Tone:%4dHz"), potFreq());
    } else {
      snprintf(line, sizeof(line), L("Adım:%2d  Nota:%4dHz", "Step:%2d  Note:%4dHz"), step + 1, NOTES[step]);
    }
    lcdRow(2, line);
  }
}
