/*
 * TR: GERÇEK PROJE - Akıllı Termostat ve Fan.
 *  - OTOMATİK mod (açılışta): NTC sıcaklık sensörü odanın sıcaklığını ölçer,
 *    potansiyometre ile istediğiniz HEDEF sıcaklığı (20-40 °C) seçersiniz.
 *    Sıcaklık hedefi geçince DC motor (fan) çalışır; ne kadar çok ısınırsa fan
 *    o kadar hızlı döner (orantılı kontrol). Fan, hedefin 0.5 °C altına inene
 *    kadar kapanmaz ("histerezis") - böylece sıcaklık tam hedefin üzerindeyken
 *    fan sürekli aç-kapa yapıp titremez. Sensör okuması son 10 ölçümün
 *    ortalaması alınarak yumuşatılır.
 *  - B3 butonu MANUEL moda geçer: fan hızını potansiyometre ile doğrudan siz
 *    ayarlarsınız (en altta fan durur). B3'e tekrar basınca otomatik moda döner.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim    / help        -> komut listesi
 *      oto       / auto        -> otomatik mod (termostat)
 *      manuel    / manual      -> manuel mod (pot = fan hızı)
 *      hedef 25  / target 25   -> hedef sıcaklık 25 °C (otomatik moda geçer)
 *      hiz 50    / speed 50    -> fan %50 (manuel moda geçer)
 *      dur       / stop        -> fanı durdur (manuel moda geçer)
 *      oku       / read        -> sıcaklık, hedef ve fan durumunu yaz
 *      dil       / lang        -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Smart Thermostat and Fan.
 *  - AUTO mode (at startup): the NTC temperature sensor measures the room,
 *    and you pick the TARGET temperature (20-40 °C) with the potentiometer.
 *    When the temperature goes above the target the DC motor (fan) runs; the
 *    hotter it gets, the faster the fan spins (proportional control). The fan
 *    only turns off once it is 0.5 °C below the target ("hysteresis") - so it
 *    does not keep switching on and off when the temperature sits right at
 *    the target. The sensor reading is smoothed by averaging the last 10
 *    samples.
 *  - Button B3 switches to MANUAL mode: you set the fan speed directly with
 *    the potentiometer (at the bottom the fan stops). Press B3 again to go
 *    back to auto mode.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help      / yardim      -> command list
 *      auto      / oto         -> auto mode (thermostat)
 *      manual    / manuel      -> manual mode (pot = fan speed)
 *      target 25 / hedef 25    -> target temperature 25 °C (switches to auto)
 *      speed 50  / hiz 50      -> fan at 50% (switches to manual)
 *      stop      / dur         -> stop the fan (switches to manual)
 *      read      / oku         -> print temperature, target and fan state
 *      lang      / dil         -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: NTC sıcaklık sensörünü P5 soketine (IO33) takın, DC
 * motoru (fan) P6 soketine takın. Potansiyometre ve B3 kart üzerindedir.
 * P2/P3'e ve trafik lambasına modül takmayın (motor IO26/IO27'yi kullanır).
 * / Plug the NTC temperature sensor into socket P5 (IO33) and the DC motor
 * (fan) into socket P6. The potentiometer and B3 are on the board. Do not
 * plug modules into P2/P3 or use the traffic light (the motor uses IO26/IO27).
 */

#include <IOTBOT.h>

IOTBOT iotbot;

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

#define NTC_PIN IO33 // NTC sensörü: P5 (analog okuma için güvenli pin) / NTC sensor: P5 (safe analog pin)

namespace {
  constexpr float kMinTargetC = 20.0f;         // En düşük hedef / lowest target
  constexpr float kMaxTargetC = 40.0f;         // En yüksek hedef / highest target
  constexpr float kHysteresisC = 0.5f;         // Kapanma payı / turn-off margin
  constexpr float kFullSpeedAboveC = 5.0f;     // Hedef+5 °C'de tam hız / full speed at target+5 °C
  constexpr int kMinFanPwm = 90;               // Fanın dönebildiği en düşük PWM / lowest PWM that spins the fan
  constexpr int kSamples = 10;                 // Ortalama için ölçüm sayısı / samples to average
  constexpr uint32_t kSampleIntervalMs = 100;  // 10 x 100 ms = 1 sn'lik ortalama / 1 s average
  constexpr uint32_t kUiIntervalMs = 500;      // LCD yenileme aralığı / LCD refresh interval
  constexpr int kPotDeadZone = 150;            // Manuelde pot bunun altında -> fan durur / manual: below this the fan stops
  constexpr int kPotMoveRaw = 120;             // Pot "gerçekten çevrildi" eşiği / pot "really turned" threshold

  float samples[kSamples];
  int sampleIndex = 0;
  int sampleCount = 0;
  float sampleSum = 0;
  float potSmooth = 0;
  bool manualMode = false;    // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
  float target = 25.0f;       // Hedef sıcaklık / target temperature
  int manualPercent = 0;      // Manuel fan hızı (0-100) / manual fan speed (0-100)
  // Pot, seri komuttan sonra ancak gerçekten çevrilince yeniden söz sahibi olur.
  // After a serial command the pot takes over again only when it is really turned.
  bool potActive = true;
  int potRawAtCommand = 0;
  bool fanOn = false;
  int fanPwm = 0;
  bool sensorOk = true;
  uint32_t lastSampleMs = 0;
  uint32_t lastUiMs = 0;
  bool lastB3 = false;
  uint32_t lastB3Ms = 0;
}

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "HIZ" -> "hiz"
// Lower-cases and simplifies Turkish letters: "HIZ" -> "hiz"
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
// Sensör yardımcıları / Sensor helpers
// ---------------------------------------------------------------------------
// Hareketli ortalama: en eski ölçümü çıkar, yenisini ekle.
// Moving average: remove the oldest sample, add the new one.
float addSample(float value) {
  if (sampleCount == kSamples) sampleSum -= samples[sampleIndex];
  else sampleCount++;
  samples[sampleIndex] = value;
  sampleSum += value;
  sampleIndex = (sampleIndex + 1) % kSamples;
  return sampleSum / sampleCount;
}

float currentTemp() {
  return sampleCount > 0 ? sampleSum / sampleCount : 0;
}

// Potansiyometre -> 20.0 ... 40.0 °C, 0.5 derecelik adımlarla.
// Potentiometer -> 20.0 ... 40.0 °C in 0.5 degree steps.
float readPotTarget() {
  potSmooth = potSmooth * 0.8f + iotbot.potentiometerRead() * 0.2f;
  int halfSteps = (int)((potSmooth / 4095.0f) * (kMaxTargetC - kMinTargetC) * 2 + 0.5f);
  return kMinTargetC + halfSteps * 0.5f;
}

int percentToPwm(int percent) {
  return percent <= 0 ? 0 : map(percent, 1, 100, kMinFanPwm, 255);
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
// lcdWriteFixedTxt Türkçe harfleri (ç, ğ, ı, ö, ş, ü) LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- AKILLI TERMOSTAT - Komutlar ----", "---- SMART THERMOSTAT - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oto           : otomatik mod (termostat)", "  auto          : auto mode (thermostat)"));
  iotbot.serialWrite(L("  manuel        : manuel mod (pot = fan hızı)", "  manual        : manual mode (pot = fan speed)"));
  iotbot.serialWrite(L("  hedef 20-40   : hedef sıcaklık (°C)", "  target 20-40 : target temperature (°C)"));
  iotbot.serialWrite(L("  hiz 0-100     : fan hızı yüzdesi", "  speed 0-100   : fan speed percent"));
  iotbot.serialWrite(L("  dur           : fanı durdur", "  stop          : stop the fan"));
  iotbot.serialWrite(L("  oku           : sıcaklık / hedef / fan", "  read          : temperature / target / fan"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : OTOMATİK <-> MANUEL", "  B3 button     : AUTO <-> MANUAL"));
}

void drawStaticScreen() {
  lcdRow(0, manualMode ? L("TERMOSTAT - MANUEL", "THERMOSTAT - MANUAL") : L("TERMOSTAT - OTOMATİK", "THERMOSTAT - AUTO"));
  lastUiMs = 0; // Değerleri hemen çiz / draw the values right away
}

void printStatus() {
  char msg[96];
  if (sensorOk && sampleCount > 0) {
    snprintf(msg, sizeof(msg), L("Sıcaklık: %.1f °C  Hedef: %.1f °C  Fan: %%%d  (%s)", "Temp: %.1f °C  Target: %.1f °C  Fan: %d%%  (%s)"),
             currentTemp(), target, fanPwm * 100 / 255, manualMode ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO"));
  } else {
    snprintf(msg, sizeof(msg), "%s", L("Sensör okunamıyor! NTC P5'e (IO33) takılı mı?", "Cannot read the sensor! Is the NTC in P5 (IO33)?"));
  }
  iotbot.serialWrite(msg);
}

void setMode(bool manual) {
  manualMode = manual;
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  potActive = true; // Yeni modda pot hemen geçerli olsun / the pot takes effect immediately in the new mode
  if (manual) {
    iotbot.serialWrite(L(">> MANUEL mod: fan hızını potansiyometre ile ayarlayın.", ">> MANUAL mode: set the fan speed with the potentiometer."));
  } else {
    fanOn = false;          // Termostat kararını baştan versin / let the thermostat decide again
    iotbot.serialWrite(L(">> OTOMATİK mod: fan sıcaklığa göre çalışır, pot = hedef.", ">> AUTO mode: the fan follows the temperature, pot = target."));
  }
  drawStaticScreen();
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  String valueText = hasValue ? cmd.substring(space + 1) : "";
  valueText.replace(',', '.'); // "25,5" de kabul / also accept "25,5"

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oto" || word == "otomatik" || word == "auto") {
    setMode(false);
  } else if (word == "manuel" || word == "manual") {
    setMode(true);
  } else if ((word == "hedef" || word == "target") && hasValue) {
    if (manualMode) setMode(false);
    target = constrain(valueText.toFloat(), kMinTargetC, kMaxTargetC);
    // Seri komutla verilen hedefi pot hemen ezmesin diye pot konumunu kaydet.
    // Remember the pot position so it does not override the serial target right away.
    potActive = false;
    potRawAtCommand = iotbot.potentiometerRead();
    iotbot.serialWrite(String(L("Hedef sıcaklık: ", "Target temperature: ")) + String(target, 1) + " °C");
  } else if ((word == "hiz" || word == "speed") && hasValue) {
    if (!manualMode) setMode(true);
    manualPercent = constrain(valueText.toInt(), 0, 100);
    potActive = false;
    potRawAtCommand = iotbot.potentiometerRead();
    iotbot.serialWrite(String(L("Fan hızı: %", "Fan speed: ")) + manualPercent + L("", "%"));
  } else if (word == "dur" || word == "stop") {
    if (!manualMode) setMode(true);
    manualPercent = 0;
    potActive = false;
    potRawAtCommand = iotbot.potentiometerRead();
    iotbot.serialWrite(L("Fan durduruldu.", "Fan stopped."));
  } else if (word == "oku" || word == "read") {
    printStatus();
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
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.moduleDCMotorStop(); // Fan DURUK başlar / the fan starts STOPPED
  potSmooth = iotbot.potentiometerRead();
  iotbot.lcdClear();
  drawStaticScreen();
  iotbot.serialWrite(L("Akıllı termostat hazır.", "Smart thermostat ready."));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) B3 -> mod değiştir (sadece basıldığı an) / B3 -> toggle mode (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3 && now - lastB3Ms > 200) {
    lastB3Ms = now;
    setMode(!manualMode);
  }
  lastB3 = b3;

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Potansiyometre: otomatikte HEDEF, manuelde FAN HIZI. Sadece çevrilince geçerli olur,
  // böylece seri porttan verilen değeri hemen ezmez.
  // 3) Potentiometer: TARGET in auto, FAN SPEED in manual. It only counts when turned, so it
  // does not override a value given from the serial port right away.
  float potTarget = readPotTarget();
  int raw = (int)potSmooth;
  if (!potActive && abs(raw - potRawAtCommand) >= kPotMoveRaw) potActive = true;
  if (potActive) {
    if (manualMode) manualPercent = (raw < kPotDeadZone) ? 0 : constrain(map(raw, kPotDeadZone, 4095, 1, 100), 1, 100);
    else target = potTarget;
  }

  if (now - lastSampleMs >= kSampleIntervalMs) {
    lastSampleMs = now;
    float t = iotbot.moduleNtcTempRead(NTC_PIN);
    // Sensör takılı değilse formül -273 °C ya da "nan" gibi saçma değerler verir.
    // If the sensor is unplugged the formula gives nonsense like -273 °C or "nan".
    sensorOk = !isnan(t) && t > -30.0f && t < 120.0f;
    if (sensorOk) addSample(t);
    float temp = currentTemp();

    bool wasOn = fanOn;
    if (manualMode) {
      fanPwm = percentToPwm(manualPercent);
      fanOn = fanPwm > 0;
    } else {
      // Histerezisli karar: hedefin üstünde AÇ, hedef-0.5'in altında KAPAT.
      // Decision with hysteresis: ON above target, OFF below target-0.5.
      if (!sensorOk || sampleCount == 0) fanOn = false;  // Güvenli taraf / fail safe
      else if (!fanOn && temp > target) fanOn = true;
      else if (fanOn && temp < target - kHysteresisC) fanOn = false;

      if (fanOn) {
        // Orantılı hız: hedefin ne kadar üstündeysek o kadar hızlı.
        // Proportional speed: the further above target, the faster.
        float over = constrain(temp - target, 0.0f, kFullSpeedAboveC);
        fanPwm = kMinFanPwm + (int)(over / kFullSpeedAboveC * (255 - kMinFanPwm));
      } else {
        fanPwm = 0;
      }
    }
    if (fanPwm > 0) iotbot.moduleDCMotorGOClockWise(fanPwm);
    else iotbot.moduleDCMotorStop();

    if (fanOn != wasOn) {
      iotbot.serialWrite(fanOn ? L("Fan açıldı.", "Fan on.") : L("Fan kapandı.", "Fan off."));
    }
  }

  if (now - lastUiMs >= kUiIntervalMs) {
    lastUiMs = now;
    char line[41];
    if (sensorOk && sampleCount > 0) {
      snprintf(line, sizeof(line), L("Sıcaklık: %5.1f C", "Temp:     %5.1f C"), currentTemp());
    } else {
      snprintf(line, sizeof(line), "%s", L("Sensör yok? (P5)", "No sensor? (P5)"));
    }
    lcdRow(1, line);
    if (manualMode) snprintf(line, sizeof(line), "%s", L("Pot: fan  B3: oto", "Pot: fan  B3: auto"));
    else snprintf(line, sizeof(line), L("Hedef:    %5.1f C", "Target:   %5.1f C"), target);
    lcdRow(2, line);
    if (fanPwm > 0) snprintf(line, sizeof(line), L("Fan: %%%d ÇALIŞIYOR", "Fan: %d%% RUNNING"), fanPwm * 100 / 255);
    else snprintf(line, sizeof(line), "%s", L("Fan: KAPALI", "Fan: OFF"));
    lcdRow(3, line);
  }
  delay(10);
}
