// TR: GERCEK PROJE - Akilli Termostat ve Fan. NTC sicaklik sensoru odanin
// sicakligini olcer, potansiyometre ile istediginiz HEDEF sicakligi (20-40 C)
// secersiniz. Sicaklik hedefi gecince DC motor (fan) calisir; ne kadar cok
// isinirsa fan o kadar hizli doner (orantili kontrol). Fan, hedefin 0.5 C
// altina inene kadar kapanmaz ("histerezis") - boylece sicaklik tam hedefin
// uzerindeyken fan surekli ac-kapa yapip titremez. Sensor okumasi son 10
// olcumun ortalamasi alinarak yumusatilir. LCD: sicaklik, hedef ve fan hizi.
// EN: A REAL PROJECT - Smart Thermostat and Fan. The NTC temperature sensor
// measures the room, and you pick the TARGET temperature (20-40 C) with the
// potentiometer. When the temperature goes above the target the DC motor
// (fan) runs; the hotter it gets, the faster the fan spins (proportional
// control). The fan only turns off once it is 0.5 C below the target
// ("hysteresis") - so it does not keep switching on and off when the
// temperature sits right at the target. The sensor reading is smoothed by
// averaging the last 10 samples. LCD: temperature, target and fan speed.
//
// Baglanti / Wiring: NTC sicaklik sensorunu P5 soketine (IO33) takin, DC
// motoru (fan) P6 soketine takin. Potansiyometre kart uzerindedir. P2/P3'e
// ve trafik lambasina modul takmayin (motor IO26/IO27'yi kullanir). / Plug
// the NTC temperature sensor into socket P5 (IO33) and the DC motor (fan)
// into socket P6. The potentiometer is on the board. Do not plug modules
// into P2/P3 or use the traffic light (the motor uses IO26/IO27).

#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define NTC_PIN IO33 // NTC sensoru: P5 (analog okuma icin guvenli pin) / NTC sensor: P5 (safe analog pin)

namespace {
  constexpr float kMinTargetC = 20.0f;         // En dusuk hedef / lowest target
  constexpr float kMaxTargetC = 40.0f;         // En yuksek hedef / highest target
  constexpr float kHysteresisC = 0.5f;         // Kapanma payi / turn-off margin
  constexpr float kFullSpeedAboveC = 5.0f;     // Hedef+5 C'de tam hiz / full speed at target+5 C
  constexpr int kMinFanPwm = 90;               // Fanin donebildigi en dusuk PWM / lowest PWM that spins the fan
  constexpr int kSamples = 10;                 // Ortalama icin olcum sayisi / samples to average
  constexpr uint32_t kSampleIntervalMs = 100;  // 10 x 100 ms = 1 sn'lik ortalama / 1 s average
  constexpr uint32_t kUiIntervalMs = 500;      // LCD yenileme araligi / LCD refresh interval

  float samples[kSamples];
  int sampleIndex = 0;
  int sampleCount = 0;
  float sampleSum = 0;
  float potSmooth = 0;
  bool fanOn = false;
  int fanPwm = 0;
  bool sensorOk = true;
  uint32_t lastSampleMs = 0;
  uint32_t lastUiMs = 0;
}

// Hareketli ortalama: en eski olcumu cikar, yenisini ekle.
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

// Potansiyometre -> 20.0 ... 40.0 C, 0.5 derecelik adimlarla.
// Potentiometer -> 20.0 ... 40.0 C in 0.5 degree steps.
float readTarget() {
  potSmooth = potSmooth * 0.8f + iotbot.potentiometerRead() * 0.2f;
  int halfSteps = (int)((potSmooth / 4095.0f) * (kMaxTargetC - kMinTargetC) * 2 + 0.5f);
  return kMinTargetC + halfSteps * 0.5f;
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.moduleDCMotorStop();
  potSmooth = iotbot.potentiometerRead();
  iotbot.lcdClear();
  iotbot.lcdWriteFixed(0, turkish ? "  AKILLI TERMOSTAT" : "  SMART THERMOSTAT");
  iotbot.serialWrite(turkish ? "Akilli termostat hazir." : "Smart thermostat ready.");
}

void loop() {
  uint32_t now = millis();
  float target = readTarget();

  if (now - lastSampleMs >= kSampleIntervalMs) {
    lastSampleMs = now;
    float t = iotbot.moduleNtcTempRead(NTC_PIN);
    // Sensor takili degilse formul -273 C ya da "nan" gibi sacma degerler verir.
    // If the sensor is unplugged the formula gives nonsense like -273 C or "nan".
    sensorOk = !isnan(t) && t > -30.0f && t < 120.0f;
    if (sensorOk) addSample(t);
    float temp = currentTemp();

    // Histerezisli karar: hedefin ustunde AC, hedef-0.5'in altinda KAPAT.
    // Decision with hysteresis: ON above target, OFF below target-0.5.
    bool wasOn = fanOn;
    if (!sensorOk || sampleCount == 0) fanOn = false;  // Guvenli taraf / fail safe
    else if (!fanOn && temp > target) fanOn = true;
    else if (fanOn && temp < target - kHysteresisC) fanOn = false;

    if (fanOn) {
      // Orantili hiz: hedefin ne kadar ustundeysek o kadar hizli.
      // Proportional speed: the further above target, the faster.
      float over = constrain(temp - target, 0.0f, kFullSpeedAboveC);
      fanPwm = kMinFanPwm + (int)(over / kFullSpeedAboveC * (255 - kMinFanPwm));
      iotbot.moduleDCMotorGOClockWise(fanPwm);
    } else {
      fanPwm = 0;
      iotbot.moduleDCMotorStop();
    }
    if (fanOn != wasOn) {
      iotbot.serialWrite(fanOn ? (turkish ? "Fan acildi." : "Fan on.")
                               : (turkish ? "Fan kapandi." : "Fan off."));
    }
  }

  if (now - lastUiMs >= kUiIntervalMs) {
    lastUiMs = now;
    char line[21];
    if (sensorOk && sampleCount > 0) {
      snprintf(line, sizeof(line), turkish ? "Sicaklik: %5.1f C" : "Temp:     %5.1f C", currentTemp());
    } else {
      snprintf(line, sizeof(line), "%s", turkish ? "Sensor yok? (P5)" : "No sensor? (P5)");
    }
    iotbot.lcdWriteFixed(1, line);
    snprintf(line, sizeof(line), turkish ? "Hedef:    %5.1f C" : "Target:   %5.1f C", target);
    iotbot.lcdWriteFixed(2, line);
    if (fanOn) snprintf(line, sizeof(line), turkish ? "Fan: %%%d CALISIYOR" : "Fan: %d%% RUNNING", fanPwm * 100 / 255);
    else snprintf(line, sizeof(line), "%s", turkish ? "Fan: KAPALI" : "Fan: OFF");
    iotbot.lcdWriteFixed(3, line);
  }
  delay(10);
}
