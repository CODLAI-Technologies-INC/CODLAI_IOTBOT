// TR: GERCEK PROJE - Kablosuz Deprem Uyari Agi (VERICI). IOTBOT'a takili
// titresim sensoru sarsinti hissedince kart kendi sirenini calar, LCD'ye
// "DEPREM!" yazar ve ESP-NOW ile odadaki TUM kartlara "deprem = 1"
// mesajini yayinlar. Bir paket kaybolsa bile sorun olmasin diye mesaj 3
// saniye boyunca her 500 ms'de bir tekrarlanir. Tehlike gecince B3'e
// basin: "deprem = 0" yayinlanir ve her yer susar. Bu kodu IOTBOT'a,
// MINIBOT_ESPNOW_Earthquake_Alert_Receiver_Example.ino dosyasini bir
// MINIBOT'a ve ROLEBOT_ESPNOW_Earthquake_Alert_Receiver_Example.ino
// dosyasini bir ROLEBOT'a yukleyin - IOTBOT'u sallayinca hepsi alarma gecer!
// EN: A REAL PROJECT - Wireless Earthquake Alert Network (SENDER). When the
// vibration sensor on the IOTBOT feels shaking, the board sounds its own
// siren, shows "EARTHQUAKE!" on the LCD and broadcasts "deprem = 1" over
// ESP-NOW to ALL boards in the room. The message is repeated every 500 ms
// for 3 seconds so a lost packet does not matter. When the danger is over,
// press B3: "deprem = 0" is broadcast and everything goes quiet. Upload this
// to an IOTBOT, MINIBOT_ESPNOW_Earthquake_Alert_Receiver_Example.ino to a
// MINIBOT and ROLEBOT_ESPNOW_Earthquake_Alert_Receiver_Example.ino to a
// ROLEBOT - shake the IOTBOT and they all go into alarm!
//
// Baglanti / Wiring: Titresim sensorunu P5 soketine (IO33) takin. P5 bir
// ADC1 pinidir, ESP-NOW acikken de analog okuma dogru calisir (P1-P3
// calismaz). / Plug the vibration sensor into socket P5 (IO33). P5 is an
// ADC1 pin, so analog reads keep working while ESP-NOW is on (P1-P3 do not).

#define USE_ESPNOW
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define VIBRATION_PIN IO33 // P5 soketi / socket P5

namespace {
  constexpr int kEspNowChannel = 1;          // Alici kartlarla AYNI kanal / SAME channel as the receivers
  constexpr uint32_t kConfirmMs = 150;       // Sinyal bu kadar surmeli (gurultu filtresi) / signal must last this long (noise filter)
  constexpr int kAnalogThreshold = 2500;     // Analog deger bunu gecerse de sarsinti / analog value above this also counts
  constexpr uint32_t kRepeatEveryMs = 500;   // Mesaj tekrar araligi / message repeat interval
  constexpr uint32_t kRepeatForMs = 3000;    // Tekrar suresi / how long to keep repeating
  constexpr uint32_t kRearmMs = 3000;        // B3'ten sonra sensoru bu kadar yok say / ignore the sensor this long after B3
  constexpr uint32_t kSirenStepMs = 300;     // Siren ton degisim hizi / siren tone switch rate

  bool alarmOn = false;
  uint32_t shakeStartMs = 0;    // Sinyalin ilk goruldugu an (0 = yok) / when the signal was first seen (0 = none)
  uint32_t lastShakeMs = 0;     // Son sarsinti zamani (0 = hic olmadi) / time of the last shake (0 = never)
  uint32_t ignoreUntilMs = 2000; // Acilista sensor otursun / let the sensor settle at power-up
  int burstValue = 0;           // Tekrar tekrar yayinlanan deger / value being repeated
  uint32_t burstUntilMs = 0, lastSendMs = 0;
  uint32_t lastSirenMs = 0, lastLcdMs = 0;
  bool sirenHigh = false;
  bool lastB3 = false;

  // Satiri tam 20 karakter olacak sekilde bosluklarla doldurup yazar (titremez).
  // Pads the row to exactly 20 characters with spaces and writes it (no flicker).
  void showRow(int row, const char *text) {
    char line[21];
    snprintf(line, sizeof(line), "%-20s", text);
    iotbot.lcdWriteFixed(row, line);
  }

  void startBurst(int value) {
    burstValue = value;
    burstUntilMs = millis() + kRepeatForMs;
    lastSendMs = millis() - kRepeatEveryMs; // Ilk mesaj hemen gitsin / send the first one right away
  }

  void showCalmScreen() {
    iotbot.lcdWriteMid(turkish ? "DEPREM AGI" : "EARTHQUAKE NETWORK",
                        turkish ? "Sakin, guvenli" : "Calm, safe", "", "");
  }
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.espNowBegin(kEspNowChannel);
  showCalmScreen();
  iotbot.serialWrite(turkish ? "Deprem vericisi hazir. Sensoru sallayin!" : "Earthquake sender ready. Shake the sensor!");
}

void loop() {
  uint32_t now = millis();

  // 1) Sarsinti var mi? Dijital sinyal 150 ms surmeli YA DA analog deger esigi gecmeli.
  // 1) Is it shaking? The digital signal must last 150 ms OR the analog value must pass the threshold.
  int analogValue = iotbot.moduleVibrationAnalogRead(VIBRATION_PIN);
  bool raw = iotbot.moduleVibrationDigitalRead(VIBRATION_PIN);
  if (!raw) shakeStartMs = 0;
  else if (shakeStartMs == 0) shakeStartMs = now;
  bool shaking = (shakeStartMs != 0 && now - shakeStartMs >= kConfirmMs) || analogValue > kAnalogThreshold;

  if (shaking && now >= ignoreUntilMs) {
    lastShakeMs = now;
    if (!alarmOn) {
      alarmOn = true;
      startBurst(1);
      iotbot.lcdWriteMid(turkish ? "!!! DEPREM !!!" : "!! EARTHQUAKE !!",
                          turkish ? "Uyari yayinlandi" : "Alert broadcast", "",
                          turkish ? "B3: Tehlike gecti" : "B3: All clear");
      iotbot.serialWrite(turkish ? "DEPREM algilandi! Uyari yayinlaniyor..." : "EARTHQUAKE detected! Broadcasting alert...");
    }
  }

  // 2) B3 = "tehlike gecti": deprem = 0 yayinla, sireni sustur.
  // 2) B3 = "all clear": broadcast deprem = 0, silence the siren.
  bool b3 = iotbot.button3Read(); // true = basili / pressed
  if (b3 && !lastB3 && alarmOn) {
    alarmOn = false;
    iotbot.buzzerStop();
    startBurst(0);
    ignoreUntilMs = now + kRearmMs; // Butona basmanin sarsintisi alarmi tekrar baslatmasin / the button press itself must not re-trigger
    showCalmScreen();
    iotbot.serialWrite(turkish ? "Tehlike gecti - deprem = 0 yayinlaniyor." : "All clear - broadcasting deprem = 0.");
  }
  lastB3 = b3;

  // 3) Mesaji 3 sn boyunca her 500 ms'de bir tekrarla (kayip paket onemsiz olsun).
  // 3) Repeat the message every 500 ms for 3 s (so a lost packet does not matter).
  if (now < burstUntilMs && now - lastSendMs >= kRepeatEveryMs) {
    lastSendMs = now;
    iotbot.espNowSendNumber("deprem", burstValue);
  }

  // 4) Iki tonlu siren - buzzerStart() beklemeden calar. / Two-tone siren - buzzerStart() does not block.
  if (alarmOn && now - lastSirenMs >= kSirenStepMs) {
    lastSirenMs = now;
    sirenHigh = !sirenHigh;
    iotbot.buzzerStart(sirenHigh ? 1400 : 800);
  }

  // 5) LCD: son sarsintidan beri gecen sure (200 ms'de bir). / LCD: time since the last shake (every 200 ms).
  if (now - lastLcdMs >= 200) {
    lastLcdMs = now;
    char text[32];
    if (lastShakeMs == 0) {
      snprintf(text, sizeof(text), turkish ? "Son sarsinti: yok" : "Last shake: none");
    } else {
      uint32_t ago = (now - lastShakeMs) / 1000;
      if (ago < 60) snprintf(text, sizeof(text), turkish ? "Son: %lu sn once" : "Last: %lu s ago", (unsigned long)ago);
      else snprintf(text, sizeof(text), turkish ? "Son: %lu dk once" : "Last: %lu min ago", (unsigned long)(ago / 60));
    }
    showRow(2, text);
    if (!alarmOn) {
      snprintf(text, sizeof(text), "Sensor: %d", analogValue); // Esigi ayarlamak icin / to tune the threshold
      showRow(3, text);
    }
  }

  delay(10);
}
