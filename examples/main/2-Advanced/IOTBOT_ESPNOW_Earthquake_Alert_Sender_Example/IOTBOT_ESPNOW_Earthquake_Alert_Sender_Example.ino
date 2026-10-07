/*
 * TR: GERÇEK PROJE - Kablosuz Deprem Uyarı Ağı (VERİCİ)
 *  - IOTBOT'a takılı titreşim sensörü sarsıntı hissedince kart kendi sirenini çalar,
 *    LCD'ye "DEPREM!" yazar ve ESP-NOW ile odadaki TÜM kartlara "deprem = 1" mesajını
 *    yayınlar. Bir paket kaybolsa bile sorun olmasın diye mesaj 3 saniye boyunca her
 *    500 ms'de bir tekrarlanır.
 *  - Tehlike geçince B3'e basın: "deprem = 0" yayınlanır ve her yer susar.
 *  - Bu kodu IOTBOT'a, MINIBOT_ESPNOW_Earthquake_Alert_Receiver_Example.ino dosyasını
 *    bir MINIBOT'a ve ROLEBOT_ESPNOW_Earthquake_Alert_Receiver_Example.ino dosyasını bir
 *    ROLEBOT'a yükleyin - IOTBOT'u sallayınca hepsi alarma geçer!
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help              -> komut listesi
 *      test                       -> sarsıntı varmış gibi alarmı başlat (deneme)
 *      tamam  / clear             -> tehlike geçti (B3 ile aynı)
 *      oku    / read              -> sensör değerini şimdi yazdır
 *      esik 2500 / threshold 2500 -> analog alarm eşiği (0-4095)
 *      durum  / status            -> alarm durumu, eşik, son sarsıntı
 *      dil    / lang              -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Wireless Earthquake Alert Network (SENDER)
 *  - When the vibration sensor on the IOTBOT feels shaking, the board sounds its own
 *    siren, shows "EARTHQUAKE!" on the LCD and broadcasts "deprem = 1" over ESP-NOW to
 *    ALL boards in the room. The message is repeated every 500 ms for 3 seconds so a
 *    lost packet does not matter.
 *  - When the danger is over press B3: "deprem = 0" is broadcast and everything goes quiet.
 *  - Upload this to an IOTBOT, MINIBOT_ESPNOW_Earthquake_Alert_Receiver_Example.ino to a
 *    MINIBOT and ROLEBOT_ESPNOW_Earthquake_Alert_Receiver_Example.ino to a ROLEBOT -
 *    shake the IOTBOT and they all go into alarm!
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim            -> command list
 *      test                       -> start the alarm as if it were shaking (trial)
 *      clear  / tamam             -> all clear (same as B3)
 *      read   / oku               -> print the sensor value now
 *      threshold 2500 / esik 2500 -> analog alarm threshold (0-4095)
 *      status / durum             -> alarm state, threshold, last shake
 *      lang   / dil               -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Titreşim sensörünü P5 soketine (IO33) takın. P5 bir ADC1 pinidir,
 * ESP-NOW açıkken de analog okuma doğru çalışır (P1-P3 çalışmaz).
 * Plug the vibration sensor into socket P5 (IO33). P5 is an ADC1 pin, so analog reads
 * keep working while ESP-NOW is on (P1-P3 do not).
 * NOT / NOTE: Yayınlanan mesaj adı ("deprem") dilden bağımsızdır; alıcılar bunu bekler.
 *             The broadcast name ("deprem") does not depend on the language; receivers expect it.
 */

#define USE_ESPNOW
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

#define VIBRATION_PIN IO33 // P5 soketi / socket P5

const int kEspNowChannel = 1;         // Alıcı kartlarla AYNI kanal / SAME channel as the receivers
const uint32_t kConfirmMs = 150;      // Sinyal bu kadar sürmeli (gürültü filtresi) / signal must last this long (noise filter)
const uint32_t kRepeatEveryMs = 500;  // Mesaj tekrar aralığı / message repeat interval
const uint32_t kRepeatForMs = 3000;   // Tekrar süresi / how long to keep repeating
const uint32_t kRearmMs = 3000;       // B3'ten sonra sensörü bu kadar yok say / ignore the sensor this long after B3
const uint32_t kSirenStepMs = 300;    // Siren ton değişim hızı / siren tone switch rate

int analogThreshold = 2500;    // Analog değer bunu geçerse de sarsıntı / analog value above this also counts
bool alarmOn = false;
uint32_t shakeStartMs = 0;     // Sinyalin ilk görüldüğü an (0 = yok) / when the signal was first seen (0 = none)
uint32_t lastShakeMs = 0;      // Son sarsıntı zamanı (0 = hiç olmadı) / time of the last shake (0 = never)
uint32_t ignoreUntilMs = 2000; // Açılışta sensör otursun / let the sensor settle at power-up
int burstValue = 0;            // Tekrar tekrar yayınlanan değer / value being repeated
uint32_t burstUntilMs = 0, lastSendMs = 0;
uint32_t lastSirenMs = 0, lastLcdMs = 0;
bool sirenHigh = false;
bool lastB3 = false;
int analogValue = 0;

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
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void startBurst(int value) {
  burstValue = value;
  burstUntilMs = millis() + kRepeatForMs;
  lastSendMs = millis() - kRepeatEveryMs; // İlk mesaj hemen gitsin / send the first one right away
}

void showCalmScreen() {
  lcdRow(0, L("DEPREM AĞI", "EARTHQUAKE NETWORK"));
  lcdRow(1, L("Sakin, güvenli", "Calm, safe"));
  lastLcdMs = 0;
}

void showAlarmScreen() {
  lcdRow(0, L("!!! DEPREM !!!", "!! EARTHQUAKE !!"));
  lcdRow(1, L("Uyarı yayınlandı", "Alert broadcast"));
  lcdRow(3, L("B3: Tehlike geçti", "B3: All clear"));
  lastLcdMs = 0;
}

void printHelp() {
  iotbot.serialWrite(L("---- DEPREM VERİCİSİ - Komutlar ----", "---- EARTHQUAKE SENDER - Commands ----"));
  iotbot.serialWrite(L("  yardim      : bu liste", "  help        : this list"));
  iotbot.serialWrite(L("  test        : alarmı dene (sarsıntı varmış gibi)", "  test        : try the alarm (as if shaking)"));
  iotbot.serialWrite(L("  tamam       : tehlike geçti (B3 ile aynı)", "  clear       : all clear (same as B3)"));
  iotbot.serialWrite(L("  oku         : sensör değerini yazdır", "  read        : print the sensor value"));
  iotbot.serialWrite(L("  esik 2500   : analog alarm eşiği (0-4095)", "  threshold 2500 : analog alarm threshold (0-4095)"));
  iotbot.serialWrite(L("  durum       : alarm durumu", "  status      : alarm state"));
  iotbot.serialWrite(L("  dil         : English'e geç", "  lang        : switch to Turkish"));
}

void raiseAlarm(uint32_t now) {
  lastShakeMs = now;
  if (alarmOn) return;
  alarmOn = true;
  startBurst(1);
  showAlarmScreen();
  iotbot.serialWrite(L("DEPREM algılandı! Uyarı yayınlanıyor...", "EARTHQUAKE detected! Broadcasting alert..."));
}

void allClear(uint32_t now) {
  if (!alarmOn) return;
  alarmOn = false;
  iotbot.buzzerStop();
  startBurst(0);
  ignoreUntilMs = now + kRearmMs; // Butona basmanın sarsıntısı alarmı tekrar başlatmasın / the button press itself must not re-trigger
  showCalmScreen();
  iotbot.serialWrite(L("Tehlike geçti - deprem = 0 yayınlanıyor.", "All clear - broadcasting deprem = 0."));
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;
  uint32_t now = millis();

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "test") {
    raiseAlarm(now);
  } else if (word == "tamam" || word == "clear") {
    if (alarmOn) allClear(now);
    else iotbot.serialWrite(L("Alarm zaten kapalı.", "The alarm is already off."));
  } else if (word == "oku" || word == "read") {
    iotbot.serialWrite(String(L("Sensör analog: ", "Sensor analog: ")) + iotbot.moduleVibrationAnalogRead(VIBRATION_PIN) +
                       L("   dijital: ", "   digital: ") + (iotbot.moduleVibrationDigitalRead(VIBRATION_PIN) ? "1" : "0") +
                       L("   eşik: ", "   threshold: ") + analogThreshold);
  } else if ((word == "esik" || word == "threshold") && hasValue) {
    analogThreshold = constrain(value, 0, 4095);
    iotbot.serialWrite(String(L("Analog eşik: ", "Analog threshold: ")) + analogThreshold);
  } else if (word == "durum" || word == "status") {
    iotbot.serialWrite(String(L("Alarm: ", "Alarm: ")) + (alarmOn ? L("AÇIK", "ON") : L("kapalı", "off")) +
                       L("   Eşik: ", "   Threshold: ") + analogThreshold + L("   Kanal: ", "   Channel: ") + kEspNowChannel);
    if (lastShakeMs == 0) iotbot.serialWrite(L("Henüz sarsıntı olmadı.", "No shake yet."));
    else iotbot.serialWrite(String(L("Son sarsıntı: ", "Last shake: ")) + ((now - lastShakeMs) / 1000) + L(" sn önce", " s ago"));
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    if (alarmOn) showAlarmScreen(); else showCalmScreen();
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.espNowBegin(kEspNowChannel);
  iotbot.lcdClear();
  showCalmScreen();
  iotbot.serialWrite(L("Deprem vericisi hazır. Sensörü sallayın!", "Earthquake sender ready. Shake the sensor!"));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) Sarsıntı var mı? Dijital sinyal 150 ms sürmeli YA DA analog değer eşiği geçmeli.
  // 1) Is it shaking? The digital signal must last 150 ms OR the analog value must pass the threshold.
  analogValue = iotbot.moduleVibrationAnalogRead(VIBRATION_PIN);
  bool raw = iotbot.moduleVibrationDigitalRead(VIBRATION_PIN);
  if (!raw) shakeStartMs = 0;
  else if (shakeStartMs == 0) shakeStartMs = now;
  bool shaking = (shakeStartMs != 0 && now - shakeStartMs >= kConfirmMs) || analogValue > analogThreshold;

  if (shaking && (int32_t)(now - ignoreUntilMs) >= 0) raiseAlarm(now);

  // 2) B3 = "tehlike geçti": deprem = 0 yayınla, sireni sustur.
  // 2) B3 = "all clear": broadcast deprem = 0, silence the siren.
  bool b3 = iotbot.button3Read(); // true = basılı / pressed
  if (b3 && !lastB3) allClear(now);
  lastB3 = b3;

  // 3) Seri komutlar / serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 4) Mesajı 3 sn boyunca her 500 ms'de bir tekrarla (kayıp paket önemsiz olsun).
  // 4) Repeat the message every 500 ms for 3 s (so a lost packet does not matter).
  if ((int32_t)(now - burstUntilMs) < 0 && now - lastSendMs >= kRepeatEveryMs) {
    lastSendMs = now;
    iotbot.espNowSendNumber("deprem", burstValue); // Ad değişmemeli: alıcılar "deprem" bekler / keep the name: receivers expect "deprem"
  }

  // 5) İki tonlu siren - buzzerStart() beklemeden çalar. / Two-tone siren - buzzerStart() does not block.
  if (alarmOn && now - lastSirenMs >= kSirenStepMs) {
    lastSirenMs = now;
    sirenHigh = !sirenHigh;
    iotbot.buzzerStart(sirenHigh ? 1400 : 800);
  }

  // 6) LCD: son sarsıntıdan beri geçen süre (200 ms'de bir). / LCD: time since the last shake (every 200 ms).
  if (now - lastLcdMs >= 200) {
    lastLcdMs = now;
    char text[41];
    if (lastShakeMs == 0) {
      snprintf(text, sizeof(text), "%s", L("Son sarsıntı: yok", "Last shake: none"));
    } else {
      uint32_t ago = (now - lastShakeMs) / 1000;
      if (ago < 60) snprintf(text, sizeof(text), L("Son: %lu sn önce", "Last: %lu s ago"), (unsigned long)ago);
      else snprintf(text, sizeof(text), L("Son: %lu dk önce", "Last: %lu min ago"), (unsigned long)(ago / 60));
    }
    lcdRow(2, text);
    if (!alarmOn) {
      snprintf(text, sizeof(text), L("Sensör: %d/%d", "Sensor: %d/%d"), analogValue, analogThreshold); // Eşiği ayarlamak için / to tune the threshold
      lcdRow(3, text);
    }
  }

  delay(10);
}
