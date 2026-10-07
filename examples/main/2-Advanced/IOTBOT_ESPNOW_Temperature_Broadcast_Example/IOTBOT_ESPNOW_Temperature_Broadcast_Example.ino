/*
 * TR: KABLOSUZ AKILLI EV FİKRİ - Sıcaklık yayını
 *  - IOTBOT'a takılı DHT sıcaklık sensörünün değerini her saniye ESP-NOW ile yayınlar
 *    (broadcast). Bu tek başına bir şey yapmaz ama aynı odadaki BAŞKA kartlar bu veriyi
 *    dinleyip kendi kararlarını verebilir - ör. "sıcaklık yükselince vantilatörü aç".
 *  - Bkz. MINIBOT_ESPNOW_Fan_Control_Reactive_Example.ino ve ROLEBOT_ESPNOW_Fan_Control_
 *    Reactive_Example.ino: onları çalıştırın, DHT sensörünü elinizle ısıtın, uzaktaki
 *    röle/vantilatör kendiliğinden çalışacak!
 *  - Sensör okunamazsa (takılı değilse) yanlış değer yayınlanmaz, uyarı yazılır.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help    -> komut listesi
 *      oku    / read    -> sıcaklığı şimdi yazdır
 *      test 35          -> sensör yerine 35 °C yayınla (alıcıyı denemek için)
 *      test             -> deneme bitti, gerçek sensöre dön
 *      durum  / status  -> MAC adresi, son değer, gönderilen paket
 *      dil    / lang    -> dili değiştir (Türkçe <-> English)
 *
 * EN: A WIRELESS SMART HOME IDEA - Temperature broadcast
 *  - Broadcasts the reading of the DHT temperature sensor on the IOTBOT over ESP-NOW
 *    every second. By itself this does nothing, but OTHER boards in the room can
 *    listen to it and decide on their own - e.g. "turn on the fan when it gets hot".
 *  - See MINIBOT_ESPNOW_Fan_Control_Reactive_Example.ino and ROLEBOT_ESPNOW_Fan_Control_
 *    Reactive_Example.ino: run one of them, warm the DHT sensor with your hand and the
 *    remote relay/fan turns on by itself!
 *  - If the sensor cannot be read (not plugged in), no wrong value is broadcast.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim  -> command list
 *      read   / oku     -> print the temperature now
 *      test 35          -> broadcast 35 °C instead of the sensor (to try a receiver)
 *      test             -> trial over, back to the real sensor
 *      status / durum   -> MAC address, last value, packets sent
 *      lang   / dil     -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: DHT sensörünü P1-P5 soketlerinden BİRİNE takın ve aşağıdaki
 * DHT_PIN değerini o soketin sinyaline göre ayarlayın. / Plug the DHT sensor into ONE
 * of the P1-P5 sockets and set DHT_PIN below to match that socket's signal.
 */

#define USE_ESPNOW
#define USE_DHT
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

#define DHT_PIN IO27 // DHT sensörünün bağlı olduğu pin / pin the DHT sensor is connected to
// Desteklenen pinler / Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Herkese / to everyone

const uint32_t kSendIntervalMs = 1000;
uint32_t lastSendMs = 0;
uint32_t packetCount = 0;
int lastTemp = -999;
bool testMode = false;  // true = sensör yerine deneme değeri / true = trial value instead of the sensor
int testTemp = 35;
bool warnedSensor = false;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "DURUM" -> "durum"
// Lower-cases and simplifies Turkish letters: "DURUM" -> "durum"
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

void drawStaticScreen() {
  lcdRow(0, L("SICAKLIK YAYINI", "TEMPERATURE BCAST"));
  lcdRow(2, L("Uzaktaki kartlar", "Remote boards can"));
  lcdRow(3, L("bunu dinleyebilir", "listen to this"));
}

void printHelp() {
  iotbot.serialWrite(L("---- SICAKLIK YAYINI - Komutlar ----", "---- TEMPERATURE BROADCAST - Commands ----"));
  iotbot.serialWrite(L("  yardim  : bu liste", "  help    : this list"));
  iotbot.serialWrite(L("  oku     : sıcaklığı yazdır", "  read    : print the temperature"));
  iotbot.serialWrite(L("  test 35 : sensör yerine 35 °C yayınla", "  test 35 : broadcast 35 °C instead of the sensor"));
  iotbot.serialWrite(L("  test    : gerçek sensöre dön", "  test    : back to the real sensor"));
  iotbot.serialWrite(L("  durum   : MAC, son değer, paket sayısı", "  status  : MAC, last value, packet count"));
  iotbot.serialWrite(L("  dil     : English'e geç", "  lang    : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oku" || word == "read") {
    int t = iotbot.moduleDhtTempReadC(DHT_PIN);
    if (t == -999) iotbot.serialWrite(L("Sensör okunamadı! DHT_PIN'i ve bağlantıyı kontrol edin.", "Sensor read failed! Check DHT_PIN and the wiring."));
    else iotbot.serialWrite(String(L("Sıcaklık: ", "Temperature: ")) + t + " °C");
  } else if (word == "test") {
    testMode = hasValue;
    if (hasValue) testTemp = constrain(value, -40, 80);
    iotbot.serialWrite(testMode ? String(L("DENEME: sensör yerine ", "TRIAL: broadcasting ")) + testTemp + L(" °C yayınlanıyor.", " °C instead of the sensor.")
                                : String(L("Deneme bitti, gerçek sensör kullanılıyor.", "Trial over, using the real sensor.")));
  } else if (word == "durum" || word == "status") {
    iotbot.serialWrite(String(L("Benim MAC adresim: ", "My MAC address: ")) + WiFi.macAddress());
    iotbot.serialWrite(String(L("Son değer: ", "Last value: ")) + (lastTemp == -999 ? String(L("okunamadı", "no reading")) : String(lastTemp) + " °C") +
                       (testMode ? L(" (deneme)", " (trial)") : "") + L("   Gönderilen paket: ", "   Packets sent: ") + packetCount);
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
  iotbot.initESPNow();
  iotbot.lcdClear();
  drawStaticScreen();
  iotbot.serialWrite(L("Sıcaklık yayını başladı.", "Temperature broadcast started."));
  printHelp();
}

void loop() {
  if (millis() - lastSendMs >= kSendIntervalMs) {
    lastSendMs = millis();
    int tempC = testMode ? testTemp : iotbot.moduleDhtTempReadC(DHT_PIN);
    lastTemp = tempC;
    char line[41];

    if (tempC == -999) {
      // Okuma hatası: alıcılar yanlış karar vermesin diye YAYINLAMA
      // Read error: do NOT broadcast, so the receivers do not decide on a wrong value
      lcdRow(1, L("Sensör okunamadı!", "Sensor read error!"));
      if (!warnedSensor) {
        warnedSensor = true;
        iotbot.serialWrite(L("UYARI: DHT okunamadı, yayın yapılmıyor. DHT_PIN'i kontrol edin.", "WARNING: DHT read failed, not broadcasting. Check DHT_PIN."));
      }
    } else {
      warnedSensor = false;
      // Paket yapısı alıcılarla AYNI kalmalı: deviceType 11, axis1 = sıcaklık (°C)
      // The packet layout must stay the SAME as on the receivers: deviceType 11, axis1 = temperature (°C)
      CodlaiESPNowMessage outgoing = {}; // Tüm alanlar sıfır / all fields zero
      outgoing.deviceType = 11;          // 11 = IOTBOT sıcaklık yayını / IOTBOT temperature broadcast
      outgoing.axis1 = tempC;            // Sıcaklık (°C) / temperature (°C)
      outgoing.axis2 = 0;
      outgoing.axis3 = 0;
      outgoing.gripper = 0;
      outgoing.action = 0;
      iotbot.sendESPNow(broadcastAddress, (const uint8_t *)&outgoing, sizeof(outgoing));
      packetCount++;
      snprintf(line, sizeof(line), L("Sıcaklık: %d C%s", "Temp: %d C%s"), tempC, testMode ? L(" (T)", " (T)") : "");
      lcdRow(1, line);
    }
  }

  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
}
