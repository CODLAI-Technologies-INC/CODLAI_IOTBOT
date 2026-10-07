/*
 * TR: GERÇEK PROJE - Park Sensörü. Ultrasonik mesafe sensörü bir cismi
 * (örneğin bir duvarı ya da başka bir aracı) algıladıkça buzzer GİDEREK
 * HIZLANAN bir şekilde "bip" sesi çıkarır - tıpkı arabalardaki park sensörü
 * gibi. Çok yaklaşınca (10 cm altı) ses sürekli/sabit hale gelir, LCD "DUR!"
 * yazar ve kart üzerindeki röleyi tetikler (örneğin kırmızı bir ikaz lambası
 * bağlayabilirsiniz). LCD'deki çubuk, cisim yaklaştıkça dolar.
 *  - B3 butonu sesi kapatır/açar (sessiz modda LCD ve röle çalışmaya devam eder).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim    / help          -> komut listesi
 *      sustur    / mute          -> sesi kapat
 *      ses       / sound         -> sesi aç
 *      esik 10   / threshold 10  -> "DUR!" mesafesi (cm)
 *      oku       / read          -> mesafeyi şimdi yaz
 *      dil       / lang          -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Parking Sensor. As the ultrasonic distance sensor
 * detects an object getting closer (e.g. a wall or another car), the buzzer
 * beeps FASTER AND FASTER - just like a real car's parking sensor. When very
 * close (under 10 cm) the sound becomes constant, the LCD shows "STOP!" and
 * it triggers the board's relay (you can wire a red warning lamp to it). The
 * bar on the LCD fills up as the object gets closer.
 *  - Button B3 turns the sound off/on (in silent mode the LCD and relay keep working).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help         / yardim     -> command list
 *      mute         / sustur     -> sound off
 *      sound        / ses        -> sound on
 *      threshold 10 / esik 10    -> "STOP!" distance (cm)
 *      read         / oku        -> print the distance now
 *      lang         / dil        -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ultrasonik sensörü herhangi bir P1-P5 soketine takmanıza
 * GEREK YOK - bu modül sabit pinler kullanır (TRIG=IO27, ECHO=IO32). Röle ve
 * B3 kart üzerindedir. / You do NOT need to plug the ultrasonic sensor into
 * any P1-P5 socket - this module uses fixed pins (TRIG=IO27, ECHO=IO32). The
 * relay and B3 are on the board.
 */

#include <IOTBOT.h>

IOTBOT iotbot;

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

namespace {
  constexpr int kMaxUsefulDistanceCm = 100;   // Bu mesafenin üstünde sessiz / above this: silent
  constexpr uint32_t kMeasureIntervalMs = 60; // Sensör en az 60 ms arayla ölçmeli / sensor needs 60 ms between pings
  constexpr uint32_t kBeepMs = 60;            // Bip uzunluğu / beep length
  constexpr int kBeepHz = 1800;               // Bip sesi / beep pitch
  constexpr uint32_t kUiIntervalMs = 200;     // LCD yenileme aralığı / LCD refresh interval

  enum Zone { ZONE_CLEAR, ZONE_NEAR, ZONE_STOP };
  int stopDistanceCm = 10;     // Bu mesafenin altında "DUR!" / below this: "STOP!"
  int distance = 0;            // Son geçerli ölçüm (0 = yok) / last valid reading (0 = none)
  Zone zone = ZONE_CLEAR;
  Zone printedZone = ZONE_STOP; // Seri porta en son yazılan bölge / last zone printed to serial
  bool muted = false;
  bool beepOn = false;
  uint32_t beepEdgeMs = 0;      // Bip son açıldığı/kapandığı an / last time the beep turned on/off
  uint32_t lastMeasureMs = 0;
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
// lcdWriteFixedTxt Türkçe harfleri (ç, ğ, ı, ö, ş, ü) LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- PARK SENSÖRÜ - Komutlar ----", "---- PARKING SENSOR - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  sustur / ses  : sesi kapat / aç", "  mute / sound  : sound off / on"));
  iotbot.serialWrite(L("  esik 3-50     : \"DUR!\" mesafesi (cm)", "  threshold 3-50: \"STOP!\" distance (cm)"));
  iotbot.serialWrite(L("  oku           : mesafeyi şimdi yaz", "  read          : print the distance now"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : sesi kapat / aç", "  B3 button     : sound off / on"));
}

void drawStaticScreen() {
  lcdRow(0, L("    PARK SENSÖRÜ", "   PARKING SENSOR"));
  lcdRow(3, muted ? L("Ses: KAPALI  B3:aç", "Sound: OFF  B3:on") : L("Ses: AÇIK  B3:kapat", "Sound: ON  B3:off"));
  lastUiMs = 0; // Değerleri hemen çiz / draw the values right away
}

void stopBeep() {
  iotbot.buzzerStop();
  beepOn = false;
}

void setMuted(bool m) {
  muted = m;
  if (muted) stopBeep();
  else iotbot.buzzerPlayTone(1200, 40);
  iotbot.serialWrite(muted ? L("Ses KAPALI (sessiz mod).", "Sound OFF (silent mode).") : L("Ses AÇIK.", "Sound ON."));
  drawStaticScreen();
}

void printDistance() {
  if (distance > 0) iotbot.serialWrite(String(L("Mesafe: ", "Distance: ")) + distance + " cm");
  else iotbot.serialWrite(L("Mesafe: menzil dışı / yankı yok", "Distance: out of range / no echo"));
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "sustur" || word == "mute") {
    setMuted(true);
  } else if (word == "ses" || word == "sound") {
    setMuted(false);
  } else if ((word == "esik" || word == "threshold") && hasValue) {
    stopDistanceCm = constrain(value, 3, 50);
    iotbot.serialWrite(String(L("\"DUR!\" mesafesi: ", "\"STOP!\" distance: ")) + stopDistanceCm + " cm");
  } else if (word == "oku" || word == "read") {
    printDistance();
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
  iotbot.relayWrite(false);
  iotbot.lcdClear();
  drawStaticScreen();
  iotbot.serialWrite(L("Park sensörü hazır.", "Parking sensor ready."));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) B3 -> sesi kapat/aç (sadece basıldığı an) / B3 -> sound off/on (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3 && now - lastB3Ms > 200) {
    lastB3Ms = now;
    setMuted(!muted);
  }
  lastB3 = b3;

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Ölçüm ve bölge / Measurement and zone
  if (now - lastMeasureMs >= kMeasureIntervalMs) {
    lastMeasureMs = now;
    int d = iotbot.moduleUltrasonicDistanceRead();
    distance = (d > 0 && d < 400) ? d : 0; // 0 = geçersiz ya da menzil dışı / invalid or out of range
    if (distance == 0 || distance > kMaxUsefulDistanceCm) zone = ZONE_CLEAR;
    else if (distance <= stopDistanceCm) zone = ZONE_STOP;
    else zone = ZONE_NEAR;
    iotbot.relayWrite(zone == ZONE_STOP); // Çok yakında ikaz lambası / warning lamp when very close

    if (zone != printedZone) {
      printedZone = zone;
      if (zone == ZONE_CLEAR) iotbot.serialWrite(L("Yol açık.", "Path clear."));
      else if (zone == ZONE_NEAR) iotbot.serialWrite(L("Cisim yaklaşıyor...", "Object getting closer..."));
      else iotbot.serialWrite(String(L("DUR! Mesafe: ", "STOP! Distance: ")) + distance + " cm");
    }
  }

  // 4) Ses (bloklamaz): yaklaştıkça bip araları kısalır, çok yakında sürekli ses.
  // 4) Sound (non-blocking): the gap between beeps shrinks as it gets closer, constant when very close.
  if (muted || zone == ZONE_CLEAR) {
    if (beepOn) stopBeep();
  } else if (zone == ZONE_STOP) {
    if (!beepOn) { iotbot.buzzerStart(kBeepHz); beepOn = true; }
  } else {
    uint32_t gapMs = map(distance, stopDistanceCm, kMaxUsefulDistanceCm, 60, 600);
    if (beepOn && now - beepEdgeMs >= kBeepMs) {
      stopBeep();
      beepEdgeMs = now;
    } else if (!beepOn && now - beepEdgeMs >= gapMs) {
      iotbot.buzzerStart(kBeepHz);
      beepOn = true;
      beepEdgeMs = now;
    }
  }

  // 5) LCD: mesafe ve yakınlık çubuğu (200 ms'de bir, titremesiz)
  // 5) LCD: distance and closeness bar (every 200 ms, no flicker)
  if (now - lastUiMs >= kUiIntervalMs) {
    lastUiMs = now;
    char line[41];
    if (zone == ZONE_CLEAR) snprintf(line, sizeof(line), "%s", L("    Yol açık", "    Path clear"));
    else if (zone == ZONE_STOP) snprintf(line, sizeof(line), L("   DUR!  %d cm", "   STOP!  %d cm"), distance);
    else snprintf(line, sizeof(line), L("   Mesafe: %d cm", "   Distance: %d cm"), distance);
    lcdRow(1, line);

    // Yakınlık çubuğu: cisim yaklaştıkça dolar. 0xFF = LCD'de dolu kutu.
    // Closeness bar: fills up as the object gets closer. 0xFF = full block on the LCD.
    int filled = 0;
    if (zone != ZONE_CLEAR) filled = constrain(map(distance, kMaxUsefulDistanceCm, stopDistanceCm, 1, 20), 1, 20);
    for (int i = 0; i < 20; i++) line[i] = (i < filled) ? '\xFF' : ' ';
    line[20] = '\0';
    lcdRow(2, line);
  }
  delay(5);
}
