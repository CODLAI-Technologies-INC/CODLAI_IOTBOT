/*
 * TR: ÇOCUKLAR İÇİN BASİT KABLOSUZ MESAJLAŞMA (ESP-NOW)
 *  - Router/WiFi ağı OLMADAN iki CODLAI kartı (IOTBOT/MINIBOT/ROLEBOT) arasında
 *    "sohbet" etmenin en basit yolu: espNowSendText / espNowSendNumber ile gönder,
 *    espNowAvailable / espNowReadText / espNowReadNumber ile oku.
 *  - Bu örnek her saniye SIRAYLA kısa bir metin ("Merhaba!") ya da artan bir sayı
 *    ("sayac") yayınlar (her biri 2 saniyede bir). Başka bir karttan gelen metin ve sayılar LCD'de ve Seri
 *    Monitör'de görünür.
 *  - B3 butonu: hemen "Merhaba!" gönder.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help                    -> komut listesi
 *      mesaj <metin> / msg <text>       -> metin gönder (en fazla 31 karakter)
 *      sayi 42 / number 42              -> "sayi" adıyla 42 gönder
 *      sayi isik 512 / number light 512 -> kendi adınızla sayı gönder
 *      dur    / pause                   -> otomatik gönderimi durdur
 *      devam  / resume                  -> otomatik gönderime devam et
 *      durum  / status                  -> MAC, gönderilen/alınan sayısı
 *      dil    / lang                    -> dili değiştir (Türkçe <-> English)
 *
 * EN: SIMPLE WIRELESS MESSAGING FOR KIDS (ESP-NOW)
 *  - The simplest way for two CODLAI boards (IOTBOT/MINIBOT/ROLEBOT) to "chat" with NO
 *    router/WiFi network: send with espNowSendText / espNowSendNumber, read with
 *    espNowAvailable / espNowReadText / espNowReadNumber.
 *  - Every second this example broadcasts, IN TURN, a short text ("Hello!") or an
 *    increasing number ("sayac") (each one every 2 seconds). Texts and numbers from another board are shown on
 *    the LCD and the Serial Monitor.
 *  - B3 button: send "Hello!" right now.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim                  -> command list
 *      msg <text> / mesaj <metin>       -> send a text (max 31 characters)
 *      number 42 / sayi 42              -> send 42 with the name "sayi"
 *      number light 512 / sayi isik 512 -> send a number with your own name
 *      pause  / dur                     -> stop the automatic sending
 *      resume / devam                   -> continue the automatic sending
 *      status / durum                   -> MAC, sent/received counts
 *      lang   / dil                     -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. İki kart da AYNI kanalda (1) olmalı.
 * NO extra module needed. Both boards must use the SAME channel (1).
 */

#define USE_ESPNOW
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

uint32_t lastSendMs = 0;
bool sendTextNext = true;  // Sırayla: metin, sayı, metin... / in turn: text, number, text...
bool autoSend = true;
int counter = 0;
int sentCount = 0, receivedCount = 0;
bool lastB3 = false;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
String rawLine; // Komutun orijinal hali (mesaj metni için) / original command text (for the message text)
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "SAYI" -> "sayi"
// Lower-cases and simplifies Turkish letters: "SAYI" -> "sayi"
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
      rawLine = cmdBuffer; rawLine.trim();
      cmd = normalizeCommand(cmdBuffer);
      cmdBuffer = "";
      return true;
    }
    if (cmdBuffer.length() < 80) cmdBuffer += c;
  }
  // "Satır sonu yok" seçiliyse: 150 ms sessizlikten sonra komutu kabul et.
  // "No line ending" selected: accept the command after 150 ms of silence.
  if (cmdBuffer.length() > 0 && millis() - lastCharMs > 150) {
    rawLine = cmdBuffer; rawLine.trim();
    cmd = normalizeCommand(cmdBuffer);
    cmdBuffer = "";
    return true;
  }
  return false;
}

// Komuttan sonraki metin (orijinal harflerle) / the text after the command word (original letters)
String argText() {
  int space = rawLine.indexOf(' ');
  if (space < 0) return "";
  String t = rawLine.substring(space + 1);
  t.trim();
  return t;
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void drawStaticScreen() {
  lcdRow(0, L("BASİT MESAJLAŞMA", "SIMPLE MESSAGING"));
  lcdRow(3, L("B3: Merhaba gönder", "B3: send Hello"));
}

void printHelp() {
  iotbot.serialWrite(L("---- BASİT MESAJLAŞMA - Komutlar ----", "---- SIMPLE MESSAGING - Commands ----"));
  iotbot.serialWrite(L("  yardim          : bu liste", "  help            : this list"));
  iotbot.serialWrite(L("  mesaj <metin>   : metin gönder (en fazla 31)", "  msg <text>      : send a text (max 31)"));
  iotbot.serialWrite(L("  sayi 42         : sayı gönder", "  number 42       : send a number"));
  iotbot.serialWrite(L("  sayi isik 512   : adıyla sayı gönder", "  number light 512: send a named number"));
  iotbot.serialWrite(L("  dur / devam     : otomatik gönderim", "  pause / resume  : automatic sending"));
  iotbot.serialWrite(L("  durum           : MAC ve sayaçlar", "  status          : MAC and counters"));
  iotbot.serialWrite(L("  dil             : English'e geç", "  lang            : switch to Turkish"));
}

void sendText(const String &text) {
  iotbot.espNowSendText(text); // En fazla 31 karakter gider / at most 31 characters are sent
  sentCount++;
  iotbot.serialWrite(String(L("Metin gönderildi: ", "Text sent: ")) + text);
  String shown = String(L("> ", "> ")) + text;
  lcdRow(1, shown.substring(0, 20).c_str());
}

void sendNumber(const String &name, float value) {
  iotbot.espNowSendNumber(name, value);
  sentCount++;
  iotbot.serialWrite(String(L("Sayı gönderildi: ", "Number sent: ")) + name + " = " + String(value, 2));
  char line[41];
  snprintf(line, sizeof(line), "> %s=%g", name.c_str(), value);
  lcdRow(1, line);
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "mesaj" || word == "msg" || word == "message") {
    String text = argText();
    if (text.length() == 0) iotbot.serialWrite(L("Kullanım: mesaj <metin>", "Usage: msg <text>"));
    else sendText(text);
  } else if (word == "sayi" || word == "number") {
    String args = argText();               // "42" ya da / or "isik 512"
    int sp = args.lastIndexOf(' ');
    String name = (sp < 0) ? String("sayi") : args.substring(0, sp);
    String valueText = (sp < 0) ? args : args.substring(sp + 1);
    if (valueText.length() == 0) iotbot.serialWrite(L("Kullanım: sayi 42  ya da  sayi isik 512", "Usage: number 42  or  number light 512"));
    else sendNumber(name, valueText.toFloat());
  } else if (word == "dur" || word == "pause" || word == "stop") {
    autoSend = false;
    iotbot.serialWrite(L("Otomatik gönderim DURDU (dinleme sürüyor).", "Automatic sending PAUSED (still listening)."));
  } else if (word == "devam" || word == "resume") {
    autoSend = true;
    iotbot.serialWrite(L("Otomatik gönderim devam ediyor.", "Automatic sending resumed."));
  } else if (word == "durum" || word == "status") {
    iotbot.serialWrite(String(L("Benim MAC adresim: ", "My MAC address: ")) + WiFi.macAddress());
    iotbot.serialWrite(String(L("Gönderilen: ", "Sent: ")) + sentCount + L("   Alınan: ", "   Received: ") + receivedCount +
                       L("   Otomatik: ", "   Auto: ") + (autoSend ? L("açık", "on") : L("kapalı", "off")));
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
  iotbot.espNowBegin(1); // Kanal 1 - dinleyen diğer kartla AYNI kanal olmalı / channel 1 - must match the other board
  iotbot.lcdClear();
  drawStaticScreen();
  lcdRow(1, L("Hazır!", "Ready!"));
  iotbot.serialWrite(L("Basit ESP-NOW mesajlaşma hazır.", "Simple ESP-NOW messaging ready."));
  printHelp();
}

void loop() {
  // 1) Her 1 saniyede bir SIRAYLA metin ya da sayı gönder. İkisini aynı anda göndermiyoruz:
  //    alıcı tek bir mesaj kutusu tutar, ikinci mesaj ilkinin üzerine yazılırdı.
  // 1) Every 1 second send a text OR a number, in turn. We do not send both at once:
  //    the receiver keeps a single message box, so the second would overwrite the first.
  if (autoSend && millis() - lastSendMs >= 1000) {
    lastSendMs = millis();
    if (sendTextNext) {
      sendText(L("Merhaba!", "Hello!"));
    } else {
      counter++;
      sendNumber("sayac", counter);
    }
    sendTextNext = !sendTextNext;
  }

  // 2) B3 = hemen "Merhaba!" (sadece basıldığı an) / B3 = "Hello!" right now (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) {
    sendText(L("Merhaba!", "Hello!"));
    iotbot.buzzerPlayTone(1500, 30);
  }
  lastB3 = b3;

  // 3) Gelen mesaj var mı? (metin ya da sayı) / any incoming message? (text or number)
  if (iotbot.espNowAvailable()) {
    char line[41];
    if (iotbot.receivedData.deviceType == 20) { // 20 = metin / text
      String text = iotbot.espNowReadText();
      receivedCount++;
      iotbot.serialWrite(String(L("Metin alındı: ", "Text received: ")) + text);
      String shown = "< " + text;
      lcdRow(2, shown.substring(0, 20).c_str());
    } else {                                    // 21 = sayı / number
      String name = iotbot.espNowReadName();
      float value = iotbot.espNowReadNumber();
      receivedCount++;
      iotbot.serialWrite(String(L("Sayı alındı: ", "Number received: ")) + name + " = " + String(value, 2));
      snprintf(line, sizeof(line), "< %s=%g", name.c_str(), value);
      lcdRow(2, line);
    }
    iotbot.buzzerPlayTone(1200, 30);
  }

  // 4) Seri komutlar / serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  delay(10);
}
