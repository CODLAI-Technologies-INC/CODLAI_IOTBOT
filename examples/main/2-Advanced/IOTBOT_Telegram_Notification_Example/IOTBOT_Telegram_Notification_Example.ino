/*
 * TR: TELEGRAM BİLDİRİMİ - IOTBOT'tan telefonunuza mesaj
 *  - IOTBOT WiFi'ye bağlanır ve açılışta Telegram'a "IOTBOT çevrimiçi" mesajı gönderir.
 *  - B3 butonuna basınca (veya "gonder" yazınca) ışık ve potansiyometre değerlerini
 *    içeren bir uyarı mesajı gönderir (iki mesaj arasında en az 5 saniye).
 *  - Gereksinimler: Telegram Bot Token (@BotFather'dan alın), Chat ID (@userinfobot
 *    veya benzerinden alın), WiFi bağlantısı.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help               -> komut listesi
 *      gonder / send               -> sensör değerlerini Telegram'a gönder
 *      mesaj <metin> / msg <text>  -> kendi metninizi gönderin (ör. "mesaj Merhaba!")
 *      durum  / status             -> WiFi durumu, gönderilen mesaj sayısı
 *      dil    / lang               -> dili değiştir (mesajlar da o dilde gider)
 *  - Not: Gönderim 1-2 saniye sürer; o sırada kart bekler.
 *
 * EN: TELEGRAM NOTIFICATION - a message from the IOTBOT to your phone
 *  - The IOTBOT joins WiFi and sends "IOTBOT is online" to Telegram at startup.
 *  - Pressing B3 (or typing "send") sends an alert with the light and potentiometer
 *    values (at least 5 seconds between two messages).
 *  - Requirements: a Telegram Bot Token (get it from @BotFather), a Chat ID (get it
 *    from @userinfobot or similar), a WiFi connection.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim             -> command list
 *      send   / gonder             -> send the sensor values to Telegram
 *      msg <text> / mesaj <metin>  -> send your own text (e.g. "msg Hello!")
 *      status / durum              -> WiFi state, messages sent
 *      lang   / dil                -> switch language (messages use it too)
 *  - Note: sending takes 1-2 seconds; the board waits meanwhile.
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. / NO extra module needed.
 */

#define USE_TELEGRAM // Telegram özelliği / Telegram feature
#define USE_WIFI     // WiFi özelliği / WiFi feature
#include <IOTBOT.h>  // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// WiFi bilgileri / WiFi credentials
const char *ssid = "YOUR_WIFI_SSID";
const char *password = "YOUR_WIFI_PASSWORD";

// Telegram bilgileri / Telegram credentials
String botToken = "YOUR_BOT_TOKEN";
String chatID = "YOUR_CHAT_ID";

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

const uint32_t kMinGapMs = 5000; // İki mesaj arası en az / minimum time between two messages
uint32_t lastSendMs = 0;
bool sentOnce = false;
int messageCount = 0;
bool lastB3 = false;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
String rawLine; // Komutun orijinal hali (mesaj metni için) / original command text (for the message text)
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "GÖNDER" -> "gonder"
// Lower-cases and simplifies Turkish letters: "GÖNDER" -> "gonder"
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
    if (cmdBuffer.length() < 120) cmdBuffer += c;
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

// Not: Mesaj, internet adresinin (URL) içinde gider. sendTelegram() boşluk, Türkçe
// harf, "&" gibi karakterleri kendisi %XX biçimine çevirir; metni olduğu gibi verin
// (kendiniz kodlarsanız iki kez kodlanır ve mesajda "%20" gibi yazılar görünür).
// Note: The message travels inside the web address (URL). sendTelegram() converts
// spaces, Turkish letters, "&" etc. to %XX itself; pass the text as it is (encoding it
// yourself would encode it twice and the message would show things like "%20").

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

bool wifiOk() { return WiFi.status() == WL_CONNECTED; }

void drawScreen(const char *status) {
  char line[41];
  lcdRow(0, L("TELEGRAM BİLDİRİMİ", "TELEGRAM NOTIFY"));
  lcdRow(1, wifiOk() ? L("WiFi: bağlı", "WiFi: connected") : L("WiFi: YOK", "WiFi: NONE"));
  snprintf(line, sizeof(line), L("Gönderilen: %d", "Sent: %d"), messageCount);
  lcdRow(2, status ? status : line);
  lcdRow(3, L("B3: uyarı gönder", "B3: send an alert"));
}

void printHelp() {
  iotbot.serialWrite(L("---- TELEGRAM - Komutlar ----", "---- TELEGRAM - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  gonder        : sensör değerlerini gönder", "  send          : send the sensor values"));
  iotbot.serialWrite(L("  mesaj <metin> : kendi metninizi gönderin", "  msg <text>    : send your own text"));
  iotbot.serialWrite(L("  durum         : WiFi ve gönderim bilgisi", "  status        : WiFi and sending info"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : uyarı gönder", "  B3 button     : send an alert"));
}

void sendMessage(const String &text) {
  if (!wifiOk()) {
    iotbot.serialWrite(L("WiFi bağlı değil, mesaj gönderilemedi.", "WiFi not connected, message not sent."));
    return;
  }
  if (sentOnce && millis() - lastSendMs < kMinGapMs) {
    iotbot.serialWrite(L("Çok hızlı! Biraz bekleyip tekrar deneyin.", "Too fast! Wait a little and try again."));
    return;
  }
  drawScreen(L("Gönderiliyor...", "Sending..."));
  iotbot.serialWrite(String(L("Telegram'a gönderiliyor: ", "Sending to Telegram: ")) + text);
  iotbot.sendTelegram(botToken, chatID, text);
  lastSendMs = millis();
  sentOnce = true;
  messageCount++;
  iotbot.buzzerPlayTone(2000, 60);
  drawScreen(nullptr);
}

void sendAlert() {
  sendMessage(String(L("Uyarı: IOTBOT üzerinde B3'e basıldı! Işık: ", "Alert: B3 was pressed on the IOTBOT! Light: ")) +
              iotbot.ldrRead() + L(", Pot: ", ", Pot: ") + iotbot.potentiometerRead());
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "gonder" || word == "send") {
    sendMessage(String(L("IOTBOT raporu - Işık: ", "IOTBOT report - Light: ")) + iotbot.ldrRead() +
                L(", Pot: ", ", Pot: ") + iotbot.potentiometerRead());
  } else if (word == "mesaj" || word == "msg" || word == "message") {
    String text = argText();
    if (text.length() == 0) iotbot.serialWrite(L("Kullanım: mesaj <metin>", "Usage: msg <text>"));
    else sendMessage(text);
  } else if (word == "durum" || word == "status") {
    iotbot.serialWrite(String("WiFi: ") + (wifiOk() ? String(L("bağlı, IP ", "connected, IP ")) + iotbot.wifiGetIPAddress() : String(L("YOK", "NONE"))));
    iotbot.serialWrite(String(L("Gönderilen mesaj: ", "Messages sent: ")) + messageCount);
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    drawScreen(nullptr);
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.lcdClear();
  lcdRow(0, L("TELEGRAM BİLDİRİMİ", "TELEGRAM NOTIFY"));
  lcdRow(1, L("WiFi'ye bağlanıyor", "Connecting WiFi"));

  iotbot.wifiStartAndConnect(ssid, password); // WiFi'ye bağlan / connect to WiFi

  if (wifiOk()) {
    // Açılış mesajı / startup message
    sendMessage(L("IOTBOT'tan merhaba! Sistem çevrimiçi.", "Hello from IOTBOT! The system is online."));
  } else {
    iotbot.serialWrite(L("WiFi bağlantısı başarısız! SSID/şifreyi kontrol edin.", "WiFi connection failed! Check SSID/password."));
    iotbot.buzzerPlayTone(400, 400);
  }
  drawScreen(nullptr);
  printHelp();
}

void loop() {
  // B3 -> uyarı mesajı (sadece basıldığı an). B3, WiFi açıkken de güvenle okunur.
  // B3 -> alert message (on press only). B3 is read reliably even with WiFi on.
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) sendAlert();
  lastB3 = b3;

  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
  delay(10);
}
