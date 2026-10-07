/*
 * TR: E-POSTA GÖNDERİCİ
 *  - IOTBOT WiFi'ye bağlanır ve açılışta "IOTBOT çevrimiçi" e-postası gönderir.
 *  - B3 butonuna basınca (veya "gonder" yazınca) ışık ve potansiyometre değerlerini
 *    içeren bir e-posta gönderir. Gereksiz e-posta yağmuru olmasın diye iki gönderim
 *    arasında en az 30 saniye beklenir.
 *  - Gereksinimler: WiFi bilgileri ve gönderen e-posta hesabının "Uygulama Şifresi".
 *    Gmail için 2 Adımlı Doğrulamayı açıp bir "Uygulama Şifresi" oluşturun (normal
 *    şifreniz ÇALIŞMAZ).
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help               -> komut listesi
 *      gonder / send               -> sensör değerlerini e-postayla gönder
 *      mesaj <metin> / msg <text>  -> kendi metninizi e-postayla gönderin
 *      durum  / status             -> WiFi durumu, gönderilen e-posta sayısı
 *      dil    / lang               -> dili değiştir (e-postalar da o dilde gider)
 *  - Not: Gönderim birkaç saniye sürer; o sırada kart başka işe bakmaz.
 *
 * EN: EMAIL SENDER
 *  - The IOTBOT connects to WiFi and sends an "IOTBOT is online" email at startup.
 *  - Pressing B3 (or typing "send") emails the light and potentiometer values. To
 *    avoid an email flood, at least 30 seconds must pass between two emails.
 *  - Requirements: WiFi credentials and an "App Password" of the sender account.
 *    For Gmail, turn on 2-Step Verification and create an "App Password" (your
 *    normal password does NOT work).
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim             -> command list
 *      send   / gonder             -> email the sensor values
 *      msg <text> / mesaj <metin>  -> email your own text
 *      status / durum              -> WiFi state, number of emails sent
 *      lang   / dil                -> switch language (emails use it too)
 *  - Note: sending takes a few seconds; the board does nothing else meanwhile.
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. / NO extra module needed.
 */

#define USE_EMAIL // E-posta özelliği (WiFi'yi de açar) / Email feature (also enables WiFi)
#define USE_WIFI
// ESP Mail Client dosya sistemi için LittleFS'i kullanır; bu satır derleyicinin onu
// bulmasını sağlar. / ESP Mail Client uses LittleFS for its file system; this line
// makes sure the build finds it.
#include <LittleFS.h>
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// WiFi bilgileri / WiFi credentials
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// E-posta bilgileri / Email credentials
#define SMTP_HOST "smtp.gmail.com"
#define SMTP_PORT 465
#define AUTHOR_EMAIL "YOUR_EMAIL@gmail.com"
#define AUTHOR_PASSWORD "YOUR_APP_PASSWORD"
#define RECIPIENT_EMAIL "RECIPIENT_EMAIL@example.com"

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

const uint32_t kMinGapMs = 30000; // İki e-posta arası en az / minimum time between two emails
uint32_t lastSendMs = 0;
bool sentOnce = false;
int emailCount = 0;
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

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

bool wifiOk() { return WiFi.status() == WL_CONNECTED; }

void drawScreen(const char *status) {
  char line[41];
  lcdRow(0, L("  E-POSTA GÖNDERİCİ", "    EMAIL SENDER"));
  lcdRow(1, wifiOk() ? L("WiFi: bağlı", "WiFi: connected") : L("WiFi: YOK", "WiFi: NONE"));
  snprintf(line, sizeof(line), L("Gönderilen: %d", "Sent: %d"), emailCount);
  lcdRow(2, status ? status : line);
  lcdRow(3, L("B3: e-posta gönder", "B3: send an email"));
}

void printHelp() {
  iotbot.serialWrite(L("---- E-POSTA - Komutlar ----", "---- EMAIL - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  gonder        : sensör değerlerini gönder", "  send          : email the sensor values"));
  iotbot.serialWrite(L("  mesaj <metin> : kendi metninizi gönderin", "  msg <text>    : email your own text"));
  iotbot.serialWrite(L("  durum         : WiFi ve gönderim bilgisi", "  status        : WiFi and sending info"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : sensör değerlerini gönder", "  B3 button     : email the sensor values"));
}

// E-posta gönderir (WiFi ve 30 sn kuralını kontrol eder) / sends an email (checks WiFi and the 30 s rule)
void sendMail(const String &subject, const String &body) {
  if (!wifiOk()) {
    iotbot.serialWrite(L("WiFi bağlı değil, e-posta gönderilemedi.", "WiFi not connected, email not sent."));
    return;
  }
  if (sentOnce && millis() - lastSendMs < kMinGapMs) {
    iotbot.serialWrite(String(L("Biraz bekleyin: ", "Please wait: ")) + ((kMinGapMs - (millis() - lastSendMs)) / 1000) +
                       L(" sn sonra tekrar gönderebilirsiniz.", " s until the next email."));
    return;
  }
  drawScreen(L("Gönderiliyor...", "Sending..."));
  iotbot.serialWrite(String(L("E-posta gönderiliyor: ", "Sending email: ")) + subject);
  // Argümanlar: SMTP sunucu, port, gönderen, uygulama şifresi, alıcı, konu, mesaj
  // Arguments: SMTP host, port, sender, app password, recipient, subject, message
  iotbot.sendEmail(SMTP_HOST, SMTP_PORT, AUTHOR_EMAIL, AUTHOR_PASSWORD, RECIPIENT_EMAIL, subject, body);
  lastSendMs = millis();
  sentOnce = true;
  emailCount++;
  iotbot.buzzerPlayTone(2000, 80);
  iotbot.serialWrite(L("İşlem tamam - gelen kutunuzu kontrol edin.", "Done - check your inbox."));
  drawScreen(nullptr);
}

void sendSensorMail() {
  String body = String(L("IOTBOT sensör raporu\n", "IOTBOT sensor report\n")) +
                L("Işık (LDR): ", "Light (LDR): ") + iotbot.ldrRead() + "\n" +
                L("Potansiyometre: ", "Potentiometer: ") + iotbot.potentiometerRead() + "\n" +
                L("Çalışma süresi: ", "Uptime: ") + (millis() / 1000) + L(" sn", " s");
  sendMail(L("IOTBOT sensör raporu", "IOTBOT sensor report"), body);
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "gonder" || word == "send") {
    sendSensorMail();
  } else if (word == "mesaj" || word == "msg" || word == "message") {
    String text = argText();
    if (text.length() == 0) iotbot.serialWrite(L("Kullanım: mesaj <metin>", "Usage: msg <text>"));
    else sendMail(L("IOTBOT'tan mesaj", "Message from IOTBOT"), text);
  } else if (word == "durum" || word == "status") {
    iotbot.serialWrite(String("WiFi: ") + (wifiOk() ? L("bağlı, IP ", "connected, IP ") + iotbot.wifiGetIPAddress() : String(L("YOK", "NONE"))));
    iotbot.serialWrite(String(L("Gönderilen e-posta: ", "Emails sent: ")) + emailCount + L("   Alıcı: ", "   Recipient: ") + RECIPIENT_EMAIL);
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
  iotbot.begin();             // IoTBot başlatılıyor / Initialize IoTBot
  iotbot.serialStart(115200); // Seri haberleşme / Serial communication
  iotbot.lcdClear();
  lcdRow(0, L("  E-POSTA GÖNDERİCİ", "    EMAIL SENDER"));
  lcdRow(1, L("WiFi'ye bağlanıyor", "Connecting WiFi"));
  iotbot.serialWrite(L("E-posta gönderici örneği başladı.", "Email sender example started."));

  // WiFi'ye bağlan / connect to WiFi
  iotbot.wifiStartAndConnect(WIFI_SSID, WIFI_PASSWORD);

  if (wifiOk()) {
    // Açılış e-postası / startup email
    sendMail(L("IOTBOT çevrimiçi", "IOTBOT is online"), L("Merhaba! IOTBOT çalışmaya başladı.", "Hello! The IOTBOT has started."));
  } else {
    iotbot.serialWrite(L("WiFi bağlantısı başarısız! SSID/şifreyi kontrol edin.", "WiFi connection failed! Check SSID/password."));
    iotbot.buzzerPlayTone(400, 400);
  }
  drawScreen(nullptr);
  printHelp();
}

void loop() {
  // B3 -> sensör e-postası (sadece basıldığı an) / B3 -> sensor email (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) sendSensorMail();
  lastB3 = b3;

  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
  delay(10);
}
