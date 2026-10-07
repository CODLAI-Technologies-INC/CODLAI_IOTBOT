/*
 * TR: IFTTT WEBHOOK - IOTBOT'tan başka servisleri tetikleyin
 *  - IOTBOT açılınca ve B3 butonuna basınca bir IFTTT Webhook olayı tetikler. IFTTT
 *    bu olayla Google Sheets'e satır ekleyebilir, Discord'a/e-postaya mesaj atabilir...
 *  - Gönderilen JSON'daki value1, value2, value3 alanlarını IFTTT applet'inizde
 *    "malzeme" (ingredient) olarak kullanabilirsiniz.
 *  - İki tetikleme arasında en az 5 saniye beklenir.
 *  - IFTTT bilgilerini nasıl alırsınız:
 *    1) https://ifttt.com/create > tetikleyici olarak Webhooks -> "Receive a web
 *       request" seçin, olay adı (Event Name) yazın (iftttEventName ile AYNI).
 *    2) "Then" kısmı için istediğiniz servisi seçin (Google Sheets, Discord, Gmail...).
 *    3) https://ifttt.com/maker_webhooks > "Documentation": örnek adresteki anahtarı
 *       (key) kopyalayıp aşağıdaki iftttWebhookKey'e yapıştırın.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help               -> komut listesi
 *      gonder / send               -> sensör değerleriyle olayı tetikle (B3 ile aynı)
 *      mesaj <metin> / msg <text>  -> value3 = sizin metniniz olarak tetikle
 *      durum  / status             -> WiFi durumu, tetikleme sayısı
 *      dil    / lang               -> dili değiştir (Türkçe <-> English)
 *  - Not: USE_IFTTT tanımlıyken WiFi otomatik açılır. İstek 1-2 sn sürer; kart bekler.
 *
 * EN: IFTTT WEBHOOK - trigger other services from the IOTBOT
 *  - The IOTBOT triggers an IFTTT Webhook event at startup and when B3 is pressed.
 *    IFTTT can then add a row to Google Sheets, send a Discord message/email...
 *  - The value1, value2, value3 fields of the JSON can be used as "ingredients" in
 *    your IFTTT applet.
 *  - At least 5 seconds must pass between two triggers.
 *  - How to get your IFTTT details:
 *    1) https://ifttt.com/create > pick Webhooks -> "Receive a web request" as the
 *       trigger, write an Event Name (the SAME as iftttEventName).
 *    2) Choose any service for the "Then" part (Google Sheets, Discord, Gmail...).
 *    3) https://ifttt.com/maker_webhooks > "Documentation": copy the key from the
 *       sample URL and paste it into iftttWebhookKey below.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim             -> command list
 *      send   / gonder             -> trigger the event with sensor values (same as B3)
 *      msg <text> / mesaj <metin>  -> trigger with value3 = your text
 *      status / durum              -> WiFi state, trigger count
 *      lang   / dil                -> switch language (Turkish <-> English)
 *  - Note: with USE_IFTTT, WiFi is enabled automatically. A request takes 1-2 s; the
 *    board waits meanwhile.
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. / NO extra module needed.
 */

#define USE_IFTTT   // IFTTT fonksiyonlarını açar / enables the IFTTT helpers
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// WiFi bilgileri / WiFi credentials
const char *ssid = "YOUR_WIFI_SSID";
const char *password = "YOUR_WIFI_PASSWORD";

// IFTTT Maker Webhooks bilgileri / IFTTT Maker Webhooks data
String iftttEventName = "YOUR_EVENT_NAME"; // Örnek / example: "iotbot_button"
String iftttWebhookKey = "YOUR_IFTTT_KEY"; // https://ifttt.com/maker_webhooks

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

const uint32_t kMinGapMs = 5000; // İki tetikleme arası en az / minimum time between two triggers
uint32_t lastTriggerMs = 0;
bool triggeredOnce = false;
int triggerCount = 0, okCount = 0;
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

// JSON içinde " ve \ karakterleri kaçışlı yazılmalı / " and \ must be escaped inside JSON
String jsonEscape(const String &text) {
  String out;
  for (unsigned int i = 0; i < text.length(); i++) {
    char c = text[i];
    if (c == '"' || c == '\\') out += '\\';
    out += c;
  }
  return out;
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

bool wifiOk() { return WiFi.status() == WL_CONNECTED; }

void drawScreen(const char *status) {
  char line[41];
  lcdRow(0, L("   IFTTT WEBHOOK", "   IFTTT WEBHOOK"));
  lcdRow(1, wifiOk() ? L("WiFi: bağlı", "WiFi: connected") : L("WiFi: YOK", "WiFi: NONE"));
  snprintf(line, sizeof(line), L("Tetik: %d  Tamam: %d", "Sent: %d  OK: %d"), triggerCount, okCount);
  lcdRow(2, status ? status : line);
  lcdRow(3, L("B3: olayı tetikle", "B3: trigger event"));
}

void printHelp() {
  iotbot.serialWrite(L("---- IFTTT WEBHOOK - Komutlar ----", "---- IFTTT WEBHOOK - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  gonder        : sensör değerleriyle tetikle", "  send          : trigger with sensor values"));
  iotbot.serialWrite(L("  mesaj <metin> : value3 = metniniz", "  msg <text>    : value3 = your text"));
  iotbot.serialWrite(L("  durum         : WiFi ve tetikleme bilgisi", "  status        : WiFi and trigger info"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : olayı tetikle", "  B3 button     : trigger the event"));
}

// value1/value2/value3 ile olayı tetikler / triggers the event with value1/value2/value3
void trigger(const String &v1, const String &v2, const String &v3) {
  if (!wifiOk()) {
    iotbot.serialWrite(L("WiFi bağlı değil, tetiklenemedi.", "WiFi not connected, not triggered."));
    return;
  }
  if (triggeredOnce && millis() - lastTriggerMs < kMinGapMs) {
    iotbot.serialWrite(L("Çok hızlı! 5 saniye bekleyin.", "Too fast! Wait 5 seconds."));
    return;
  }
  String payload = "{\"value1\":\"" + jsonEscape(v1) + "\",\"value2\":\"" + jsonEscape(v2) + "\",\"value3\":\"" + jsonEscape(v3) + "\"}";
  drawScreen(L("Gönderiliyor...", "Sending..."));
  iotbot.serialWrite(String(L("IFTTT tetikleniyor: ", "Triggering IFTTT: ")) + payload);
  bool ok = iotbot.triggerIFTTTEvent(iftttEventName, iftttWebhookKey, payload);
  lastTriggerMs = millis();
  triggeredOnce = true;
  triggerCount++;
  if (ok) okCount++;
  iotbot.serialWrite(ok ? L("[IFTTT] Olay teslim edildi.", "[IFTTT] Event delivered.") : L("[IFTTT] Olay gönderilemedi.", "[IFTTT] Event failed."));
  iotbot.buzzerPlayTone(ok ? 2000 : 400, ok ? 60 : 200);
  drawScreen(nullptr);
}

void sendSensorEvent() {
  String sensors = String("LDR:") + iotbot.ldrRead() + " POT:" + iotbot.potentiometerRead();
  trigger("IoTBOT", L("B3 basıldı", "B3 pressed"), sensors);
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "gonder" || word == "send") {
    sendSensorEvent();
  } else if (word == "mesaj" || word == "msg" || word == "message") {
    String text = argText();
    if (text.length() == 0) iotbot.serialWrite(L("Kullanım: mesaj <metin>", "Usage: msg <text>"));
    else trigger("IoTBOT", L("Mesaj", "Message"), text);
  } else if (word == "durum" || word == "status") {
    iotbot.serialWrite(String("WiFi: ") + (wifiOk() ? String(L("bağlı, IP ", "connected, IP ")) + iotbot.wifiGetIPAddress() : String(L("YOK", "NONE"))));
    iotbot.serialWrite(String(L("Olay adı: ", "Event name: ")) + iftttEventName + L("   Tetikleme: ", "   Triggers: ") + triggerCount +
                       L("   Başarılı: ", "   Successful: ") + okCount);
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
  lcdRow(0, L("   IFTTT WEBHOOK", "   IFTTT WEBHOOK"));
  lcdRow(1, L("WiFi'ye bağlanıyor", "Connecting WiFi"));

  iotbot.wifiStartAndConnect(ssid, password);

  if (wifiOk()) {
    // Açılış bildirimi: otomasyonlar IOTBOT'un çevrimiçi olduğunu öğrensin
    // Boot notification so the automations know the IOTBOT is online
    trigger("IoTBOT", L("Açılış", "Boot"), L("Sistem çevrimiçi", "System Online"));
  } else {
    iotbot.serialWrite(L("WiFi bağlantısı başarısız! SSID/şifreyi kontrol edin.", "WiFi connection failed! Check SSID/password."));
    iotbot.buzzerPlayTone(400, 400);
  }
  drawScreen(nullptr);
  printHelp();
}

void loop() {
  // B3 -> olayı tetikle (sadece basıldığı an). B1 yerine B3: WiFi açıkken B1/B2 güvenilir değil.
  // B3 -> trigger the event (on press only). B3 instead of B1: B1/B2 are unreliable with WiFi on.
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) sendSensorEvent();
  lastB3 = b3;

  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
  delay(10);
}
