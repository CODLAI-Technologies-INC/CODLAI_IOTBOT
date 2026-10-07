/*
 * TR: ESP-NOW GÖNDERİCİ (SENDER) - kendi veri yapımızla
 *  - Her 3 saniyede bir şu paketi gönderir: bir metin ("Merhaba IOTBOT"), bir değer
 *    (potansiyometreden 0-100), bir sıcaklık (24.5) ve bir durum (açık/kapalı).
 *    IOTBOT_ESP_NOW_Receiver_Example.ino yüklü kart bu paketleri gösterir.
 *  - Gönderimin başarılı olup olmadığı LCD'de görünür. (Yayın adresine gönderirken
 *    "başarılı" sadece paketin havaya çıktığı anlamına gelir.)
 *  - B3 butonu: paketi HEMEN gönder.
 *  - Önemli: gönderim sonucu fonksiyonu (OnDataSent) WiFi görevinin içinde çalışır;
 *    orada LCD/buzzer/delay KULLANMAYIN. Sonucu kaydedip loop() içinde gösteriyoruz.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help                -> komut listesi
 *      gonder / send                -> paketi hemen gönder
 *      mesaj <metin> / msg <text>   -> paketteki metni değiştir (en fazla 31 karakter)
 *      anahtar / switch             -> paketteki durumu (açık/kapalı) değiştir
 *      aralik 5 / interval 5        -> otomatik gönderim aralığı (saniye, 1-60)
 *      dur    / pause               -> otomatik gönderimi durdur
 *      devam  / resume              -> otomatik gönderime devam et
 *      durum  / status              -> son gönderim bilgisi
 *      dil    / lang                -> dili değiştir (Türkçe <-> English)
 *
 * EN: ESP-NOW SENDER - with our own data structure
 *  - Every 3 seconds it sends this packet: a text ("Merhaba IOTBOT"), a value (0-100
 *    from the potentiometer), a temperature (24.5) and a status (on/off). A board
 *    running IOTBOT_ESP_NOW_Receiver_Example.ino shows these packets.
 *  - The LCD shows whether sending succeeded. (When sending to the broadcast address,
 *    "success" only means the packet went out into the air.)
 *  - B3 button: send the packet RIGHT NOW.
 *  - Important: the send-result function (OnDataSent) runs inside the WiFi task; do
 *    NOT use the LCD/buzzer/delay there. We store the result and show it in loop().
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim              -> command list
 *      send   / gonder              -> send the packet now
 *      msg <text> / mesaj <metin>   -> change the text in the packet (max 31 chars)
 *      switch / anahtar             -> toggle the status (on/off) in the packet
 *      interval 5 / aralik 5        -> automatic send interval (seconds, 1-60)
 *      pause  / dur                 -> stop the automatic sending
 *      resume / devam               -> continue the automatic sending
 *      status / durum               -> last send information
 *      lang   / dil                 -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. / NO extra module needed.
 */
#define USE_ESPNOW
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// ALICI MAC ADRESİ / RECEIVER MAC ADDRESS
// Tüm cihazlara göndermek için (yayın): {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
// To send to all devices (broadcast): {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
// Belirli bir cihaza göndermek için o cihazın MAC adresini yazın (alıcı LCD'de gösterir).
// To send to one device, write its MAC address (the receiver shows it on its LCD).
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Gönderilecek veri yapısı (alıcı ile AYNI olmalı) / data structure to send (must match the receiver)
typedef struct struct_message {
  char msg[32];
  int value;
  float temp;
  bool status;
} struct_message;

struct_message myData;
String messageText = "Merhaba IOTBOT"; // Paketteki metin / text in the packet
bool statusFlag = true;                // Paketteki durum / status in the packet
uint32_t intervalMs = 3000;
bool autoSend = true;
uint32_t lastSendMs = 0;
int sentCount = 0, okCount = 0;
bool lastB3 = false;

volatile bool sendDone = false;   // Gönderim sonucu geldi mi? / did a send result arrive?
volatile bool sendOk = false;     // Sonuç başarılı mı? / was it successful?

// Gönderim bitince WiFi görevi bunu çağırır: SADECE sonucu kaydet.
// The WiFi task calls this when a send finishes: ONLY store the result.
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  sendOk = (status == ESP_NOW_SEND_SUCCESS);
  sendDone = true;
}

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

void printHelp() {
  iotbot.serialWrite(L("---- ESP-NOW GÖNDERİCİ - Komutlar ----", "---- ESP-NOW SENDER - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  gonder        : paketi hemen gönder", "  send          : send the packet now"));
  iotbot.serialWrite(L("  mesaj <metin> : paketteki metni değiştir", "  msg <text>    : change the text in the packet"));
  iotbot.serialWrite(L("  anahtar       : durumu (açık/kapalı) değiştir", "  switch        : toggle the status (on/off)"));
  iotbot.serialWrite(L("  aralik 5      : gönderim aralığı (sn)", "  interval 5    : send interval (s)"));
  iotbot.serialWrite(L("  dur / devam   : otomatik gönderim", "  pause / resume: automatic sending"));
  iotbot.serialWrite(L("  durum         : son gönderim bilgisi", "  status        : last send information"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : hemen gönder,  Pot: değer (0-100)", "  B3 button     : send now,  Pot: value (0-100)"));
}

void sendPacket() {
  // Verileri hazırla / prepare the data
  memset(&myData, 0, sizeof(myData));
  strncpy(myData.msg, messageText.c_str(), sizeof(myData.msg) - 1);
  myData.value = map(iotbot.potentiometerRead(), 0, 4095, 0, 100);
  myData.temp = 24.5;
  myData.status = statusFlag;

  char line[41];
  lcdRow(0, L("Gönderilen veri:", "Data sent:"));
  lcdRow(1, myData.msg);
  snprintf(line, sizeof(line), L("Değer: %d  %s", "Value: %d  %s"), myData.value, statusFlag ? L("AÇIK", "ON") : L("KAPALI", "OFF"));
  lcdRow(2, line);
  lcdRow(3, L("Gönderiliyor...", "Sending..."));

  iotbot.sendESPNow(broadcastAddress, (uint8_t *)&myData, sizeof(myData)); // Veriyi gönder / send the data
  sentCount++;
  lastSendMs = millis();
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "gonder" || word == "send") {
    sendPacket();
  } else if (word == "mesaj" || word == "msg" || word == "message") {
    String text = argText();
    if (text.length() == 0) {
      iotbot.serialWrite(L("Kullanım: mesaj <metin>", "Usage: msg <text>"));
    } else {
      messageText = text;
      iotbot.serialWrite(String(L("Yeni metin: ", "New text: ")) + messageText + (messageText.length() > 31 ? L(" (ilk 31 bayt gider)", " (first 31 bytes are sent)") : ""));
      sendPacket();
    }
  } else if (word == "anahtar" || word == "switch") {
    statusFlag = !statusFlag;
    iotbot.serialWrite(String(L("Durum: ", "Status: ")) + (statusFlag ? L("AÇIK", "ON") : L("KAPALI", "OFF")));
    sendPacket();
  } else if ((word == "aralik" || word == "interval") && hasValue) {
    intervalMs = (uint32_t)constrain(value, 1, 60) * 1000UL;
    iotbot.serialWrite(String(L("Gönderim aralığı: ", "Send interval: ")) + (intervalMs / 1000) + L(" sn", " s"));
  } else if (word == "dur" || word == "pause" || word == "stop") {
    autoSend = false;
    iotbot.serialWrite(L("Otomatik gönderim DURDU.", "Automatic sending PAUSED."));
  } else if (word == "devam" || word == "resume") {
    autoSend = true;
    iotbot.serialWrite(L("Otomatik gönderim devam ediyor.", "Automatic sending resumed."));
  } else if (word == "durum" || word == "status") {
    iotbot.serialWrite(String(L("Gönderilen: ", "Sent: ")) + sentCount + L("   Başarılı: ", "   Successful: ") + okCount +
                       L("   Aralık: ", "   Interval: ") + (intervalMs / 1000) + L(" sn", " s") +
                       L("   Otomatik: ", "   Auto: ") + (autoSend ? L("açık", "on") : L("kapalı", "off")));
    iotbot.serialWrite(String(L("Metin: ", "Text: ")) + messageText + L("   Durum: ", "   Status: ") + (statusFlag ? L("AÇIK", "ON") : L("KAPALI", "OFF")));
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);

  iotbot.lcdShowLoading(L("Gönderici başlıyor", "Starting sender"));
  iotbot.buzzerPlayTone(1000, 200);

  iotbot.initESPNow();                    // ESP-NOW başlat / initialize ESP-NOW
  esp_now_register_send_cb(OnDataSent);   // Gönderim sonucu fonksiyonunu kaydet / register the send callback

  iotbot.lcdShowStatus(L("Gönderici (TX)", "Sender (TX)"), L("Hazır", "Ready"), true);
  iotbot.serialWrite(L("Gönderici hazır.", "Sender ready."));
  printHelp();
  delay(1000);
  iotbot.lcdClear();
}

void loop() {
  // 1) Otomatik gönderim / automatic sending
  if (autoSend && millis() - lastSendMs >= intervalMs) sendPacket();

  // 2) B3 = hemen gönder (sadece basıldığı an) / B3 = send now (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) sendPacket();
  lastB3 = b3;

  // 3) Gönderim sonucunu göster / show the send result
  if (sendDone) {
    sendDone = false;
    if (sendOk) okCount++;
    lcdRow(3, sendOk ? L("Gönderim: BAŞARILI", "Send: SUCCESS") : L("Gönderim: BAŞARISIZ", "Send: FAILED"));
    iotbot.serialWrite(String(L("Gönderim sonucu: ", "Send status: ")) + (sendOk ? L("Başarılı", "Success") : L("Başarısız", "Fail")) +
                       L("   değer=", "   value=") + myData.value);
    if (sendOk) iotbot.buzzerPlayTone(2000, 40);
    else iotbot.buzzerPlayTone(500, 200);
  }

  // 4) Seri komutlar / serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
}
