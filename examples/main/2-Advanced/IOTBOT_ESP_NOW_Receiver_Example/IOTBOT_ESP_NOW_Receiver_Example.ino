/*
 * TR: ESP-NOW ALICI (RECEIVER) - kendi veri yapımızla
 *  - IOTBOT_ESP_NOW_Sender_Example.ino'nun gönderdiği paketleri dinler: bir metin,
 *    bir tam sayı, bir sıcaklık (ondalık) ve bir durum (açık/kapalı).
 *  - Gelen her paket LCD'de ve Seri Monitör'de gösterilir, kısa bir bip duyulur.
 *  - Önemli: ESP-NOW alma fonksiyonu (OnDataRecv) WiFi görevinin içinde çalışır;
 *    orada LCD/buzzer/delay KULLANMAYIN. Biz sadece veriyi kopyalayıp bir bayrak
 *    kaldırıyoruz, ekrana yazma işini loop() yapıyor.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help    -> komut listesi
 *      durum  / status  -> MAC adresi, alınan paket sayısı, son veri
 *      sessiz / mute    -> paket gelince bip sesi aç/kapat
 *      dil    / lang    -> dili değiştir (Türkçe <-> English)
 *
 * EN: ESP-NOW RECEIVER - with our own data structure
 *  - Listens to the packets sent by IOTBOT_ESP_NOW_Sender_Example.ino: a text, an
 *    integer, a temperature (decimal) and a status (on/off).
 *  - Each packet is shown on the LCD and the Serial Monitor, with a short beep.
 *  - Important: the ESP-NOW receive function (OnDataRecv) runs inside the WiFi task;
 *    do NOT use the LCD/buzzer/delay there. We only copy the data and raise a flag,
 *    loop() does the displaying.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim  -> command list
 *      status / durum   -> MAC address, packets received, last data
 *      mute   / sessiz  -> beep on/off when a packet arrives
 *      lang   / dil     -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. Gönderici kart, bu kartın MAC adresini (LCD'de
 * ve Seri Monitör'de yazar) ya da yayın adresini (FF:FF:FF:FF:FF:FF) kullanmalıdır.
 * NO extra module needed. The sender must use this board's MAC address (shown on the
 * LCD and Serial Monitor) or the broadcast address (FF:FF:FF:FF:FF:FF).
 */
#define USE_ESPNOW
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Gelen veri yapısı (gönderici ile AYNI olmalı) / incoming data structure (must match the sender)
typedef struct struct_message {
  char msg[32];
  int value;
  float temp;
  bool status;
} struct_message;

struct_message incomingData;          // Son gelen paket / last packet received
volatile bool packetArrived = false;  // Callback'ten loop'a haber / news from the callback to loop
volatile int packetLen = 0;
int packetCount = 0;
bool beepOn = true;

// Veri alındığında WiFi görevi bu fonksiyonu çağırır: SADECE kopyala ve bayrak kaldır.
// The WiFi task calls this when data arrives: ONLY copy and raise a flag.
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingBytes, int len) {
  size_t n = (size_t)len < sizeof(incomingData) ? (size_t)len : sizeof(incomingData); // Taşmayı önle / prevent overflow
  memset(&incomingData, 0, sizeof(incomingData));
  memcpy(&incomingData, incomingBytes, n);
  incomingData.msg[sizeof(incomingData.msg) - 1] = '\0'; // Metin her zaman bitsin / always terminate the text
  packetLen = len;
  packetArrived = true;
}

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "SESSİZ" -> "sessiz"
// Lower-cases and simplifies Turkish letters: "SESSİZ" -> "sessiz"
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

void drawWaitingScreen() {
  String mac = WiFi.macAddress();
  lcdRow(0, L("ALICI MODU (RX)", "RECEIVER MODE (RX)"));
  lcdRow(1, L("MAC adresim:", "My MAC address:"));
  lcdRow(2, mac.c_str()); // 17 karakter / 17 characters
  lcdRow(3, L("Veri bekleniyor...", "Waiting for data..."));
}

void showPacket() {
  char line[41];
  snprintf(line, sizeof(line), L("Veri alındı! #%d", "Data received! #%d"), packetCount);
  lcdRow(0, line);
  lcdRow(1, incomingData.msg);
  snprintf(line, sizeof(line), L("Değer: %d  %s", "Value: %d  %s"), incomingData.value, incomingData.status ? L("AÇIK", "ON") : L("KAPALI", "OFF"));
  lcdRow(2, line);
  snprintf(line, sizeof(line), L("Sıcaklık: %.1f C", "Temperature: %.1f C"), incomingData.temp);
  lcdRow(3, line);
}

void printPacket() {
  iotbot.serialWrite(String(L("Paket #", "Packet #")) + packetCount + "  (" + packetLen + L(" bayt)", " bytes)"));
  iotbot.serialWrite(String(L("  Mesaj   : ", "  Message : ")) + incomingData.msg);
  iotbot.serialWrite(String(L("  Değer   : ", "  Value   : ")) + incomingData.value);
  iotbot.serialWrite(String(L("  Sıcaklık: ", "  Temp    : ")) + String(incomingData.temp, 1) + " C");
  iotbot.serialWrite(String(L("  Durum   : ", "  Status  : ")) + (incomingData.status ? L("AÇIK", "ON") : L("KAPALI", "OFF")));
}

void printHelp() {
  iotbot.serialWrite(L("---- ESP-NOW ALICI - Komutlar ----", "---- ESP-NOW RECEIVER - Commands ----"));
  iotbot.serialWrite(L("  yardim : bu liste", "  help   : this list"));
  iotbot.serialWrite(L("  durum  : MAC adresi, paket sayısı, son veri", "  status : MAC address, packet count, last data"));
  iotbot.serialWrite(L("  sessiz : bip sesini aç/kapat", "  mute   : beep on/off"));
  iotbot.serialWrite(L("  dil    : English'e geç", "  lang   : switch to Turkish"));
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "durum" || cmd == "status") {
    iotbot.serialWrite(String(L("Benim MAC adresim: ", "My MAC address: ")) + WiFi.macAddress());
    iotbot.serialWrite(String(L("Alınan paket: ", "Packets received: ")) + packetCount);
    if (packetCount > 0) printPacket();
  } else if (cmd == "sessiz" || cmd == "mute") {
    beepOn = !beepOn;
    iotbot.serialWrite(beepOn ? L("Bip sesi AÇIK.", "Beep ON.") : L("Bip sesi KAPALI.", "Beep OFF."));
  } else if (cmd == "dil" || cmd == "lang" || cmd == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    if (packetCount > 0) showPacket(); else drawWaitingScreen();
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);

  iotbot.lcdShowLoading(L("Alıcı başlatılıyor", "Starting receiver"));
  iotbot.buzzerPlayTone(1000, 200);

  iotbot.initESPNow();                // ESP-NOW başlat / initialize ESP-NOW
  iotbot.registerOnRecv(OnDataRecv);  // Alıcı fonksiyonunu kaydet / register the receive callback

  iotbot.lcdClear();
  drawWaitingScreen();
  iotbot.serialWrite(String(L("Alıcı hazır. Benim MAC adresim: ", "Receiver ready. My MAC address: ")) + WiFi.macAddress());
  printHelp();
}

void loop() {
  // Yeni paket geldiyse göster (asıl iş burada) / show a new packet (the real work happens here)
  if (packetArrived) {
    packetArrived = false;
    packetCount++;
    showPacket();
    printPacket();
    if (beepOn) {
      iotbot.buzzerPlayTone(1500, 60);
      iotbot.buzzerPlayTone(2000, 60);
    }
  }

  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
}
