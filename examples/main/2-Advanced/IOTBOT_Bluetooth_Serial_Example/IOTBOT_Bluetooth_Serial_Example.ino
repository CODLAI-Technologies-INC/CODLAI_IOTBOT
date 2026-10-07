/*
 * TR: BLUETOOTH SERİ HABERLEŞME - Telefon <-> IOTBOT <-> Seri Monitör
 *  - Telefonunuzda bir Bluetooth terminal uygulaması açın (ör. "Serial Bluetooth
 *    Terminal"), "IOTBOT_BT" cihazına bağlanın (PIN: 1234).
 *  - Telefondan yazdığınız her metin LCD'de ve Seri Monitör'de görünür, IOTBOT
 *    "Alındı: ..." diye cevap verir.
 *  - Telefondan gönderilebilen komutlar (Türkçe veya İngilizce):
 *      merhaba / hello   -> IOTBOT selam verir ve bip sesi çıkarır
 *      bip     / beep    -> kısa bip
 *      durum   / status  -> ışık (LDR) ve potansiyometre değerini gönderir
 *      yardim  / help    -> telefon komut listesi
 *  - Seri port komutları (115200 baud):
 *      yardim / help         -> komut listesi
 *      mesaj <metin> / msg <text> -> metni telefona gönder (ör. "mesaj Selam")
 *      durum  / status       -> Bluetooth bağlantı durumu
 *      dil    / lang         -> dili değiştir (Türkçe <-> English)
 *
 * EN: BLUETOOTH SERIAL - Phone <-> IOTBOT <-> Serial Monitor
 *  - Open a Bluetooth terminal app on your phone (e.g. "Serial Bluetooth
 *    Terminal") and connect to "IOTBOT_BT" (PIN: 1234).
 *  - Every text you send from the phone is shown on the LCD and the Serial
 *    Monitor, and the IOTBOT replies "Received: ...".
 *  - Commands you can send from the phone (Turkish or English):
 *      hello  / merhaba  -> the IOTBOT says hello and beeps
 *      beep   / bip      -> short beep
 *      status / durum    -> sends the light (LDR) and potentiometer values
 *      help   / yardim   -> phone command list
 *  - Serial port commands (115200 baud):
 *      help   / yardim       -> command list
 *      msg <text> / mesaj <metin> -> send the text to the phone (e.g. "msg Hi")
 *      status / durum        -> Bluetooth connection state
 *      lang   / dil          -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. / NO extra module needed.
 */

#define USE_BLUETOOTH // Bluetooth özelliğini etkinleştir / Enable Bluetooth
#include <IOTBOT.h>   // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

#define BT_NAME "IOTBOT_BT" // Telefonda görünen ad / name shown on the phone
#define BT_PIN "1234"       // Eşleşme PIN'i / pairing PIN

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
String rawLine; // Komutun orijinal hali (mesaj metni için) / original command text (for the message text)
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

// Komuttan sonraki metin (orijinal harflerle): "mesaj Selam!" -> "Selam!"
// The text after the command word (original letters): "msg Hi!" -> "Hi!"
String argText() {
  int space = rawLine.indexOf(' ');
  if (space < 0) return "";
  String t = rawLine.substring(space + 1);
  t.trim();
  return t;
}

// ---------------------------------------------------------------------------
// Bluetooth satır okuyucu (beklemesiz) / Bluetooth line reader (non-blocking)
// iotbot.bluetoothRead() o an gelen metni tek parça döndürür (~40 ms bekler) ama
// satırlara ayırmaz; bu okuyucu karakterleri tek tek toplayıp satır satır verir,
// loop hiç durmaz.
// iotbot.bluetoothRead() returns the text that just arrived in one piece (waits
// ~40 ms) but does not split it into lines; this reader collects the characters
// one by one and hands them over line by line, so loop never stops.
// ---------------------------------------------------------------------------
String btBuffer;
uint32_t lastBtCharMs = 0;

bool readBluetoothLine(String &line) {
  BluetoothSerial *bt = iotbot.getBluetoothObject();
  while (bt->available() > 0) {
    char c = bt->read();
    lastBtCharMs = millis();
    if (c == '\n' || c == '\r') {
      if (btBuffer.length() == 0) continue;
      line = btBuffer; line.trim();
      btBuffer = "";
      return line.length() > 0;
    }
    if (btBuffer.length() < 80) btBuffer += c;
  }
  if (btBuffer.length() > 0 && millis() - lastBtCharMs > 150) {
    line = btBuffer; line.trim();
    btBuffer = "";
    return line.length() > 0;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void drawScreen() {
  lcdRow(0, L("BLUETOOTH SERİ", "BLUETOOTH SERIAL"));
  lcdRow(1, L("Cihaz: " BT_NAME, "Device: " BT_NAME));
  lcdRow(2, "PIN: " BT_PIN);
  lcdRow(3, L("Bağlantı bekleniyor", "Waiting connection"));
}

// LCD'de 2 satırlık mesaj: başlık + metnin ilk 20 karakteri
// 2-row message on the LCD: a title + the first 20 characters of the text
void showMessage(const char *title, const String &text) {
  lcdRow(2, title);
  lcdRow(3, text.substring(0, 20).c_str());
}

void printHelp() {
  iotbot.serialWrite(L("---- BLUETOOTH SERİ - Komutlar ----", "---- BLUETOOTH SERIAL - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  mesaj <metin> : metni telefona gönder", "  msg <text>    : send the text to the phone"));
  iotbot.serialWrite(L("  durum         : Bluetooth bağlantı durumu", "  status        : Bluetooth connection state"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  Telefondan    : merhaba, bip, durum, yardim", "  From phone    : hello, beep, status, help"));
}

void sendPhoneHelp() {
  iotbot.bluetoothWrite(L("Komutlar: merhaba, bip, durum, yardim. Diğer her metin LCD'de gösterilir.",
                          "Commands: hello, beep, status, help. Any other text is shown on the LCD."));
}

// Telefondan gelen satırı işler / handles a line coming from the phone
void handlePhoneLine(const String &line) {
  String cmd = normalizeCommand(line);
  iotbot.serialWrite(String(L("Telefondan alındı: ", "From phone: ")) + line);
  showMessage(L("BT'den alındı:", "BT received:"), line);

  if (cmd == "merhaba" || cmd == "hello" || cmd == "hi" || cmd == "selam") {
    iotbot.bluetoothWrite(L("IOTBOT'tan merhaba!", "Hello from IOTBOT!"));
    iotbot.buzzerPlayTone(1000, 150);
  } else if (cmd == "bip" || cmd == "beep") {
    iotbot.bluetoothWrite(L("Bip!", "Beep!"));
    iotbot.buzzerPlayTone(1500, 80);
  } else if (cmd == "durum" || cmd == "status") {
    iotbot.bluetoothWrite(String(L("Işık: ", "Light: ")) + iotbot.ldrRead() + L("  Pot: ", "  Pot: ") + iotbot.potentiometerRead());
  } else if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    sendPhoneHelp();
  } else {
    // Komut değilse sadece "alındı" de / not a command: just confirm
    iotbot.bluetoothWrite(String(L("IOTBOT aldı: ", "IOTBOT received: ")) + line);
  }
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "mesaj" || word == "msg" || word == "message") {
    String text = argText();
    if (text.length() == 0) {
      iotbot.serialWrite(L("Kullanım: mesaj <metin>", "Usage: msg <text>"));
    } else if (!iotbot.getBluetoothObject()->hasClient()) {
      iotbot.serialWrite(L("Telefon bağlı değil, mesaj gönderilemedi.", "No phone connected, message not sent."));
    } else {
      iotbot.bluetoothWrite(text);
      iotbot.serialWrite(String(L("Telefona gönderildi: ", "Sent to phone: ")) + text);
      showMessage(L("BT'ye gönderildi:", "Sent to BT:"), text);
    }
  } else if (word == "durum" || word == "status") {
    iotbot.serialWrite(String(L("Bluetooth adı: ", "Bluetooth name: ")) + BT_NAME + "  PIN: " BT_PIN);
    iotbot.serialWrite(iotbot.getBluetoothObject()->hasClient() ? L("Telefon BAĞLI.", "Phone CONNECTED.")
                                                                : L("Telefon bağlı değil.", "No phone connected."));
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    drawScreen();
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
  drawScreen();

  // Bluetooth'u isim ve PIN ile başlat / Start Bluetooth with a name and PIN
  iotbot.bluetoothStart(BT_NAME, BT_PIN);

  iotbot.serialWrite(L("Bluetooth başladı! Telefondan 'IOTBOT_BT' cihazına bağlanın (PIN 1234).",
                       "Bluetooth started! Connect to 'IOTBOT_BT' from your phone (PIN 1234)."));
  printHelp();
}

void loop() {
  static bool wasConnected = false;

  // Telefon bağlandı / ayrıldı bildirimi / phone connected / disconnected notice
  bool connected = iotbot.getBluetoothObject()->hasClient();
  if (connected != wasConnected) {
    wasConnected = connected;
    iotbot.serialWrite(connected ? L("Telefon bağlandı.", "Phone connected.") : L("Telefon ayrıldı.", "Phone disconnected."));
    lcdRow(3, connected ? L("Telefon bağlı", "Phone connected") : L("Bağlantı bekleniyor", "Waiting connection"));
    if (connected) {
      iotbot.buzzerPlayTone(1800, 60);
      sendPhoneHelp();
    }
  }

  // 1) Telefondan gelen satır / line from the phone
  String line;
  if (readBluetoothLine(line)) handlePhoneLine(line);

  // 2) Seri komutlar / serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
}
