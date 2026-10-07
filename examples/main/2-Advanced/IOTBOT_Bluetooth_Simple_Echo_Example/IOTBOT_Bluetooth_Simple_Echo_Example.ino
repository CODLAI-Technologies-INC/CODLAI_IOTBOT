/*
 * TR: BLUETOOTH'A İLK ADIM - en basit örnek ("yankı" / echo).
 *  - Telefonunuzdan bir Bluetooth terminal uygulamasıyla (ör. "Serial Bluetooth
 *    Terminal") "IOTBOT_ECHO" cihazına bağlanın (PIN gerekmez).
 *  - Ne yazarsanız yazın IOTBOT aynısını "Yankı: ..." diye geri gönderir ve LCD'de
 *    gösterir. Sadece telefondan "yardim" / "help" yazınca kısa bir açıklama gelir.
 *  - Gerçek komutlarla LED/röle kontrolü için sonra
 *    IOTBOT_Bluetooth_TR_EN_Control_Example.ino örneğine bakın.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help              -> komut listesi
 *      mesaj <metin> / msg <text> -> metni telefona gönder
 *      durum  / status            -> bağlantı durumu ve yankı sayısı
 *      dil    / lang              -> dili değiştir (Türkçe <-> English)
 *
 * EN: FIRST STEP INTO BLUETOOTH - the simplest example ("echo").
 *  - Connect to the "IOTBOT_ECHO" device from a Bluetooth terminal app on your
 *    phone (e.g. "Serial Bluetooth Terminal") - no PIN needed.
 *  - Whatever you type, the IOTBOT sends it right back as "Echo: ..." and shows it
 *    on the LCD. Only "help" / "yardim" from the phone returns a short description.
 *  - Later, see IOTBOT_Bluetooth_TR_EN_Control_Example.ino to control LEDs/relay
 *    with real commands.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim            -> command list
 *      msg <text> / mesaj <metin> -> send the text to the phone
 *      status / durum             -> connection state and echo count
 *      lang   / dil               -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. / NO extra module needed.
 */

#define USE_BLUETOOTH
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

#define BT_NAME "IOTBOT_ECHO" // Telefonda görünen ad / name shown on the phone

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

int echoCount = 0; // Kaç mesaj geri gönderildi / how many messages were echoed

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

// Komuttan sonraki metin (orijinal harflerle) / the text after the command word (original letters)
String argText() {
  int space = rawLine.indexOf(' ');
  if (space < 0) return "";
  String t = rawLine.substring(space + 1);
  t.trim();
  return t;
}

// ---------------------------------------------------------------------------
// Bluetooth satır okuyucu (beklemesiz) / Bluetooth line reader (non-blocking)
// iotbot.bluetoothRead() mesajı tek parça okur (~40 ms bekler); bu okuyucu hiç
// beklemez ve satır satır çalışır.
// iotbot.bluetoothRead() reads a message in one piece (waits ~40 ms); this reader
// never waits and works line by line.
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
  lcdRow(0, L("BLUETOOTH YANKI", "BLUETOOTH ECHO"));
  lcdRow(1, L("Cihaz: " BT_NAME, "Device: " BT_NAME));
  lcdRow(2, L("Bağlantı bekleniyor", "Waiting connection"));
  lcdRow(3, "");
}

void printHelp() {
  iotbot.serialWrite(L("---- BLUETOOTH YANKI - Komutlar ----", "---- BLUETOOTH ECHO - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  mesaj <metin> : metni telefona gönder", "  msg <text>    : send the text to the phone"));
  iotbot.serialWrite(L("  durum         : bağlantı durumu", "  status        : connection state"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
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
      iotbot.serialWrite(L("Telefon bağlı değil.", "No phone connected."));
    } else {
      iotbot.bluetoothWrite(text);
      iotbot.serialWrite(String(L("Telefona gönderildi: ", "Sent to phone: ")) + text);
    }
  } else if (word == "durum" || word == "status") {
    iotbot.serialWrite(iotbot.getBluetoothObject()->hasClient() ? L("Telefon BAĞLI.", "Phone CONNECTED.")
                                                                : L("Telefon bağlı değil.", "No phone connected."));
    iotbot.serialWrite(String(L("Geri gönderilen mesaj: ", "Messages echoed: ")) + echoCount);
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
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.lcdClear();
  drawScreen();
  iotbot.bluetoothStart(BT_NAME); // PIN yok / no PIN
  iotbot.serialWrite(L("Bluetooth başlatıldı: IOTBOT_ECHO", "Bluetooth started: IOTBOT_ECHO"));
  printHelp();
}

void loop() {
  String message;
  if (readBluetoothLine(message)) {
    String cmd = normalizeCommand(message);
    if (cmd == "yardim" || cmd == "help") {
      // Telefona kısa açıklama / short description for the phone
      iotbot.bluetoothWrite(L("Bu bir yankı örneği: ne yazarsanız geri gönderirim.",
                              "This is an echo example: I send back whatever you type."));
    } else {
      // Aynısını geri gönder / send it right back
      iotbot.bluetoothWrite(String(L("Yankı: ", "Echo: ")) + message);
    }
    echoCount++;
    iotbot.serialWrite(String(L("Alınan: ", "Received: ")) + message);
    lcdRow(1, L("ALINAN MESAJ:", "MESSAGE RECEIVED:"));
    lcdRow(2, message.substring(0, 20).c_str());
    lcdRow(3, L("Geri gönderildi", "Sent back"));
    iotbot.buzzerPlayTone(1200, 50);
  }

  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);
}
