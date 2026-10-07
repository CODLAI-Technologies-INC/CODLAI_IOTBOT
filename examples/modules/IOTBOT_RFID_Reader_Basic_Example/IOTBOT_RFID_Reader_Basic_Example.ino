/*
 * TR: RFID KART OKUYUCU (RC522) MODÜLÜ
 *  - Okuyucuya bir RFID kart veya anahtarlık yaklaştırın: kartın numarası (ID)
 *    LCD'de ve seri portta görünür, kısa bir bip çalar. Kaç kart okunduğunu sayar.
 *  - Bu numaraları kendi projelerinizde (ör. kapı kilidi) "hangi kart?" sorusu
 *    için kullanabilirsiniz. Seri porttaki "Kodda kullanmak için" satırını
 *    olduğu gibi kopyalayıp kendi sketch'inize yapıştırabilirsiniz.
 *  - ID, kartın UID'sinin ilk 4 baytından üretilir (eksi sayı da olabilir).
 *    NOT: Kütüphanenin eski sürümleri aynı kart için FARKLI bir sayı veriyordu;
 *    eski ID'leri kullandıysanız kartları yeniden okutun.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help  -> komut listesi
 *      oku     / read  -> son okunan kartı yaz
 *      sifirla / reset -> son kartı ve sayacı sıfırla
 *      dil     / lang  -> dili değiştir (Türkçe <-> English)
 *  - RFID özelliklerini açmak için "#define USE_RFID" satırı #include'dan ÖNCE olmalıdır.
 *
 * EN: RFID CARD READER (RC522) MODULE
 *  - Hold an RFID card or key fob near the reader: the card's number (ID) is
 *    shown on the LCD and the serial port with a short beep. It counts the cards read.
 *  - Use these numbers in your projects (e.g. a door lock) to know "which card?".
 *    Copy the "To use in code" line from the serial port into your own sketch.
 *  - The ID is built from the first 4 bytes of the card's UID (it may be negative).
 *    NOTE: older library versions gave a DIFFERENT number for the same card;
 *    if you used old IDs, scan the cards again.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help  / yardim  -> command list
 *      read  / oku     -> print the last card read
 *      reset / sifirla -> clear the last card and the counter
 *      lang  / dil     -> switch language (Turkish <-> English)
 *  - To enable the RFID features, "#define USE_RFID" must come BEFORE #include.
 *
 * Bağlantı / Wiring: RC522 modülünü kartın RFID haberleşme portuna takın.
 *                    Plug the RC522 module into the board's RFID port.
 */

#define USE_RFID
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

int lastId = 0;          // Son okunan kart (0 = henüz yok) / last card read (0 = none yet)
int cardCount = 0;       // Kaç kart okundu / how many cards were read
uint32_t lastReadMs = 0;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "AÇI" -> "aci"
// Lower-cases and simplifies Turkish letters: "AÇI" -> "aci"
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
  iotbot.serialWrite(L("---- RFID OKUYUCU - Komutlar ----", "---- RFID READER - Commands ----"));
  iotbot.serialWrite(L("  yardim  : bu liste", "  help    : this list"));
  iotbot.serialWrite(L("  oku     : son okunan kartı yaz", "  read    : print the last card read"));
  iotbot.serialWrite(L("  sifirla : son kartı ve sayacı sıfırla", "  reset   : clear the last card and the counter"));
  iotbot.serialWrite(L("  dil     : English'e geç", "  lang    : switch to Turkish"));
}

void drawScreen() {
  char line[41];
  lcdRow(0, L("   RFID OKUYUCU", "    RFID READER"));
  if (cardCount == 0) {
    lcdRow(1, L("Kartı okutun...", "Hold a card near..."));
    lcdRow(2, "");
  } else {
    lcdRow(1, L("Kart ID:", "Card ID:"));
    snprintf(line, sizeof(line), "%d", lastId);
    lcdRow(2, line);
  }
  snprintf(line, sizeof(line), L("Okunan kart: %d", "Cards read: %d"), cardCount);
  lcdRow(3, line);
}

void printLast() {
  if (cardCount == 0) iotbot.serialWrite(L("Henüz kart okunmadı - bir kart yaklaştırın.", "No card yet - hold a card near the reader."));
  else {
    // HEX gösterim UID baytlarıdır (ör. 9A2B3C4D) / the HEX form shows the UID bytes (e.g. 9A2B3C4D)
    char hex[9];
    snprintf(hex, sizeof(hex), "%08lX", (unsigned long)(uint32_t)lastId);
    iotbot.serialWrite(String(L("Son kart ID: ", "Last card ID: ")) + lastId + "  (HEX " + hex + ")");
    iotbot.serialWrite(String(L("  Kodda kullanmak için: ", "  To use in code: ")) + "const int myCard = " + lastId + ";");
  }
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "oku" || cmd == "read") {
    printLast();
  } else if (cmd == "sifirla" || cmd == "reset") {
    lastId = 0;
    cardCount = 0;
    drawScreen();
    iotbot.serialWrite(L("Sıfırlandı.", "Cleared."));
  } else if (cmd == "dil" || cmd == "lang" || cmd == "language") {
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
  iotbot.serialWrite(L("RFID okuyucu hazır. Bir kart yaklaştırın!", "RFID reader ready. Hold a card near it!"));
  printHelp();
}

void loop() {
  // 1) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 2) 100 ms'de bir kart var mı bak (0 = yeni kart yok) / check for a card every 100 ms (0 = no new card)
  if (millis() - lastReadMs < 100) return;
  lastReadMs = millis();

  int id = iotbot.moduleRFIDRead();
  if (id != 0) {
    lastId = id;
    cardCount++;
    printLast();
    drawScreen();
    iotbot.buzzerPlayTone(1800, 80); // Kart okundu bip'i / card read beep
  }
}
