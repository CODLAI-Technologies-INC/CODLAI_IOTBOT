/*
 * TR: GERÇEK PROJE - Kapı Erişim Kontrolü (RFID Kilit).
 *  - OTOMATİK mod (açılışta): önceden belirlediğimiz "izinli" kartı okutunca
 *    röle 3 saniyeliğine açılır (gerçek bir kapı kilidi/elektrikli mandal
 *    bağlayabilirsiniz), LCD "HOŞ GELDİN" yazar ve onay sesi çalar. Başka bir
 *    kart okutulursa "ERİŞİM RED" yazar ve alarm sesi çalar.
 *  - B3 butonu = içerideki "ÇIKIŞ" butonu: kapıyı kartsız 3 saniye açar
 *    (gerçek binalarda da içeriden çıkarken böyle bir buton vardır).
 *  - MANUEL mod (seri porttan "manuel"): kapı serbest bırakılır, kilit sürekli
 *    açık kalır (örneğin bir etkinlik sırasında). "oto" ile normale döner.
 *  - Kendi kartınızın ID'sini öğrenmek için herhangi bir kartı okutup Seri
 *    Port'u izleyin; "ekle" komutuyla son okunan kartı izinli listeye
 *    ekleyebilirsiniz (kart kapanınca unutulur - kalıcı olması için ID'yi
 *    aşağıdaki allowedCardIds dizisine yazın; "liste" komutu diziyi kopyalanmaya
 *    hazır bir satır olarak da yazar).
 *  - NOT: Kütüphanenin eski sürümleri aynı kart için FARKLI bir ID veriyordu;
 *    eski ID'leri yazdıysanız kartları yeniden okutup güncelleyin.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim  / help     -> komut listesi
 *      ac      / open     -> kapıyı 3 sn aç
 *      manuel  / manual   -> kapı serbest (kilit sürekli açık)
 *      oto     / auto     -> normal kart kontrolü
 *      ekle    / add      -> son okunan kartı izinli yap
 *      liste   / list     -> izinli kartları yaz
 *      dil     / lang     -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Door Access Control (RFID Lock).
 *  - AUTO mode (at startup): scanning our pre-defined "authorized" card opens
 *    the relay for 3 seconds (you can wire a real door lock/electric strike to
 *    it), the LCD shows "WELCOME" and plays a confirmation tone. Scanning any
 *    other card shows "ACCESS DENIED" and plays an alarm tone.
 *  - Button B3 = the "EXIT" button inside: opens the door for 3 seconds
 *    without a card (real buildings have such a button for leaving).
 *  - MANUAL mode (serial "manual"): the door is released, the lock stays open
 *    (e.g. during an event). "auto" goes back to normal.
 *  - To learn your own card's ID, scan any card and watch the Serial Monitor;
 *    the "add" command authorizes the last scanned card (forgotten at power
 *    off - to make it permanent write the ID into the allowedCardIds array; the
 *    "list" command also prints the array as a ready-to-copy line).
 *  - NOTE: older library versions gave a DIFFERENT ID for the same card; if you
 *    wrote old IDs here, scan the cards again and update them.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help    / yardim   -> command list
 *      open    / ac       -> open the door for 3 s
 *      manual  / manuel   -> door released (lock stays open)
 *      auto    / oto      -> normal card control
 *      add     / ekle     -> authorize the last scanned card
 *      list    / liste    -> print the authorized cards
 *      lang    / dil      -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: RC522 RFID modülünü kartın RFID haberleşme portuna takın.
 * Röle ve B3 kart üzerindedir. / Plug the RC522 RFID module into the board's
 * RFID communication port. The relay and B3 are on the board.
 */

#define USE_RFID
#include <IOTBOT.h>

IOTBOT iotbot;

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// ÖNEMLİ: Kendi kartınızın ID'sini önce okutup Seri Port'tan öğrenin, sonra buraya yazın.
// Birden fazla izinli kart ekleyebilirsiniz (en fazla kMaxCards).
// IMPORTANT: Scan your own card first, learn its ID from the Serial Monitor, then put it
// here. You can add more than one allowed card (up to kMaxCards).
constexpr int kMaxCards = 8;
int allowedCardIds[kMaxCards] = {123456789, 987654321}; // Örnek/placeholder değerler / example values
int allowedCardCount = 2;

namespace {
  constexpr uint32_t kOpenMs = 3000;        // Kapı bu kadar açık kalır / door stays unlocked this long
  constexpr uint32_t kDeniedShowMs = 1500;  // "RED" ekranı süresi / "DENIED" screen time

  enum State { IDLE, GRANTED, DENIED };
  State state = IDLE;
  uint32_t stateStartMs = 0;
  bool manualMode = false;   // true = kapı serbest (kilit sürekli açık) / true = door released (lock open)
  int lastCardId = 0;        // Son okunan kart (0 = yok) / last scanned card (0 = none)
  bool lastB3 = false;
  uint32_t lastB3Ms = 0;
  uint32_t lastCountdownMs = 0;

  bool isAllowed(int cardId) {
    for (int i = 0; i < allowedCardCount; ++i) {
      if (allowedCardIds[i] == cardId) return true;
    }
    return false;
  }
}

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "AÇ" -> "ac"
// Lower-cases and simplifies Turkish letters: "AÇ" -> "ac"
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
  iotbot.serialWrite(L("---- ERİŞİM KONTROLÜ - Komutlar ----", "---- ACCESS CONTROL - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  ac            : kapıyı 3 sn aç", "  open          : open the door for 3 s"));
  iotbot.serialWrite(L("  manuel        : kapı serbest (kilit sürekli açık)", "  manual        : door released (lock stays open)"));
  iotbot.serialWrite(L("  oto           : normal kart kontrolü", "  auto          : normal card control"));
  iotbot.serialWrite(L("  ekle          : son okunan kartı izinli yap", "  add           : authorize the last scanned card"));
  iotbot.serialWrite(L("  liste         : izinli kartlar", "  list          : authorized cards"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : ÇIKIŞ (kapıyı 3 sn aç)", "  B3 button     : EXIT (open the door for 3 s)"));
}

void showIdleScreen() {
  if (manualMode) {
    iotbot.lcdWriteMid(L("ERİŞİM KONTROLÜ", "ACCESS CONTROL"), L("KAPI SERBEST", "DOOR RELEASED"), L("(manuel mod)", "(manual mode)"),
                       L("oto: normale dön", "auto: back to normal"));
  } else {
    iotbot.lcdWriteMid(L("ERİŞİM KONTROLÜ", "ACCESS CONTROL"), L("Kartınızı okutun", "Scan your card"), "",
                       L("B3: çıkış", "B3: exit"));
  }
}

// Kapıyı 3 sn aç (kart, B3 ya da seri komut). / Open the door for 3 s (card, B3 or serial).
void openDoor(const char *title, const char *reason, int cardId) {
  state = GRANTED;
  stateStartMs = millis();
  lastCountdownMs = 0;
  char idLine[24];
  if (cardId != 0) snprintf(idLine, sizeof(idLine), "ID: %d", cardId);
  else idLine[0] = '\0';
  iotbot.lcdWriteMid(title, reason, idLine, "");
  iotbot.buzzerPlayTone(1200, 100);
  iotbot.buzzerPlayTone(1600, 150);
  iotbot.relayWrite(true);
  iotbot.serialWrite(String(L("Kapı açıldı: ", "Door opened: ")) + reason);
}

void setManual(bool manual) {
  manualMode = manual;
  state = IDLE;
  iotbot.relayWrite(manual); // Manuelde kilit sürekli açık / lock stays open in manual
  iotbot.buzzerPlayTone(manual ? 1500 : 1000, 60);
  iotbot.serialWrite(manual ? L(">> MANUEL mod: kapı serbest, kilit açık.", ">> MANUAL mode: door released, lock open.")
                            : L(">> OTOMATİK mod: kapı kartla açılır.", ">> AUTO mode: the door opens with a card."));
  showIdleScreen();
}

void listCards() {
  iotbot.serialWrite(String(L("İzinli kartlar (", "Authorized cards (")) + allowedCardCount + "):");
  for (int i = 0; i < allowedCardCount; i++) iotbot.serialWrite(String("  ") + allowedCardIds[i]);
  // Kalıcı yapmak için bu satırı kopyalayıp yukarıdaki diziyle değiştirin
  // To make it permanent, copy this line over the array above
  String line = "int allowedCardIds[kMaxCards] = {";
  for (int i = 0; i < allowedCardCount; i++) line += (i ? ", " : "") + String(allowedCardIds[i]);
  line += "};  int allowedCardCount = " + String(allowedCardCount) + ";";
  iotbot.serialWrite(L("Kodda kullanmak için:", "To use in code:"));
  iotbot.serialWrite(line);
}

void handleCommand(const String &cmd) {
  if (cmd == "yardim" || cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "ac" || cmd == "open") {
    if (manualMode) iotbot.serialWrite(L("Kapı zaten serbest (manuel mod).", "The door is already released (manual mode)."));
    else openDoor(L("KAPI AÇIK", "DOOR OPEN"), L("Seri komut", "Serial command"), 0);
  } else if (cmd == "manuel" || cmd == "manual") {
    setManual(true);
  } else if (cmd == "oto" || cmd == "otomatik" || cmd == "auto") {
    setManual(false);
  } else if (cmd == "ekle" || cmd == "add") {
    if (lastCardId == 0) {
      iotbot.serialWrite(L("Önce bir kart okutun.", "Scan a card first."));
    } else if (isAllowed(lastCardId)) {
      iotbot.serialWrite(L("Bu kart zaten izinli.", "This card is already authorized."));
    } else if (allowedCardCount >= kMaxCards) {
      iotbot.serialWrite(L("Liste dolu!", "The list is full!"));
    } else {
      allowedCardIds[allowedCardCount++] = lastCardId;
      iotbot.serialWrite(String(L("Kart eklendi: ", "Card added: ")) + lastCardId);
    }
  } else if (cmd == "liste" || cmd == "list") {
    listCards();
  } else if (cmd == "dil" || cmd == "lang" || cmd == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    if (state == IDLE) showIdleScreen();
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
  showIdleScreen();
  iotbot.serialWrite(L("Erişim kontrolü hazır. Kart bekleniyor...", "Access control ready. Waiting for a card..."));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) B3 = içerideki ÇIKIŞ butonu (sadece basıldığı an) / B3 = the EXIT button inside (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3 && now - lastB3Ms > 300) {
    lastB3Ms = now;
    if (!manualMode) openDoor(L("GÜLE GÜLE!", "GOODBYE!"), L("Çıkış butonu", "Exit button"), 0);
  }
  lastB3 = b3;

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) Kart okuma (0 = kart yok) / Card reading (0 = no card)
  int cardId = iotbot.moduleRFIDRead();
  if (cardId != 0) {
    lastCardId = cardId;
    iotbot.serialWrite(String(L("Okunan kart ID: ", "Card ID read: ")) + cardId);
    if (manualMode) {
      iotbot.buzzerPlayTone(1500, 40); // Kapı zaten serbest / door already released
    } else if (isAllowed(cardId)) {
      openDoor(L("HOŞ GELDİN!", "WELCOME!"), L("Erişim onaylandı", "Access granted"), cardId);
    } else {
      state = DENIED;
      stateStartMs = now;
      iotbot.relayWrite(false);
      char idLine[24];
      snprintf(idLine, sizeof(idLine), "ID: %d", cardId);
      iotbot.lcdWriteMid(L("ERİŞİM RED!", "ACCESS DENIED!"), L("Bu kart tanımlı", "This card is not"), L("değil", "recognized"), idLine);
      iotbot.serialWrite(L("Erişim reddedildi. (Bu kartı eklemek için: ekle)", "Access denied. (To add this card: add)"));
      iotbot.buzzerPlayTone(400, 400);
    }
  }

  // 4) Zamanlayıcılar (bloklamaz) / Timers (non-blocking)
  if (state == GRANTED) {
    if (now - stateStartMs >= kOpenMs) {
      iotbot.relayWrite(false); // Kapı yeniden kilitlendi / door locked again
      iotbot.serialWrite(L("Kapı kilitlendi.", "Door locked."));
      state = IDLE;
      showIdleScreen();
    } else if (now - lastCountdownMs >= 250) {
      lastCountdownMs = now;
      char line[41];
      snprintf(line, sizeof(line), L("Kilitlenmeye: %lu sn", "Locking in: %lu s"),
               (unsigned long)((kOpenMs - (now - stateStartMs) + 999) / 1000));
      lcdRow(3, line);
    }
  } else if (state == DENIED && now - stateStartMs >= kDeniedShowMs) {
    state = IDLE;
    showIdleScreen();
  }
  delay(50);
}
