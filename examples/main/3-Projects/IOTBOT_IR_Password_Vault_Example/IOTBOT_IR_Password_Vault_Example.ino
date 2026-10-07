/*
 * TR: GERÇEK PROJE - IR Kumandalı Şifreli Kasa. Kumandadan 3 tuşluk bir şifre
 * girersiniz, LCD her tuşta bir yıldız gösterir ("ŞİFRE: * * _"). Şifre
 * doğruysa servo kilidi açar (0 -> 110 derece), LCD "KASA AÇIK" yazar ve
 * kutlama melodisi çalar. Kasa 8 saniye sonra, kumandadan herhangi bir tuşa
 * basınca ya da B3'e basınca kendiliğinden tekrar kilitlenir. Şifre yanlışsa
 * alarm öter; üst üste 3 yanlış denemede kasa 30 saniye boyunca geri sayımla
 * kilitlenir (tıpkı telefonlardaki gibi).
 *  - Bu bir güvenlik projesi olduğu için "manuel aç" butonu YOKTUR: kasa sadece
 *    doğru şifreyle açılır (seri porttan da şifre girilebilir).
 *  - KENDİ KUMANDANIZIN KODLARINI ÖĞRENMEK: Her kumanda markası tuşları farklı
 *    sayılarla gönderir. Bu sketch aldığı HER kodu Seri Monitör'e yazar.
 *    1) Sketch'i yükleyin, Seri Monitör'ü 115200 baud ile açın.
 *    2) Şifrede kullanmak istediğiniz tuşlara tek tek basın, "IR kod: 69" gibi
 *       satırlardaki sayıları not edin.
 *    3) Bu sayıları aşağıdaki kPassword dizisine yazıp tekrar yükleyin.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim       / help             -> komut listesi
 *      sifre 1 2 3  / password 1 2 3   -> şifreyi seri porttan gir (IR kodları)
 *      kilitle      / lock             -> açık kasayı hemen kilitle
 *      durum        / status           -> kasanın durumunu yaz
 *      dil          / lang             -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - IR Remote Password Vault. You enter a 3-key password
 * with the remote and the LCD shows one star per key ("PASSWORD: * * _"). If
 * the password is right, the servo opens the lock (0 -> 110 degrees), the
 * LCD shows "VAULT OPEN" and a celebration melody plays. The vault locks
 * itself again after 8 seconds, when you press any key on the remote or when
 * you press B3. A wrong password sounds an alarm; after 3 wrong tries in a
 * row the vault is locked out for 30 seconds with a countdown (just like on
 * phones).
 *  - This is a security project, so there is NO "manual open" button: the
 *    vault only opens with the right password (which can also be typed on the
 *    serial port).
 *  - LEARNING YOUR OWN REMOTE'S CODES: every remote brand sends its keys as
 *    different numbers. This sketch prints EVERY code it receives to Serial.
 *    1) Upload the sketch, open the Serial Monitor at 115200 baud.
 *    2) Press the keys you want in your password one by one and note the
 *       numbers in lines like "IR code: 69".
 *    3) Put those numbers into the kPassword array below and upload again.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help             / yardim        -> command list
 *      password 1 2 3   / sifre 1 2 3   -> type the password on serial (IR codes)
 *      lock             / kilitle       -> lock an open vault right now
 *      status           / durum         -> print the vault state
 *      lang             / dil           -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: IR alıcı modülünü P2 soketine (IO26), servo motoru P1
 * soketine (IO25) takın. B3 kart üzerindedir. / Plug the IR receiver module
 * into socket P2 (IO26) and the servo motor into socket P1 (IO25). B3 is on
 * the board.
 */

#define USE_IR
#define USE_SERVO
#include <IOTBOT.h>

IOTBOT iotbot;

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

#define IR_PIN IO26    // P2 soketi / socket P2
#define SERVO_PIN IO25 // P1 soketi / socket P1

namespace {
  // ÖRNEK değerler! CODLAI kumandasında 1-2-3 tuşları 1,2,3 gönderebilir ama her kumanda
  // böyle DEĞİLDİR - kendi kodlarınızı Serial'den öğrenip buraya yazın (bkz. yukarısı).
  // EXAMPLE values! The CODLAI remote may send 1,2,3 for keys 1-2-3 but NOT every remote
  // does - learn your own codes from Serial and write them here (see above).
  constexpr int kPassword[] = {1, 2, 3};
  constexpr int kPasswordLength = 3;          // Şifre uzunluğu / password length
  constexpr int kLockedAngle = 0;             // Kilitli servo açısı / locked servo angle
  constexpr int kOpenAngle = 110;             // Açık servo açısı / open servo angle
  constexpr int kServoMsPerDegree = 5;        // Servo yumuşak hareket hızı / smooth servo speed
  constexpr uint32_t kAutoLockMs = 8000;      // Otomatik kilit süresi / auto-lock time
  constexpr uint32_t kLockoutMs = 30000;      // 3 yanlıştan sonra bekleme / lockout after 3 wrong
  constexpr uint32_t kEntryTimeoutMs = 10000; // Yarım şifre unutulursa sıfırla / reset half-typed password
  constexpr uint32_t kKeyGapMs = 250;         // Aynı tuşun çift gelmesini engelle / ignore double sends
  constexpr int kMaxWrongAttempts = 3;        // Kilitlenmeden önce hak / tries before lockout
  constexpr int kIrRepeatCode = 255;          // "Tuş basılı tutuluyor" tekrar kodu / NEC key-held repeat code

  enum VaultState { STATE_ENTERING, STATE_OPEN, STATE_LOCKOUT };
  VaultState state = STATE_ENTERING;
  int entered[kPasswordLength];
  int enteredCount = 0;
  int wrongAttempts = 0;
  uint32_t stateStartMs = 0;
  uint32_t lastKeyMs = 0;
  uint32_t lastScreenMs = 0;
  bool lastB3 = false;
}

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "ŞİFRE" -> "sifre"
// Lower-cases and simplifies Turkish letters: "ŞİFRE" -> "sifre"
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
  iotbot.serialWrite(L("---- ŞİFRELİ KASA - Komutlar ----", "---- PASSWORD VAULT - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  sifre A B C   : şifreyi gir (IR kodları)", "  password A B C: enter the password (IR codes)"));
  iotbot.serialWrite(L("  kilitle       : kasayı hemen kilitle", "  lock          : lock the vault now"));
  iotbot.serialWrite(L("  durum         : kasanın durumu", "  status        : vault state"));
  iotbot.serialWrite(L("  dil           : English'e geç", "  lang          : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu     : açık kasayı kilitle", "  B3 button     : lock an open vault"));
}

// "ŞİFRE: * * _" satırını çizer: girilen her tuş bir yıldız. / Draws the "* * _" row.
void showPasswordRow() {
  char marks[8];
  for (int i = 0; i < kPasswordLength; i++) {
    marks[i * 2] = (i < enteredCount) ? '*' : '_';
    marks[i * 2 + 1] = ' ';
  }
  marks[kPasswordLength * 2 - 1] = '\0';
  char line[41];
  snprintf(line, sizeof(line), L("   ŞİFRE: %s", "   PASSWORD: %s"), marks);
  lcdRow(1, line);
}

void showEntryScreen(const char *message) {
  iotbot.lcdWriteMid(L("ŞİFRELİ KASA", "PASSWORD VAULT"), "", message, L("Kumandadan girin", "Use the remote"));
  showPasswordRow();
}

void showOpenScreen() {
  iotbot.lcdWriteMid(L("KASA AÇIK", "VAULT OPEN"), L("Şifre doğru!", "Correct password!"), "",
                     L("Tuş/B3 = kilitle", "Key/B3 = lock now"));
}

void showLockoutScreen() {
  iotbot.lcdWriteMid(L("!! KİLİTLENDİ !!", "!! LOCKED OUT !!"), L("3 yanlış deneme", "3 wrong tries"), "",
                     L("Lütfen bekleyin", "Please wait"));
}

void lockVault() {
  iotbot.moduleServoGoAngle(SERVO_PIN, kLockedAngle, kServoMsPerDegree);
  iotbot.buzzerPlayTone(900, 80);
  state = STATE_ENTERING;
  enteredCount = 0;
  showEntryScreen(L("Kasa kilitlendi", "Vault locked"));
  iotbot.serialWrite(L("Kasa kilitlendi.", "Vault locked."));
}

void openVault() {
  state = STATE_OPEN;
  wrongAttempts = 0;
  showOpenScreen();
  iotbot.serialWrite(L("Şifre doğru, kasa açıldı.", "Correct password, vault opened."));
  iotbot.moduleServoGoAngle(SERVO_PIN, kOpenAngle, kServoMsPerDegree);
  iotbot.buzzerPlayMelody(4); // Kısa kutlama melodisi / short celebration melody
  stateStartMs = millis();     // 8 sn melodiden SONRA başlasın / start the 8 s AFTER the melody
}

void wrongPassword() {
  wrongAttempts++;
  iotbot.serialWrite(L("Yanlış şifre!", "Wrong password!"));
  // Alarm: tiz-pes iki ton / alarm: high-low two tones
  iotbot.buzzerPlayTone(500, 150);
  iotbot.buzzerPlayTone(250, 350);
  enteredCount = 0;
  if (wrongAttempts >= kMaxWrongAttempts) {
    state = STATE_LOCKOUT;
    stateStartMs = millis();
    showLockoutScreen();
    iotbot.serialWrite(L("3 yanlış deneme: kasa 30 saniye kilitli.", "3 wrong tries: the vault is locked for 30 seconds."));
    return;
  }
  char msg[41];
  snprintf(msg, sizeof(msg), L("YANLIŞ! Kalan hak: %d", "WRONG! Tries left: %d"), kMaxWrongAttempts - wrongAttempts);
  showEntryScreen(msg);
}

void checkPassword() {
  for (int i = 0; i < kPasswordLength; i++) {
    if (entered[i] != kPassword[i]) {
      wrongPassword();
      return;
    }
  }
  openVault();
}

// Bir tuş (kumandadan ya da seri porttan) geldi. / A key arrived (from the remote or serial).
void handleKey(int code) {
  lastKeyMs = millis();
  switch (state) {
    case STATE_ENTERING:
      entered[enteredCount++] = code;
      iotbot.buzzerPlayTone(1500, 40); // Tuş onay sesi / key click
      showPasswordRow();
      if (enteredCount == kPasswordLength) checkPassword();
      break;
    case STATE_OPEN:
      lockVault(); // Açıkken herhangi bir tuş kilitler / any key locks an open vault
      break;
    case STATE_LOCKOUT:
      break; // Bekleme süresinde tuşlar yok sayılır / keys are ignored during the lockout
  }
}

void printStatus() {
  if (state == STATE_OPEN) {
    iotbot.serialWrite(L("Durum: KASA AÇIK", "State: VAULT OPEN"));
  } else if (state == STATE_LOCKOUT) {
    iotbot.serialWrite(String(L("Durum: KİLİTLİ, kalan süre ", "State: LOCKED OUT, time left ")) +
                       (kLockoutMs - (millis() - stateStartMs) + 999) / 1000 + L(" sn", " s"));
  } else {
    iotbot.serialWrite(String(L("Durum: şifre bekleniyor, girilen tuş: ", "State: waiting for password, keys typed: ")) + enteredCount +
                       L(", kalan hak: ", ", tries left: ") + (kMaxWrongAttempts - wrongAttempts));
  }
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  String rest = hasValue ? cmd.substring(space + 1) : "";

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if ((word == "sifre" || word == "password") && hasValue) {
    if (state == STATE_LOCKOUT) {
      iotbot.serialWrite(L("Kasa kilitli, lütfen bekleyin.", "The vault is locked out, please wait."));
      return;
    }
    if (state == STATE_OPEN) {
      iotbot.serialWrite(L("Kasa zaten açık.", "The vault is already open."));
      return;
    }
    // "1 2 3" -> her sayı bir tuş gibi girilir / "1 2 3" -> each number is entered like a key
    int start = 0;
    while (start < (int)rest.length() && state == STATE_ENTERING) {
      int next = rest.indexOf(' ', start);
      if (next < 0) next = rest.length();
      String part = rest.substring(start, next);
      if (part.length() > 0) handleKey(part.toInt());
      start = next + 1;
    }
  } else if (word == "kilitle" || word == "lock") {
    if (state == STATE_OPEN) lockVault();
    else iotbot.serialWrite(L("Kasa zaten kilitli.", "The vault is already locked."));
  } else if (word == "durum" || word == "status") {
    printStatus();
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    if (state == STATE_OPEN) showOpenScreen();
    else if (state == STATE_LOCKOUT) showLockoutScreen();
    else showEntryScreen(L("Şifreyi girin", "Enter the password"));
    lastScreenMs = 0;
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.moduleServoGoAngle(SERVO_PIN, kLockedAngle, 1); // Başlangıçta kilitli / start locked
  showEntryScreen(L("Şifreyi girin", "Enter the password"));
  iotbot.serialWrite(L("Şifreli kasa hazır. Tuşlara basın, kodlar burada görünür.",
                       "Password vault ready. Press keys, their codes appear here."));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // 1) B3: açık kasayı hemen kilitle (sadece basıldığı an) / B3: lock an open vault now (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3 && state == STATE_OPEN) lockVault();
  lastB3 = b3;

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 3) IR: 0 = sinyal yok. Gelen HER kodu Serial'e yazıyoruz ki kendi kumandanızı öğrenebilesiniz.
  // 3) IR: 0 = no signal. We print EVERY code to Serial so you can learn your own remote.
  int code = iotbot.moduleIRReadDecimalx8(IR_PIN);
  if (code > 0) {
    char msg[64];
    snprintf(msg, sizeof(msg), L("IR kod: %d%s", "IR code: %d%s"), code,
             code == kIrRepeatCode ? L(" (tekrar kodu, yok sayıldı)", " (repeat code, ignored)") : "");
    iotbot.serialWrite(msg);
    if (code != kIrRepeatCode && now - lastKeyMs >= kKeyGapMs) handleKey(code);
  }

  // 4) Zamanlayıcılar / Timers
  switch (state) {
    case STATE_ENTERING:
      if (enteredCount > 0 && now - lastKeyMs >= kEntryTimeoutMs) {
        enteredCount = 0; // Yarım kalan şifreyi unut / forget a half-typed password
        showEntryScreen(L("Süre doldu, baştan", "Timeout, start again"));
      }
      break;

    case STATE_OPEN:
      if (now - stateStartMs >= kAutoLockMs) {
        lockVault();
      } else if (now - lastScreenMs >= 250) {
        lastScreenMs = now;
        char line[41];
        snprintf(line, sizeof(line), L(" Otomatik kilit: %lu", " Auto-lock in: %lu"),
                 (unsigned long)((kAutoLockMs - (now - stateStartMs) + 999) / 1000));
        lcdRow(2, line);
      }
      break;

    case STATE_LOCKOUT:
      if (now - stateStartMs >= kLockoutMs) {
        wrongAttempts = 0;
        enteredCount = 0;
        state = STATE_ENTERING;
        showEntryScreen(L("Tekrar deneyin", "Try again"));
        iotbot.serialWrite(L("Bekleme bitti, tekrar deneyebilirsiniz.", "Lockout over, you can try again."));
      } else if (now - lastScreenMs >= 250) {
        lastScreenMs = now;
        char line[41];
        snprintf(line, sizeof(line), L("   Kalan süre: %lu sn", "   Time left: %lu s"),
                 (unsigned long)((kLockoutMs - (now - stateStartMs) + 999) / 1000));
        lcdRow(2, line);
      }
      break;
  }
  delay(10);
}
