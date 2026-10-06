// TR: GERCEK PROJE - IR Kumandali Sifreli Kasa. Kumandadan 3 tusluk bir
// sifre girersiniz, LCD her tusta bir yildiz gosterir ("SIFRE: * * _").
// Sifre dogruysa servo kilidi acar (0 -> 110 derece), LCD "KASA ACIK" yazar
// ve kutlama melodisi calar. Kasa 8 saniye sonra ya da kumandadan herhangi
// bir tusa basinca kendiliginden tekrar kilitlenir. Sifre yanlissa alarm
// ciner; ust uste 3 yanlis denemede kasa 30 saniye boyunca geri sayimla
// kilitlenir (tipki telefonlardaki gibi).
// KENDI KUMANDANIZIN KODLARINI OGRENMEK: Her kumanda markasi tuslari farkli
// sayilarla gonderir. Bu sketch aldigi HER kodu Serial Monitor'e yazar.
// 1) Sketch'i yukleyin, Serial Monitor'u 115200 baud ile acin.
// 2) Sifrede kullanmak istediginiz tuslara tek tek basin, "IR kod: 69" gibi
//    satirlardaki sayilari not edin.
// 3) Bu sayilari asagidaki kPassword dizisine yazip tekrar yukleyin.
// EN: A REAL PROJECT - IR Remote Password Vault. You enter a 3-key password
// with the remote and the LCD shows one star per key ("SIFRE: * * _"). If
// the password is right, the servo opens the lock (0 -> 110 degrees), the
// LCD shows "VAULT OPEN" and a celebration melody plays. The vault locks
// itself again after 8 seconds or when you press any key on the remote. A
// wrong password sounds an alarm; after 3 wrong tries in a row the vault is
// locked out for 30 seconds with a countdown (just like on phones).
// LEARNING YOUR OWN REMOTE'S CODES: every remote brand sends its keys as
// different numbers. This sketch prints EVERY code it receives to Serial.
// 1) Upload the sketch, open the Serial Monitor at 115200 baud.
// 2) Press the keys you want in your password one by one and note the
//    numbers in lines like "IR code: 69".
// 3) Put those numbers into the kPassword array below and upload again.
//
// Baglanti / Wiring: IR alici modulunu P2 soketine (IO26), servo motoru P1
// soketine (IO25) takin. / Plug the IR receiver module into socket P2
// (IO26) and the servo motor into socket P1 (IO25).

#define USE_IR
#define USE_SERVO
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define IR_PIN IO26    // P2 soketi / socket P2
#define SERVO_PIN IO25 // P1 soketi / socket P1

namespace {
  // ORNEK degerler! CODLAI kumandasinda 1-2-3 tuslari 1,2,3 gonderebilir ama her kumanda
  // boyle DEGILDIR - kendi kodlarinizi Serial'den ogrenip buraya yazin (bkz. yukarisi).
  // EXAMPLE values! The CODLAI remote may send 1,2,3 for keys 1-2-3 but NOT every remote
  // does - learn your own codes from Serial and write them here (see above).
  constexpr int kPassword[] = {1, 2, 3};
  constexpr int kPasswordLength = 3;          // Sifre uzunlugu / password length
  constexpr int kLockedAngle = 0;             // Kilitli servo acisi / locked servo angle
  constexpr int kOpenAngle = 110;             // Acik servo acisi / open servo angle
  constexpr int kServoMsPerDegree = 5;        // Servo yumusak hareket hizi / smooth servo speed
  constexpr uint32_t kAutoLockMs = 8000;      // Otomatik kilit suresi / auto-lock time
  constexpr uint32_t kLockoutMs = 30000;      // 3 yanlistan sonra bekleme / lockout after 3 wrong
  constexpr uint32_t kEntryTimeoutMs = 10000; // Yarim sifre unutulursa sifirla / reset half-typed password
  constexpr uint32_t kKeyGapMs = 250;         // Ayni tusun cift gelmesini engelle / ignore double sends
  constexpr int kMaxWrongAttempts = 3;        // Kilitlenmeden once hak / tries before lockout
  constexpr int kIrRepeatCode = 255;          // "Tus basili tutuluyor" tekrar kodu / NEC key-held repeat code

  enum VaultState { STATE_ENTERING, STATE_OPEN, STATE_LOCKOUT };
  VaultState state = STATE_ENTERING;
  int entered[kPasswordLength];
  int enteredCount = 0;
  int wrongAttempts = 0;
  uint32_t stateStartMs = 0;
  uint32_t lastKeyMs = 0;
  uint32_t lastScreenMs = 0;
}

// lcdWriteFixed() her zaman 20 karakter yazar; metni once bosluklarla 20'ye tamamliyoruz ki
// eski yazidan kalinti kalmasin. / lcdWriteFixed() always writes 20 chars; we pad the text
// with spaces first so no leftovers of older text stay on the row.
void writeRow(int row, const char *text) {
  char line[21];
  snprintf(line, sizeof(line), "%-20s", text);
  iotbot.lcdWriteFixed(row, line);
}

// "SIFRE: * * _" satirini cizer: girilen her tus bir yildiz. / Draws the "* * _" row.
void showPasswordRow() {
  char marks[8];
  for (int i = 0; i < kPasswordLength; i++) {
    marks[i * 2] = (i < enteredCount) ? '*' : '_';
    marks[i * 2 + 1] = ' ';
  }
  marks[kPasswordLength * 2 - 1] = '\0';
  char line[21];
  snprintf(line, sizeof(line), turkish ? "   SIFRE: %s" : "   PASSWORD: %s", marks);
  writeRow(1, line);
}

void showEntryScreen(const char *message) {
  iotbot.lcdWriteMid(turkish ? "SIFRELI KASA" : "PASSWORD VAULT", "", message,
                      turkish ? "Kumandadan girin" : "Use the remote");
  showPasswordRow();
}

void lockVault() {
  iotbot.moduleServoGoAngle(SERVO_PIN, kLockedAngle, kServoMsPerDegree);
  iotbot.buzzerPlayTone(900, 80);
  state = STATE_ENTERING;
  enteredCount = 0;
  showEntryScreen(turkish ? "Kasa kilitlendi" : "Vault locked");
  iotbot.serialWrite(turkish ? "Kasa kilitlendi." : "Vault locked.");
}

void openVault() {
  state = STATE_OPEN;
  wrongAttempts = 0;
  iotbot.lcdWriteMid(turkish ? "KASA ACIK" : "VAULT OPEN", turkish ? "Sifre dogru!" : "Correct password!", "",
                      turkish ? "Tus = hemen kilitle" : "Any key = lock now");
  iotbot.serialWrite(turkish ? "Sifre dogru, kasa acildi." : "Correct password, vault opened.");
  iotbot.moduleServoGoAngle(SERVO_PIN, kOpenAngle, kServoMsPerDegree);
  iotbot.buzzerPlayMelody(4); // Kisa kutlama melodisi / short celebration melody
  stateStartMs = millis();     // 8 sn melodiden SONRA baslasin / start the 8 s AFTER the melody
}

void wrongPassword() {
  wrongAttempts++;
  iotbot.serialWrite(turkish ? "Yanlis sifre!" : "Wrong password!");
  // Alarm: tiz-pes iki ton / alarm: high-low two tones
  iotbot.buzzerPlayTone(500, 150);
  iotbot.buzzerPlayTone(250, 350);
  enteredCount = 0;
  if (wrongAttempts >= kMaxWrongAttempts) {
    state = STATE_LOCKOUT;
    stateStartMs = millis();
    iotbot.lcdWriteMid(turkish ? "!! KILITLENDI !!" : "!! LOCKED OUT !!", turkish ? "3 yanlis deneme" : "3 wrong tries", "",
                        turkish ? "Lutfen bekleyin" : "Please wait");
    return;
  }
  char msg[21];
  snprintf(msg, sizeof(msg), turkish ? "YANLIS! Kalan hak: %d" : "WRONG! Tries left: %d",
           kMaxWrongAttempts - wrongAttempts);
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

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.moduleServoGoAngle(SERVO_PIN, kLockedAngle, 1); // Baslangicta kilitli / start locked
  showEntryScreen(turkish ? "Sifreyi girin" : "Enter the password");
  iotbot.serialWrite(turkish ? "Sifreli kasa hazir. Tuslara basin, kodlar burada gorunur."
                             : "Password vault ready. Press keys, their codes appear here.");
}

void loop() {
  uint32_t now = millis();

  // 0 = sinyal yok. Gelen HER kodu Serial'e yaziyoruz ki kendi kumandanizi ogrenebilesiniz.
  // 0 = no signal. We print EVERY code to Serial so you can learn your own remote.
  bool gotKey = false;
  int code = iotbot.moduleIRReadDecimalx8(IR_PIN);
  if (code > 0) {
    char msg[48];
    snprintf(msg, sizeof(msg), turkish ? "IR kod: %d%s" : "IR code: %d%s", code,
             code == kIrRepeatCode ? (turkish ? " (tekrar kodu, yok sayildi)" : " (repeat code, ignored)") : "");
    iotbot.serialWrite(msg);
    if (code != kIrRepeatCode && now - lastKeyMs >= kKeyGapMs) {
      gotKey = true;
      lastKeyMs = now;
    }
  }

  switch (state) {
    case STATE_ENTERING:
      if (gotKey) {
        entered[enteredCount++] = code;
        iotbot.buzzerPlayTone(1500, 40); // Tus onay sesi / key click
        showPasswordRow();
        if (enteredCount == kPasswordLength) {
          checkPassword();
        }
      } else if (enteredCount > 0 && now - lastKeyMs >= kEntryTimeoutMs) {
        enteredCount = 0; // Yarim kalan sifreyi unut / forget a half-typed password
        showEntryScreen(turkish ? "Sure doldu, bastan" : "Timeout, start again");
      }
      break;

    case STATE_OPEN:
      if (gotKey || now - stateStartMs >= kAutoLockMs) {
        lockVault();
      } else if (now - lastScreenMs >= 250) {
        lastScreenMs = now;
        char line[21];
        snprintf(line, sizeof(line), turkish ? " Otomatik kilit: %lu" : " Auto-lock in: %lu",
                 (unsigned long)((kAutoLockMs - (now - stateStartMs) + 999) / 1000));
        writeRow(2, line);
      }
      break;

    case STATE_LOCKOUT:
      if (now - stateStartMs >= kLockoutMs) {
        wrongAttempts = 0;
        enteredCount = 0;
        state = STATE_ENTERING;
        showEntryScreen(turkish ? "Tekrar deneyin" : "Try again");
      } else if (now - lastScreenMs >= 250) {
        lastScreenMs = now;
        char line[21];
        snprintf(line, sizeof(line), turkish ? "   Kalan sure: %lu sn" : "   Time left: %lu s",
                 (unsigned long)((kLockoutMs - (now - stateStartMs) + 999) / 1000));
        writeRow(2, line);
      }
      break;
  }
  delay(10);
}
