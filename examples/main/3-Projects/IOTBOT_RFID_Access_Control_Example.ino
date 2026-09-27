// TR: GERCEK PROJE - Kapi Erisim Kontrolu (RFID Kilit). Onceden
// belirledigimiz "izinli" karti okutunca role 3 saniyeligine acilir
// (gercek bir kapi kilidi/elektrikli mandal baglayabilirsiniz), LCD
// "HOS GELDIN" yazar ve onay sesi calar. Baska bir kart okutulursa
// "ERISIM RED" yazar ve alarm sesi calar. Ilk acilista, kendi kartinizin
// ID'sini ogrenmek icin herhangi bir karti okutup Seri Port'u izleyin.
// EN: A REAL PROJECT - Door Access Control (RFID Lock). Scanning our
// pre-defined "authorized" card opens the relay for 3 seconds (you can
// wire a real door lock/electric strike to it), the LCD shows "WELCOME"
// and plays a confirmation tone. Scanning any other card shows "ACCESS
// DENIED" and plays an alarm tone. On first boot, scan any card and
// watch the Serial Monitor to learn its ID.

#define USE_RFID
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

// ONEMLI: Kendi kartinizin ID'sini once okutup Seri Port'tan ogrenin,
// sonra buraya yazin. Birden fazla izinli kart ekleyebilirsiniz.
// IMPORTANT: Scan your own card first, learn its ID from the Serial
// Monitor, then put it here. You can add more than one allowed card.
constexpr int kAllowedCardCount = 2;
int allowedCardIds[kAllowedCardCount] = {123456789, 987654321}; // Ornek/placeholder degerler

namespace {
  bool isAllowed(int cardId) {
    for (int i = 0; i < kAllowedCardCount; ++i) {
      if (allowedCardIds[i] == cardId) return true;
    }
    return false;
  }
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.relayWrite(false);
  iotbot.lcdWriteMid(turkish ? "ERISIM KONTROLU" : "ACCESS CONTROL",
                      turkish ? "Kartinizi okutun" : "Scan your card",
                      "", "");
  iotbot.serialWrite(turkish ? "Erisim kontrolu hazir. Kart bekleniyor..."
                             : "Access control ready. Waiting for a card...");
}

void loop() {
  int cardId = iotbot.moduleRFIDRead();
  if (cardId == 0) {
    delay(100);
    return;
  }

  iotbot.serialWrite(turkish ? "Okunan kart ID: " + String(cardId) : "Card ID read: " + String(cardId));

  if (isAllowed(cardId)) {
    iotbot.lcdWriteMid(turkish ? "HOS GELDIN!" : "WELCOME!",
                        turkish ? "Erisim onaylandi" : "Access granted",
                        ("ID: " + String(cardId)).c_str(),
                        "");
    iotbot.buzzerPlayTone(1200, 100);
    iotbot.buzzerPlayTone(1600, 150);
    iotbot.relayWrite(true);
    delay(3000); // Kapi 3 saniye acik kalir / door stays unlocked for 3 seconds
    iotbot.relayWrite(false);
  } else {
    iotbot.lcdWriteMid(turkish ? "ERISIM RED!" : "ACCESS DENIED!",
                        turkish ? "Bu kart tanimli" : "This card is not",
                        turkish ? "degil" : "recognized",
                        ("ID: " + String(cardId)).c_str());
    iotbot.buzzerPlayTone(400, 400);
  }

  delay(1500);
  iotbot.lcdWriteMid(turkish ? "ERISIM KONTROLU" : "ACCESS CONTROL",
                      turkish ? "Kartinizi okutun" : "Scan your card",
                      "", "");
}
