/*
 * TR: FIREBASE (BULUT VERİTABANI) - Kullanıcı doğrulamalı temel örnek
 *  - IOTBOT WiFi'ye bağlanır, Firebase'e e-posta/şifre ile giriş yapar (kimlik
 *    doğrulama), birkaç değer yazar ve 10 saniyede bir geri okuyup LCD'de gösterir.
 *    Firebase Console'da değerleri değiştirin, IOTBOT'un ekranında görün!
 *  - Yazılan yollar: /device/temperature (sayı), /device/status (metin),
 *    /device/active (doğru/yanlış), /device/light (ışık sensörü).
 *  - B3 butonu: /device/active değerini tersine çevirip yazar.
 *  - Firebase kurulum adımları:
 *    1) Firebase Console > Realtime Database: veritabanı oluşturun, URL'yi kopyalayın
 *       ("https://...firebaseio.com/") ve FIREBASE_PROJECT_URL'ye yazın.
 *    2) Project Settings > General: "Web API Key" değerini FIREBASE_API_KEY'e yazın.
 *    3) Authentication > Sign-in method: Email/Password girişini açın, "Users"
 *       sekmesinden bir kullanıcı oluşturun; bilgilerini USER_EMAIL / USER_PASSWORD'a yazın.
 *    4) USE_FIREBASE tanımlıyken WiFi otomatik açılır; ayrıca USE_WIFI gerekmez.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help                 -> komut listesi
 *      oku    / read                 -> değerleri şimdi oku
 *      sicaklik 30 / temp 30         -> /device/temperature'a yaz
 *      mesaj <metin> / msg <text>    -> /device/status'a metin yaz
 *      aktif  / active               -> /device/active'i tersine çevir (B3 ile aynı)
 *      gonder / send                 -> ışık değerini /device/light'a yaz
 *      durum  / status               -> WiFi ve Firebase durumu
 *      dil    / lang                 -> dili değiştir (Türkçe <-> English)
 *  - Not: Her okuma/yazma birkaç yüz ms sürer; o sırada kart bekler.
 *
 * EN: FIREBASE (CLOUD DATABASE) - Basic example with user authentication
 *  - The IOTBOT joins WiFi, signs in to Firebase with email/password (authentication),
 *    writes a few values and reads them back every 10 seconds onto the LCD. Change the
 *    values in the Firebase Console and see them on the IOTBOT's screen!
 *  - Paths written: /device/temperature (number), /device/status (text),
 *    /device/active (true/false), /device/light (light sensor).
 *  - B3 button: flips /device/active and writes it.
 *  - Firebase setup steps:
 *    1) Firebase Console > Realtime Database: create a database, copy its URL
 *       ("https://...firebaseio.com/") into FIREBASE_PROJECT_URL.
 *    2) Project Settings > General: copy the "Web API Key" into FIREBASE_API_KEY.
 *    3) Authentication > Sign-in method: enable Email/Password, create a user under the
 *       "Users" tab and put its details into USER_EMAIL / USER_PASSWORD.
 *    4) With USE_FIREBASE, WiFi is enabled automatically; no extra USE_WIFI needed.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help   / yardim               -> command list
 *      read   / oku                  -> read the values now
 *      temp 30 / sicaklik 30         -> write to /device/temperature
 *      msg <text> / mesaj <metin>    -> write a text to /device/status
 *      active / aktif                -> flip /device/active (same as B3)
 *      send   / gonder               -> write the light value to /device/light
 *      status / durum                -> WiFi and Firebase state
 *      lang   / dil                  -> switch language (Turkish <-> English)
 *  - Note: each read/write takes a few hundred ms; the board waits meanwhile.
 *
 * Bağlantı / Wiring: Ek modül GEREKMEZ. / NO extra module needed.
 */
#define USE_FIREBASE
#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object

// Firebase yapılandırması / Firebase configuration
#define FIREBASE_PROJECT_URL "https://YOUR_PROJECT_ID-default-rtdb.firebaseio.com/" // Firebase veritabanı URL'si / Firebase database URL
#define FIREBASE_API_KEY "YOUR_FIREBASE_WEB_API_KEY"                             // Web API Key

// Firebase kullanıcı bilgileri / Firebase user credentials
#define USER_EMAIL "YOUR_USER_EMAIL@example.com" // Firebase'de oluşturduğunuz kullanıcının e-postası / email of the Firebase user
#define USER_PASSWORD "YOUR_USER_PASSWORD"       // O kullanıcının şifresi / that user's password

// WiFi ayarları / WiFi settings
#define WIFI_SSID "WIFI_SSID" // Bağlanılacak WiFi ağının adı / name of the WiFi network
#define WIFI_PASS "WiFi_PASS" // WiFi şifresi / WiFi password

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

const uint32_t kReadEveryMs = 10000; // 10 saniyede bir oku / read every 10 seconds
bool firebaseStarted = false;
uint32_t lastReadMs = 0, lastWifiCheckMs = 0;
bool activeValue = true;
bool lastB3 = false;

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
String rawLine; // Komutun orijinal hali (mesaj metni için) / original command text (for the message text)
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "SICAKLIK" -> "sicaklik"
// Lower-cases and simplifies Turkish letters: "SICAKLIK" -> "sicaklik"
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

bool wifiOk() { return WiFi.status() == WL_CONNECTED; }

void printHelp() {
  iotbot.serialWrite(L("---- FIREBASE - Komutlar ----", "---- FIREBASE - Commands ----"));
  iotbot.serialWrite(L("  yardim        : bu liste", "  help          : this list"));
  iotbot.serialWrite(L("  oku           : değerleri şimdi oku", "  read          : read the values now"));
  iotbot.serialWrite(L("  sicaklik 30   : /device/temperature yaz", "  temp 30       : write /device/temperature"));
  iotbot.serialWrite(L("  mesaj <metin> : /device/status yaz", "  msg <text>    : write /device/status"));
  iotbot.serialWrite(L("  aktif         : /device/active tersine çevir", "  active        : flip /device/active"));
  iotbot.serialWrite(L("  gonder        : ışık değerini /device/light'a yaz", "  send          : write the light value to /device/light"));
  iotbot.serialWrite(L("  durum, dil", "  status, lang"));
  iotbot.serialWrite(L("  B3 butonu     : /device/active tersine çevir", "  B3 button     : flip /device/active"));
}

// Firebase'den oku, Seri Port'a ve LCD'ye yaz / read from Firebase, print to Serial and the LCD
void readAndShow() {
  lastReadMs = millis();
  if (!firebaseStarted) return;
  int temp = iotbot.fbServerGetInt("/device/temperature");
  String status = iotbot.fbServerGetString("/device/status");
  bool active = iotbot.fbServerGetBool("/device/active");
  activeValue = active;

  iotbot.serialWrite(String(L("Sıcaklık: ", "Temperature: ")) + temp + L("   Durum: ", "   Status: ") + status +
                     L("   Aktif: ", "   Active: ") + (active ? L("Evet", "Yes") : L("Hayır", "No")));

  char line[41];
  snprintf(line, sizeof(line), L("Sıcaklık: %d", "Temp: %d"), temp);
  lcdRow(0, line);
  String s = String(L("Durum: ", "Status: ")) + status;
  lcdRow(1, s.substring(0, 20).c_str());
  snprintf(line, sizeof(line), L("Aktif: %s", "Active: %s"), active ? L("Evet", "Yes") : L("Hayır", "No"));
  lcdRow(2, line);
  lcdRow(3, L("Firebase'den okundu", "Read from Firebase"));
}

void startFirebase() {
  iotbot.lcdShowStatus(L("WiFi Bağlandı", "WiFi Connected"), L("Firebase başlıyor", "Init Firebase"), true);
  iotbot.buzzerPlayTone(1500, 200);

  // Firebase'i başlat ve kullanıcıyı doğrula (en fazla ~25 sn) / start Firebase and verify the user (max ~25 s)
  iotbot.fbServerSetandStartWithUser(FIREBASE_PROJECT_URL, FIREBASE_API_KEY, USER_EMAIL, USER_PASSWORD);
  firebaseStarted = true;

  iotbot.lcdShowStatus(L("Firebase Hazır", "Firebase Ready"), L("Veri gönderiliyor", "Sending Data"), true);
  iotbot.buzzerPlayTone(2000, 300);

  // Firebase'e ilk verileri yaz / write the first data to Firebase
  iotbot.fbServerSetInt("/device/temperature", 25);
  iotbot.fbServerSetString("/device/status", "Online");
  iotbot.fbServerSetBool("/device/active", true);
  iotbot.serialWrite(L("Veriler Firebase'e gönderildi.", "Data sent to Firebase."));
  readAndShow();
}

void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  bool hasValue = space > 0;
  int value = hasValue ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
    return;
  }
  if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    printHelp();
    return;
  }
  if (word == "durum" || word == "status") {
    iotbot.serialWrite(String("WiFi: ") + (wifiOk() ? String(L("bağlı, IP ", "connected, IP ")) + iotbot.wifiGetIPAddress() : String(L("YOK", "NONE"))));
    iotbot.serialWrite(String("Firebase: ") + (firebaseStarted ? L("başlatıldı", "started") : L("henüz başlamadı (WiFi bekleniyor)", "not started yet (waiting for WiFi)")));
    return;
  }
  if (!firebaseStarted) {
    iotbot.serialWrite(L("Firebase henüz hazır değil (WiFi bekleniyor).", "Firebase is not ready yet (waiting for WiFi)."));
    return;
  }

  if (word == "oku" || word == "read") {
    readAndShow();
  } else if ((word == "sicaklik" || word == "temp") && hasValue) {
    iotbot.fbServerSetInt("/device/temperature", value);
    readAndShow();
  } else if (word == "mesaj" || word == "msg" || word == "message") {
    String text = argText();
    if (text.length() == 0) {
      iotbot.serialWrite(L("Kullanım: mesaj <metin>", "Usage: msg <text>"));
      return;
    }
    iotbot.fbServerSetString("/device/status", text);
    readAndShow();
  } else if (word == "aktif" || word == "active") {
    activeValue = !activeValue;
    iotbot.fbServerSetBool("/device/active", activeValue);
    readAndShow();
  } else if (word == "gonder" || word == "send") {
    int light = iotbot.ldrRead();
    iotbot.fbServerSetInt("/device/light", light);
    iotbot.serialWrite(String(L("Işık değeri yazıldı: ", "Light value written: ")) + light);
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.serialWrite(L("IoTBot Firebase örneği başlıyor...", "IoTBot Firebase example starting..."));

  iotbot.lcdShowLoading(L("WiFi bağlanıyor", "Connecting WiFi"));
  iotbot.buzzerPlayTone(1000, 200);

  // 1. adım: WiFi'ye bağlan / step 1: connect to WiFi
  iotbot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS);

  if (wifiOk()) {
    startFirebase(); // 2. ve 3. adım / steps 2 and 3
  } else {
    // Eskiden burada bekleyen sonsuz bir döngü vardı; artık loop() bağlantıyı bekler,
    // bu sırada seri komutlar da çalışır.
    // There used to be a blocking endless loop here; now loop() waits for the
    // connection, and serial commands keep working meanwhile.
    iotbot.serialWrite(L("WiFi bağlanamadı, tekrar deneniyor...", "WiFi failed, retrying..."));
    iotbot.lcdShowStatus(L("WiFi Yok", "WiFi Failed"), L("Tekrar deneniyor", "Retrying..."), false);
  }
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // WiFi gelince Firebase'i bir kez başlat / start Firebase once when WiFi is up
  if (!firebaseStarted && now - lastWifiCheckMs >= 1000) {
    lastWifiCheckMs = now;
    if (wifiOk()) {
      iotbot.serialWrite(L("Bağlantı başarılı! Firebase başlatılıyor.", "Connection success! Starting Firebase."));
      startFirebase();
    }
  }

  // B3 -> /device/active tersine çevir (sadece basıldığı an) / B3 -> flip /device/active (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3 && firebaseStarted) {
    activeValue = !activeValue;
    iotbot.fbServerSetBool("/device/active", activeValue);
    iotbot.buzzerPlayTone(1500, 40);
    readAndShow();
  }
  lastB3 = b3;

  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  // 4. adım: 10 saniyede bir Firebase'den oku / step 4: read from Firebase every 10 seconds
  if (firebaseStarted && now - lastReadMs >= kReadEveryMs) readAndShow();
  delay(10);
}
