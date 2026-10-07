/*
 * TR: SÜPER BLUETOOTH KUMANDA - İki IOTBOT arasında Kumanda (Master) / Robot (Slave)
 *  Bu kodu İKİ IOTBOT'a yükleyin:
 *  1) Rol seçimi: B1 = Kumanda, B2 = Robot (veya seri porttan "kumanda" / "robot").
 *  2) Robot bağlantı bekler. Kumanda 5 saniye etraftaki Bluetooth cihazlarını tarar
 *     ve listeyi LCD'de gösterir (Joystick Y: listede gez, B1: bağlan).
 *  3) Robot "Bağlantı isteği - izin ver?" diye sorar: B1 = EVET, B2 = HAYIR.
 *  4) Kumanda 4 haneli PIN'i girer (Joystick Y: rakamı değiştir, Joystick X sağa
 *     veya B2: sonraki hane, B1: gönder). Doğru PIN: 1234.
 *  5) Eşleşince kumanda menüsü: B1 = "Merhaba", B2 = sıcaklık, Joystick butonu =
 *     mesafe gönderir. Robot gelen veriyi LCD'de gösterir ve "ACK" ile cevap verir.
 *  - Bir telefon da Robot'a bağlanabilir: "baglan" / "connect" yazıp Robot'ta B1'e
 *    basın, sonra PIN'i (1234) yazın. Bağlıyken telefondan "merhaba"/"hello",
 *    "bip"/"beep", "durum"/"status" komutları çalışır.
 *  - Seri port komutları (115200 baud), Türkçe veya İngilizce:
 *      yardim / help         -> komut listesi
 *      kumanda / controller  -> rol: kumanda          robot / slave -> rol: robot
 *      sec 2 / select 2      -> listeden 2. cihaza bağlan (kumanda)
 *      tara / scan           -> yeniden tara (kumanda)
 *      pin 1234              -> PIN gönder (kumanda)
 *      evet / yes, hayir / no-> bağlantı isteğine cevap (robot)
 *      merhaba / hello, sicaklik / temp, mesafe / distance -> veri gönder (kumanda)
 *      mesaj <metin> / msg <text> -> karşı tarafa metin gönder (eşleşince)
 *      durum / status        -> durum bilgisi
 *      yeniden / restart     -> kartı yeniden başlat (rolü tekrar seçmek için)
 *      dil / lang            -> dili değiştir (Türkçe <-> English)
 *
 * EN: SUPER BLUETOOTH CONTROLLER - Controller (Master) / Robot (Slave) between two IOTBOTs
 *  Upload this code to TWO IOTBOTs:
 *  1) Role: B1 = Controller, B2 = Robot (or type "controller" / "robot" on serial).
 *  2) The Robot waits for a connection. The Controller scans nearby Bluetooth
 *     devices for 5 seconds and lists them on the LCD (Joystick Y: scroll, B1: connect).
 *  3) The Robot asks "Connection request - allow?": B1 = YES, B2 = NO.
 *  4) The Controller enters the 4-digit PIN (Joystick Y: change digit, Joystick X
 *     right or B2: next digit, B1: send). The correct PIN is 1234.
 *  5) Once paired, Controller menu: B1 = "Hello", B2 = temperature, Joystick button =
 *     distance. The Robot shows the data on its LCD and answers with "ACK".
 *  - A phone can also connect to the Robot: type "connect" / "baglan", press B1 on
 *    the Robot, then type the PIN (1234). While connected the phone can send
 *    "hello"/"merhaba", "beep"/"bip", "status"/"durum".
 *  - Serial port commands (115200 baud), Turkish or English:
 *      help / yardim         -> command list
 *      controller / kumanda  -> role: controller      slave / robot -> role: robot
 *      select 2 / sec 2      -> connect to device 2 of the list (controller)
 *      scan / tara           -> scan again (controller)
 *      pin 1234              -> send the PIN (controller)
 *      yes / evet, no / hayir-> answer the connection request (robot)
 *      hello / merhaba, temp / sicaklik, distance / mesafe -> send data (controller)
 *      msg <text> / mesaj <metin> -> send a text to the other side (when paired)
 *      status / durum        -> status information
 *      restart / yeniden     -> restart the board (to choose the role again)
 *      lang / dil            -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: Sıcaklık için DHT11'i IO25'e (P3), mesafe için ultrasonik
 * sensörü IO27 (TRIG) + IO32 (ECHO) soketine takın. Takılı değilse "sensör yok" gönderilir.
 * Plug a DHT11 into IO25 (P3) for temperature and the ultrasonic sensor into the
 * IO27 (TRIG) + IO32 (ECHO) socket for distance. If missing, "no sensor" is sent.
 *
 * Not / Note: Tarama (5 sn) ve bağlanma (birkaç sn) Bluetooth kütüphanesinde
 * bekleyen işlemlerdir; o sırada butonlar tepki vermez. / Scanning (5 s) and
 * connecting (a few s) are blocking calls of the Bluetooth library; the buttons do
 * not react during them.
 */

#define USE_BLUETOOTH
#define USE_DHT // Sıcaklık örneği için / for the temperature example
#include <IOTBOT.h>

IOTBOT iotbot;
BluetoothSerial *bt;

#define DHT_PIN IO25 // DHT11 sensörünün pini / DHT11 sensor pin

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Uygulama durumları / Application states
enum AppState {
  ROLE_SELECT,
  SLAVE_WAIT,        // Robot: bağlantı bekliyor / waiting for a connection
  SLAVE_WAIT_REQ,    // Robot: "REQ_AUTH" bekliyor / waiting for "REQ_AUTH"
  SLAVE_ASK,         // Robot: kullanıcıya soruyor / asking the user
  SLAVE_WAIT_PIN,    // Robot: PIN bekliyor / waiting for the PIN
  SLAVE_CONNECTED,
  MASTER_SCAN,
  MASTER_LIST,
  MASTER_WAIT_AUTH,  // Kumanda: robotun izni bekleniyor / waiting for the robot's permission
  MASTER_AUTH_PIN,   // Kumanda: PIN giriliyor / entering the PIN
  MASTER_WAIT_PIN,   // Kumanda: PIN sonucu bekleniyor / waiting for the PIN result
  MASTER_MENU
};

AppState state = ROLE_SELECT;
bool screenDirty = true;       // Ekran yeniden çizilmeli mi? / does the screen need a redraw?
uint32_t stateStartMs = 0;     // Bu duruma girilen an / when this state was entered

const char *SLAVE_NAME = "IOTBOT_SLAVE";
const char *MASTER_NAME = "IOTBOT_MASTER";
const char *SECRET_PIN = "1234"; // Robotun istediği PIN / the PIN the robot requires

BTScanResults *scanResults = nullptr;
bool scanBlocked = false; // Yeniden tarama mümkün değil / scanning again is not possible
int deviceCount = 0;
int selectedIndex = 0;

char enteredPIN[5] = "0000";
int pinCursor = 0;

// Kısa bilgi mesajı (ör. "Gönderildi: ...") - süresi dolunca ekran normale döner
// Short info message (e.g. "Sent: ...") - the screen returns to normal when it expires
String infoText;
uint32_t infoUntilMs = 0;

// Buton kenar algılama / button edge detection
bool lastB1 = false, lastB2 = false, lastJoyBtn = false;
uint32_t lastJoyMoveMs = 0;

void setState(AppState s) {
  state = s;
  stateStartMs = millis();
  screenDirty = true;
}

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
// Bluetooth satır okuyucu (beklemesiz) / Bluetooth line reader (non-blocking)
// ---------------------------------------------------------------------------
String btBuffer;
uint32_t lastBtCharMs = 0;

bool readBluetoothLine(String &line) {
  if (!bt) return false;
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

bool linked() { return bt && bt->hasClient(); } // Bağlı bir karşı taraf var mı? / is the other side connected?

// ---------------------------------------------------------------------------
// Ekran / Screen
// ---------------------------------------------------------------------------
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }
void lcdRow(int row, const String &text) { lcdRow(row, text.c_str()); }

void showInfo(const String &line1, const String &line2, uint32_t ms) {
  lcdRow(0, line1);
  lcdRow(1, line2);
  lcdRow(2, "");
  lcdRow(3, "");
  infoText = line1;
  infoUntilMs = millis() + ms;
  screenDirty = true; // Süre dolunca normal ekran / normal screen when it expires
}

void drawScreen() {
  char line[41];
  switch (state) {
    case ROLE_SELECT:
      lcdRow(0, L("Rol seçin:", "Select role:"));
      lcdRow(1, L("B1: Kumanda", "B1: Controller"));
      lcdRow(2, L("B2: Robot", "B2: Robot (slave)"));
      lcdRow(3, L("(veya seri komut)", "(or serial command)"));
      break;
    case SLAVE_WAIT:
    case SLAVE_WAIT_REQ:
      lcdRow(0, L("Rol: Robot", "Role: Robot"));
      lcdRow(1, SLAVE_NAME);
      lcdRow(2, state == SLAVE_WAIT ? L("Bağlantı bekleniyor", "Waiting connection") : L("Cihaz bağlandı!", "Device connected!"));
      lcdRow(3, state == SLAVE_WAIT ? "" : L("İstek bekleniyor...", "Waiting request..."));
      break;
    case SLAVE_ASK:
      lcdRow(0, L("Bağlantı isteği!", "Connection request!"));
      lcdRow(1, L("İzin verilsin mi?", "Allow it?"));
      lcdRow(2, L("B1: EVET  B2: HAYIR", "B1: YES   B2: NO"));
      lcdRow(3, "");
      break;
    case SLAVE_WAIT_PIN:
      lcdRow(0, L("PIN kodu", "Waiting for"));
      lcdRow(1, L("bekleniyor...", "PIN code..."));
      lcdRow(2, "");
      lcdRow(3, "");
      break;
    case SLAVE_CONNECTED:
      lcdRow(0, L("Eşleşme tamam!", "Pairing done!"));
      lcdRow(1, L("Veri bekleniyor...", "Waiting for data..."));
      lcdRow(2, "");
      lcdRow(3, "");
      break;
    case MASTER_SCAN:
      lcdRow(0, L("Taranıyor...", "Scanning..."));
      lcdRow(1, L("Lütfen bekleyin", "Please wait"));
      lcdRow(2, L("(5 saniye)", "(5 seconds)"));
      lcdRow(3, "");
      break;
    case MASTER_LIST: {
      BTAdvertisedDevice *device = scanResults->getDevice(selectedIndex);
      snprintf(line, sizeof(line), L("Cihaz seç (%d/%d):", "Select (%d/%d):"), selectedIndex + 1, deviceCount);
      lcdRow(0, line);
      String name = device->getName().c_str();
      if (name.length() == 0) name = "?";
      lcdRow(1, ("> " + name).substring(0, 20));
      lcdRow(2, String(device->getAddress().toString().c_str()));
      lcdRow(3, L("Y:gez  B1:bağlan", "Y:scroll B1:connect"));
      break;
    }
    case MASTER_WAIT_AUTH:
      lcdRow(0, L("Bağlandı!", "Connected!"));
      lcdRow(1, L("Robotun izni", "Waiting for the"));
      lcdRow(2, L("bekleniyor...", "robot's permission"));
      lcdRow(3, "");
      break;
    case MASTER_AUTH_PIN: {
      lcdRow(0, L("PIN kodunu girin:", "Enter PIN code:"));
      String shown = "  ";
      for (int i = 0; i < 4; i++) {
        if (i == pinCursor) shown += "[" + String(enteredPIN[i]) + "]";
        else shown += " " + String(enteredPIN[i]) + " ";
      }
      lcdRow(1, shown);
      lcdRow(2, L("Y:rakam X/B2:sonraki", "Y:digit X/B2:next"));
      lcdRow(3, L("B1: gönder", "B1: send"));
      break;
    }
    case MASTER_WAIT_PIN:
      lcdRow(0, L("PIN gönderildi...", "PIN sent..."));
      lcdRow(1, "");
      lcdRow(2, "");
      lcdRow(3, "");
      break;
    case MASTER_MENU:
      lcdRow(0, L("Ana Menü:", "Main Menu:"));
      lcdRow(1, L("B1: Merhaba gönder", "B1: Send hello"));
      lcdRow(2, L("B2: Sıcaklık gönder", "B2: Send temp"));
      lcdRow(3, L("Joy btn: Mesafe", "Joy btn: Distance"));
      break;
  }
}

// ---------------------------------------------------------------------------
// Ortak işlemler / Common actions
// ---------------------------------------------------------------------------
void printHelp() {
  iotbot.serialWrite(L("---- SÜPER BLUETOOTH KUMANDA - Komutlar ----", "---- SUPER BLUETOOTH CONTROLLER - Commands ----"));
  iotbot.serialWrite(L("  kumanda / robot   : rol seç (açılışta)", "  controller / robot : choose the role (at startup)"));
  iotbot.serialWrite(L("  tara, sec 2       : tara / listeden seç (kumanda)", "  scan, select 2    : scan / pick from list (controller)"));
  iotbot.serialWrite(L("  pin 1234          : PIN gönder (kumanda)", "  pin 1234          : send the PIN (controller)"));
  iotbot.serialWrite(L("  evet / hayir      : isteğe cevap (robot)", "  yes / no          : answer the request (robot)"));
  iotbot.serialWrite(L("  merhaba, sicaklik, mesafe : veri gönder (kumanda)", "  hello, temp, distance : send data (controller)"));
  iotbot.serialWrite(L("  mesaj <metin>     : karşı tarafa metin gönder", "  msg <text>        : send a text to the other side"));
  iotbot.serialWrite(L("  durum, yeniden, dil, yardim", "  status, restart, lang, help"));
}

void printStatus() {
  const char *names[] = {"ROLE_SELECT", "SLAVE_WAIT", "SLAVE_WAIT_REQ", "SLAVE_ASK", "SLAVE_WAIT_PIN", "SLAVE_CONNECTED",
                         "MASTER_SCAN", "MASTER_LIST", "MASTER_WAIT_AUTH", "MASTER_AUTH_PIN", "MASTER_WAIT_PIN", "MASTER_MENU"};
  iotbot.serialWrite(String(L("Durum: ", "State: ")) + names[state] +
                     L("   Bağlantı: ", "   Link: ") + (linked() ? L("VAR", "YES") : L("YOK", "NO")));
  if (state == MASTER_LIST) {
    for (int i = 0; i < deviceCount; i++) {
      BTAdvertisedDevice *d = scanResults->getDevice(i);
      iotbot.serialWrite(String("  ") + (i + 1) + ") " + d->getName().c_str() + "  " + d->getAddress().toString().c_str());
    }
  }
}

void startAsController() {
  // Kumanda MASTER modunda başlamalı (begin(ad, true)); yoksa connect() hep başarısız olur.
  // The controller must start in MASTER mode (begin(name, true)); otherwise connect() always fails.
  bt->begin(MASTER_NAME, true);
  iotbot.serialWrite(L("Rol: KUMANDA. Tarama başlıyor...", "Role: CONTROLLER. Starting scan..."));
  iotbot.buzzerPlayTone(1500, 60);
  setState(MASTER_SCAN);
}

void startAsRobot() {
  iotbot.bluetoothStart(SLAVE_NAME); // Robot = slave (normal mod) / robot = slave (normal mode)
  iotbot.serialWrite(L("Rol: ROBOT. Bağlantı bekleniyor (IOTBOT_SLAVE)...", "Role: ROBOT. Waiting for a connection (IOTBOT_SLAVE)..."));
  iotbot.buzzerPlayTone(1000, 60);
  setState(SLAVE_WAIT);
}

void connectToSelected() {
  BTAdvertisedDevice *device = scanResults->getDevice(selectedIndex);
  String name = device->getName().c_str();
  showInfo(L("Bağlanılıyor:", "Connecting to:"), name, 0);
  iotbot.serialWrite(String(L("Bağlanılıyor: ", "Connecting to: ")) + name);
  if (bt->connect(device->getAddress())) { // Kütüphane burada birkaç sn bekler / the library waits a few s here
    bt->println("REQ_AUTH"); // Protokolü başlat / start the protocol
    iotbot.serialWrite(L("Bağlandı, robotun izni bekleniyor...", "Connected, waiting for the robot's permission..."));
    infoUntilMs = 0;
    setState(MASTER_WAIT_AUTH);
  } else {
    iotbot.serialWrite(L("Bağlantı başarısız. Yeniden taranıyor.", "Connection failed. Scanning again."));
    showInfo(L("Bağlantı", "Connection"), L("başarısız!", "failed!"), 2000);
    setState(MASTER_SCAN);
  }
}

void sendPin() {
  bt->println(enteredPIN);
  iotbot.serialWrite(String(L("PIN gönderildi: ", "PIN sent: ")) + enteredPIN);
  setState(MASTER_WAIT_PIN);
}

void masterSend(const String &text, const String &shortText) {
  if (!linked()) {
    iotbot.serialWrite(L("Bağlantı yok.", "Not connected."));
    return;
  }
  bt->println(text);
  iotbot.serialWrite(String(L("Gönderildi: ", "Sent: ")) + text);
  showInfo(L("Gönderildi:", "Sent:"), shortText, 1500);
}

void sendHello() { masterSend(L("Kumandadan merhaba!", "Hello from the controller!"), L("Merhaba", "Hello")); }

void sendTemperature() {
  int temp = iotbot.moduleDhtTempReadC(DHT_PIN);
  if (temp == -999) masterSend(L("Sıcaklık: sensör yok", "Temp: no sensor"), L("Sıcaklık: yok", "Temp: none"));
  else masterSend(String(L("Sıcaklık: ", "Temp: ")) + temp + " C", String(L("Sıcaklık: ", "Temp: ")) + temp + " C");
}

void sendDistance() {
  int dist = iotbot.moduleUltrasonicDistanceRead();
  if (dist == 0) masterSend(L("Mesafe: ölçülemedi", "Distance: no echo"), L("Mesafe: yok", "Distance: none"));
  else masterSend(String(L("Mesafe: ", "Distance: ")) + dist + " cm", String(L("Mesafe: ", "Distance: ")) + dist + " cm");
}

void slaveAnswer(bool allow) {
  if (allow) {
    bt->println("AUTH_OK"); // Kumanda PIN'e geçsin / the controller moves on to the PIN
    iotbot.serialWrite(L("İzin verildi, PIN bekleniyor.", "Allowed, waiting for the PIN."));
    setState(SLAVE_WAIT_PIN);
  } else {
    bt->println("AUTH_DENY");
    iotbot.serialWrite(L("İstek reddedildi.", "Request denied."));
    bt->disconnect();
    setState(SLAVE_WAIT);
  }
}

// Bağlantı koptuysa ilgili bekleme durumuna dön / go back to waiting if the link dropped
void checkLinkLost() {
  bool slaveSide = (state == SLAVE_WAIT_REQ || state == SLAVE_ASK || state == SLAVE_WAIT_PIN || state == SLAVE_CONNECTED);
  bool masterSide = (state == MASTER_WAIT_AUTH || state == MASTER_AUTH_PIN || state == MASTER_WAIT_PIN || state == MASTER_MENU);
  if ((slaveSide || masterSide) && !linked()) {
    iotbot.serialWrite(L("Bağlantı koptu.", "Connection lost."));
    iotbot.buzzerPlayTone(400, 200);
    showInfo(L("Bağlantı koptu", "Connection lost"), L("Yeniden başlıyor...", "Restarting..."), 2000);
    setState(slaveSide ? SLAVE_WAIT : MASTER_SCAN);
  }
}

// ---------------------------------------------------------------------------
// Bluetooth'tan gelen satırlar / Lines coming over Bluetooth
// ---------------------------------------------------------------------------
void handleBluetoothLine(const String &line) {
  String cmd = normalizeCommand(line);
  iotbot.serialWrite(String(L("BT'den: ", "From BT: ")) + line);

  switch (state) {
    case SLAVE_WAIT_REQ:
      // "REQ_AUTH" kumandadan; telefon kolaylığı için "baglan"/"connect" de olur
      // "REQ_AUTH" comes from the controller; "baglan"/"connect" also works for phones
      if (line == "REQ_AUTH" || cmd == "baglan" || cmd == "connect") {
        iotbot.buzzerPlayTone(1800, 100);
        setState(SLAVE_ASK);
      }
      break;
    case SLAVE_WAIT_PIN:
      if (line == SECRET_PIN) {
        bt->println("PIN_OK");
        iotbot.serialWrite(L("PIN doğru - eşleşme tamam!", "Correct PIN - pairing done!"));
        iotbot.buzzerPlayTone(1000, 300);
        setState(SLAVE_CONNECTED);
      } else {
        bt->println("PIN_FAIL");
        iotbot.serialWrite(L("Yanlış PIN! Bağlantı kesildi.", "Wrong PIN! Disconnected."));
        showInfo(L("Yanlış PIN!", "Wrong PIN!"), L("Bağlantı kesildi", "Disconnected"), 2000);
        bt->disconnect();
        setState(SLAVE_WAIT);
      }
      break;
    case SLAVE_CONNECTED:
      // Telefon/kumanda komutları (TR + EN) / phone/controller commands (TR + EN)
      if (cmd == "bip" || cmd == "beep") {
        iotbot.buzzerPlayTone(1500, 100);
        bt->println(L("Bip!", "Beep!"));
      } else if (cmd == "durum" || cmd == "status") {
        bt->println(String(L("Işık: ", "Light: ")) + iotbot.ldrRead() + L("  Pot: ", "  Pot: ") + iotbot.potentiometerRead());
      } else if (cmd == "merhaba" || cmd == "hello") {
        bt->println(L("Robottan merhaba!", "Hello from the robot!"));
        iotbot.buzzerPlayTone(1200, 80);
      } else {
        bt->println("ACK: " + line); // Gelen veriyi onayla / acknowledge the data
        iotbot.buzzerPlayTone(2000, 50);
      }
      showInfo(L("Gelen veri:", "Received data:"), line.substring(0, 20), 3000);
      break;
    case MASTER_WAIT_AUTH:
      if (line == "AUTH_OK") {
        strcpy(enteredPIN, "0000");
        pinCursor = 0;
        setState(MASTER_AUTH_PIN);
      } else if (line == "AUTH_DENY") {
        showInfo(L("İzin verilmedi!", "Access denied!"), L("(robot reddetti)", "(by the robot)"), 2000);
        bt->disconnect();
        setState(MASTER_SCAN);
      }
      break;
    case MASTER_WAIT_PIN:
      if (line == "PIN_OK") {
        iotbot.serialWrite(L("Erişim izni verildi! Menü açıldı.", "Access granted! Menu opened."));
        iotbot.buzzerPlayTone(1000, 300);
        setState(MASTER_MENU);
      } else if (line == "PIN_FAIL") {
        showInfo(L("Erişim reddedildi!", "Access denied!"), L("Yanlış PIN", "Wrong PIN"), 2000);
        bt->disconnect();
        setState(MASTER_SCAN);
      }
      break;
    case MASTER_MENU:
      showInfo(L("Robottan cevap:", "Robot replied:"), line.substring(0, 20), 1500);
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// Seri komutlar / Serial commands
// ---------------------------------------------------------------------------
void handleCommand(const String &cmd) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  int value = (space > 0) ? cmd.substring(space + 1).toInt() : 0;

  if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    screenDirty = true;
  } else if (word == "durum" || word == "status") {
    printStatus();
  } else if (word == "yeniden" || word == "restart") {
    iotbot.serialWrite(L("Yeniden başlatılıyor...", "Restarting..."));
    delay(100);
    ESP.restart();
  } else if ((word == "kumanda" || word == "controller" || word == "master") && state == ROLE_SELECT) {
    startAsController();
  } else if ((word == "robot" || word == "slave") && state == ROLE_SELECT) {
    startAsRobot();
  } else if ((word == "tara" || word == "scan") && (state == MASTER_LIST || state == MASTER_SCAN)) {
    setState(MASTER_SCAN);
  } else if ((word == "sec" || word == "select") && state == MASTER_LIST) {
    if (value >= 1 && value <= deviceCount) {
      selectedIndex = value - 1;
      connectToSelected();
    } else {
      iotbot.serialWrite(String(L("1 ile ", "Pick 1 to ")) + deviceCount + L(" arası seçin.", "."));
    }
  } else if (word == "pin" && state == MASTER_AUTH_PIN) {
    String p = argText();
    if (p.length() == 4) {
      strncpy(enteredPIN, p.c_str(), 4);
      enteredPIN[4] = '\0';
      sendPin();
    } else {
      iotbot.serialWrite(L("PIN 4 haneli olmalı (ör. pin 1234).", "The PIN must have 4 digits (e.g. pin 1234)."));
    }
  } else if ((word == "evet" || word == "yes") && state == SLAVE_ASK) {
    slaveAnswer(true);
  } else if ((word == "hayir" || word == "no") && state == SLAVE_ASK) {
    slaveAnswer(false);
  } else if ((word == "merhaba" || word == "hello") && state == MASTER_MENU) {
    sendHello();
  } else if ((word == "sicaklik" || word == "temp" || word == "temperature") && state == MASTER_MENU) {
    sendTemperature();
  } else if ((word == "mesafe" || word == "distance") && state == MASTER_MENU) {
    sendDistance();
  } else if (word == "mesaj" || word == "msg" || word == "message") {
    String text = argText();
    if (text.length() == 0) iotbot.serialWrite(L("Kullanım: mesaj <metin>", "Usage: msg <text>"));
    else if (state != MASTER_MENU && state != SLAVE_CONNECTED) iotbot.serialWrite(L("Önce eşleşme tamamlanmalı.", "Finish pairing first."));
    else masterSend(text, text.substring(0, 20));
  } else {
    iotbot.serialWrite(String(L("Bu durumda geçersiz/bilinmeyen komut: ", "Invalid here / unknown command: ")) + cmd +
                       L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  bt = iotbot.getBluetoothObject();
  iotbot.lcdClear();
  setState(ROLE_SELECT);
  iotbot.serialWrite(L("Süper Bluetooth Kumanda. Rol seçin: B1 = Kumanda, B2 = Robot.",
                       "Super Bluetooth Controller. Choose a role: B1 = Controller, B2 = Robot."));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  // Butonlar: sadece basıldığı an / buttons: only at the moment of pressing
  bool b1 = iotbot.button1Read();
  bool b2 = iotbot.button2Read();
  bool joyBtn = !iotbot.joystickButtonRead(); // LOW = basılı / LOW = pressed
  bool b1Pressed = b1 && !lastB1, b2Pressed = b2 && !lastB2, joyPressed = joyBtn && !lastJoyBtn;
  lastB1 = b1; lastB2 = b2; lastJoyBtn = joyBtn;

  // Joystick: 250 ms'de en fazla bir adım / at most one step every 250 ms
  int joyY = iotbot.joystickYRead();
  int joyX = iotbot.joystickXRead();
  int yStep = 0;
  bool xRight = false;
  if (now - lastJoyMoveMs >= 250) {
    if (joyY > 3000) yStep = 1;
    else if (joyY < 1000) yStep = -1;
    xRight = joyX > 3000;
    if (yStep != 0 || xRight) lastJoyMoveMs = now;
  }

  String cmd;
  if (readCommand(cmd)) handleCommand(cmd);

  String line;
  if (readBluetoothLine(line)) handleBluetoothLine(line);

  checkLinkLost();

  switch (state) {
    case ROLE_SELECT:
      if (b1Pressed) startAsController();
      else if (b2Pressed) startAsRobot();
      break;

    case SLAVE_WAIT:
      if (linked()) {
        iotbot.serialWrite(L("Bir cihaz bağlandı, istek bekleniyor.", "A device connected, waiting for its request."));
        setState(SLAVE_WAIT_REQ);
      }
      break;

    case SLAVE_ASK:
      if (b1Pressed) slaveAnswer(true);
      else if (b2Pressed) slaveAnswer(false);
      break;

    case SLAVE_WAIT_PIN:
      if (now - stateStartMs > 30000) { // 30 sn içinde PIN gelmezse kes / drop it if no PIN within 30 s
        bt->println("PIN_FAIL");
        bt->disconnect();
        setState(SLAVE_WAIT);
      }
      break;

    case MASTER_SCAN:
      // Önceki bilgi mesajı (ör. "bağlantı başarısız") okunsun diye bekle
      // Wait so the previous info message (e.g. "connection failed") can be read
      if (scanBlocked || (infoUntilMs != 0 && (int32_t)(now - infoUntilMs) < 0)) break;
      drawScreen(); // Tarama ekranını hemen göster / show the scan screen right away
      scanResults = bt->discover(5000); // 5 sn sürer / takes 5 s
      if (scanResults == nullptr) {
        // Kütüphane, daha önce bir adrese bağlanılmışsa yeniden taramaya izin vermiyor.
        // The library refuses to scan again after it has connected to an address once.
        scanBlocked = true;
        showInfo(L("Tarama yapılamadı", "Cannot scan again"), L("'yeniden' yazın", "type 'restart'"), 3600000UL);
        iotbot.serialWrite(L("Tekrar taramak için kartı yeniden başlatın: yeniden", "Restart the board to scan again: restart"));
      } else if (scanResults->getCount() > 0) {
        deviceCount = scanResults->getCount();
        selectedIndex = 0;
        iotbot.serialWrite(String(L("Bulunan cihaz: ", "Devices found: ")) + deviceCount);
        setState(MASTER_LIST);
        printStatus();
      } else {
        iotbot.serialWrite(L("Cihaz bulunamadı, tekrar taranıyor.", "No devices found, scanning again."));
        showInfo(L("Cihaz bulunamadı", "No devices found"), L("Tekrar deneniyor...", "Retrying..."), 1500);
      }
      break;

    case MASTER_LIST:
      if (yStep != 0) {
        selectedIndex = (selectedIndex + yStep + deviceCount) % deviceCount;
        screenDirty = true;
      }
      if (b1Pressed) connectToSelected();
      break;

    case MASTER_WAIT_AUTH:
      if (now - stateStartMs > 20000) { // Robottaki kişiye 20 sn süre / 20 s for the person at the robot
        showInfo(L("Cevap yok", "No response"), L("Zaman aşımı", "Timeout"), 2000);
        bt->disconnect();
        setState(MASTER_SCAN);
      }
      break;

    case MASTER_AUTH_PIN:
      if (yStep != 0) {
        char c = enteredPIN[pinCursor];
        if (yStep > 0) c = (c < '9') ? c + 1 : '0';
        else c = (c > '0') ? c - 1 : '9';
        enteredPIN[pinCursor] = c;
        screenDirty = true;
      }
      if (xRight || b2Pressed) {
        pinCursor = (pinCursor + 1) % 4;
        screenDirty = true;
      }
      if (b1Pressed) sendPin();
      break;

    case MASTER_WAIT_PIN:
      if (now - stateStartMs > 5000) {
        showInfo(L("Cevap yok", "No response"), L("Zaman aşımı", "Timeout"), 2000);
        bt->disconnect();
        setState(MASTER_SCAN);
      }
      break;

    case MASTER_MENU:
      if (b1Pressed) sendHello();
      else if (b2Pressed) sendTemperature();
      else if (joyPressed) sendDistance();
      break;

    default:
      break;
  }

  // Bilgi mesajı süresi dolduysa (veya ekran değiştiyse) normal ekranı çiz
  // Draw the normal screen when the info message expired (or the screen changed)
  if (screenDirty && (infoUntilMs == 0 || (int32_t)(now - infoUntilMs) >= 0)) {
    screenDirty = false;
    infoUntilMs = 0;
    drawScreen();
  }
}
