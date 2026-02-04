/*
 * CODLAI IOTBOT Library
 * 
 * Structure Information:
 * This library uses a modular structure to optimize memory usage and compilation time.
 * Features are enabled/disabled via definitions in the main sketch (e.g., #define USE_DHT).
 * 
 * IMPORTANT: Define feature flags BEFORE including this library in your sketch.
 */

#ifndef IOTBOT_H
#define IOTBOT_H

#include <Arduino.h>

#if defined(ESP32)

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <time.h>
#include <Stepper.h>
#include <LittleFS.h> // Ensure LittleFS is available for dependencies like Firebase
#include <WiFi.h> // Ensure WiFi is available for dependencies like Firebase

// Feature dependent includes
#if defined(USE_SERVO)
#include <ESP32Servo.h>
#endif

#if defined(USE_DHT)
#include <DHT.h>
#endif

#if defined(USE_NEOPIXEL)
#include <Adafruit_NeoPixel.h>
#endif

#if defined(USE_IR)
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#endif

#if defined(USE_RFID)
#include <SPI.h>
#include <MFRC522.h>
#endif

#if defined(USE_SERVER)
#ifndef USE_WIFI
#define USE_WIFI
#endif
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#endif

#if defined(USE_FIREBASE)
#ifndef USE_WIFI
#define USE_WIFI
#endif
#include <Firebase_ESP_Client.h>
#include <ArduinoJson.h>
#endif

#if defined(USE_OTA)
#ifndef USE_WIFI
#define USE_WIFI
#endif
#include <ArduinoOTA.h>
#endif

#if defined(USE_WIFI)
#include <WiFi.h>
#endif

#if defined(USE_ESPNOW)
#ifndef USE_WIFI
#define USE_WIFI
#endif
#include <esp_now.h>
#if defined(ESP32)
#include <esp_wifi.h>
#endif
#endif

#if defined(USE_EMAIL)
#ifndef USE_WIFI
#define USE_WIFI
#endif
#include <ESP_Mail_Client.h>
#endif

#if defined(USE_TELEGRAM) || defined(USE_WEATHER) || defined(USE_WIKIPEDIA) || defined(USE_IFTTT)
#ifndef USE_WIFI
#define USE_WIFI
#endif
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#endif

#if defined(USE_BLUETOOTH) && defined(ESP32)
#include "BluetoothSerial.h"
#endif

// Structure to receive data via ESP-NOW
#ifndef CODLAI_ESPNOW_MESSAGE_DEFINED
#define CODLAI_ESPNOW_MESSAGE_DEFINED
typedef struct {
  uint8_t deviceType; // 1 = Armbot
  int axis1;
  int axis2;
  int axis3;
  int gripper;
  uint8_t action; // 0=None, 1=Horn, 2=Note
} CodlaiESPNowMessage;
#endif

// Pins
#define JOYSTICK_Y_PIN 34
#define JOYSTICK_X_PIN 35  //DEĞİŞti  15 olacaktı.
#define JOYSTICK_BUTTON_PIN 15//DEĞİŞti  35 olacaktı.
#define ENCODER_A_PIN 16
#define ENCODER_B_PIN 17
#define ENCODER_BUTTON_PIN 13
#define B1_AND_B2_BUTTON_PIN 4 //DEĞİŞECEK // wifi acıksa dijital okuma yapacak. 
#define RELAY_PIN 14
#define BUZZER_PIN 12
#define LDR_PIN 39
#define POT_PIN 36
#define B3_BUTTON_PIN 0
#define LCD_ADRESS 0x27
#define LED_BUILTIN 1

#define IO25 25
#define IO26 26
#define IO27 27
#define IO32 32
#define IO33 33

class IOTBOT
{
public:
  IOTBOT();
  void begin();
  void playIntro();

  /*********************************** Serial Port ***********************************
   */
  void serialStart(int baundrate);
  void serialWrite(const char *message);
  void serialWrite(String message);
  void serialWrite(long value);
  void serialWrite(int value);
  void serialWrite(float value);
  void serialWrite(bool value);
  int serialAvailable();
  String serialReadStringUntil(char terminator);

  /*********************************** BUZZER ***********************************
   */
  void buzzerPlayTone(int frequency, int duration);
  void buzzerPlay(int frequency, int duration); // Alias for consistency
  void buzzerStart(int frequency);
  void buzzerStop();
  void buzzerSoundIntro();
  void buzzertest();

  /*********************************** LCD SCREEN ***********************************
   */

  void lcdSetCursor(int col, int row);
  void lcdWriteMid(const char *line1, const char *line2, const char *line3, const char *line4);
  void lcdWrite(String text);
  void lcdWrite(int value);
  void lcdWrite(float value);
  void lcdWrite(bool value);
  void lcdWriteCR(int col, int row, const char *text);
  void lcdWriteCR(int col, int row, String text);
  void lcdWriteCR(int col, int row, int value);
  void lcdWriteCR(int col, int row, float value);
  void lcdWriteCR(int col, int row, bool value);
  void lcdWrite(const char *text);
  void lcdClear();
  void lcdtest();
  void lcdShowLoading(String message);
  void lcdShowStatus(String title, String status, bool isSuccess);

  /*********************************** LDR ***********************************
   */
  int ldrRead();
  void ldrtest();

  /*********************************** RELAY ***********************************
   */
  void relayWrite(bool status);
  void relaytest();

  /*********************************** POTANTIOMETER ***********************************
   */
  int potentiometerRead();
  void potentiometertest();

  /*********************************** JOYSTICK ***********************************
   */
  int joystickXRead();
  int joystickYRead();
  bool joystickButtonRead();
  void joysticktest();

  /*********************************** BUTTONS ***********************************
   */
  bool button1Read();
  bool button2Read();
  bool button3Read();
  void buttonsAnalogtest();

  /*********************************** ENCODER ***********************************
   */
  int encoderRead();
  bool encoderButtonRead();
  void encodertest();

  /*********************************** DRIVER AND MOTORS ***********************************
   */
  void moduleStepMotorMotion(int step, bool rotation, int accelometer, int speed);
  void moduleDCMotorGOClockWise(int speed);
  void moduleDCMotorGOCounterClockWise(int speed);
  void moduleDCMotorStop();
  void moduleDCMotorBrake();

  /*********************************** NTC Temp Sensor ***********************************
   */
  float moduleNtcTempRead(int pin);

  /*********************************** Magnetic Sensor ***********************************
   */
  bool moduleMagneticRead(int pin);

  /*********************************** Matris Button Sensor ***********************************
   */
  int moduleMatrisButtonAnalogRead(int pin);
  int moduleMatrisButtonNumberRead(int pin);

  /*********************************** Vibration Sensor ***********************************
   */
  bool moduleVibrationDigitalRead(int pin);
  int moduleVibrationAnalogRead(int pin);

  /*********************************** Ultrasonic Distance Sensor ***********************************
   */
  int moduleUltrasonicDistanceRead();

  /*********************************** Trafic Ligh Sensor ***********************************
   */
  void moduleTraficLightWrite(bool red, bool yellow, bool green);
  void moduleTraficLightWriteRed(bool red);
  void moduleTraficLightWriteYellow(bool yellow);
  void moduleTraficLightWriteGreen(bool green);

  /*********************************** Motion Sensor ***********************************
   */
  bool moduleMotionRead(int pin);

  /*********************************** Smoke Sensor ***********************************
   */
  int moduleSmokeRead(int pin);

  /*********************************** Mic Sensor ***********************************
   */
  int moduleMicRead(int pin);

  /*********************************** Soil Moisture Sensor ***********************************
   */
  int moduleSoilMoistureRead(int pin);

  /*********************************** Relay Sensor ***********************************
   */
  void moduleRelayWrite(int pin, bool status);

  /*********************************** OTHER PINS ***********************************
   */
  int analogReadPin(int pin);
  void analogWritePin(int pin, int value);
  bool digitalReadPin(int pin);
  void digitalWritePin(int pin, bool value);

  /*********************************** EEPROM  ***********************************
   */
  bool eepromBegin(size_t size = 1024);
  bool eepromCommit();
  void eepromEnd();

  bool eepromWriteByte(int address, uint8_t value);
  uint8_t eepromReadByte(int address, uint8_t defaultValue = 0);

  // NOTE: eepromWriteInt/eepromReadInt store 16-bit (2 bytes) for backward compatibility.
  void eepromWriteInt(int address, int value);
  int eepromReadInt(int address);

  bool eepromWriteInt32(int address, int32_t value);
  int32_t eepromReadInt32(int address, int32_t defaultValue = 0);
  bool eepromWriteUInt32(int address, uint32_t value);
  uint32_t eepromReadUInt32(int address, uint32_t defaultValue = 0);
  bool eepromWriteFloat(int address, float value);
  float eepromReadFloat(int address, float defaultValue = 0.0f);

  // Stores: [uint16 length][bytes...]
  bool eepromWriteString(int address, const String &value, uint16_t maxLen = 128);
  String eepromReadString(int address, uint16_t maxLen = 128);

  bool eepromWriteBytes(int address, const uint8_t *data, size_t len);
  bool eepromReadBytes(int address, uint8_t *data, size_t len);
  bool eepromClear(int startAddress = 0, size_t length = 0, uint8_t fill = 0xFF);

  // Record helpers (recommended): Stores a blob with header + CRC32
  // Layout: [uint16 magic][uint16 version][uint16 len][uint32 crc32][bytes...]
  uint32_t eepromCrc32(const uint8_t *data, size_t len, uint32_t seed = 0xFFFFFFFF);
  bool eepromWriteRecord(int address, const uint8_t *data, uint16_t len, uint16_t version = 1);
  bool eepromReadRecord(int address, uint8_t *out, uint16_t maxLen, uint16_t *outLen = nullptr, uint16_t *outVersion = nullptr);

  /*********************************** Servo Motor Sensor ***********************************
   */
#if defined(USE_SERVO)
  void moduleServoGoAngle(int pin, int angle, int acceleration);
#endif

  /*********************************** DHT Sensor ***********************************
   */
#if defined(USE_DHT)
  int moduleDhtTempReadC(int pin);
  int moduleDthFeelingTempC(int pin);
  int moduleDhtTempReadF(int pin);
  int moduleDthFeelingTempF(int pin);
  int moduleDhtHumRead(int pin);
#endif

  /*********************************** Smart LED Sensor ***********************************
   */
#if defined(USE_NEOPIXEL)
  void extendSmartLEDPrepare(int pin, int numLEDs);
  void extendSmartLEDFill(int startLED, int endLED, int red, int green, int blue);
  void moduleSmartLEDPrepare(int pin);                             // Initialize NeoPixel strip
  void moduleSmartLEDWrite(int led, int red, int green, int blue); // Write RGB values to specific LED
  void moduleSmartLEDRainbowEffect(int wait);                      // Rainbow effect
  void moduleSmartLEDRainbowTheaterChaseEffect(int wait);          // Rainbow theater chase effect
  void moduleSmartLEDTheaterChaseEffect(uint32_t color, int wait); // Theater chase effect
  void moduleSmartLEDColorWipeEffect(uint32_t color, int wait);    // Color wipe effect
  uint32_t getColor(int red, int green, int blue);                 // Helper function for creating colors
#endif

  /*********************************** IR Sensor ***********************************
   */
#if defined(USE_IR)
  String moduleIRReadHex(int pin);
  int moduleIRReadDecimalx32(int pin);
  int moduleIRReadDecimalx8(int pin);
#endif

  /*********************************** RFID Sensor ***********************************
   */
#if defined(USE_RFID)
  int moduleRFIDRead(); // RFID kart ID oku / Read RFID card ID
#endif

  /*********************************** Server  ***********************************
   */
#if defined(USE_SERVER)
  void serverStart(const char *mode, const char *ssid, const char *password);
  void serverCreateLocalPage(const char *url, const char *WEBPageScript, const char *WEBPageCSS, const char *WEBPageHTML, size_t bufferSize = 4096);
  void serverHandleDNS();
  void serverContinue();
#endif

  /*********************************** Firebase Server  ***********************************
   */
#if defined(USE_FIREBASE)
  // 📡 Firebase Server Functions
  void fbServerSetandStartWithUser(const char *projectURL, const char *secretKey, const char *userMail, const char *mailPass); // projectURL: YOUR_FIREBASE_PROJECT_ID.firebaseio.com / secretKey: YOUR_FIREBASE_DATABASE_SECRET

  // 🔄 Firebase Database Write Functions
  void fbServerSetInt(const char *dataPath, int data);
  void fbServerSetFloat(const char *dataPath, float data);
  void fbServerSetString(const char *dataPath, String data);
  void fbServerSetDouble(const char *dataPath, double data);
  void fbServerSetBool(const char *dataPath, bool data);
  void fbServerSetJSON(const char *dataPath, String data);

  // 📥 Firebase Database Read Functions
  int fbServerGetInt(const char *dataPath);
  float fbServerGetFloat(const char *dataPath);
  String fbServerGetString(const char *dataPath);
  double fbServerGetDouble(const char *dataPath);
  bool fbServerGetBool(const char *dataPath);
  String fbServerGetJSON(const char *dataPath);
#endif

  /*********************************** WiFi  ***********************************
   */
#if defined(USE_WIFI)
  void wifiStartAndConnect(const char *ssid, const char *pass);
  bool wifiConnectionControl();
  String wifiGetMACAddress();
  String wifiGetIPAddress();
#endif

  /*********************************** OTA (Over-The-Air) ***********************************
   * TR: WiFi baglantisindan SONRA cagirilmalidir.
   * EN: Must be called AFTER WiFi connection is established.
   */
#if defined(USE_OTA)
  void otaBegin(const char *hostname = "CODLAI-IOTBOT", const char *password = "1234", uint16_t port = 3232);
  void otaHandle();
#endif

  /*********************************** NTP Time ***********************************
   * TR: WiFi baglantisindan SONRA cagirilmalidir.
   * EN: Must be called AFTER WiFi connection is established.
   */
  // TR/EN: Recommended simple setup for blocks.
  // timezoneHours: UTC offset in hours (e.g., Turkey is +3)
  bool ntpBegin(int timezoneHours = 0, const char *ntpServer = "pool.ntp.org", int daylightOffsetHours = 0, uint32_t timeoutMs = 10000);
  bool ntpSync(const char *ntpServer = "pool.ntp.org", long gmtOffsetSec = 0, int daylightOffsetSec = 0, uint32_t timeoutMs = 10000);
  bool ntpIsTimeValid(time_t minEpoch = 1609459200);
  time_t ntpGetEpoch();
  String ntpGetDateTimeString();

  /*********************************** ESP-NOW ***********************************
   */
#if defined(USE_ESPNOW)
  void initESPNow();
  void setWiFiChannel(int channel);
  void sendESPNow(const uint8_t *macAddr, const uint8_t *data, int len);
  void registerOnRecv(esp_now_recv_cb_t cb);

  // ESP-NOW Data Handling
  CodlaiESPNowMessage receivedData;
  volatile bool newData = false;
  volatile bool lastSendStatus = false; // Added for send status tracking

  // Static helper to get instance for callbacks
  static IOTBOT*& _getInstance() {
      static IOTBOT* i = nullptr;
      return i;
  }

  // Internal callback for send status
  static void _onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
      if (_getInstance()) {
          _getInstance()->lastSendStatus = (status == ESP_NOW_SEND_SUCCESS);
      }
  }

  void startListening() {
      _getInstance() = this;
      #if defined(ESP32)
      registerOnRecv([](const uint8_t *mac, const uint8_t *incomingData, int len) {
      #elif defined(ESP8266)
      registerOnRecv([](uint8_t *mac, uint8_t *incomingData, uint8_t len) {
      #endif
          if (_getInstance() && len == sizeof(CodlaiESPNowMessage)) {
              memcpy(&_getInstance()->receivedData, incomingData, sizeof(CodlaiESPNowMessage));
              _getInstance()->newData = true;
          }
      });
  }
#endif

  /*********************************** Telegram ***********************************
   */
#if defined(USE_TELEGRAM)
  void sendTelegram(String token, String chatId, String message);
#endif

  /*********************************** IFTTT ***********************************
   */
#if defined(USE_IFTTT)
  bool triggerIFTTTEvent(const String &eventName, const String &webhookKey, const String &jsonPayload = "{}");
#endif

  /*********************************** Bluetooth ***********************************
   */
#if defined(USE_BLUETOOTH) && defined(ESP32)
  void bluetoothStart(String name, String pin = "");
  bool bluetoothConnect(String remoteName);
  void bluetoothWrite(String message);
  String bluetoothRead();
  BluetoothSerial* getBluetoothObject(); // Gelişmiş işlemler için nesneye erişim
#endif

  /*********************************** Multi-Tasking (ESP32 Only) ***********************************
   */
#if defined(ESP32)
  void createTask(TaskFunction_t taskFunction, const char *name, int coreID = 1, int stackSize = 10000, int priority = 1);
  void createTask(void (*taskFunction)(), const char *name, int coreID = 1, int stackSize = 10000, int priority = 1);
  void createLoopTask(void (*taskFunction)(), const char *name, int coreID = 1, int priority = 1, int stackSize = 10000);
  void taskDelay(int ms);
  static void _taskEntry(void *pvParameters);
  static void _taskLoopEntry(void *pvParameters);
#endif

  /*********************************** Email ***********************************
   */
#if defined(USE_EMAIL)
  void sendEmail(String smtpHost, int smtpPort, String authorEmail, String authorPassword, String recipientEmail, String subject, String message);
#endif

  /*********************************** Weather ***********************************
   */
#if defined(USE_WEATHER)
  String getWeather(String city, String apiKey);
#endif

  /*********************************** Wikipedia ***********************************
   */
#if defined(USE_WIKIPEDIA)
  String getWikipedia(String query, String lang = "en");
#endif
/*********************************** Helpers ***********************************
   */
  void lcdWriteFixedTxt(int col, int row, const char *txt, int width);
  void lcdWriteFixed(int col, int row, int value, int width);
  void calibrateJoystick(int &xCenter, int &yCenter, int samples = 20);
#if defined(USE_ESPNOW)
  bool addBroadcastPeer(int channel = 1);
#endif

private:
  static constexpr uint16_t _EEPROM_RECORD_MAGIC = 0xCD1A;
  static constexpr time_t _NTP_VALID_EPOCH = 1609459200; // 2021-01-01

  bool _eepromReady = false;
  size_t _eepromSize = 0;

  bool _eepromEnsure(size_t minSize);

  void createTurkishChars();
  String convertTR(String text);

  LiquidCrystal_I2C lcd;
  int encoderCount; // Stores the encoder's position
  int lastStateA;   // Stores the last state of A pin
  int lastStateB;   // Stores the last state of B pin
  int counterBuzzer, counterLCD, counterLDR, counterRelay, counterPot, counterJoystick, counterButtons, counterEncoder = 0;

#if defined(USE_SERVO)
  Servo servoModule; // Create a Servo object for controlling the servo motor
  int currentAngle = 0;
#endif

#if defined(USE_DHT)
  void initializeDht(int pin, uint8_t type);
  DHT *dhtSensor; // Pointer to DHT sensor object
#endif

#if defined(USE_NEOPIXEL)
  Adafruit_NeoPixel *pixels; // NeoPixel object pointer
#endif

#if defined(USE_IR)

  void initializeIR(int pin);
  IRrecv *irrecv = nullptr; // Pointer to IR receiver / IR alıcısı için pointer
  decode_results results;   // Stores received IR results / Alınan IR sinyallerini saklar
  int irPin;                // Store the IR receiver pin / IR alıcı pini sakla
  long irRawValue = 0;      // Stores last received IR value / En son alınan IR değerini saklar

#endif

#if defined(USE_RFID)
  void beginRFID();             // RFID başlat / Initialize RFID module
  MFRC522 rfid{5, 10};          // RFID modülü nesnesi / RFID module object (SS_PIN: 5, RST_PIN: 10)
  bool rfidInitialized = false; // RFID'nin başlatılıp başlatılmadığını kontrol eden bayrak / Flag to check initialization
#endif

#if defined(USE_SERVER)
  const IPAddress apIP = IPAddress(192, 168, 4, 1); // Sabit IP adresi tanımlanıyor / Define static IP address
  DNSServer dnsServer;                              // DNS sunucusu tanımlanıyor / Define DNS Server
  AsyncWebServer serverCODLAI{80};                  // Web server objesi
  AsyncWebSocket *serverCODLAIWebSocket;            // Pointer olarak tanımla
#endif

#if defined(USE_FIREBASE)
  FirebaseData firebaseData;     // Data object to handle Firebase communication
  FirebaseAuth firebaseAuth;     // Authentication credentials for user verification
  FirebaseConfig firebaseConfig; // Configuration settings for Firebase
  char uid[128] = "";            // User ID storage
#endif

#if defined(USE_BLUETOOTH) && defined(ESP32)
  BluetoothSerial serialBT;
#endif
};

#if defined(USE_ESPNOW)
// Static member definition moved to main.cpp or implementation file to avoid multiple definition errors
// IOTBOT* IOTBOT::_instance = nullptr; 
#endif

/*********************************** IMPLEMENTATION ***********************************/

/*********************************** Constructor ***********************************/

inline IOTBOT::IOTBOT() : lcd(LCD_ADRESS, 20, 4)
{
#if defined(USE_SERVER)
  serverCODLAIWebSocket = new AsyncWebSocket("/serverCODLAIWebSocket");
#endif
}

/*********************************** Helper Functions ***********************************/
inline void IOTBOT::createTurkishChars() {
  byte char_ch[8] = {0x00,0x0E,0x10,0x10,0x11,0x0E,0x04,0x00}; // ç
  byte char_g[8]  = {0x0E,0x00,0x0F,0x11,0x0F,0x01,0x0E,0x00}; // ğ
  byte char_i[8]  = {0x00,0x00,0x0C,0x04,0x04,0x04,0x0E,0x00}; // ı
  byte char_o[8]  = {0x0A,0x00,0x0E,0x11,0x11,0x11,0x0E,0x00}; // ö
  byte char_s[8]  = {0x00,0x0F,0x10,0x0E,0x01,0x1E,0x04,0x00}; // ş
  byte char_u[8]  = {0x0A,0x00,0x11,0x11,0x11,0x13,0x0D,0x00}; // ü
  byte char_I[8]  = {0x04,0x00,0x0C,0x04,0x04,0x04,0x0E,0x00}; // İ
  // byte char_C[8]  = {0x0E,0x11,0x10,0x10,0x11,0x0E,0x04,0x00}; // Ç (Not used to save space/avoid 0x00)

  lcd.createChar(1, char_ch);
  lcd.createChar(2, char_g);
  lcd.createChar(3, char_i);
  lcd.createChar(4, char_o);
  lcd.createChar(5, char_s);
  lcd.createChar(6, char_u);
  lcd.createChar(7, char_I);
}

inline String IOTBOT::convertTR(String text) {
  // Replace Turkish characters with custom char indices
  text.replace("ç", "\x01");
  text.replace("ğ", "\x02");
  text.replace("ı", "\x03");
  text.replace("ö", "\x04");
  text.replace("ş", "\x05");
  text.replace("ü", "\x06");
  text.replace("İ", "\x07");
  
  // Map uppercase to ASCII or similar
  text.replace("Ç", "C");
  text.replace("Ş", "S");
  text.replace("Ğ", "G");
  text.replace("Ü", "U");
  text.replace("Ö", "O");
  
  return text;
}

/*********************************** BEGIN ***********************************/
inline void IOTBOT::begin()
{
  pinMode(JOYSTICK_Y_PIN, INPUT);
  pinMode(JOYSTICK_X_PIN, INPUT);
  pinMode(JOYSTICK_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);
  pinMode(ENCODER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(B1_AND_B2_BUTTON_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(B3_BUTTON_PIN, INPUT_PULLUP);

  // Reset encoder count and read initial states
  encoderCount = 0;
  lastStateA = digitalRead(ENCODER_A_PIN);
  lastStateB = digitalRead(ENCODER_B_PIN);

  // LCD begin
  lcd.init();
  lcd.backlight();
  createTurkishChars(); // Initialize Turkish characters
  lcd.clear();

  analogWrite(BUZZER_PIN, 750);
  delay(125);
  analogWrite(BUZZER_PIN, 0);
  delay(100);
  analogWrite(BUZZER_PIN, 700);
  delay(100);
  analogWrite(BUZZER_PIN, 0);

  lcd.setCursor(3, 1);
  lcd.print("--- IoTBot ---");
  lcd.setCursor(4, 3);
  lcd.print("from  CODLAI");
}

inline void IOTBOT::playIntro()
{
  lcd.clear();
  delay(50);

  lcd.setCursor(3, 1);
  lcd.print("Hi There!");
  delay(200);
  lcd.setCursor(10, 2);
  lcd.print("Merhaba!");
  delay(200);
  lcd.setCursor(0, 0);
  lcd.print("Bonjour!");
  delay(200);
  lcd.setCursor(1, 3);
  lcd.print("Hallo!");
  delay(200);
  lcd.setCursor(12, 0);
  lcd.print("Hola!");
  delay(200);
  lcd.setCursor(14, 1);
  lcd.print("Ciao!");
  delay(200);
  lcd.setCursor(2, 2);
  lcd.print("Ola!");
  delay(200);
  lcd.setCursor(9, 3);
  lcd.print("Konnichiwa!");
  delay(1000);

  lcd.clear();
  lcd.setCursor(3, 1);
  lcd.print("--- IoTBot ---");
  lcd.setCursor(4, 3);
  lcd.print("from  CODLAI");
  delay(500);

  tone(BUZZER_PIN, 800, 200);
  delay(200);
  tone(BUZZER_PIN, 1000, 100);
  delay(250);
  tone(BUZZER_PIN, 1500, 400);
  delay(400);
}

/*********************************** Serial Port ***********************************
 */
inline void IOTBOT::serialStart(int baudrate)
{
  Serial.begin(baudrate);
}

inline void IOTBOT::serialWrite(const char *message) // Overloaded function for const char* / `const char*` için fonksiyon
{
  Serial.println(message);
}

inline void IOTBOT::serialWrite(String message) // Overloaded function for String / `String` için özel fonksiyon
{
  Serial.println(message.c_str()); // Convert String to const char*
}

inline void IOTBOT::serialWrite(long value) // Overloaded function for long / `long` için özel fonksiyon
{
  Serial.println(String(value).c_str());
}

inline void IOTBOT::serialWrite(int value) // Overloaded function for int / `int` için fonksiyon
{
  Serial.println(String(value).c_str());
}

inline void IOTBOT::serialWrite(float value) // Overloaded function for float / `float` için fonksiyon
{
  Serial.println(String(value).c_str());
}

inline void IOTBOT::serialWrite(bool value) // Overloaded function for bool / `bool` için fonksiyon
{
  Serial.println(value ? "true" : "false");
}

inline int IOTBOT::serialAvailable()
{
  return Serial.available();
}

inline String IOTBOT::serialReadStringUntil(char terminator)
{
  return Serial.readStringUntil(terminator);
}

/*********************************** BUZZER ***********************************
 */
inline void IOTBOT::buzzerPlayTone(int frequency, int duration)
{
  tone(BUZZER_PIN, frequency, duration);
  delay(duration);
}

inline void IOTBOT::buzzerPlay(int frequency, int duration)
{
  buzzerPlayTone(frequency, duration);
}

inline void IOTBOT::buzzerStart(int frequency)
{
  tone(BUZZER_PIN, frequency);
}

inline void IOTBOT::buzzerStop()
{
  noTone(BUZZER_PIN);
}

inline void IOTBOT::buzzerSoundIntro()
{
  tone(BUZZER_PIN, 750, 125);
  delay(125);
  delay(150);
  tone(BUZZER_PIN, 1000, 100);
  delay(100);
  delay(200);
  tone(BUZZER_PIN, 800, 400);
  delay(400);
}

inline void IOTBOT::buzzertest()
{
  if (counterBuzzer < 1)
  {
    lcd.clear();
    lcd.setCursor(6, 0);
    lcd.print("-IotBot-");
    lcd.setCursor(5, 1);
    lcd.print("Buzzer Test");

    counterBuzzer++;
  }
  lcd.setCursor(6, 3);
  lcd.print("Beeping!");
  buzzerPlayTone(1000, 500);
  lcd.setCursor(6, 3);
  lcd.print(" Silent.");
  delay(500);
}

/*********************************** LCD SCREEN ***********************************
 */
inline void IOTBOT::lcdWriteMid(const char *line1, const char *line2, const char *line3, const char *line4)
{
  lcd.clear();

  // Convert to String to handle TR chars
  String s1 = convertTR(String(line1));
  String s2 = convertTR(String(line2));
  String s3 = convertTR(String(line3));
  String s4 = convertTR(String(line4));

  // Define the number of columns in the LCD (e.g., 20 for a 20x4 LCD display)
  const int lcdColumns = 20;

  // Calculate the length of each line
  int len1 = s1.length();
  int len2 = s2.length();
  int len3 = s3.length();
  int len4 = s4.length();

  // Calculate the starting column for each line to center-align the text
  int startCol1 = (lcdColumns - len1) / 2;
  int startCol2 = (lcdColumns - len2) / 2;
  int startCol3 = (lcdColumns - len3) / 2;
  int startCol4 = (lcdColumns - len4) / 2;

  // Ensure the starting column is not negative (in case the text is too long)
  startCol1 = max(0, startCol1);
  startCol2 = max(0, startCol2);
  startCol3 = max(0, startCol3);
  startCol4 = max(0, startCol4);

  // Print each line on the LCD at the calculated starting column
  lcd.setCursor(startCol1, 0); // Line 0
  lcd.print(s1);

  lcd.setCursor(startCol2, 1); // Line 1
  lcd.print(s2);

  lcd.setCursor(startCol3, 2); // Line 2
  lcd.print(s3);

  lcd.setCursor(startCol4, 3); // Line 3
  lcd.print(s4);
}

inline void IOTBOT::lcdWrite(const char *text) // Overloaded function for const char* / `const char*` için fonksiyon
{
  lcd.print(convertTR(String(text)));
}

inline void IOTBOT::lcdWrite(String text) // Overloaded function for String / `String` için özel fonksiyon
{
  lcd.print(convertTR(text)); 
}

inline void IOTBOT::lcdWrite(int value)
{
  lcd.print(String(value));
}

inline void IOTBOT::lcdWrite(float value) // Overloaded function for float / `float` için fonksiyon
{
  lcd.print(String(value));
}

inline void IOTBOT::lcdWrite(bool value) // Overloaded function for bool / `bool` için fonksiyon
{
  lcd.print(value ? "true" : "false");
}

inline void IOTBOT::lcdSetCursor(int col, int row)
{
  lcd.setCursor(col, row);
}

inline void IOTBOT::lcdWriteCR(int col, int row, const char *text)
{
  if (row < 0 || row >= 4 || col < 0 || col >= 20)
    return;
  lcd.setCursor(col, row);
  lcd.print(convertTR(String(text)));
}

inline void IOTBOT::lcdWriteCR(int col, int row, String text)
{
  if (row < 0 || row >= 4 || col < 0 || col >= 20)
    return;
  lcd.setCursor(col, row);
  lcd.print(convertTR(text));
}

inline void IOTBOT::lcdWriteCR(int col, int row, int value)
{
  if (row < 0 || row >= 4 || col < 0 || col >= 20)
    return;
  lcd.setCursor(col, row);
  lcd.print(String(value));
}

inline void IOTBOT::lcdWriteCR(int col, int row, float value)
{
  if (row < 0 || row >= 4 || col < 0 || col >= 20)
    return;
  lcd.setCursor(col, row);
  lcd.print(String(value));
}

inline void IOTBOT::lcdWriteCR(int col, int row, bool value)
{
  if (row < 0 || row >= 4 || col < 0 || col >= 20)
    return;
  lcd.setCursor(col, row);
  lcd.print(value ? "true" : "false");
}

inline void IOTBOT::lcdClear()
{
  lcd.clear();
}

inline void IOTBOT::lcdtest()
{
  if (counterLCD < 1)
  {
    lcd.clear();
    lcd.setCursor(6, 0);
    lcd.print("-IotBot-");
    lcd.setCursor(7, 1);
    lcd.print("LCD Test");

    counterLCD++;
  }

  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("Hi, my name is");
  lcd.setCursor(3, 1);
  lcd.print("--- IoTBot ---");
  lcd.setCursor(4, 3);
  lcd.print("from  CODLAI");
  delay(5000);
}

inline void IOTBOT::lcdShowLoading(String message) {
    lcd.clear();
    message = convertTR(message);
    lcd.setCursor((20 - message.length()) / 2, 1);
    lcd.print(message);
    
    for(int i=0; i<3; i++) {
        lcd.setCursor(8, 2);
        lcd.print(".   ");
        delay(200);
        lcd.setCursor(8, 2);
        lcd.print("..  ");
        delay(200);
        lcd.setCursor(8, 2);
        lcd.print("... ");
        delay(200);
        lcd.setCursor(8, 2);
        lcd.print("....");
        delay(200);
    }
}

inline void IOTBOT::lcdShowStatus(String title, String status, bool isSuccess) {
    lcd.clear();
    title = convertTR(title);
    status = convertTR(status);
    
    lcd.setCursor((20 - title.length()) / 2, 0);
    lcd.print(title);
    
    lcd.setCursor((20 - status.length()) / 2, 2);
    lcd.print(status);
    
    lcd.setCursor(8, 3);
    if(isSuccess) {
        lcd.print("[ OK ]");
    } else {
        lcd.print("[FAIL]");
    }
}

/*********************************** LDR ***********************************
 */
inline int IOTBOT::ldrRead()
{
  return analogRead(LDR_PIN);
}

inline void IOTBOT::ldrtest()
{
  if (counterLDR < 1)
  {
    lcd.clear();
    lcd.setCursor(6, 0);
    lcd.print("-IotBot-");
    lcd.setCursor(6, 1);
    lcd.print("LDR Test");
    lcd.setCursor(5, 3);
    lcd.print("Value: ");
    counterLDR++;
  }
  int value = analogRead(LDR_PIN);
  lcd.setCursor(12, 3);
  lcd.print(value);
  lcd.print("   ");
}

/*********************************** RELAY ***********************************
 */
inline void IOTBOT::relayWrite(bool status)
{
  digitalWrite(RELAY_PIN, status);
}

inline void IOTBOT::relaytest()
{
  if (counterRelay < 1)
  {
    lcd.clear();
    lcd.setCursor(6, 0);
    lcd.print("-IotBot-");
    lcd.setCursor(5, 1);
    lcd.print("Relay Test");

    counterRelay++;
  }
  lcd.setCursor(6, 3);
  lcd.print(" Open!");
  digitalWrite(RELAY_PIN, HIGH);
  delay(500);
  lcd.setCursor(6, 3);
  lcd.print("Close!");
  digitalWrite(RELAY_PIN, LOW);
  delay(500);
}

/*********************************** POTANTIOMETER ***********************************
 */
inline int IOTBOT::potentiometerRead()
{
  return analogRead(POT_PIN);
}

inline void IOTBOT::potentiometertest()
{
  if (counterPot < 1)
  {
    lcd.clear();
    lcd.setCursor(6, 0);
    lcd.print("-IotBot-");
    lcd.setCursor(1, 1);
    lcd.print("Potantiometer Test");
    lcd.setCursor(4, 3);
    lcd.print("Value: ");

    counterPot++;
  }
  int value = analogRead(POT_PIN);
  lcd.setCursor(11, 3);
  lcd.print(value);
  lcd.print("   ");
}

/*********************************** JOYSTICK ***********************************
 */
inline int IOTBOT::joystickXRead()
{
  return analogRead(JOYSTICK_X_PIN);
}

inline int IOTBOT::joystickYRead()
{
  return analogRead(JOYSTICK_Y_PIN);
}

inline bool IOTBOT::joystickButtonRead()
{
  return digitalRead(JOYSTICK_BUTTON_PIN);
}

inline void IOTBOT::joysticktest()
{
  if (counterJoystick < 1)
  {
    lcd.clear();
    lcd.setCursor(6, 0);
    lcd.print("-IotBot-");
    lcd.setCursor(3, 1);
    lcd.print("Joystick Test");
    lcd.setCursor(0, 2);
    lcd.print("X: ");
    lcd.setCursor(10, 2);
    lcd.print("Y: ");

    lcd.setCursor(2, 3);
    lcd.print("Button Status: ");

    counterJoystick++;
  }

  int xValue = analogRead(JOYSTICK_X_PIN);
  int yValue = analogRead(JOYSTICK_Y_PIN);
  int buttonStatus = digitalRead(JOYSTICK_BUTTON_PIN);

  lcd.setCursor(3, 2);
  lcd.print(xValue);
  lcd.print("   ");
  lcd.setCursor(13, 2);
  lcd.print(yValue);
  lcd.print("   ");
  lcd.setCursor(17, 3);
  lcd.print(buttonStatus);
}

/*********************************** BUTTONS ***********************************
 */
inline bool IOTBOT::button1Read()
{
#if defined(ESP32) && (defined(USE_WIFI) || defined(USE_ESPNOW))
  // ADC2 Pin 4 workaround: When WiFi is active, analogRead fails.
  // Fallback to digitalRead as requested.
  return digitalRead(B1_AND_B2_BUTTON_PIN);
#else
  int value = analogRead(B1_AND_B2_BUTTON_PIN);

  if (value > 3500)
  {
    return true;
  }
  else
  {
    return false;
  }
#endif
}

inline bool IOTBOT::button2Read()
{
#if defined(ESP32) && (defined(USE_WIFI) || defined(USE_ESPNOW))
  // ADC2 Pin 4 workaround: When WiFi is active, analogRead fails.
  // Fallback to digitalRead. Note: Cannot distinguish B1/B2 levels in digital mode.
  return digitalRead(B1_AND_B2_BUTTON_PIN);
#else
  int value = analogRead(B1_AND_B2_BUTTON_PIN);

  if (value > 1500 && value < 3000)
  {
    return true;
  }
  else
  {
    return false;
  }
#endif
}

inline bool IOTBOT::button3Read()
{
  int value = digitalRead(B3_BUTTON_PIN);

  if (value == LOW)
  {
    return true;
  }
  else
  {
    return false;
  }
}

inline void IOTBOT::buttonsAnalogtest()
{
  if (counterButtons < 1)
  {
    lcd.clear();
    lcd.setCursor(6, 0);
    lcd.print("-IotBot-");
    lcd.setCursor(0, 1);
    lcd.print("Analog Button Test");
    lcd.setCursor(1, 2);
    lcd.print("Button value : ");

    lcd.setCursor(1, 3);
    counterButtons++;
  }

  int buttonValue = analogRead(B1_AND_B2_BUTTON_PIN);
  int B3value = digitalRead(B3_BUTTON_PIN);

  lcd.setCursor(16, 2);
  lcd.print(buttonValue);

  if (buttonValue > 3500)
  {
    lcd.setCursor(6, 3);
    lcd.print("B1");
  }
  else if (buttonValue > 1500 && buttonValue < 3000)
  {
    lcd.setCursor(6, 3);
    lcd.print("B2");
  }
  else
  {
    lcd.setCursor(17, 2);
    lcd.print("   ");
    lcd.setCursor(6, 3);
    lcd.print("  ");
  }

  if (B3value == LOW)
  {
    lcd.setCursor(14, 3);
    lcd.print("B3");
  }
  else
  {
    lcd.setCursor(14, 3);
    lcd.print("  ");
  }
}

/*********************************** ENCODER ***********************************
 */
inline int IOTBOT::encoderRead()
{
  // Read current states of A and B pins
  int currentStateA = digitalRead(ENCODER_A_PIN);
  int currentStateB = digitalRead(ENCODER_B_PIN);

  // Check if A pin state has changed
  if (currentStateA != lastStateA)
  {
    if (currentStateA == HIGH)
    {
      // Determine direction based on B pin state
      if (currentStateB == LOW)
      {
        encoderCount--; // Counterclockwise
      }
      else
      {
        encoderCount++; // Clockwise
      }
    }
  }

  // Update last states
  lastStateA = currentStateA;
  lastStateB = currentStateB;

  // Return the current encoder count
  return encoderCount;
}

inline bool IOTBOT::encoderButtonRead()
{
  int value = digitalRead(ENCODER_BUTTON_PIN);

  if (value)
  {
    return true;
  }
  else
  {
    return false;
  }
}

inline void IOTBOT::encodertest()
{
  if (counterEncoder < 1)
  {
    lcd.clear();
    lcd.setCursor(6, 0);
    lcd.print("-IotBot-");
    lcd.setCursor(4, 1);
    lcd.print("Encoder Test");
    lcd.setCursor(1, 2);
    lcd.print("Encoder value : ");

    lcd.setCursor(1, 3);
    lcd.print("Encoder button: ");
    counterEncoder++;
  }

  int buttonStatus = digitalRead(ENCODER_BUTTON_PIN);
  int currentStateA = digitalRead(ENCODER_A_PIN);
  int currentStateB = digitalRead(ENCODER_B_PIN);

  // Check if A pin state has changed
  if (currentStateA != lastStateA)
  {
    if (currentStateA == HIGH)
    {
      // Determine direction based on B pin state
      if (currentStateB == LOW)
      {
        encoderCount--; // Counterclockwise
      }
      else
      {
        encoderCount++; // Clockwise
      }
    }
  }

  // Update last states
  lastStateA = currentStateA;
  lastStateB = currentStateB;

  lcd.setCursor(17, 2);
  lcd.print(encoderCount);
  lcd.print("   ");
  lcd.setCursor(17, 3);
  lcd.print(buttonStatus);
}

/*********************************** DRIVER AND MOTORS ***********************************
 */
/*********************************** Stepper Motor Motion ***********************************
 * Controls the stepper motor.
 * rotation: True for clockwise, false for counterclockwise.
 * accelometer: Number of steps to move.
 * speed: Speed of the stepper motor in RPM.
 */
inline void IOTBOT::moduleStepMotorMotion(int step, bool rotation, int accelometer, int speed)
{
  // Stepper motor object (shared across the library)
  Stepper stepMotor(step, IO26, IO33, IO32, IO27); // 50 steps per revolution, pins 26, 33, 32, 27

  stepMotor.setSpeed(speed); // Set the speed of the stepper motor

  if (rotation)
  {
    stepMotor.step(accelometer); // Move forward (clockwise)
  }
  else
  {
    stepMotor.step(-accelometer); // Move backward (counterclockwise)
  }
}

/*********************************** DC Motor Clockwise ***********************************
 * Rotates the DC motor clockwise at the specified speed.
 * speed: Motor speed (0 to 255 for PWM).
 */
inline void IOTBOT::moduleDCMotorGOClockWise(int speed)
{
  pinMode(IO26, OUTPUT); // Direction control pin
  pinMode(IO27, OUTPUT); // PWM control pin

  // Map speed from 0-100 to 0-255 for PWM
  int pwmValue = map(speed, 0, 100, 0, 255);

  digitalWrite(IO26, LOW);     // Set direction to clockwise
  analogWrite(IO27, pwmValue); // Set motor speed using PWM
}

/*********************************** DC Motor Counter-Clockwise ***********************************
 * Rotates the DC motor counterclockwise at the specified speed.
 * speed: Motor speed (0 to 255 for PWM).
 */
inline void IOTBOT::moduleDCMotorGOCounterClockWise(int speed)
{
  pinMode(IO26, OUTPUT); // Direction control pin
  pinMode(IO27, OUTPUT); // PWM control pin

  // Map speed from 0-100 to 0-255 for PWM
  int pwmValue = map(speed, 0, 100, 0, 255);

  digitalWrite(IO26, pwmValue); // Set direction to counterclockwise
  analogWrite(IO27, LOW);       // Set motor speed using PWM
}

/*********************************** DC Motor Stop ***********************************
 * Stops the DC motor by setting PWM to 0.
 */
inline void IOTBOT::moduleDCMotorStop()
{
  pinMode(IO26, OUTPUT);
  pinMode(IO27, OUTPUT);

  digitalWrite(IO26, LOW);
  digitalWrite(IO27, LOW);
}

/*********************************** DC Motor Brake ***********************************
 * Brakes the DC motor using a hardware brake.
 */
inline void IOTBOT::moduleDCMotorBrake()
{
  pinMode(IO26, OUTPUT);
  pinMode(IO27, OUTPUT);

  digitalWrite(IO26, HIGH);
  digitalWrite(IO27, HIGH);
}

/*********************************** NTC Temp Sensor ***********************************
 * Reads the NTC temperature sensor value and calculates the temperature in Celsius.
 * pin: The analog pin where the NTC is connected.
 */
inline float IOTBOT::moduleNtcTempRead(int pin)
{
  // Configure pins
  pinMode(pin, INPUT);

  // NTC constants
  const float R_NOMINAL = 10000.0;       // Nominal resistance at 25°C (10k ohms)
  const float T_NOMINAL = 25.0;          // Nominal temperature (25°C)
  const float B_COEFFICIENT = 3950.0;    // Beta coefficient of the thermistor
  const float SERIES_RESISTOR = 10000.0; // Resistor value in series with the NTC

  // Read the analog value from the pin
  int analogValue = analogRead(pin);

  // Convert the analog value to resistance
  float resistance = SERIES_RESISTOR / ((4095.0 / analogValue) - 1.0);

  // Calculate the temperature using the Steinhart-Hart equation
  float steinhart = resistance / R_NOMINAL; // (R / R_NOMINAL)
  steinhart = log(steinhart);               // ln(R/R_NOMINAL)
  steinhart /= B_COEFFICIENT;               // 1/B * ln(R/R_NOMINAL)
  steinhart += 1.0 / (T_NOMINAL + 273.15);  // + (1/T_NOMINAL)
  steinhart = 1.0 / steinhart;              // Invert
  steinhart -= 273.15;                      // Convert Kelvin to Celsius

  return steinhart;
}

/*********************************** Magnetic Sensor ***********************************
 */
inline bool IOTBOT::moduleMagneticRead(int pin)
{
  // Configure pins
  pinMode(pin, INPUT);
  return !digitalRead(pin);
}

/*********************************** Matris Button Sensor ***********************************
 */
inline int IOTBOT::moduleMatrisButtonAnalogRead(int pin)
{
  // Configure pins
  pinMode(pin, INPUT);
  return analogRead(pin);
}

inline int IOTBOT::moduleMatrisButtonNumberRead(int pin)
{
  // Configure pins
  pinMode(pin, INPUT);
  int value = analogRead(pin);
  if (value > 4000)
  {
    return 1;
  }
  else if (value > 2350 && value < 2500)
  {
    return 2;
  }
  else if (value > 2210 && value < 2350)
  {
    return 3;
  }
  else if (value > 2150 && value < 2210)
  {
    return 4;
  }
  else if (value > 2000 && value < 2150)
  {
    return 5;
  }
  else
  {
    return 0;
  }
}

/*********************************** Vibration Sensor ***********************************
 */
inline bool IOTBOT::moduleVibrationDigitalRead(int pin)
{
  // Configure pins
  pinMode(pin, INPUT);
  return digitalRead(pin);
}

inline int IOTBOT::moduleVibrationAnalogRead(int pin)
{
  // Configure pins
  pinMode(pin, INPUT);
  return analogRead(pin);
}

/*********************************** Ultrasonic Distance Sensor ***********************************
 * Reads distance from the ultrasonic sensor using echo and trigger pins.
 * Automatically adjusts for ESP32 and ESP8266 platforms.
 * Returns 0 if the distance exceeds the maximum measurable range (400 cm).
 */
inline int IOTBOT::moduleUltrasonicDistanceRead()
{
#if defined(ESP32)
  const int TRIG_PIN = IO27;
  const int ECHO_PIN = IO32;
#else
#error "Unsupported platform! Only ESP32 and ESP8266 are supported."
#endif

  // Maximum measurable distance for HC-SR04 (in centimeters)
  const int MAX_DISTANCE = 400;

  // Configure pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Send a 10-microsecond pulse on the TRIG_PIN
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(ECHO_PIN, HIGH, MAX_DISTANCE * 58); // Timeout for MAX_DISTANCE

  // If no echo is received, return 0 (out of range)
  if (duration == 0)
  {
    return 0; // Out of range or no object detected
  }

  // Calculate the distance in centimeters
  int distance = duration * 0.034 / 2; // Sound speed: 0.034 cm/µs, divide by 2 for round trip

  // If the calculated distance exceeds the maximum range, return 0
  if (distance > MAX_DISTANCE)
  {
    return 000;
  }

  return distance; // Return the measured distance
}

/*********************************** Trafic Ligh Sensor ***********************************
 */
inline void IOTBOT::moduleTraficLightWrite(bool red, bool yellow, bool green)
{
#if defined(ESP32)
  const int RED_PIN = IO32;
  const int YELLOW_PIN = IO26;
  const int GREEN_PIN = IO25;
#else
#error "Unsupported platform! Only ESP32 and ESP8266 are supported."
#endif

  // Configure pins
  pinMode(RED_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);

  if (red)
  {
    digitalWrite(RED_PIN, HIGH);
  }
  else
  {
    digitalWrite(RED_PIN, LOW);
  }
  if (yellow)
  {
    digitalWrite(YELLOW_PIN, HIGH);
  }
  else
  {
    digitalWrite(YELLOW_PIN, LOW);
  }
  if (green)
  {
    digitalWrite(GREEN_PIN, HIGH);
  }
  else
  {
    digitalWrite(GREEN_PIN, LOW);
  }
}

inline void IOTBOT::moduleTraficLightWriteRed(bool red)
{
#if defined(ESP32)
  const int RED_PIN = IO32;
#else
#error "Unsupported platform! Only ESP32 and ESP8266 are supported."
#endif

  // Configure pins
  pinMode(RED_PIN, OUTPUT);

  if (red)
  {
    digitalWrite(RED_PIN, HIGH);
  }
  else
  {
    digitalWrite(RED_PIN, LOW);
  }
}
inline void IOTBOT::moduleTraficLightWriteYellow(bool yellow)
{
#if defined(ESP32)
  const int YELLOW_PIN = IO26;
#else
#error "Unsupported platform! Only ESP32 and ESP8266 are supported."
#endif

  // Configure pins
  pinMode(YELLOW_PIN, OUTPUT);

  if (yellow)
  {
    digitalWrite(YELLOW_PIN, HIGH);
  }
  else
  {
    digitalWrite(YELLOW_PIN, LOW);
  }
}

inline void IOTBOT::moduleTraficLightWriteGreen(bool green)
{
#if defined(ESP32)
  const int GREEN_PIN = IO25;
#else
#error "Unsupported platform! Only ESP32 and ESP8266 are supported."
#endif

  // Configure pins
  pinMode(GREEN_PIN, OUTPUT);

  if (green)
  {
    digitalWrite(GREEN_PIN, HIGH);
  }
  else
  {
    digitalWrite(GREEN_PIN, LOW);
  }
}

/*********************************** Motion Sensor ***********************************
 */
inline bool IOTBOT::moduleMotionRead(int pin)
{
  // Configure pins
  pinMode(pin, INPUT);
  return digitalRead(pin);
}

/*********************************** Smoke Sensor ***********************************
 */
inline int IOTBOT::moduleSmokeRead(int pin)
{
  // Configure pins
  pinMode(pin, INPUT);
  return analogRead(pin);
}

/*********************************** Mic Sensor ***********************************
 */
inline int IOTBOT::moduleMicRead(int pin)
{
  // Configure pins
  pinMode(pin, INPUT);
  return analogRead(pin);
}

/*********************************** Soil Moisture Sensor ***********************************
 */
inline int IOTBOT::moduleSoilMoistureRead(int pin)
{
  // Configure pins
  pinMode(pin, INPUT);
  return analogRead(pin);
}

/*********************************** Relay Sensor ***********************************
 */
inline void IOTBOT::moduleRelayWrite(int pin, bool status)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, status);
}

/*********************************** OTHER PINS ***********************************
 */
inline int IOTBOT::analogReadPin(int pin)
{
  // Configure pins
  pinMode(pin, INPUT);
  return analogRead(pin);
}

inline void IOTBOT::analogWritePin(int pin, int value)
{
  pinMode(pin, OUTPUT);
  // int pwmChannel = pin % 16;
  // ledcAttachPin(pin, pwmChannel);
  // ledcWrite(pwmChannel, value);
  analogWrite(pin, value); // ESP8266 için normal PWM
}

inline bool IOTBOT::digitalReadPin(int pin)
{
  // Configure pins
  pinMode(pin, INPUT);
  return digitalRead(pin);
}

inline void IOTBOT::digitalWritePin(int pin, bool value)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, value);
}

/*********************************** EEPROM  ***********************************
 */
inline bool IOTBOT::_eepromEnsure(size_t minSize)
{
  if (_eepromReady && _eepromSize >= minSize)
  {
    return true;
  }

  size_t targetSize = _eepromSize;
  if (targetSize == 0)
  {
    targetSize = 1024;
  }
  if (targetSize < minSize)
  {
    targetSize = minSize;
  }

  return eepromBegin(targetSize);
}

inline bool IOTBOT::eepromBegin(size_t size)
{
  if (size == 0)
  {
    size = 1024;
  }

  if (_eepromReady && _eepromSize == size)
  {
    return true;
  }

  _eepromSize = size;
#if defined(ESP8266)
  EEPROM.begin(size);
  _eepromReady = true;
#else
  _eepromReady = EEPROM.begin(size);
#endif
  return _eepromReady;
}

inline bool IOTBOT::eepromCommit()
{
  if (!_eepromEnsure(2))
  {
    return false;
  }

  return EEPROM.commit();
}

inline void IOTBOT::eepromEnd()
{
  if (_eepromReady)
  {
    EEPROM.end();
  }
  _eepromReady = false;
  _eepromSize = 0;
}

inline bool IOTBOT::eepromWriteByte(int address, uint8_t value)
{
  if (address < 0)
  {
    return false;
  }

  if (!_eepromEnsure((size_t)address + 1))
  {
    return false;
  }

  EEPROM.write(address, value);
  return EEPROM.commit();
}

inline uint8_t IOTBOT::eepromReadByte(int address, uint8_t defaultValue)
{
  if (address < 0)
  {
    return defaultValue;
  }

  if (!_eepromEnsure((size_t)address + 1))
  {
    return defaultValue;
  }

  return EEPROM.read(address);
}

inline void IOTBOT::eepromWriteInt(int address, int value) // EEPROM'a güvenli bir şekilde int türünde veri yazmak için fonksiyon
{
  if (address < 0)
  {
    return;
  }

  if (!_eepromEnsure((size_t)address + 2))
  {
    return;
  }

  uint8_t hi = highByte(value); // int'in yüksek baytını al
  uint8_t lo = lowByte(value);  // int'in düşük baytını al

  EEPROM.write(address, hi);      // İlk baytı EEPROM'a yaz
  EEPROM.write(address + 1, lo);  // İkinci baytı EEPROM'a yaz
  EEPROM.commit();                    // Değişiklikleri kaydetmek için commit işlemi yapılmalıdır
}

inline int IOTBOT::eepromReadInt(int address) // EEPROM'dan int türünde veri okumak için fonksiyon
{
  if (address < 0)
  {
    return 0;
  }

  if (!_eepromEnsure((size_t)address + 2))
  {
    return 0;
  }

  uint8_t hi = EEPROM.read(address);     // İlk baytı oku
  uint8_t lo = EEPROM.read(address + 1); // İkinci baytı oku
  return word(hi, lo);                   // Yüksek ve düşük baytları birleştirerek int değeri oluştur
}

inline bool IOTBOT::eepromWriteInt32(int address, int32_t value)
{
  if (address < 0)
  {
    return false;
  }
  if (!_eepromEnsure((size_t)address + sizeof(int32_t)))
  {
    return false;
  }

  EEPROM.put(address, value);
  return EEPROM.commit();
}

inline int32_t IOTBOT::eepromReadInt32(int address, int32_t defaultValue)
{
  if (address < 0)
  {
    return defaultValue;
  }
  if (!_eepromEnsure((size_t)address + sizeof(int32_t)))
  {
    return defaultValue;
  }

  int32_t value;
  EEPROM.get(address, value);
  return value;
}

inline bool IOTBOT::eepromWriteUInt32(int address, uint32_t value)
{
  if (address < 0)
  {
    return false;
  }
  if (!_eepromEnsure((size_t)address + sizeof(uint32_t)))
  {
    return false;
  }

  EEPROM.put(address, value);
  return EEPROM.commit();
}

inline uint32_t IOTBOT::eepromReadUInt32(int address, uint32_t defaultValue)
{
  if (address < 0)
  {
    return defaultValue;
  }
  if (!_eepromEnsure((size_t)address + sizeof(uint32_t)))
  {
    return defaultValue;
  }

  uint32_t value;
  EEPROM.get(address, value);
  return value;
}

inline bool IOTBOT::eepromWriteFloat(int address, float value)
{
  if (address < 0)
  {
    return false;
  }
  if (!_eepromEnsure((size_t)address + sizeof(float)))
  {
    return false;
  }

  EEPROM.put(address, value);
  return EEPROM.commit();
}

inline float IOTBOT::eepromReadFloat(int address, float defaultValue)
{
  if (address < 0)
  {
    return defaultValue;
  }
  if (!_eepromEnsure((size_t)address + sizeof(float)))
  {
    return defaultValue;
  }

  float value;
  EEPROM.get(address, value);
  return value;
}

inline bool IOTBOT::eepromWriteBytes(int address, const uint8_t *data, size_t len)
{
  if (address < 0 || data == nullptr)
  {
    return false;
  }

  if (len == 0)
  {
    return true;
  }

  if (!_eepromEnsure((size_t)address + len))
  {
    return false;
  }

  for (size_t i = 0; i < len; i++)
  {
    EEPROM.write(address + (int)i, data[i]);
  }

  return EEPROM.commit();
}

inline bool IOTBOT::eepromReadBytes(int address, uint8_t *data, size_t len)
{
  if (address < 0 || data == nullptr)
  {
    return false;
  }

  if (len == 0)
  {
    return true;
  }

  if (!_eepromEnsure((size_t)address + len))
  {
    return false;
  }

  for (size_t i = 0; i < len; i++)
  {
    data[i] = EEPROM.read(address + (int)i);
  }

  return true;
}

inline bool IOTBOT::eepromWriteString(int address, const String &value, uint16_t maxLen)
{
  if (address < 0)
  {
    return false;
  }

  if (maxLen == 0)
  {
    return false;
  }

  uint16_t len = (uint16_t)value.length();
  if (len > maxLen)
  {
    len = maxLen;
  }

  size_t total = sizeof(uint16_t) + (size_t)len;
  if (!_eepromEnsure((size_t)address + total))
  {
    return false;
  }

  EEPROM.put(address, len);
  for (uint16_t i = 0; i < len; i++)
  {
    EEPROM.write(address + (int)sizeof(uint16_t) + (int)i, (uint8_t)value[i]);
  }

  return EEPROM.commit();
}

inline String IOTBOT::eepromReadString(int address, uint16_t maxLen)
{
  if (address < 0 || maxLen == 0)
  {
    return String("");
  }

  if (!_eepromEnsure((size_t)address + sizeof(uint16_t)))
  {
    return String("");
  }

  uint16_t len = 0;
  EEPROM.get(address, len);

  if (len > maxLen)
  {
    len = maxLen;
  }

  if (!_eepromEnsure((size_t)address + sizeof(uint16_t) + (size_t)len))
  {
    return String("");
  }

  String out;
  out.reserve(len);
  for (uint16_t i = 0; i < len; i++)
  {
    out += (char)EEPROM.read(address + (int)sizeof(uint16_t) + (int)i);
  }
  return out;
}

inline bool IOTBOT::eepromClear(int startAddress, size_t length, uint8_t fill)
{
  if (startAddress < 0)
  {
    return false;
  }

  if (length == 0)
  {
    length = (_eepromSize == 0) ? 1024 : _eepromSize;
  }

  if (!_eepromEnsure((size_t)startAddress + length))
  {
    return false;
  }

  for (size_t i = 0; i < length; i++)
  {
    EEPROM.write(startAddress + (int)i, fill);
  }

  return EEPROM.commit();
}

inline uint32_t IOTBOT::eepromCrc32(const uint8_t *data, size_t len, uint32_t seed)
{
  if (data == nullptr)
  {
    return 0;
  }

  uint32_t crc = seed;
  for (size_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
    {
      if (crc & 1)
      {
        crc = (crc >> 1) ^ 0xEDB88320UL;
      }
      else
      {
        crc >>= 1;
      }
    }
  }
  return crc ^ 0xFFFFFFFFUL;
}

inline bool IOTBOT::eepromWriteRecord(int address, const uint8_t *data, uint16_t len, uint16_t version)
{
  if (address < 0 || data == nullptr)
  {
    return false;
  }

  const size_t headerSize = 2 + 2 + 2 + 4;
  if (!_eepromEnsure((size_t)address + headerSize + (size_t)len))
  {
    return false;
  }

  const uint16_t magic = _EEPROM_RECORD_MAGIC;
  const uint32_t crc = eepromCrc32(data, len);

  EEPROM.put(address, magic);
  EEPROM.put(address + 2, version);
  EEPROM.put(address + 4, len);
  EEPROM.put(address + 6, crc);

  for (uint16_t i = 0; i < len; i++)
  {
    EEPROM.write(address + (int)headerSize + (int)i, data[i]);
  }

  return EEPROM.commit();
}

inline bool IOTBOT::eepromReadRecord(int address, uint8_t *out, uint16_t maxLen, uint16_t *outLen, uint16_t *outVersion)
{
  if (address < 0 || out == nullptr || maxLen == 0)
  {
    return false;
  }

  const size_t headerSize = 2 + 2 + 2 + 4;
  if (!_eepromEnsure((size_t)address + headerSize))
  {
    return false;
  }

  uint16_t magic = 0;
  uint16_t version = 0;
  uint16_t len = 0;
  uint32_t storedCrc = 0;

  EEPROM.get(address, magic);
  if (magic != _EEPROM_RECORD_MAGIC)
  {
    return false;
  }

  EEPROM.get(address + 2, version);
  EEPROM.get(address + 4, len);
  EEPROM.get(address + 6, storedCrc);

  if (len > maxLen)
  {
    return false;
  }

  if (!_eepromEnsure((size_t)address + headerSize + (size_t)len))
  {
    return false;
  }

  for (uint16_t i = 0; i < len; i++)
  {
    out[i] = EEPROM.read(address + (int)headerSize + (int)i);
  }

  const uint32_t calcCrc = eepromCrc32(out, len);
  if (calcCrc != storedCrc)
  {
    return false;
  }

  if (outLen)
  {
    *outLen = len;
  }
  if (outVersion)
  {
    *outVersion = version;
  }
  return true;
}

inline bool IOTBOT::ntpSync(const char *ntpServer, long gmtOffsetSec, int daylightOffsetSec, uint32_t timeoutMs)
{
  if (ntpServer == nullptr || ntpServer[0] == '\0')
  {
    ntpServer = "pool.ntp.org";
  }

  configTime(gmtOffsetSec, daylightOffsetSec, ntpServer);

  const uint32_t startMs = millis();
  while ((millis() - startMs) < timeoutMs)
  {
    time_t now = time(nullptr);
    if (now >= _NTP_VALID_EPOCH)
    {
      return true;
    }
    delay(50);
  }

  return false;
}

inline bool IOTBOT::ntpBegin(int timezoneHours, const char *ntpServer, int daylightOffsetHours, uint32_t timeoutMs)
{
  const long gmtOffsetSec = (long)timezoneHours * 3600L;
  const int daylightOffsetSec = daylightOffsetHours * 3600;
  return ntpSync(ntpServer, gmtOffsetSec, daylightOffsetSec, timeoutMs);
}

inline bool IOTBOT::ntpIsTimeValid(time_t minEpoch)
{
  if (minEpoch <= 0)
  {
    minEpoch = _NTP_VALID_EPOCH;
  }
  return time(nullptr) >= minEpoch;
}

inline time_t IOTBOT::ntpGetEpoch()
{
  return time(nullptr);
}

inline String IOTBOT::ntpGetDateTimeString()
{
  time_t now = time(nullptr);
  if (now < _NTP_VALID_EPOCH)
  {
    return String("");
  }

  struct tm tmInfo;
  localtime_r(&now, &tmInfo);

  char buf[80];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           tmInfo.tm_year + 1900,
           tmInfo.tm_mon + 1,
           tmInfo.tm_mday,
           tmInfo.tm_hour,
           tmInfo.tm_min,
           tmInfo.tm_sec);
  return String(buf);
}

/*********************************** Servo Angle Control ***********************************
 * Moves a servo to the specified angle with optional acceleration control.
 * pin: The GPIO pin connected to the servo signal.
 * angle: The target angle for the servo (0° to 180°).
 * acceleration: The delay (in milliseconds) between incremental movements.
 */
#ifdef USE_SERVO // Eğer main.cpp içinde tanımlandıysa, burada aktif olur

inline void IOTBOT::moduleServoGoAngle(int pin, int angle, int acceleration)
{
  // Ensure acceleration is valid
  acceleration = max(acceleration, 1); // Minimum 1 ms gecikme

  // Attach the servo to the specified pin if not already attached
  if (!servoModule.attached())
  {
    servoModule.attach(pin, 1000, 2000); // Sadece bağlı değilse ata
  }

  // Ensure angle is within valid bounds (0 to 180 degrees)
  angle = constrain(angle, 0, 180);

  // Get the current position of the servo
  // int currentAngle = servoModule.read(); // Read the current angle (0° to 180°)

  // Determine movement direction (1 for increasing, -1 for decreasing)
  int step = (angle > currentAngle) ? 1 : -1;

  // Gradually move the servo to the target angle with acceleration
  for (int pos = currentAngle; pos != angle; pos += step)
  {
    servoModule.write(pos); // Move servo to the next position
    delay(acceleration);    // Delay for acceleration control
  }
  currentAngle = angle; // Read the current angle (0° to 180°)

  // Ensure the final angle is set correctly
  servoModule.write(angle);
}
#endif

/*********************************** DHT Sensor Initialization ***********************************
 * Configures the DHT sensor.
 * This is automatically initialized when reading temperature or humidity.
 */
#if defined(USE_DHT)

inline void IOTBOT::initializeDht(int pin, uint8_t type)
{
  if (!dhtSensor)
  {
    dhtSensor = new DHT(pin, type); // Create a new DHT object
    dhtSensor->begin();             // Initialize the sensor
  }
}

inline int IOTBOT::moduleDhtTempReadC(int pin) // Read Temperature
{
  initializeDht(pin, DHT11); // Ensure DHT11 is initialized
  float temp = dhtSensor->readTemperature();

  if (isnan(temp)) // Check if reading failed
    return -999;

  return static_cast<int>(temp);
}

inline int IOTBOT::moduleDthFeelingTempC(int pin) // Calculate Heat Index (Feeling Temperature)
{
  initializeDht(pin, DHT11); // Ensure DHT11 is initialized

  float temp = dhtSensor->readTemperature();
  float hum = dhtSensor->readHumidity();

  if (isnan(temp) || isnan(hum)) // Check if readings failed
    return -999;

  float heatIndex = dhtSensor->computeHeatIndex(temp, hum, false); // Calculate heat index in Celsius
  return static_cast<int>(heatIndex);
}

inline int IOTBOT::moduleDhtTempReadF(int pin) // Read Temperature in Fahrenheit
{
  initializeDht(pin, DHT11);                     // Ensure DHT11 is initialized
  float temp = dhtSensor->readTemperature(true); // **Fahrenheit sıcaklık okuma**

  if (isnan(temp)) // Check if reading failed
    return -999;

  return static_cast<int>(temp);
}

inline int IOTBOT::moduleDthFeelingTempF(int pin) // Calculate Heat Index (Feeling Temperature in Fahrenheit)
{
  initializeDht(pin, DHT11); // Ensure DHT11 is initialized

  float temp = dhtSensor->readTemperature(true); // **Fahrenheit sıcaklık okuma**
  float hum = dhtSensor->readHumidity();         // **Nem okuma**

  if (isnan(temp) || isnan(hum)) // Check if readings failed
    return -999;

  float heatIndex = dhtSensor->computeHeatIndex(temp, hum, true); // **Fahrenheit olarak hissedilen sıcaklık hesapla**
  return static_cast<int>(heatIndex);
}

inline int IOTBOT::moduleDhtHumRead(int pin) // Read Humidity
{
  initializeDht(pin, DHT11); // Ensure DHT11 is initialized
  float hum = dhtSensor->readHumidity();

  if (isnan(hum)) // Check if reading failed
    return -999;

  return static_cast<int>(hum);
}
#endif

/*********************************** Smart LED Sensor ***********************************
 */
#if defined(USE_NEOPIXEL)
inline void IOTBOT::extendSmartLEDPrepare(int pin, int numLEDs)
{
  // Create a new Adafruit_NeoPixel object dynamically
  pixels = new Adafruit_NeoPixel(numLEDs, pin, NEO_GRB + NEO_KHZ800);
  pixels->begin(); // Initialize the NeoPixel strip
  pixels->show();  // Turn off all LEDs initially
}

inline void IOTBOT::extendSmartLEDFill(int startLED, int endLED, int red, int green, int blue)
{
  if (pixels)
  {
    // Set the color for a range of LEDs
    for (int i = startLED; i <= endLED; i++)
    {
      pixels->setPixelColor(i, pixels->Color(red, green, blue));
    }
    pixels->show(); // Update the LEDs
  }
}

inline void IOTBOT::moduleSmartLEDPrepare(int pin)
{
  pixels = new Adafruit_NeoPixel(3, pin, NEO_GRB + NEO_KHZ800);
  pixels->begin();
  pixels->show(); // Clear all LEDs
}

inline void IOTBOT::moduleSmartLEDWrite(int led, int red, int green, int blue)
{
  if (pixels)
  {
    pixels->setPixelColor(led, pixels->Color(red, green, blue));
    pixels->show();
  }
}

inline uint32_t IOTBOT::getColor(int red, int green, int blue)
{
  return pixels->Color(red, green, blue);
}

inline void IOTBOT::moduleSmartLEDRainbowEffect(int wait)
{
  if (pixels)
  {
    for (long firstPixelHue = 0; firstPixelHue < 3 * 65536; firstPixelHue += 256)
    {
      for (int i = 0; i < pixels->numPixels(); i++)
      {
        int pixelHue = firstPixelHue + (i * 65536L / pixels->numPixels());
        pixels->setPixelColor(i, pixels->gamma32(pixels->ColorHSV(pixelHue)));
      }
      pixels->show();
      delay(wait);
    }
  }
}

inline void IOTBOT::moduleSmartLEDRainbowTheaterChaseEffect(int wait)
{
  if (pixels)
  {
    int firstPixelHue = 0;
    for (int a = 0; a < 30; a++)
    {
      for (int b = 0; b < 3; b++)
      {
        pixels->clear();
        for (int c = b; c < pixels->numPixels(); c += 3)
        {
          int hue = firstPixelHue + c * 65536L / pixels->numPixels();
          uint32_t color = pixels->gamma32(pixels->ColorHSV(hue));
          pixels->setPixelColor(c, color);
        }
        pixels->show();
        delay(wait);
        firstPixelHue += 65536 / 90;
      }
    }
  }
}

inline void IOTBOT::moduleSmartLEDTheaterChaseEffect(uint32_t color, int wait)
{
  if (pixels)
  {
    for (int a = 0; a < 10; a++)
    {
      for (int b = 0; b < 3; b++)
      {
        pixels->clear();
        for (int c = b; c < pixels->numPixels(); c += 3)
        {
          pixels->setPixelColor(c, color);
        }
        pixels->show();
        delay(wait);
      }
    }
  }
}

inline void IOTBOT::moduleSmartLEDColorWipeEffect(uint32_t color, int wait)
{
  if (pixels)
  {
    for (int i = 0; i < pixels->numPixels(); i++)
    {
      pixels->setPixelColor(i, color);
      pixels->show();
      delay(wait);
    }
  }
}
#endif

/*********************************** IR Sensor ***********************************
 */
#if defined(USE_RFID)

inline void IOTBOT::beginRFID()
{
  SPI.begin();            // SPI başlat
  rfid.PCD_Init();        // RFID başlat
  rfidInitialized = true; // RFID'nin başlatıldığını işaretle / Mark RFID as initialized
}

inline int IOTBOT::moduleRFIDRead()
{
  // Eğer RFID başlatılmadıysa, otomatik başlat / If RFID is not initialized, initialize it
  if (!rfidInitialized)
  {
    beginRFID();
  }

  String rfidNum = "";

  if (!rfid.PICC_IsNewCardPresent())
    return 0;
  if (!rfid.PICC_ReadCardSerial())
    return 0;

  for (byte i = 0; i < 4; i++)
  {
    rfidNum += String(rfid.uid.uidByte[i]);
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  return rfidNum.toInt();
}
#endif

/*********************************** IR Sensor ***********************************
 */
#if defined(USE_IR)

inline void IOTBOT::initializeIR(int pin) // Initialize the IR module / IR modülünü başlat
{
  if (!irrecv || irPin != pin)
  {                                           // Eğer IR alıcı yoksa veya pin değişmişse baştan başlat
    irPin = pin;                              // Store the IR receiver pin / IR alıcı pini sakla
    delete irrecv;                            // Önceki nesneyi temizle
    irrecv = new IRrecv(pin, 1024, 50, true); // Create a new IRrecv instance / Yeni bir IRrecv nesnesi oluştur
    irrecv->enableIRIn();                     // Start the IR receiver / IR alıcıyı başlat
  }
}

inline String IOTBOT::moduleIRReadHex(int pin) // Read IR signal in hexadecimal format / IR sinyalini HEX formatında oku
{
  initializeIR(pin); // Ensure IR is initialized / IR'nin başlatıldığından emin ol
  if (irrecv->decode(&results))
  {
    String hexCode = "0x" + String(results.value, HEX); // Convert to HEX / HEX formatına çevir
    irrecv->resume();                                   // Continue receiving new data / Yeni veri almak için devam et
    return hexCode;
  }
  return "0"; // No signal received / Sinyal yoksa 0 döndür
}

inline int IOTBOT::moduleIRReadDecimalx32(int pin) // Read IR signal as a full 32-bit decimal value / IR sinyalini tam 32-bit ondalık formatta oku
{
  initializeIR(pin); // Ensure IR is initialized / IR'nin başlatıldığından emin ol
  if (irrecv->decode(&results))
  {
    int decimalCode = results.value; // Return the full 32-bit value / Tam 32-bit değeri döndür
    irrecv->resume();                // Continue receiving new data / Yeni veri almak için devam et
    return decimalCode;
  }
  return 0; // No signal received / Sinyal yoksa 0 döndür
}

inline int IOTBOT::moduleIRReadDecimalx8(int pin) // Read IR signal as only the last 8 bits (for smaller values) / IR sinyalini sadece son 8 bit olarak oku (küçük değerler için)
{
  initializeIR(pin); // Ensure IR is initialized / IR'nin başlatıldığından emin ol
  if (irrecv->decode(&results))
  {
    int smallCode = results.value & 0xFF; // Extract only the last 8 bits / Sadece son 8 biti al
    irrecv->resume();                     // Continue receiving new data / Yeni veri almak için devam et
    return smallCode;
  }
  return 0; // No signal received / Sinyal yoksa 0 döndür
}
#endif

/*********************************** WiFi ***********************************/
#if defined(USE_WIFI)
inline void IOTBOT::wifiStartAndConnect(const char *ssid, const char *pass)
{
  Serial.printf("[WiFi]: Connection Starting!\r\n[WiFi]: SSID: %s\r\n[WiFi]: Pass: %s\r\n", ssid, pass);

  WiFi.begin(ssid, pass);
  int count = 0;
  while (count < 30)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.printf("\n[WiFi]: Connected!\r\n[WiFi]: Local IP: %s\r\n", WiFi.localIP().toString().c_str());
      Serial.printf("[WiFi]: MAC Address: %s\r\n", WiFi.macAddress().c_str());
      return;
    }
    Serial.print(".");
    delay(500);
    count++;
  }
  Serial.println();
  Serial.println("[WiFi]: Connection Timeout!");
}

inline bool IOTBOT::wifiConnectionControl()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("[WiFi]: Connection OK!");
    return true;
  }
  else
  {
    Serial.println("[WiFi]: Connection ERROR!");
    return false;
  }
}

inline String IOTBOT::wifiGetMACAddress()
{
  return WiFi.macAddress();
}

inline String IOTBOT::wifiGetIPAddress()
{
  return WiFi.localIP().toString();
}
#endif

/*********************************** OTA (Over-The-Air) ***********************************/
#if defined(USE_OTA)
inline void IOTBOT::otaBegin(const char *hostname, const char *password, uint16_t port)
{
  static String otaHost;
  if (hostname && strlen(hostname) > 0)
  {
    otaHost = hostname;
  }
  else
  {
    String mac = WiFi.macAddress();
    mac.replace(":", "");
    otaHost = String("IOTBOT-") + mac;
  }
  ArduinoOTA.setHostname(otaHost.c_str());

  if (password && strlen(password) > 0)
  {
    ArduinoOTA.setPassword(password);
  }
  else
  {
    ArduinoOTA.setPassword("1234");
  }

  ArduinoOTA.setPort(port);

  ArduinoOTA.onStart([]() {
    Serial.println("[OTA]: Start");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA]: End");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (total > 0)
    {
      Serial.printf("[OTA]: Progress: %u%%\r", (progress * 100) / total);
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA]: Error[%u]\n", error);
  });

  ArduinoOTA.begin();
  Serial.println("[OTA]: Ready");
}

inline void IOTBOT::otaHandle()
{
  ArduinoOTA.handle();
}
#endif

/*********************************** Server ***********************************/
#if defined(USE_SERVER)

inline void IOTBOT::serverStart(const char *mode, const char *ssid, const char *password)
{
  if (strcmp(mode, "STA") == 0)
  {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    Serial.printf("\n[STA Mode]: Connecting to WiFi: %s\n", ssid);

    int retries = 30;
    while (WiFi.status() != WL_CONNECTED && retries > 0)
    {
      delay(1000);
      Serial.print(".");
      retries--;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("\n[STA Mode]: Connected!");
      Serial.printf("[STA Mode]: IP Address: http://%s\n", WiFi.localIP().toString().c_str());
    }
    else
    {
      Serial.println("\n[STA Mode]: Connection Failed! Switching to AP Mode...");
      serverStart("AP", ssid, password);
      return;
    }
  }
  else if (strcmp(mode, "AP") == 0)
  {
    WiFi.softAP(ssid, password);
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    dnsServer.start(53, "*", IPAddress(192, 168, 4, 1));

    Serial.printf("\n[AP Mode]: Access Point Started!\n");
    Serial.printf("[AP Mode]: SSID: \"%s\"\n", ssid);
    Serial.printf("[AP Mode]: Password: \"%s\"\n", password);
    Serial.printf("[AP Mode]: AP IP Address: http://%s\n", WiFi.softAPIP().toString().c_str());
  }

  // 📌 Sayfaları tanımla
  serverCODLAI.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
                  {
      Serial.println("[Local Server]: Root URL Accessed!");
      request->send(200, "text/plain", "CODLAI Server is Running!"); });

  // 📌 404 Hatası
  serverCODLAI.onNotFound([](AsyncWebServerRequest *request)
                          {
      Serial.println("[Local Server]: Received an Unknown Request!");
      request->send(404, "text/plain", "Not Found"); });

  // 📌 **WebSocket Olaylarını Bağla**
  serverCODLAIWebSocket->onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
                                 {
      if (type == WS_EVT_CONNECT) {
          Serial.println("WebSocket Client Connected");
      } else if (type == WS_EVT_DISCONNECT) {
          Serial.println("WebSocket Client Disconnected");
      } });

  // 📌 WebSocket'i Sunucuya Bağla
  serverCODLAI.addHandler(serverCODLAIWebSocket);

  // 📌 **En son sunucuyu başlat!**
  serverCODLAI.begin();
  Serial.println("[Local Server]: Server Started! ✅");
}

inline void IOTBOT::serverCreateLocalPage(const char *url, const char *WEBPageScript, const char *WEBPageCSS, const char *WEBPageHTML, size_t bufferSize)
{
  // 📌 Sayfa içeriğini oluştur
  serverCODLAI.on(("/" + String(url)).c_str(), HTTP_GET, [WEBPageScript, WEBPageCSS, WEBPageHTML, bufferSize](AsyncWebServerRequest *request)
                  {
                    // Buffer boyutu kullanıcının belirttiği veya varsayılan değerle tanımlanır
                    char *buffer = new char[bufferSize];
                    int len = snprintf(buffer, bufferSize, WEBPageHTML, WEBPageScript, WEBPageCSS);

                    if ((size_t)len >= bufferSize)
                    {
                      Serial.println("[ERROR]: Buffer size insufficient, content truncated!");
                    }

                    request->send(200, "text/html", buffer);
                    delete[] buffer; // Dinamik olarak ayrılan belleği serbest bırakın
                  });

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.printf("[Local Server]: Page created at: http://%s/%s\n", WiFi.localIP().toString().c_str(), url);
  }
  else
  {
    Serial.printf("[Local Server]: Page created at: http://%s/%s\n", apIP.toString().c_str(), url);
  }
}

inline void IOTBOT::serverHandleDNS()
{
  dnsServer.processNextRequest();
}

inline void IOTBOT::serverContinue()
{
  if (WiFi.getMode() == WIFI_AP)
  {
    serverHandleDNS();
  }
}
#endif

/*********************************** Firebase Server Functions ***********************************/

#if defined(USE_FIREBASE)

// Initialize Firebase connection with SignUp Authentication
inline void IOTBOT::fbServerSetandStartWithUser(const char *projectURL, const char *secretKey, const char *userMail, const char *mailPass)
{
  firebaseData.setResponseSize(1024); // Optimize memory usage

  // Firebase Configuration Settings
  firebaseConfig.api_key = secretKey;
  firebaseConfig.database_url = projectURL;
  firebaseAuth.user.email = userMail;
  firebaseAuth.user.password = mailPass;

  // Zaman aşımı ayarları
  firebaseConfig.timeout.socketConnection = 10 * 1000; // 10 saniye bağlantı zaman aşımı

  // Token durumu izleme ayarı
  // firebaseConfig.token_status_callback = tokenStatusCallback;
  firebaseConfig.max_token_generation_retry = 5; // Daha fazla token yenileme denemesi

  // Wi-Fi bağlantısı kaybolduğunda otomatik yeniden bağlanma
  Firebase.reconnectWiFi(true);

  // Firebase başlat
  Serial.println("[Firebase]: Firebase connection starting...");
  Firebase.begin(&firebaseConfig, &firebaseAuth);

  Serial.println("[Firebase]: Verifying user credentials...");
  uint8_t id_count = 0;
  while (firebaseAuth.token.uid == "" && id_count < 50)
  {
    Serial.print('.');
    delay(500);
    id_count++;
  }
  if (firebaseAuth.token.uid == "")
  {
    Serial.println("\n[ERROR]: Authentication timeout.");
  }
  else
  {
    if (Firebase.ready())
    {
      strncpy(uid, firebaseAuth.token.uid.c_str(), 128 - 1); // UID'yi kopyala ve taşma kontrolü yap
      uid[128 - 1] = '\0';                                   // Diziyi null karakter ile sonlandır
      Serial.print("\n[Info]: Doğrulanan Kimlik ID: ");
      Serial.println(uid);
    }
    else
    {
      Serial.print("[ERROR]: Sign-up failed. Reason: ");
      Serial.println(firebaseData.errorReason());
    }
  }
}

/*********************************** Firebase Write Functions ***********************************/

inline void IOTBOT::fbServerSetInt(const char *dataPath, int data)
{
  // Corrected function call
  if (Firebase.RTDB.setInt(&firebaseData, dataPath, data))
  {
    Serial.println("[SUCCESS]: Integer data sent successfully!");
  }
  else
  {
    Serial.print("[ERROR]: Failed to send integer data. ");
    Serial.printf("HTTP Code: %d\n", firebaseData.httpCode());
    Serial.println("Reason: " + firebaseData.errorReason());
  }
}

inline void IOTBOT::fbServerSetFloat(const char *dataPath, float data)
{
  if (Firebase.RTDB.setFloat(&firebaseData, dataPath, data))
  {
    Serial.println("[SUCCESS]: Float data sent successfully!");
  }
  else
  {
    Serial.print("[ERROR]: Failed to send float data. ");
    Serial.printf("HTTP Code: %d\n", firebaseData.httpCode());
    Serial.println("Reason: " + firebaseData.errorReason());
  }
}

inline void IOTBOT::fbServerSetString(const char *dataPath, String data)
{
  if (Firebase.RTDB.setString(&firebaseData, dataPath, data))
  {
    Serial.println("[SUCCESS]: String data sent successfully!");
  }
  else
  {
    Serial.print("[ERROR]: Failed to send string data. ");
    Serial.printf("HTTP Code: %d\n", firebaseData.httpCode());
    Serial.println("Reason: " + firebaseData.errorReason());
  }
}

inline void IOTBOT::fbServerSetDouble(const char *dataPath, double data)
{
  if (Firebase.RTDB.setDouble(&firebaseData, dataPath, data))
  {
    Serial.println("[SUCCESS]: Double data sent successfully!");
  }
  else
  {
    Serial.print("[ERROR]: Failed to send double data. ");
    Serial.printf("HTTP Code: %d\n", firebaseData.httpCode());
    Serial.println("Reason: " + firebaseData.errorReason());
  }
}

inline void IOTBOT::fbServerSetBool(const char *dataPath, bool data)
{
  if (Firebase.RTDB.setBool(&firebaseData, dataPath, data))
  {
    Serial.println("[SUCCESS]: Boolean data sent successfully!");
  }
  else
  {
    Serial.print("[ERROR]: Failed to send boolean data. ");
    Serial.printf("HTTP Code: %d\n", firebaseData.httpCode());
    Serial.println("Reason: " + firebaseData.errorReason());
  }
}

inline void IOTBOT::fbServerSetJSON(const char *dataPath, String data)
{
  FirebaseJson json;
  json.set(dataPath, data);

  if (Firebase.RTDB.setJSON(&firebaseData, dataPath, &json))
  {
    Serial.println("[SUCCESS]: JSON data sent successfully!");
  }
  else
  {
    Serial.print("[ERROR]: Failed to send JSON data. ");
   
    Serial.printf("HTTP Code: %d\n", firebaseData.httpCode());
    Serial.println("Reason: " + firebaseData.errorReason());
  }
}

/*********************************** Firebase Read Functions ***********************************/

inline int IOTBOT::fbServerGetInt(const char *dataPath)
{
  if (Firebase.RTDB.getInt(&firebaseData, dataPath))
  {
    Serial.println("[SUCCESS]: Integer data retrieved successfully!");
    return firebaseData.intData();
  }
  Serial.println("[ERROR]: Failed to retrieve integer data.");
  return -1;
}

inline float IOTBOT::fbServerGetFloat(const char *dataPath)
{
  if (Firebase.RTDB.getFloat(&firebaseData, dataPath))
  {
    Serial.println("[SUCCESS]: Float data retrieved successfully!");
    return firebaseData.floatData();
  }
  Serial.println("[ERROR]: Failed to retrieve float data.");
  return -1.0;
}

inline String IOTBOT::fbServerGetString(const char *dataPath)
{
  if (Firebase.RTDB.getString(&firebaseData, dataPath))
  {
    Serial.println("[SUCCESS]: String data retrieved successfully!");
    return firebaseData.stringData();
  }
  Serial.println("[ERROR]: Failed to retrieve string data.");
  return "";
}

inline double IOTBOT::fbServerGetDouble(const char *dataPath)
{
  if (Firebase.RTDB.getDouble(&firebaseData, dataPath))
  {
    Serial.println("[SUCCESS]: Double data retrieved successfully!");
    return firebaseData.doubleData();
  }
  Serial.println("[ERROR]: Failed to retrieve double data.");
  return -1.0;
}

inline bool IOTBOT::fbServerGetBool(const char *dataPath)
{
  if (Firebase.RTDB.getBool(&firebaseData, dataPath))
  {
    Serial.println("[SUCCESS]: Boolean data retrieved successfully!");
    return firebaseData.boolData();
  }
  Serial.println("[ERROR]: Failed to retrieve boolean data.");
  return false;
}

inline String IOTBOT::fbServerGetJSON(const char *dataPath)
{
  if (Firebase.RTDB.getJSON(&firebaseData, dataPath))
  {
    Serial.println("[SUCCESS]: JSON data retrieved successfully!");
    return firebaseData.jsonString();
  }
  Serial.println("[ERROR]: Failed to retrieve JSON data.");
  return "{}";
}
#endif

/*********************************** ESP-NOW ***********************************/
#if defined(USE_ESPNOW)
inline void IOTBOT::initESPNow()
{
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != 0)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register send callback automatically
  _getInstance() = this;
  esp_now_register_send_cb(_onDataSent);
  
  Serial.println("ESP-NOW Initialized");
}

inline void IOTBOT::setWiFiChannel(int channel)
{
  #if defined(ESP32)
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  #elif defined(ESP8266)
    wifi_set_channel(channel);
  #endif
}

inline void IOTBOT::sendESPNow(const uint8_t *macAddr, const uint8_t *data, int len)
{
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, macAddr, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(macAddr))
  {
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
      Serial.println("Failed to add peer");
      return;
    }
  }

  esp_err_t result = esp_now_send(macAddr, data, len);
  if (result == ESP_OK)
  {
    // Serial.println("Sent with success");
  }
  else
  {
    Serial.println("Error sending the data");
  }
}

inline void IOTBOT::registerOnRecv(esp_now_recv_cb_t cb)
{
  esp_now_register_recv_cb(cb);
}
#endif

/*********************************** Telegram ***********************************/
#if defined(USE_TELEGRAM)
inline void IOTBOT::sendTelegram(String token, String chatId, String message)
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi not connected!");
    return;
  }

  WiFiClientSecure client;
  client.setInsecure(); // Educational use: skip certificate validation
  
  HTTPClient http;
  String url = "https://api.telegram.org/bot" + token + "/sendMessage?chat_id=" + chatId + "&text=" + message;
  
  Serial.println("[Telegram]: Sending message...");
  
  http.begin(client, url);
  int httpCode = http.GET();
  
  if (httpCode > 0)
  {
    String payload = http.getString();
    Serial.println("[Telegram]: Message sent! Response: " + payload);
  }
  else
  {
    Serial.println("[Telegram]: Error sending message. HTTP Code: " + String(httpCode));
  }
  
  http.end();
}
#endif

/*********************************** IFTTT ***********************************/
#if defined(USE_IFTTT)
inline bool IOTBOT::triggerIFTTTEvent(const String &eventName, const String &webhookKey, const String &jsonPayload)
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("[IFTTT]: WiFi not connected!");
    return false;
  }

  if (eventName.length() == 0 || webhookKey.length() == 0)
  {
    Serial.println("[IFTTT]: Event name or webhook key is missing.");
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  String url = "https://maker.ifttt.com/trigger/" + eventName + "/with/key/" + webhookKey;

  Serial.println("[IFTTT]: Triggering '" + eventName + "'...");

  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  int httpCode = http.POST(jsonPayload);

  if (httpCode > 0)
  {
    Serial.println("[IFTTT]: HTTP " + String(httpCode));
    String payload = http.getString();
    Serial.println("[IFTTT]: Response => " + payload);
  }
  else
  {
    Serial.println("[IFTTT]: Error sending request => " + http.errorToString(httpCode));
  }

  http.end();
  return httpCode == HTTP_CODE_OK;
}
#endif

/*********************************** Bluetooth ***********************************/
#if defined(USE_BLUETOOTH) && defined(ESP32)
inline void IOTBOT::bluetoothStart(String name, String pin)
{
  if (pin != "") {
    serialBT.setPin(pin.c_str());
    Serial.println("[Bluetooth]: PIN set to " + pin);
  }
  serialBT.begin(name); // Bluetooth device name
  Serial.println("[Bluetooth]: Started as " + name);
}

inline bool IOTBOT::bluetoothConnect(String remoteName)
{
  Serial.println("[Bluetooth]: Connecting to " + remoteName + "...");
  bool connected = serialBT.connect(remoteName);
  if(connected) {
    Serial.println("[Bluetooth]: Connected successfully!");
  } else {
    Serial.println("[Bluetooth]: Connection failed!");
  }
  return connected;
}

inline void IOTBOT::bluetoothWrite(String message)
{
  if (serialBT.hasClient()) {
    serialBT.println(message);
  }
}

inline String IOTBOT::bluetoothRead()
{
  if (serialBT.available()) {
    return serialBT.readString();
  }
  return "";
}

inline BluetoothSerial* IOTBOT::getBluetoothObject()
{
  return &serialBT;
}
#endif

/*********************************** Email ***********************************/
#if defined(USE_EMAIL)
inline void IOTBOT::sendEmail(String smtpHost, int smtpPort, String authorEmail, String authorPassword, String recipientEmail, String subject, String messageStr)
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi not connected!");
    return;
  }

  SMTPSession smtp;
  Session_Config config;
  config.server.host_name = smtpHost;
  config.server.port = smtpPort;
  config.login.email = authorEmail;
  config.login.password = authorPassword;
  config.login.user_domain = "";

  SMTP_Message message;
  message.sender.name = "IOTBOT";
  message.sender.email = authorEmail;
  message.subject = subject;
  message.addRecipient("User", recipientEmail);
  message.text.content = messageStr.c_str();

  smtp.connect(&config);
  if (!MailClient.sendMail(&smtp, &message))
    Serial.println("Error sending Email, " + smtp.errorReason());
  else
    Serial.println("Email sent successfully!");
}
#endif

/*********************************** Weather ***********************************/
#if defined(USE_WEATHER)
#include <ArduinoJson.h>
inline String IOTBOT::getWeather(String city, String apiKey)
{
  if (WiFi.status() != WL_CONNECTED)
    return "WiFi Error";

  HTTPClient http;
  String url;

  // Eğer API Key girilmemişse veya varsayılan değerdeyse wttr.in kullan (API Key gerektirmez)
  // If API Key is empty or default, use wttr.in (No API Key required)
  if (apiKey == "" || apiKey == "YOUR_API_KEY") {
      Serial.println("[Weather]: Using wttr.in (Free Service)...");
      
      WiFiClientSecure client;
      client.setInsecure(); // Sertifika doğrulamasını atla / Skip certificate validation
      client.setHandshakeTimeout(20000); // 20 saniye handshake zaman aşımı

      // wttr.in format: %t (Temperature), %C (Condition)
      url = "https://wttr.in/" + city + "?format=%t+%C";
      
      Serial.println("[Weather]: Requesting URL: " + url);
      
      http.begin(client, url);
      http.setConnectTimeout(20000); // 20 saniye bağlantı zaman aşımı
      http.setUserAgent("curl/7.68.0"); // User-Agent ekle

      Serial.println("[Weather]: Sending GET request...");
      int httpCode = http.GET();
      Serial.println("[Weather]: GET request finished. HTTP Code: " + String(httpCode));

      if (httpCode > 0) {
          String payload = http.getString();
          http.end();
          payload.trim(); // Boşlukları temizle / Trim whitespace
          Serial.println("[Weather]: Data received: " + payload);
          return payload;
      } else {
          String errorStr = http.errorToString(httpCode);
          http.end();
          Serial.println("[Weather]: Error: " + errorStr);
          return "Error: " + errorStr;
      }
  } 
  else {
      // OpenWeatherMap kullan
      url = "http://api.openweathermap.org/data/2.5/weather?q=" + city + "&appid=" + apiKey + "&units=metric";

      http.begin(url);
      int httpCode = http.GET();

      if (httpCode > 0)
      {
        String payload = http.getString();
        JsonDocument doc; // DynamicJsonDocument yerine JsonDocument kullanıyoruz (ArduinoJson v7)
        deserializeJson(doc, payload);
        float temp = doc["main"]["temp"];
        String weather = doc["weather"][0]["description"];
        http.end();
        return String(temp) + "C, " + weather;
      }
      else
      {
        http.end();
        return "Error (OWM)";
      }
  }
}
#endif

/*********************************** Wikipedia ***********************************/
#if defined(USE_WIKIPEDIA)
#include <ArduinoJson.h>
inline String IOTBOT::getWikipedia(String query, String lang)
{
  if (WiFi.status() != WL_CONNECTED)
    return "WiFi Error";

  WiFiClientSecure client;
  client.setInsecure(); // Sertifika doğrulamasını atla / Skip certificate validation
  client.setHandshakeTimeout(20000); // 20 saniye handshake zaman aşımı

  HTTPClient http;
  // Dil seçeneğine göre URL oluştur / Create URL based on language option
  String url = "https://" + lang + ".wikipedia.org/api/rest_v1/page/summary/" + query;

  Serial.println("[Wikipedia]: Requesting URL: " + url);
  
  http.begin(client, url);
  http.setConnectTimeout(20000); // 20 saniye bağlantı zaman aşımı
  http.setUserAgent("curl/7.68.0"); // User-Agent ekle

  int httpCode = http.GET();

  if (httpCode > 0)
  {
    String payload = http.getString();
    JsonDocument doc; // DynamicJsonDocument yerine JsonDocument kullanıyoruz (ArduinoJson v7)
    deserializeJson(doc, payload);
    
    // Extract alanını kontrol et / Check extract field
    if (doc.containsKey("extract")) {
        String extract = doc["extract"].as<String>();
        http.end();
        return extract;
    } else {
        http.end();
        return "No Summary Found";
    }
  }
  else
  {
    String errorStr = http.errorToString(httpCode);
    http.end();
    Serial.println("[Wikipedia]: Error: " + errorStr);
    return "Error: " + errorStr;
  }
}
#endif

/*********************************** Helpers Implementation ***********************************/

inline void IOTBOT::lcdWriteFixedTxt(int col, int row, const char *txt, int width)
{
  lcd.setCursor(col, row);
  String s = convertTR(String(txt));
  lcd.print(s);
  for (int i = s.length(); i < width; i++) lcd.print(" ");
}

inline void IOTBOT::lcdWriteFixed(int col, int row, int value, int width)
{
  lcd.setCursor(col, row);
  String s = String(value);
  lcd.print(s);
  for (int i = s.length(); i < width; i++) lcd.print(" ");
}

inline void IOTBOT::calibrateJoystick(int &xCenter, int &yCenter, int samples)
{
  long xSum = 0, ySum = 0;
  for (int i = 0; i < samples; i++)
  {
    xSum += joystickXRead();
    ySum += joystickYRead();
    delay(10);
  }
  xCenter = xSum / samples;
  yCenter = ySum / samples;
}

#if defined(USE_ESPNOW)
inline bool IOTBOT::addBroadcastPeer(int channel)
{
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  // Broadcast address: FF:FF:FF:FF:FF:FF
  const uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = channel;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(broadcastAddress))
  {
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
      return false;
    }
  }
  return true;
}
#endif

/*********************************** Multi-Tasking Implementation ***********************************/
#if defined(ESP32)
inline void IOTBOT::taskDelay(int ms)
{
  vTaskDelay(ms / portTICK_PERIOD_MS);
}

inline void IOTBOT::_taskEntry(void *pvParameters)
{
  void (*taskFunc)() = (void (*)())pvParameters;
  if (taskFunc)
  {
    taskFunc();
  }
  vTaskDelete(NULL);
}

inline void IOTBOT::_taskLoopEntry(void *pvParameters)
{
  void (*taskFunc)() = (void (*)())pvParameters;
  if (taskFunc)
  {
    for (;;)
    {
      taskFunc();
      // Safety delay to prevent WDT reset if user forgets delay in empty loop
      // Kullanıcı boş döngü yaparsa WDT resetlenmesini önlemek için güvenlik gecikmesi
      vTaskDelay(1); 
    }
  }
  vTaskDelete(NULL);
}

inline void IOTBOT::createTask(TaskFunction_t taskFunction, const char *name, int coreID, int stackSize, int priority)
{
  xTaskCreatePinnedToCore(
      taskFunction, /* Task function */
      name,         /* Name of task */
      stackSize,    /* Stack size of task */
      NULL,         /* Parameter of the task */
      priority,     /* Priority of the task */
      NULL,         /* Task handle to keep track of created task */
      coreID);      /* Pin task to core */
}

inline void IOTBOT::createTask(void (*taskFunction)(), const char *name, int coreID, int stackSize, int priority)
{
  xTaskCreatePinnedToCore(
      _taskEntry,          /* Wrapper function */
      name,                /* Name of task */
      stackSize,           /* Stack size of task */
      (void *)taskFunction, /* Pass user function as parameter */
      priority,            /* Priority of the task */
      NULL,                /* Task handle */
      coreID);             /* Pin task to core */
}

inline void IOTBOT::createLoopTask(void (*taskFunction)(), const char *name, int coreID, int priority, int stackSize)
{
  xTaskCreatePinnedToCore(
      _taskLoopEntry,      /* Wrapper function */
      name,                /* Name of task */
      stackSize,           /* Stack size of task */
      (void *)taskFunction, /* Pass user function as parameter */
      priority,            /* Priority of the task */
      NULL,                /* Task handle */
      coreID);             /* Pin task to core */
}
#endif

#else
#error "Unsupported platform! Only ESP32 and ESP8266 are supported."
#endif
#endif // IOTBOT_H
