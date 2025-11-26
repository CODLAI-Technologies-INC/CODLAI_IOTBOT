# CODLAI IOTBOT Library Reference / Kütüphane Referansı

## Introduction / Giriş
**EN:** The IOTBOT library is the most comprehensive library in the CODLAI ecosystem, designed for the Master Controller (ESP32). It manages inputs (Joystick, Encoder, Buttons), outputs (LCD, Buzzer, Motors), and advanced communication (ESP-NOW, Bluetooth, WiFi, Firebase).
**TR:** IOTBOT kütüphanesi, CODLAI ekosistemindeki en kapsamlı kütüphanedir ve Ana Kontrolcü (ESP32) için tasarlanmıştır. Girişleri (Joystick, Enkoder, Butonlar), çıkışları (LCD, Buzzer, Motorlar) ve gelişmiş haberleşmeyi (ESP-NOW, Bluetooth, WiFi, Firebase) yönetir.

## Functions / Fonksiyonlar

### System & Initialization / Sistem & Başlatma

#### begin
**EN:** Initializes all onboard sensors, pins, LCD, and plays the startup sound.
**TR:** Tüm dahili sensörleri, pinleri, LCD'yi başlatır ve açılış sesini çalar.
**Syntax:** `void begin()`

#### playIntro
**EN:** Displays a multilingual greeting on the LCD and plays a melody.
**TR:** LCD'de çok dilli bir selamlama gösterir ve bir melodi çalar.
**Syntax:** `void playIntro()`

### LCD Display / LCD Ekran

#### lcdWrite
**EN:** Writes text, numbers, or boolean values to the LCD. Handles Turkish characters automatically.
**TR:** LCD'ye metin, sayı veya boolean değerler yazar. Türkçe karakterleri otomatik olarak işler.
**Syntax:** `void lcdWrite(String text)`

#### lcdWriteCR
**EN:** Writes data to a specific Column and Row.
**TR:** Belirli bir Sütun ve Satıra veri yazar.
**Syntax:** `void lcdWriteCR(int col, int row, String text)`

#### lcdWriteMid
**EN:** Writes 4 lines of text, centered on the screen.
**TR:** Ekrana ortalanmış 4 satır metin yazar.
**Syntax:** `void lcdWriteMid(const char *l1, const char *l2, const char *l3, const char *l4)`

#### lcdClear
**EN:** Clears the LCD screen.
**TR:** LCD ekranını temizler.
**Syntax:** `void lcdClear()`

#### lcdShowLoading
**EN:** Shows a loading animation with a message.
**TR:** Bir mesaj ile yükleniyor animasyonu gösterir.
**Syntax:** `void lcdShowLoading(String message)`

#### lcdShowStatus
**EN:** Displays a status screen (e.g., "WiFi [OK]").
**TR:** Bir durum ekranı gösterir (örneğin, "WiFi [OK]").
**Syntax:** `void lcdShowStatus(String title, String status, bool isSuccess)`

### Inputs / Girişler

#### joystickXRead / joystickYRead
**EN:** Reads the X and Y axis values of the joystick (0-4095).
**TR:** Joystick'in X ve Y eksen değerlerini okur (0-4095).
**Syntax:** `int joystickXRead()`

#### calibrateJoystick
**EN:** Calibrates the joystick center position by taking multiple samples.
**TR:** Birden fazla örnek alarak joystick merkez pozisyonunu kalibre eder.
**Syntax:** `void calibrateJoystick(int &xCenter, int &yCenter, int samples)`

#### button1Read / button2Read / button3Read
**EN:** Reads the state of the onboard buttons.
**TR:** Dahili butonların durumunu okur.
**Syntax:** `bool button1Read()`

#### encoderRead
**EN:** Returns the current position count of the rotary encoder.
**TR:** Döner enkoderin mevcut pozisyon sayısını döndürür.
**Syntax:** `int encoderRead()`

#### potentiometerRead
**EN:** Reads the potentiometer value.
**TR:** Potansiyometre değerini okur.
**Syntax:** `int potentiometerRead()`

#### ldrRead
**EN:** Reads the Light Dependent Resistor (LDR) value.
**TR:** Işığa Duyarlı Direnç (LDR) değerini okur.
**Syntax:** `int ldrRead()`

### Actuators / Eyleyiciler

#### moduleDCMotorGOClockWise / CounterClockWise
**EN:** Rotates the DC motor in the specified direction at a given speed (0-100).
**TR:** DC motoru belirtilen yönde ve hızda (0-100) döndürür.
**Syntax:** `void moduleDCMotorGOClockWise(int speed)`

#### moduleStepMotorMotion
**EN:** Controls a stepper motor (steps, direction, acceleration, speed).
**TR:** Bir step motoru kontrol eder (adım, yön, ivme, hız).
**Syntax:** `void moduleStepMotorMotion(int step, bool rotation, int accel, int speed)`

#### moduleServoGoAngle
**EN:** Moves a servo to a specific angle with speed control.
**TR:** Bir servoyu hız kontrolü ile belirli bir açıya hareket ettirir.
**Syntax:** `void moduleServoGoAngle(int pin, int angle, int acceleration)`

#### relayWrite
**EN:** Controls the onboard relay.
**TR:** Dahili röleyi kontrol eder.
**Syntax:** `void relayWrite(bool status)`

#### buzzerPlayTone
**EN:** Plays a tone on the buzzer.
**TR:** Buzzer'da bir ton çalar.
**Syntax:** `void buzzerPlayTone(int frequency, int duration)`

### Communication / Haberleşme

#### initESPNow
**EN:** Initializes ESP-NOW protocol.
**TR:** ESP-NOW protokolünü başlatır.
**Syntax:** `void initESPNow()`

#### addBroadcastPeer
**EN:** Adds the broadcast address (FF:FF:FF:FF:FF:FF) as a peer for broadcasting messages.
**TR:** Mesaj yayını yapmak için yayın adresini (FF:FF:FF:FF:FF:FF) eş (peer) olarak ekler.
**Syntax:** `bool addBroadcastPeer(int channel)`

#### sendESPNow
**EN:** Sends data to a specific MAC address via ESP-NOW.
**TR:** ESP-NOW üzerinden belirli bir MAC adresine veri gönderir.
**Syntax:** `void sendESPNow(const uint8_t *macAddr, const uint8_t *data, int len)`

#### wifiStartAndConnect
**EN:** Connects to a WiFi network.
**TR:** Bir WiFi ağına bağlanır.
**Syntax:** `void wifiStartAndConnect(const char *ssid, const char *pass)`

#### bluetoothStart
**EN:** Starts the Bluetooth Serial interface (ESP32 only).
**TR:** Bluetooth Seri arayüzünü başlatır (Sadece ESP32).
**Syntax:** `void bluetoothStart(String name)`

#### fbServerSet... / fbServerGet...
**EN:** Functions to interact with Firebase Realtime Database.
**TR:** Firebase Gerçek Zamanlı Veritabanı ile etkileşim fonksiyonları.
**Syntax:** `void fbServerSetInt(...)`, `int fbServerGetInt(...)`

#### sendTelegram
**EN:** Sends a message via Telegram.
**TR:** Telegram üzerinden mesaj gönderir.
**Syntax:** `void sendTelegram(String token, String chatId, String message)`

### Sensors / Sensörler
**EN:** Supports reading from various modules: DHT, Ultrasonic, RFID, IR, Soil Moisture, Smoke, etc.
**TR:** Çeşitli modüllerden okumayı destekler: DHT, Ultrasonik, RFID, IR, Toprak Nemi, Duman vb.
*   `moduleDhtTempReadC(int pin)`
*   `moduleUltrasonicDistanceRead()`
*   `moduleRFIDRead()`
*   `moduleIRReadHex(int pin)`

### Serial Communication / Seri Haberleşme

#### serialStart
**EN:** Starts serial communication at the specified baud rate.
**TR:** Seri haberleşmeyi belirtilen baud hızında başlatır.
**Syntax:** `void serialStart(int baudrate)`

#### serialWrite
**EN:** Writes data to the serial port (Overloaded for String, int, float, bool).
**TR:** Seri porta veri yazar (String, int, float, bool için aşırı yüklenmiştir).
**Syntax:** `void serialWrite(data)`
