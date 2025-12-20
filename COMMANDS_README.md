# CODLAI_IOTBOT Library Documentation / Kütüphane Dokümantasyonu

**EN:** The `IOTBOT` library is the core library developed for the Master Controller (ESP32). It manages a wide range of sensors, actuators (motors, relays, etc.), displays, and communication protocols.
**TR:** `IOTBOT` kütüphanesi, Ana kontrolcü (Master - ESP32) için geliştirilmiş çekirdek kütüphanedir. Çok çeşitli sensörleri, eyleyicileri (motorlar, röleler vb.), ekranları ve iletişim protokollerini yönetir.

### Initialization & System / Başlatma ve Sistem
*   `IOTBOT()`
    *   **EN:** Constructor function. Prepares the LCD object and basic variables.
    *   **TR:** Kurucu fonksiyon. LCD nesnesini ve temel değişkenleri hazırlar.
*   `void begin()`
    *   **EN:** Initializes the system. Activates all onboard components like Joystick, Buttons, Encoder, Buzzer, Relay, LDR, and Potentiometer. Turns on the LCD and plays the startup sound.
    *   **TR:** Sistemi başlatır. Joystick, Butonlar, Enkoder, Buzzer, Röle, LDR ve Potansiyometre gibi tüm dahili bileşenleri aktif hale getirir. LCD ekranı açar ve başlangıç sesini çalar.
*   `void playIntro()`
    *   **EN:** Displays a multilingual greeting message on the LCD and plays the opening melody.
    *   **TR:** LCD ekranda çok dilli bir selamlama mesajı gösterir ve açılış melodisini çalar.

### Serial Communication / Seri Haberleşme
*   `void serialStart(int baudrate)`
    *   **EN:** Starts serial communication at the specified baud rate.
    *   **TR:** Seri haberleşmeyi belirtilen hızda başlatır.
*   `void serialWrite(const char *message)`
    *   **EN:** Writes text to the serial port.
    *   **TR:** Seri porta metin yazar.
*   `void serialWrite(String message)`
    *   **EN:** Writes a String to the serial port.
    *   **TR:** Seri porta String yazar.
*   `void serialWrite(long/int/float/bool value)`
    *   **EN:** Writes numerical or boolean values to the serial port.
    *   **TR:** Sayısal veya mantıksal değerleri seri porta yazar.
*   `int serialAvailable()`
    *   **EN:** Checks if there is data available to read from the serial port.
    *   **TR:** Seri porttan okunabilir veri olup olmadığını kontrol eder.
*   `String serialReadStringUntil(char terminator)`
    *   **EN:** Reads from the serial port until the specified character is received.
    *   **TR:** Belirtilen karakter gelene kadar seri porttan okuma yapar.

### Display (LCD) / Ekran (LCD)
*   `void lcdWrite(String text)`
    *   **EN:** Writes text to the current cursor position. Automatically converts Turkish characters (ç, ğ, ı, ö, ş, ü) to LCD-compatible characters.
    *   **TR:** İmlecin bulunduğu konuma metin yazar. Türkçe karakterleri (ç, ğ, ı, ö, ş, ü) otomatik olarak LCD uyumlu karakterlere dönüştürür.
*   `void lcdWrite(int/float/bool value)`
    *   **EN:** Writes numerical or boolean values to the screen.
    *   **TR:** Sayısal veya mantıksal değerleri ekrana yazar.
*   `void lcdWriteCR(int col, int row, String text)`
    *   **EN:** Writes text to the specified Column and Row.
    *   **TR:** Belirtilen Sütun (Column) ve Satıra (Row) metin yazar.
*   `void lcdWriteCR(int col, int row, int/float/bool value)`
    *   **EN:** Writes a numerical value to the specified position.
    *   **TR:** Belirtilen konuma sayısal değer yazar.
*   `void lcdWriteMid(const char *line1, const char *line2, const char *line3, const char *line4)`
    *   **EN:** Writes 4 lines of text, centered on the screen.
    *   **TR:** 4 satırlık metni ekrana ortalayarak yazar.
*   `void lcdWriteFixedTxt(int col, int row, const char *txt, int width)`
    *   **EN:** Writes fixed-width text to the specified position (prevents overflow).
    *   **TR:** Belirtilen genişlikte sabit metin yazar (taşmaları engeller).
*   `void lcdWriteFixed(int col, int row, int value, int width)`
    *   **EN:** Writes a number with a fixed width.
    *   **TR:** Belirtilen genişlikte sayı yazar.
*   `void lcdSetCursor(int col, int row)`
    *   **EN:** Moves the cursor to the specified position.
    *   **TR:** İmleci belirtilen konuma taşır.
*   `void lcdClear()`
    *   **EN:** Clears the LCD screen completely.
    *   **TR:** LCD ekranını tamamen temizler.
*   `void lcdShowLoading(String message)`
    *   **EN:** Shows a loading animation along with the provided message.
    *   **TR:** Ekranda bir yükleme animasyonu ile birlikte verilen mesajı gösterir.
*   `void lcdShowStatus(String title, String status, bool isSuccess)`
    *   **EN:** Creates a status screen (e.g., "WiFi [OK]" or "Error [X]").
    *   **TR:** Bir durum ekranı oluşturur (Örn: "WiFi [OK]" veya "Hata [X]").
*   `void lcdtest()`
    *   **EN:** Tests the LCD screen.
    *   **TR:** LCD ekranını test eder.

### Inputs / Giriş Birimleri
*   **Joystick**:
    *   `int joystickXRead()`
        *   **EN:** Reads the X-axis value (0-4095).
        *   **TR:** X ekseni değerini okur (0-4095 arası).
    *   `int joystickYRead()`
        *   **EN:** Reads the Y-axis value (0-4095).
        *   **TR:** Y ekseni değerini okur (0-4095 arası).
    *   `bool joystickButtonRead()`
        *   **EN:** Reads whether the joystick button is pressed (Active LOW).
        *   **TR:** Joystick butonuna basılıp basılmadığını okur (Aktif DÜŞÜK/LOW).
    *   `void calibrateJoystick(int &xCenter, int &yCenter, int samples)`
        *   **EN:** Calibrates the center point of the joystick.
        *   **TR:** Joystick'in merkez noktasını kalibre eder.
    *   `void joysticktest()`
        *   **EN:** Tests the joystick by displaying values on the screen.
        *   **TR:** Joystick değerlerini ekranda göstererek test eder.
*   **Buttons / Butonlar**:
    *   `bool button1Read()`
        *   **EN:** Reads Button 1 (Works as Analog or Digital depending on WiFi status).
        *   **TR:** Buton 1'i okur (WiFi durumuna göre Analog veya Dijital çalışır).
    *   `bool button2Read()`
        *   **EN:** Reads Button 2.
        *   **TR:** Buton 2'yi okur.
    *   `bool button3Read()`
        *   **EN:** Reads Button 3 (Digital).
        *   **TR:** Buton 3'ü okur (Dijital).
    *   `void buttonsAnalogtest()`
        *   **EN:** Tests the buttons.
        *   **TR:** Butonları test eder.
*   **Encoder / Enkoder**:
    *   `int encoderRead()`
        *   **EN:** Returns the current position value of the rotary encoder.
        *   **TR:** Döner kodlayıcının o anki pozisyon değerini döndürür.
    *   `bool encoderButtonRead()`
        *   **EN:** Reads whether the encoder button is pressed.
        *   **TR:** Enkoderin üzerindeki butona basılıp basılmadığını okur.
    *   `void encodertest()`
        *   **EN:** Tests the encoder.
        *   **TR:** Enkoderi test eder.
*   **Potentiometer / Potansiyometre**:
    *   `int potentiometerRead()`
        *   **EN:** Reads the potentiometer value (0-4095).
        *   **TR:** Potansiyometre değerini okur (0-4095).
    *   `void potentiometertest()`
        *   **EN:** Tests the potentiometer.
        *   **TR:** Potansiyometreyi test eder.
*   **Sensors / Sensörler**:
    *   `int ldrRead()`
        *   **EN:** Reads the Light Dependent Resistor (LDR) value (Ambient light level).
        *   **TR:** Işık Bağımlı Direnç (LDR) değerini okur (Ortam ışık seviyesi).
    *   `void ldrtest()`
        *   **EN:** Tests the LDR sensor.
        *   **TR:** LDR sensörünü test eder.
    *   `int moduleMicRead(int pin)`
        *   **EN:** Reads the analog sound level from the microphone module.
        *   **TR:** Mikrofon modülünden analog ses seviyesini okur.
    *   `bool moduleMotionRead(int pin)`
        *   **EN:** Reads digital output from the motion sensor (PIR).
        *   **TR:** Hareket sensöründen (PIR) dijital okuma yapar.
    *   `int moduleSoilMoistureRead(int pin)`
        *   **EN:** Reads the soil moisture sensor value.
        *   **TR:** Toprak nem sensörü değerini okur.
    *   `int moduleSmokeRead(int pin)`
        *   **EN:** Reads the smoke/gas sensor value.
        *   **TR:** Duman/Gaz sensörü değerini okur.
    *   `bool moduleMagneticRead(int pin)`
        *   **EN:** Reads the magnetic door/window sensor.
        *   **TR:** Manyetik kapı/pencere sensörünü okur.
    *   `bool moduleVibrationDigitalRead(int pin)`
        *   **EN:** Reads the vibration sensor digitally.
        *   **TR:** Titreşim sensörünü dijital olarak okur.
    *   `int moduleVibrationAnalogRead(int pin)`
        *   **EN:** Reads the vibration sensor analog value.
        *   **TR:** Titreşim sensörünü analog olarak okur.
    *   `float moduleNtcTempRead(int pin)`
        *   **EN:** Reads temperature from the NTC sensor (Celsius).
        *   **TR:** NTC sensöründen sıcaklık okur (Celsius).
    *   `int moduleMatrisButtonAnalogRead(int pin)`
        *   **EN:** Reads analog value from the matrix button.
        *   **TR:** Matris butondan analog değer okur.
    *   `int moduleMatrisButtonNumberRead(int pin)`
        *   **EN:** Reads the pressed key number (1-5) from the matrix button.
        *   **TR:** Matris butondan basılan tuş numarasını (1-5) okur.

### Actuators / Eyleyiciler
*   **Motors / Motorlar**:
    *   `void moduleDCMotorGOClockWise(int speed)`
        *   **EN:** Rotates the DC motor clockwise (Speed 0-100).
        *   **TR:** DC motoru saat yönünde döndürür (Hız 0-100 arası).
    *   `void moduleDCMotorGOCounterClockWise(int speed)`
        *   **EN:** Rotates the DC motor counter-clockwise.
        *   **TR:** DC motoru saat yönünün tersine döndürür.
    *   `void moduleDCMotorStop()`
        *   **EN:** Stops the DC motor (Free coasting).
        *   **TR:** DC motoru durdurur (Serbest duruş).
    *   `void moduleDCMotorBrake()`
        *   **EN:** Stops the DC motor with braking.
        *   **TR:** DC motoru frenleyerek durdurur.
    *   `void moduleStepMotorMotion(int step, bool rotation, int accelometer, int speed)`
        *   **EN:** Controls a stepper motor (Steps, Direction, Acceleration, Speed).
        *   **TR:** Step motoru kontrol eder (Adım sayısı, Yön, İvme, Hız).
*   **Servos / Servolar**:
    *   `void moduleServoGoAngle(int pin, int angle, int acceleration)`
        *   **EN:** Moves the servo motor to the specified angle at the specified speed (acceleration).
        *   **TR:** Servo motoru belirtilen açıya, belirtilen hızda (ivme) götürür.
*   **Relay / Röle**:
    *   `void relayWrite(bool status)`
        *   **EN:** Turns the onboard relay on/off.
        *   **TR:** Kart üzerindeki dahili röleyi açar/kapatır.
    *   `void moduleRelayWrite(int pin, bool status)`
        *   **EN:** Controls an external relay module.
        *   **TR:** Harici bir röle modülünü kontrol eder.
    *   `void relaytest()`
        *   **EN:** Tests the onboard relay.
        *   **TR:** Dahili röleyi test eder.
*   **Buzzer (Sound) / Buzzer (Ses)**:
    *   `void buzzerPlayTone(int frequency, int duration)`
        *   **EN:** Plays a tone at the specified frequency and duration.
        *   **TR:** Belirtilen frekans ve sürede bir ton çalar.
    *   `void buzzerPlay(int frequency, int duration)`
        *   **EN:** Same as `buzzerPlayTone`.
        *   **TR:** `buzzerPlayTone` ile aynıdır.
    *   `void buzzerStart(int frequency)`
        *   **EN:** Starts sound at the specified frequency (Continuous).
        *   **TR:** Belirtilen frekansta sesi başlatır (Süresiz).
    *   `void buzzerStop()`
        *   **EN:** Stops the sound.
        *   **TR:** Sesi durdurur.
    *   `void buzzerSoundIntro()`
        *   **EN:** Plays the standard startup melody.
        *   **TR:** Standart açılış melodisini çalar.
    *   `void buzzertest()`
        *   **EN:** Tests the buzzer.
        *   **TR:** Buzzer'ı test eder.
*   **Traffic Light / Trafik Işığı**:
    *   `void moduleTraficLightWrite(bool red, bool yellow, bool green)`
        *   **EN:** Controls Red, Yellow, and Green lights on the traffic light module.
        *   **TR:** Trafik ışığı modülündeki Kırmızı, Sarı ve Yeşil ışıkları kontrol eder.
    *   `void moduleTraficLightWriteRed(bool red)`
        *   **EN:** Controls only the red light.
        *   **TR:** Sadece kırmızı ışığı kontrol eder.
    *   `void moduleTraficLightWriteYellow(bool yellow)`
        *   **EN:** Controls only the yellow light.
        *   **TR:** Sadece sarı ışığı kontrol eder.
    *   `void moduleTraficLightWriteGreen(bool green)`
        *   **EN:** Controls only the green light.
        *   **TR:** Sadece yeşil ışığı kontrol eder.
*   **Smart LED (NeoPixel) / Akıllı LED**:
    *   `void moduleSmartLEDPrepare(int pin)`
        *   **EN:** Initializes a standard 3-LED module.
        *   **TR:** 3 LED'li standart modülü başlatır.
    *   `void extendSmartLEDPrepare(int pin, int numLEDs)`
        *   **EN:** Initializes a strip with the specified number of LEDs.
        *   **TR:** Belirtilen sayıda LED içeren şeridi başlatır.
    *   `void moduleSmartLEDWrite(int led, int red, int green, int blue)`
        *   **EN:** Sets the color of the specified LED.
        *   **TR:** Belirtilen sıradaki LED'in rengini ayarlar.
    *   `void extendSmartLEDFill(int startLED, int endLED, int red, int green, int blue)`
        *   **EN:** Fills a range of LEDs with the same color.
        *   **TR:** Belirli bir aralıktaki LED'leri aynı renge boyar.
    *   `void moduleSmartLEDRainbowEffect(int wait)`
        *   **EN:** Creates a rainbow effect.
        *   **TR:** Gökkuşağı efekti yapar.
    *   `void moduleSmartLEDRainbowTheaterChaseEffect(int wait)`
        *   **EN:** Creates a rainbow chase effect.
        *   **TR:** Gökkuşağı takip efekti yapar.
    *   `void moduleSmartLEDTheaterChaseEffect(uint32_t color, int wait)`
        *   **EN:** Creates a single-color chase effect.
        *   **TR:** Tek renk takip efekti yapar.
    *   `void moduleSmartLEDColorWipeEffect(uint32_t color, int wait)`
        *   **EN:** Creates a color wipe effect.
        *   **TR:** Renk silme efekti yapar.
    *   `uint32_t getColor(int red, int green, int blue)`
        *   **EN:** Generates a color code from RGB values.
        *   **TR:** RGB değerlerinden renk kodu üretir.

### Advanced Sensors / Gelişmiş Sensörler
*   **DHT (Temperature/Humidity) / DHT (Sıcaklık/Nem)**:
    *   `int moduleDhtTempReadC(int pin)`
        *   **EN:** Reads temperature in Celsius (°C).
        *   **TR:** Sıcaklığı Santigrat (°C) cinsinden okur.
    *   `int moduleDhtTempReadF(int pin)`
        *   **EN:** Reads temperature in Fahrenheit (°F).
        *   **TR:** Sıcaklığı Fahrenheit (°F) cinsinden okur.
    *   `int moduleDhtHumRead(int pin)`
        *   **EN:** Reads humidity percentage (%).
        *   **TR:** Nem oranını (%) okur.
    *   `int moduleDthFeelingTempC(int pin)`
        *   **EN:** Calculates the Heat Index (Feels like temperature) in °C.
        *   **TR:** Hissedilen sıcaklığı (Isı İndeksi - °C) hesaplar.
    *   `int moduleDthFeelingTempF(int pin)`
        *   **EN:** Calculates the Heat Index (Feels like temperature) in °F.
        *   **TR:** Hissedilen sıcaklığı (Isı İndeksi - °F) hesaplar.
*   **Ultrasonic (Distance) / Ultrasonik (Mesafe)**:
    *   `int moduleUltrasonicDistanceRead()`
        *   **EN:** Measures distance in centimeters (cm) (Pins are defined in the library).
        *   **TR:** Mesafeyi santimetre (cm) cinsinden ölçer (Pinler kütüphanede tanımlıdır).
*   **RFID (Card Reader) / RFID (Kart Okuyucu)**:
    *   `int moduleRFIDRead()`
        *   **EN:** Returns the ID number of the read RFID card.
        *   **TR:** Okunan RFID kartının kimlik numarasını (ID) döndürür.
*   **IR Receiver (Remote) / IR Alıcı (Kumanda)**:
    *   `String moduleIRReadHex(int pin)`
        *   **EN:** Reads the signal from the IR remote as a Hexadecimal String.
        *   **TR:** Kızılötesi kumandadan gelen sinyali Hex (Onaltılık) formatında String olarak okur.
    *   `int moduleIRReadDecimalx32(int pin)`
        *   **EN:** Reads the signal as a 32-bit decimal number.
        *   **TR:** Sinyali 32-bit ondalık sayı olarak okur.
    *   `int moduleIRReadDecimalx8(int pin)`
        *   **EN:** Reads the last 8 bits of the signal as a decimal number.
        *   **TR:** Sinyalin son 8 bitini ondalık sayı olarak okur.

### General Pin Control & EEPROM / Genel Pin Kontrolü ve EEPROM
*   `int analogReadPin(int pin)`
    *   **EN:** Performs analog reading from the specified pin.
    *   **TR:** Belirtilen pinden analog okuma yapar.
*   `void analogWritePin(int pin, int value)`
    *   **EN:** Performs analog (PWM) writing to the specified pin.
    *   **TR:** Belirtilen pine analog (PWM) yazma yapar.
*   `bool digitalReadPin(int pin)`
    *   **EN:** Performs digital reading from the specified pin.
    *   **TR:** Belirtilen pinden dijital okuma yapar.
*   `void digitalWritePin(int pin, bool value)`
    *   **EN:** Performs digital writing to the specified pin.
    *   **TR:** Belirtilen pine dijital yazma yapar.
*   `void eepromWriteInt(int address, int value)`
    *   **EN:** Writes a legacy 16-bit (2-byte) integer to EEPROM.
    *   **TR:** EEPROM'a eski tip 16-bit (2 bayt) tam sayı yazar.
*   `int eepromReadInt(int address)`
    *   **EN:** Reads a legacy 16-bit (2-byte) integer from EEPROM.
    *   **TR:** EEPROM'dan eski tip 16-bit (2 bayt) tam sayı okur.
*   `bool eepromBegin(size_t size = 1024)`
    *   **EN:** Initializes EEPROM emulation.
    *   **TR:** EEPROM emülasyonunu başlatır.
*   `bool eepromCommit()` / `void eepromEnd()`
    *   **EN:** Commits pending changes / ends EEPROM usage.
    *   **TR:** Bekleyen değişiklikleri yazar / EEPROM kullanımını bitirir.
*   `bool eepromWriteByte(int address, uint8_t value)` / `uint8_t eepromReadByte(int address, uint8_t defaultValue = 0)`
    *   **EN:** Single byte read/write.
    *   **TR:** Tek bayt okuma/yazma.
*   `bool eepromWriteInt32(int address, int32_t value)` / `int32_t eepromReadInt32(int address, int32_t defaultValue = 0)`
    *   **EN:** 32-bit integer read/write.
    *   **TR:** 32-bit tam sayı okuma/yazma.
*   `bool eepromWriteUInt32(int address, uint32_t value)` / `uint32_t eepromReadUInt32(int address, uint32_t defaultValue = 0)`
    *   **EN:** 32-bit unsigned integer read/write.
    *   **TR:** 32-bit işaretsiz tam sayı okuma/yazma.
*   `bool eepromWriteFloat(int address, float value)` / `float eepromReadFloat(int address, float defaultValue = 0.0f)`
    *   **EN:** Float read/write.
    *   **TR:** Float okuma/yazma.
*   `bool eepromWriteString(int address, const String &value, uint16_t maxLen = 128)` / `String eepromReadString(int address, uint16_t maxLen = 128)`
    *   **EN:** Stores string as `[uint16 length][bytes...]`.
    *   **TR:** String'i `[uint16 uzunluk][baytlar...]` formatında saklar.
*   `bool eepromWriteBytes(int address, const uint8_t *data, size_t len)` / `bool eepromReadBytes(int address, uint8_t *data, size_t len)`
    *   **EN:** Raw bytes read/write.
    *   **TR:** Ham bayt okuma/yazma.
*   `bool eepromClear(int startAddress = 0, size_t length = 0, uint8_t fill = 0xFF)`
    *   **EN:** Fill a region (or whole EEPROM when length=0).
    *   **TR:** Bir bölgeyi (veya length=0 ise tüm EEPROM'u) doldurur.
*   `uint32_t eepromCrc32(const uint8_t *data, size_t len, uint32_t seed = 0xFFFFFFFF)`
    *   **EN:** CRC32 for raw bytes.
    *   **TR:** Ham bayt verisi için CRC32.
*   `bool eepromWriteRecord(int address, const uint8_t *data, uint16_t len, uint16_t version = 1)`
    *   **EN:** CRC-protected record write.
    *   **TR:** CRC korumalı record yazma.
*   `bool eepromReadRecord(int address, uint8_t *out, uint16_t maxLen, uint16_t *outLen = nullptr, uint16_t *outVersion = nullptr)`
    *   **EN:** CRC-protected record read (validates magic/len/crc).
    *   **TR:** CRC korumalı record okuma (magic/len/crc kontrolü).

### Communication / İletişim
*   **WiFi**:
    *   `void wifiStartAndConnect(const char *ssid, const char *pass)`
        *   **EN:** Connects to a WiFi network.
        *   **TR:** WiFi ağına bağlanır.
    *   `bool wifiConnectionControl()`
        *   **EN:** Checks the connection status.
        *   **TR:** Bağlantı durumunu kontrol eder.
    *   `String wifiGetIPAddress()`
        *   **EN:** Returns the device's local IP address.
        *   **TR:** Cihazın yerel IP adresini döndürür.
    *   `String wifiGetMACAddress()`
        *   **EN:** Returns the device's MAC address.
        *   **TR:** Cihazın MAC adresini döndürür.
*   **NTP Time / Saat Senkron**:
    *   `bool ntpBegin(int timezoneHours = 0, const char *ntpServer = "pool.ntp.org", int daylightOffsetHours = 0, uint32_t timeoutMs = 10000)`
        *   **EN:** Recommended one-call setup for blocks (timezone in hours). Call after WiFi connection.
        *   **TR:** Bloklar için önerilen tek çağrıda kurulum (saat cinsinden zaman dilimi). WiFi bağlantısından sonra çağırın.
    *   `bool ntpSync(const char *ntpServer = "pool.ntp.org", long gmtOffsetSec = 0, int daylightOffsetSec = 0, uint32_t timeoutMs = 10000)`
        *   **EN:** Advanced variant (offsets in seconds).
        *   **TR:** Gelişmiş kullanım (offset değerleri saniye cinsinden).
    *   `bool ntpIsTimeValid(time_t minEpoch = 1609459200)`
        *   **EN:** Returns true if time is valid.
        *   **TR:** Saat geçerli ise true döndürür.
    *   `time_t ntpGetEpoch()` / `String ntpGetDateTimeString()`
        *   **EN:** Returns epoch / formatted datetime string.
        *   **TR:** Epoch / formatlı tarih-saat string'i döndürür.
*   **ESP-NOW**:
    *   `void initESPNow()`
        *   **EN:** Initializes the ESP-NOW protocol.
        *   **TR:** ESP-NOW protokolünü başlatır.
    *   `void setWiFiChannel(int channel)`
        *   **EN:** Sets the WiFi channel.
        *   **TR:** WiFi kanalını ayarlar.
    *   `void sendESPNow(const uint8_t *macAddr, const uint8_t *data, int len)`
        *   **EN:** Sends data to the specified MAC address.
        *   **TR:** Belirtilen MAC adresine veri gönderir.
    *   `bool addBroadcastPeer(int channel)`
        *   **EN:** Adds the broadcast address (FF:FF:FF:FF:FF:FF) as a peer.
        *   **TR:** Yayın adresini (FF:FF:FF:FF:FF:FF) eş (peer) olarak ekler.
    *   `void registerOnRecv(esp_now_recv_cb_t cb)`
        *   **EN:** Registers the function to run when data is received.
        *   **TR:** Veri alındığında çalışacak fonksiyonu kaydeder.
    *   `void startListening()`
        *   **EN:** Starts automatic data listening (fills the receivedData structure).
        *   **TR:** Otomatik veri dinlemeyi başlatır (receivedData yapısını doldurur).
*   **Bluetooth (ESP32)**:
    *   `void bluetoothStart(String name, String pin)`
        *   **EN:** Starts Bluetooth Serial connection (PIN is optional).
        *   **TR:** Bluetooth Seri bağlantısını başlatır (PIN opsiyonel).
    *   `bool bluetoothConnect(String remoteName)`
        *   **EN:** Connects to a remote Bluetooth device.
        *   **TR:** Uzak bir Bluetooth cihazına bağlanır.
    *   `void bluetoothWrite(String message)`
        *   **EN:** Sends text via Bluetooth.
        *   **TR:** Bluetooth üzerinden metin gönderir.
    *   `String bluetoothRead()`
        *   **EN:** Reads data received via Bluetooth.
        *   **TR:** Bluetooth üzerinden gelen veriyi okur.
    *   `BluetoothSerial* getBluetoothObject()`
        *   **EN:** Provides access to the raw BluetoothSerial object.
        *   **TR:** Ham BluetoothSerial nesnesine erişim sağlar.
*   **Firebase**:
    *   `void fbServerSetandStartWithUser(...)`
        *   **EN:** Connects to Firebase Realtime Database.
        *   **TR:** Firebase Gerçek Zamanlı Veritabanına bağlanır.
    *   `void fbServerSetInt/Float/String/Double/Bool/JSON(...)`
        *   **EN:** Writes data to Firebase.
        *   **TR:** Firebase'e veri yazar.
    *   `int fbServerGetInt(...)`
        *   **EN:** Reads an integer from Firebase.
        *   **TR:** Firebase'den tamsayı okur.
    *   `float fbServerGetFloat(...)`
        *   **EN:** Reads a float from Firebase.
        *   **TR:** Firebase'den ondalıklı sayı okur.
    *   `String fbServerGetString(...)`
        *   **EN:** Reads text from Firebase.
        *   **TR:** Firebase'den metin okur.
    *   `double fbServerGetDouble(...)`
        *   **EN:** Reads a double from Firebase.
        *   **TR:** Firebase'den double okur.
    *   `bool fbServerGetBool(...)`
        *   **EN:** Reads a boolean value from Firebase.
        *   **TR:** Firebase'den mantıksal değer okur.
    *   `String fbServerGetJSON(...)`
        *   **EN:** Reads JSON from Firebase.
        *   **TR:** Firebase'den JSON okur.
*   **Telegram**:
    *   `void sendTelegram(String token, String chatId, String message)`
        *   **EN:** Sends a message via a Telegram bot.
        *   **TR:** Telegram botu üzerinden mesaj gönderir.
*   **IFTTT**:
    *   `bool triggerIFTTTEvent(const String &eventName, const String &webhookKey, const String &jsonPayload = "{}")`
        *   **EN:** Triggers an IFTTT Webhook event with an optional JSON payload. Returns `true` when the webhook responds with HTTP 200.
        *   **TR:** Opsiyonel JSON gövdesiyle IFTTT Webhook olayını tetikler. Webhook HTTP 200 döndüğünde `true` verir.
*   **Email / E-posta**:
    *   `void sendEmail(...)`
        *   **EN:** Sends an email via SMTP protocol.
        *   **TR:** SMTP protokolü üzerinden e-posta gönderir.
*   **Web Server / Web Sunucusu**:
    *   `void serverStart(const char *mode, const char *ssid, const char *password)`
        *   **EN:** Starts the web server (STA or AP mode).
        *   **TR:** Web sunucusunu başlatır (STA veya AP modu).
    *   `void serverCreateLocalPage(...)`
        *   **EN:** Creates a local web page.
        *   **TR:** Yerel bir web sayfası oluşturur.
    *   `void serverHandleDNS()`
        *   **EN:** Handles DNS requests.
        *   **TR:** DNS isteklerini işler.
    *   `void serverContinue()`
        *   **EN:** Continues the server loop.
        *   **TR:** Sunucu döngüsünü sürdürür.
*   **Internet Services / İnternet Servisleri**:
    *   `String getWeather(String city, String apiKey)`
        *   **EN:** Fetches weather information (OpenWeatherMap or wttr.in).
        *   **TR:** Hava durumu bilgisini çeker (OpenWeatherMap veya wttr.in).
    *   `String getWikipedia(String query, String lang)`
        *   **EN:** Fetches summary information from Wikipedia.
        *   **TR:** Wikipedia'dan özet bilgi çeker.

### Multi-Tasking (ESP32 Only) / Çoklu Görev (Sadece ESP32)
*   `void createTask(TaskFunction_t taskFunction, const char *name, int coreID, int stackSize, int priority)`
    *   **EN:** Creates a standard FreeRTOS task.
    *   **TR:** Standart bir FreeRTOS görevi oluşturur.
*   `void createLoopTask(void (*taskFunction)(), const char *name, int coreID, int priority, int stackSize)`
    *   **EN:** Creates a task that automatically loops the provided function. Ideal for block-based coding (Scratch, etc.).
    *   **TR:** Verilen fonksiyonu otomatik olarak sonsuz döngüye sokan bir görev oluşturur. Blok tabanlı kodlama (Scratch vb.) için idealdir.
*   `void taskDelay(int ms)`
    *   **EN:** Non-blocking delay function to be used within tasks (Wrapper for `vTaskDelay`).
    *   **TR:** Görev içinde kullanılması gereken, bloklamayan bekleme fonksiyonudur (`vTaskDelay` sarmalayıcısı).
