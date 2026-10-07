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
    *   **EN:** Writes fixed-width text to the specified position: shorter text is padded with spaces, longer text is CUT to `width` (and to the end of the 20-column row), so it never spills onto another row. Turkish letters are converted first.
    *   **TR:** Belirtilen genişlikte sabit metin yazar: kısa metin boşlukla doldurulur, uzun metin `width` kadar (ve 20 kolonluk satırın sonunda) KESİLİR, böylece başka satıra taşmaz. Türkçe harfler önce dönüştürülür.
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
    *   **EN:** Shows a loading animation along with the provided message (centered; text longer than 20 characters is cut to 20).
    *   **TR:** Ekranda bir yükleme animasyonu ile birlikte verilen mesajı gösterir (ortalanır; 20 karakterden uzun metin 20'de kesilir).
*   `void lcdShowStatus(String title, String status, bool isSuccess)`
    *   **EN:** Creates a status screen (e.g., "WiFi [OK]" or "Error [X]"). Title/status longer than 20 characters are cut to 20.
    *   **TR:** Bir durum ekranı oluşturur (Örn: "WiFi [OK]" veya "Hata [X]"). 20 karakterden uzun başlık/durum 20'de kesilir.
*   `void lcdtest()`
    *   **EN:** Tests the LCD screen.
    *   **TR:** LCD ekranını test eder.

### Inputs / Giriş Birimleri
*   **Joystick**:
    *   `int joystickXRead()`
        *   **EN:** Reads the X-axis value (0-4095).
        *   **TR:** X ekseni değerini okur (0-4095 arası).
        *   **Not / Note:** X ekseni GPIO15'tedir (ADC2); `USE_WIFI`/`USE_ESPNOW` açıkken ESP32 bu pini okuyamaz (genelde 0) - kablosuz projelerde joystick Y (GPIO34, ADC1) veya potansiyometre kullanın. / X is on GPIO15 (ADC2); the ESP32 cannot read it while `USE_WIFI`/`USE_ESPNOW` is on (usually 0) - use joystick Y (GPIO34, ADC1) or the potentiometer in wireless projects.
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
        *   **EN:** Reads temperature from the NTC sensor (Celsius). Returns `-999` if the sensor is unplugged or shorted (ADC 0 or 4095) - the same error value as the DHT functions.
        *   **TR:** NTC sensöründen sıcaklık okur (Celsius). Sensör takılı değilse ya da kısa devreyse (ADC 0 veya 4095) `-999` döndürür - DHT fonksiyonlarıyla aynı hata değeri.
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
        *   **EN:** Controls a stepper motor: `step` = steps per revolution (used for the speed), `rotation` = direction, `accelometer` = number of steps to move, `speed` = RPM. Blocks until the move ends. The coil phase is kept between calls, so a long move can be split into small chunks of any size (even 1 step) without jerking.
        *   **TR:** Step motoru kontrol eder: `step` = tur başına adım (hız için kullanılır), `rotation` = yön, `accelometer` = atılacak adım sayısı, `speed` = devir/dakika. Hareket bitene kadar bekler. Bobin fazı çağrılar arasında korunur; uzun bir hareket her boyutta (1 adım bile) küçük parçalara bölünebilir, motor geri sıçramaz.
        *   **Gerekli / Required:** `#include <IOTBOT.h>`'tan ÖNCE `#define USE_STEP_MOTOR` (1.4.0'dan beri; yoksa "moduleStepMotorMotion was not declared" derleme hatası). Sabit pinler IO26, IO33, IO32, IO27 - trafik ışığı, ultrasonik ve P2-P5 ile aynı anda kullanmayın. / `#define USE_STEP_MOTOR` BEFORE `#include <IOTBOT.h>` (since 1.4.0; otherwise a "moduleStepMotorMotion was not declared" compile error). Fixed pins IO26, IO33, IO32, IO27 - do not combine with the traffic light, ultrasonic or P2-P5.
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
        *   **EN:** Generates a color code from RGB values (0-255; also works before `moduleSmartLEDPrepare`).
        *   **TR:** RGB değerlerinden renk kodu üretir (0-255; `moduleSmartLEDPrepare`'den önce de çalışır).
    *   `void moduleSmartLEDSetBrightness(int brightness)`
        *   **EN:** Sets the brightness (0-255). Write the color again (Fill/Write) after changing it: the Adafruit brightness scaling is lossy and `0` erases the stored colors. All `moduleSmartLED*` functions safely do nothing before `moduleSmartLEDPrepare`.
        *   **TR:** Parlaklığı ayarlar (0-255). Değiştirdikten sonra rengi tekrar yazın (Fill/Write): Adafruit parlaklık ölçeklemesi kayıplıdır ve `0` saklanan renkleri siler. Tüm `moduleSmartLED*` fonksiyonları `moduleSmartLEDPrepare`'den önce çağrılırsa güvenle hiçbir şey yapmaz.
    *   `void moduleSmartLEDBreathe(int red, int green, int blue, int ms)`
        *   **EN:** "Breathing" effect: fades the color in and out in `ms` milliseconds (blocking), respecting the brightness set with `moduleSmartLEDSetBrightness`; afterwards the LEDs return to their previous state.
        *   **TR:** "Nefes alma" efekti: rengi `ms` milisaniyede yavaşça yakıp söndürür (bekletir), `moduleSmartLEDSetBrightness` ile ayarlanan parlaklığa uyar; bitince LED'ler önceki haline döner.

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
        *   **EN:** Measures distance in centimeters (cm) (Pins are defined in the library). Returns `0` when there is no valid reading: no echo (nothing within 400 cm / sensor unplugged) or farther than 400 cm.
        *   **TR:** Mesafeyi santimetre (cm) cinsinden ölçer (Pinler kütüphanede tanımlıdır). Geçerli ölçüm yoksa `0` döndürür: yankı yok (400 cm içinde cisim yok / sensör takılı değil) ya da 400 cm'den uzak.
*   **RFID (Card Reader) / RFID (Kart Okuyucu)**:
    *   `int moduleRFIDRead()`
        *   **EN:** Returns the ID number of the read RFID card (`0` = no new card). The ID is the first 4 UID bytes as a big-endian 32-bit number (may look negative as `int`; longer UIDs fold the rest in). NOTE: IDs from library versions before this change are DIFFERENT (the old decimal-string method overflowed and collided) - scan the cards again.
        *   **TR:** Okunan RFID kartının kimlik numarasını (ID) döndürür (`0` = yeni kart yok). ID, UID'nin ilk 4 baytının büyük-endian 32 bit sayısıdır (`int` olarak eksi görünebilir; uzun UID'lerde kalan baytlar da katılır). DİKKAT: Bu değişiklikten önceki kütüphane sürümlerinin verdiği ID'ler FARKLIDIR (eski ondalık metin yöntemi taşıyor ve çakışıyordu) - kartları yeniden okutun.
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
*   **EEPROM address map / EEPROM adres haritası** (kütüphanenin kendisi sabit adres kullanmaz / the library itself uses no fixed address):
    *   **EN:** Sketches generated by editor.codlai.com call `eepromBegin(1024)` and own: `0-255` memory-block numbers (64 x 4 bytes), `256-895` memory-block texts (10 x 64 bytes), `896-1019` editor reserve (ESP-NOW pairing record via `eepromWriteRecord(960, ...)`, 960-975), `1020-1023` editor marker `0xC0D1A001`. Hand-written projects mixed with editor blocks should stay above 1024 (call `eepromBegin` with a larger size).
    *   **TR:** editor.codlai.com'un ürettiği programlar `eepromBegin(1024)` çağırır ve şu alanları kullanır: `0-255` hafıza bloğu sayıları (64 x 4 bayt), `256-895` hafıza bloğu metinleri (10 x 64 bayt), `896-1019` editör rezervi (ESP-NOW eşleşme kaydı `eepromWriteRecord(960, ...)` ile 960-975), `1020-1023` editör işareti `0xC0D1A001`. Editör bloklarıyla birlikte kullanılan elle yazılmış kod 1024'ün üstünde kalmalı (`eepromBegin`'i daha büyük boyutla çağırın).
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
    *   **EN:** Reads a legacy 16-bit (2-byte) integer from EEPROM as a SIGNED value (-32768..32767), so negative numbers come back correctly. A never-written (0xFF) EEPROM reads `-1` (was 65535).
    *   **TR:** EEPROM'dan eski tip 16-bit (2 bayt) tam sayıyı İŞARETLİ (-32768..32767) okur; eksi sayılar doğru geri gelir. Hiç yazılmamış (0xFF) EEPROM `-1` okunur (eskiden 65535).
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
    *   **EN:** Stores string as `[uint16 length][bytes...]`. Reading a never-written (0xFF) area returns `""`.
    *   **TR:** String'i `[uint16 uzunluk][baytlar...]` formatında saklar. Hiç yazılmamış (0xFF) alan okunursa `""` döner.
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
        *   **EN:** Connects to a WiFi network (waits max ~15 s). The password is masked (`********`) in the serial output.
        *   **TR:** WiFi ağına bağlanır (en fazla ~15 sn bekler). Şifre seri port çıktısında gizlenir (`********`).
    *   `bool wifiConnectionControl()`
        *   **EN:** Checks the connection status. Prints a line to Serial only when the state changes (safe to call in `loop()`).
        *   **TR:** Bağlantı durumunu kontrol eder. Seri porta sadece durum değişince yazar (`loop()` içinde çağrılabilir).
    *   `String wifiGetIPAddress()`
        *   **EN:** Returns the device's local IP address.
        *   **TR:** Cihazın yerel IP adresini döndürür.
    *   `String wifiGetMACAddress()`
        *   **EN:** Returns the device's MAC address.
        *   **TR:** Cihazın MAC adresini döndürür.
*   **OTA (Over-The-Air)**:
    *   `void otaBegin(const char *hostname = "CODLAI-IOTBOT", const char *password = nullptr, uint16_t port = 3232)`
        *   **EN:** Starts OTA service (call after WiFi connection).
        *   **TR:** OTA servisini baslatir (WiFi baglantisindan sonra cagirin).
    *   `void otaHandle()`
        *   **EN:** Processes OTA updates. Call continuously in `loop()`.
        *   **TR:** OTA guncellemelerini isler. `loop()` icinde surekli cagirin.
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
    *   `bool ntpUpdate()`
        *   **EN:** Re-syncs the clock NOW with the last `ntpBegin`/`ntpSync` settings ("update internet time" block): it really asks the server again and waits for a fresh answer (max 10 s). Returns `true` only if that fresh sync succeeded (on `false` the old clock keeps running). The core also re-syncs by itself about every hour.
        *   **TR:** Saati son `ntpBegin`/`ntpSync` ayarlariyla HEMEN yeniden esitler ("Internet saatini guncelle" blogu): sunucuya gercekten yeniden sorar ve yeni cevabi bekler (en fazla 10 sn). Sadece bu yeni esitleme basariliysa `true` doner (`false` olsa da eski saat calismaya devam eder). Cekirdek ayrica yaklasik saatte bir kendiliginden esitler.
    *   `int ntpGetHour()` / `ntpGetMinute()` / `ntpGetSecond()` / `ntpGetDay()` / `ntpGetMonth()` / `ntpGetYear()` / `ntpGetWeekday()`
        *   **EN:** Parts of the local time; weekday 1=Monday ... 7=Sunday. Return -1 while the time is not valid.
        *   **TR:** Yerel saatin parcalari; haftanin gunu 1=Pazartesi ... 7=Pazar. Saat gecerli degilken -1 dondurur.
    *   `String ntpGetTimeString()` / `String ntpGetDateString()`
        *   **EN:** "14:05:09" / "29.09.2026" ("--:--:--" / "--.--.----" while not valid).
        *   **TR:** "14:05:09" / "29.09.2026" (gecerli degilken "--:--:--" / "--.--.----").
    *   `bool ntpTimeIs(int hour, int minute)`
        *   **EN:** True during that whole minute ("if the time is HH:MM" block).
        *   **TR:** O dakika boyunca true ("saat SS:DD ise" blogu).
    *   `bool ntpTimeReached(int hour, int minute)`
        *   **EN:** True only ONCE when that minute starts ("when the time is HH:MM" block) - e.g. an alarm that must not repeat for a whole minute. Up to 8 different times are tracked.
        *   **TR:** O dakikaya girildiginde SADECE BIR KEZ true ("saat SS:DD olunca" blogu) - ornegin bir dakika boyunca tekrar etmemesi gereken alarm. En fazla 8 farkli saat takip edilir.
    *   `bool ntpTimeIsBetween(int startHour, int startMinute, int endHour, int endMinute)`
        *   **EN:** True if start <= now < end; ranges crossing midnight (22:00-06:00) work.
        *   **TR:** baslangic <= simdi < bitis ise true; gece yarisini asan araliklar (22:00-06:00) de calisir.
*   **ESP-NOW**:
    *   **`deviceType` haritası / map** (`CodlaiESPNowMessage.deviceType`):
        *   **EN:** 1 = Armbot command, 2 = Carbot command, 3 = Carbot telemetry, 4 = Armbot signal, 10 = IOTBOT LDR broadcast, 11 = IOTBOT temperature broadcast, 20 = simple text message, 21 = simple number message, 22-29 = RESERVED for editor.codlai.com private/paired messaging, 30-39 = RESERVED for the CODLAI Robots autonomous project, 40-49 = library example board IDs (40 IOTBOT, 41 MINIBOT, 42 ROLEBOT) used by the Broadcast_Simple / Pair / SmartLED_Remote examples.
        *   **TR:** 1 = Armbot komutu, 2 = Carbot komutu, 3 = Carbot telemetrisi, 4 = Armbot sinyali, 10 = IOTBOT LDR yayını, 11 = IOTBOT sıcaklık yayını, 20 = basit metin mesajı, 21 = basit sayı mesajı, 22-29 = editor.codlai.com özel/eşleşmeli mesajlaşma için REZERVE, 30-39 = CODLAI Robotları Otonom projesi için REZERVE, 40-49 = kütüphane örneklerinin kart kimlikleri (40 IOTBOT, 41 MINIBOT, 42 ROLEBOT) - Broadcast_Simple / Pair / SmartLED_Remote örnekleri kullanır.
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
        *   **EN:** Reads data received via Bluetooth (`""` if nothing arrived). Waits at most ~40 ms after the last character (was ~1 s), so `loop()` stays responsive.
        *   **TR:** Bluetooth üzerinden gelen veriyi okur (gelen yoksa `""`). Son karakterden sonra en fazla ~40 ms bekler (eskiden ~1 sn), `loop()` takılmaz.
    *   `BluetoothSerial* getBluetoothObject()`
        *   **EN:** Provides access to the raw BluetoothSerial object.
        *   **TR:** Ham BluetoothSerial nesnesine erişim sağlar.
*   **Firebase**:
    *   `void fbServerSetandStartWithUser(const char *projectURL, const char *apiKey, const char *userMail, const char *mailPass)`
        *   **EN:** Connects to Firebase Realtime Database and signs in with an email/password user. The 2nd parameter is the project's **Web API Key** (Project settings > General) - NOT the Database Secret.
        *   **TR:** Firebase Gerçek Zamanlı Veritabanına bağlanır ve e-posta/şifre kullanıcısıyla giriş yapar. 2. parametre projenin **Web API Key**'idir (Proje ayarları > Genel) - Database Secret DEĞİL.
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
        *   **EN:** Sends a message via a Telegram bot. Pass plain text: the library URL-encodes it (UTF-8 `%XX`: Turkish letters, spaces, `&`, `#`, `+`, newlines). Do NOT encode it yourself (it would be encoded twice).
        *   **TR:** Telegram botu üzerinden mesaj gönderir. Düz metin verin: kütüphane metni URL için kodlar (UTF-8 `%XX`: Türkçe harf, boşluk, `&`, `#`, `+`, satır sonu). Kendiniz KODLAMAYIN (iki kez kodlanır).
    *   `static String urlEncode(const String &text)` (`USE_TELEGRAM` / `USE_WEATHER` / `USE_WIKIPEDIA` / `USE_IFTTT`)
        *   **EN:** UTF-8 percent-encoding helper (`A-Z a-z 0-9 - _ . ~` are kept). Only needed for your own URLs.
        *   **TR:** UTF-8 yüzde-kodlama yardımcısı (`A-Z a-z 0-9 - _ . ~` aynen kalır). Sadece kendi oluşturduğunuz adresler için gerekir.
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
        *   **EN:** Starts the web server (STA or AP mode). AP passwords must be at least 8 characters: a 1-7 character password is replaced by `12345678` (empty = open network). If "STA" cannot connect in ~30 s, a fallback AP named `CODLAI-IOTBOT` starts (password: the given one if it has 8+ characters, otherwise `12345678`); its name, password and address (`http://192.168.4.1`) are printed to Serial. A default "CODLAI Server is Running!" page is served at `/` until you register your own `/` page.
        *   **TR:** Web sunucusunu başlatır (STA veya AP modu). AP şifresi en az 8 karakter olmalıdır: 1-7 karakterlik şifre yerine `12345678` kullanılır (boş = şifresiz ağ). "STA" ~30 sn içinde bağlanamazsa `CODLAI-IOTBOT` adlı yedek bir AP açılır (şifre: verilen şifre 8+ karakterse o, değilse `12345678`); adı, şifresi ve adresi (`http://192.168.4.1`) seri porta yazılır. Kendi `/` sayfanızı tanımlayana kadar `/` adresinde varsayılan "CODLAI Server is Running!" sayfası gösterilir.
    *   `void serverCreateLocalPage(const char *url, ...)` / `void serverOnRequest(const char *url, std::function<String()> callback)`
        *   **EN:** Creates a local web page / runs your callback on a GET request. `url` may be written with or without the leading `/` (`"panel"` = `"/panel"`); `"/"` replaces the default home page.
        *   **TR:** Yerel bir web sayfası oluşturur / GET isteğinde fonksiyonunuzu çalıştırır. `url` başında `/` olsa da olmasa da olur (`"panel"` = `"/panel"`); `"/"` varsayılan ana sayfanın yerine geçer.
    *   `void serverHandleDNS()`
        *   **EN:** Handles DNS requests.
        *   **TR:** DNS isteklerini işler.
    *   `void serverContinue()`
        *   **EN:** Continues the server loop (handles the captive-portal DNS in AP and AP+STA modes). Call it in `loop()`.
        *   **TR:** Sunucu döngüsünü sürdürür (AP ve AP+STA modlarında DNS isteklerini işler). `loop()` içinde çağırın.
*   **Internet Services / İnternet Servisleri**:
    *   `String getWeather(String city, String apiKey)`
        *   **EN:** Fetches weather information (wttr.in when `apiKey` is empty/"YOUR_API_KEY", otherwise OpenWeatherMap over HTTPS). Pass the city as plain text (e.g. "New York", "Kahramanmaraş"); the library URL-encodes it.
        *   **TR:** Hava durumu bilgisini çeker (`apiKey` boş/"YOUR_API_KEY" ise wttr.in, değilse HTTPS üzerinden OpenWeatherMap). Şehri düz metin olarak verin (ör. "Kahramanmaraş"); kütüphane adrese uygun hale getirir.
    *   `String getWikipedia(String query, String lang)`
        *   **EN:** Fetches summary information from Wikipedia. Pass the topic as plain text (e.g. "Ada Lovelace"): spaces become `_` and the title is URL-encoded by the library - do not pre-encode it.
        *   **TR:** Wikipedia'dan özet bilgi çeker. Konuyu düz metin olarak verin (ör. "Mustafa Kemal Atatürk"): boşluklar `_` olur ve başlık kütüphanede kodlanır - kendiniz kodlamayın.

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
