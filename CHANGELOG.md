# Changelog

# CODLAI ERA (New Models)

## [Unreleased]

## [1.7.1] - 2026-09-27
### Changed
- ESP-NOW alicisi (`startListening()`) artik SADECE tam olarak `sizeof(CodlaiESPNowMessage)` boyutunda paketleri degil, ondan KUCUK (eski kutuphane surumleriyle gonderilmis) paketleri de kabul ediyor: yapi once sifirlaniyor, sonra sadece gercekten gelen kadar byte kopyalaniyor (eksik alanlar - ornegin text/value - 0/bos kalir). Boylece ESKI surumle derlenmis bir gonderici, YENI surumle derlenmis bir aliciyla hala konusabilir (tersi degil - eski aliciler hala yeni/daha buyuk paketleri reddeder). Editor ajaninin gecis-donemi uyumluluk onerisi uzerine eklendi.

## [1.7.0] - 2026-09-27
### Added
- **Basit ESP-NOW mesajlasma** (cocuklar/blok kod icin): `espNowBegin(channel=1)`, `espNowSendText(text)`, `espNowSendNumber(name, value)`, `espNowAvailable()`, `espNowReadText()`, `espNowReadName()`, `espNowReadNumber()`. `CodlaiESPNowMessage` yapisina `char text[32]` ve `float value` alanlari eklendi (Kol/Arac kontrolunu bozmadan) - ayni surumdeki tum CODLAI kartlari arasinda uyumlu.
- **Melodi**: `buzzerPlayNote(note, durationMs)` (nota adiyla calma, ornegin "C4"/"D#5"), `buzzerPlayMelody(melodyId)` (1=Dogum Gunu, 2=Twinkle Twinkle, 3=Jingle Bells, 4=Baslangic Melodisi, 5=Daha Dun Annemizin [DOGRULANMAMIS, basitlestirilmis yer tutucu]), `buzzerSetTempo(bpm)`.
- **NeoPixel**: `moduleSmartLEDFill(r,g,b)`, `moduleSmartLEDClear()`, `moduleSmartLEDSetBrightness(0-255)`, `moduleSmartLEDBlink(r,g,b,times,ms)`, `moduleSmartLEDBreathe(r,g,b,ms)`.
- Yeni ornekler: `IOTBOT_ESPNOW_Simple_Messaging_Example.ino`, `IOTBOT_Buzzer_Melody_Example.ino`, `IOTBOT_NeoPixel_Effects_Example.ino`.

## [1.6.1] - 2026-09-27
### Fixed
- `moduleDCMotorGOClockWise(speed)` / `moduleDCMotorGOCounterClockWise(speed)`: fonksiyonun kendi belgelemesi `speed`'in 0-255 araliginda oldugunu soylerken, govde icinde yanlislikla `map(speed, 0, 100, 0, 255)` ile tekrar 0-255'e olceklendiriliyordu. Bu yuzden ornegin 200 gonderildiginde PWM degeri 510'a tasip motor pratikte her zaman tam hizda calisiyordu. `map()` kaldirildi, artik dogrudan `constrain(speed, 0, 255)` kullaniliyor - fonksiyonun belgelenen sozlesmesiyle artik tutarli. (Editor ajaninin derleme/blok entegrasyonu testinde bulundu.)

## [1.6.0] - 2026-09-27
### Added
- Yeni "3-Projects" ornek klasoru: kablosuz (ESP-NOW/WiFi) haberlesme gerektirmeyen, tek basina calisan, gercek hayattan basit proje ornekleri.
- `IOTBOT_Parking_Assistant_Example.ino` - ultrasonik mesafe sensoru + buzzer (mesafeye gore hizlanan bip) + role (cok yaklasinca tetiklenir), araba park sensoru mantigi.
- `IOTBOT_RFID_Access_Control_Example.ino` - RFID kart beyaz listesi + role (kapi kilidi) + buzzer + LCD, basit erisim kontrol sistemi.
- `IOTBOT_PIR_Security_Alarm_Example.ino` - PIR hareket sensoru + role (siren/isik) + surekli buzzer alarmi + B3 butonuyla susturma.
- `IOTBOT_Plant_Watering_Reminder_Example.ino` - toprak nemi sensoru, kalibrasyonlu (baslangic) baz degere gore kuruma tespiti + periyodik hatirlatma sesi.
- `IOTBOT_Smoke_Gas_Alarm_Example.ino` - duman/gaz sensoru, temiz hava baz degerine gore esik asimi tespiti + role + buzzer alarmi (egitim amacli, gercek yangin alarmi yerine kullanilmamalidir).

- Yeni ornek: `IOTBOT_ESPNOW_Temperature_Broadcast_Example.ino` - DHT sicaklik degerini surekli yayinlar; MINIBOT/ROLEBOT_ESPNOW_Fan_Control_Reactive_Example.ino ile eslestirilip "kablosuz otomatik vantilator" senaryosunu ogretir.

### Fixed
- `initESPNow()` icinde kosulsuz `WiFi.mode(WIFI_STA)` cagrisi, ayni sketch'te onceden acilmis bir AP'yi (ornegin bir web sunucusu/OTA icin `softAP()`) sessizce dusuruyordu. Artik mevcut mod AP ya da AP_STA ise `WIFI_AP_STA`'ya geciliyor, AP kapatilmiyor. (Editor ajaninin canli-mod WiFi AP + OTA + ESP-NOW birlikte kullanma senaryosu icin bulundu.)

## [1.5.0] - 2026-09-26
### Added
- `serverOnRequest(url, callback)`: `serverCreateLocalPage` SADECE sabit/statik bir HTML sayfasi render eder; bu yeni fonksiyon, bir adrese (ornegin `/led-on`) istek geldiginde GERCEKTEN kod calistirmaniza (bir GPIO/role/LED'i tetiklemenize) izin verir. Web tabanli, gercekten interaktif kontrol panelleri icin gerekliydi.
- Yeni ornek: `IOTBOT_WiFi_Web_Control_Example.ino` - telefon/tarayicidan LED ve role kontrolu (AP modu, `serverOnRequest` kullanir).
- Yeni ornek: `IOTBOT_Bluetooth_TR_EN_Control_Example.ino` - Bluetooth terminal uygulamasindan tek harfli komutlarla LED/role kontrolu, iki dilli.
- Yeni ornek: `IOTBOT_MiniBot_ESPNOW_Pair_Example.ino` - router/WiFi agi olmadan (ESP-NOW ile) bir MINIBOT ile dogrudan, iki yonlu haberlesme; gercek donanimda (iki kart, canli MAC adresleriyle) dogrulandi.
- Yeni baslangic seviyesi ornekler: `IOTBOT_WiFi_Simple_Status_Example.ino` (MAC/sunucu gerekmeyen en basit WiFi baglanma ornegi), `IOTBOT_ESPNOW_Broadcast_Simple_Example.ino` (MAC adresi bilmeden yayin/broadcast ile herhangi bir CODLAI kartina konusma), `IOTBOT_Bluetooth_Simple_Echo_Example.ino` (en basit Bluetooth yanki ornegi) - egitim mufredati icin "once bunu dene" niteliginde.
- Yeni ornek: `IOTBOT_ESPNOW_LightSensor_Broadcast_Example.ino` - LDR degerini surekli yayinlar; MINIBOT/ROLEBOT_ESPNOW_NightLight_Reactive_Example.ino ile eslestirilip "kablosuz otomatik gece lambasi" senaryosunu ogretir.
- Yeni ornek: `IOTBOT_MiniBot_SmartLED_Remote_Example.ino` - bir MINIBOT'un butonuyla akilli LED (NeoPixel) efektini uzaktan degistirir (bkz. MINIBOT_IoTBot_SmartLED_Remote_Example.ino).

### Fixed
- Acilistan sonraki ilk `tone()` cagrisinda (`playIntro`, `buzzerPlayTone` vb.) seri monitore dusen `E ledc: ledc_get_duty(745): LEDC is not initialized` hatasi giderildi: Arduino-ESP32 2.0.x `tone()` LEDC kanal 0'i `ledcSetup` yapmadan bagliyor; `begin()` artik bu kanal grubunu onceden kuruyor. Islevsel bir etkisi yoktu, sadece hata satiri basiliyordu.
- **v1.4.0 regresyonu**: `USE_ESPNOW` (ve tek basina USE_SERVER/USE_FIREBASE/USE_OTA/USE_EMAIL/USE_TELEGRAM/USE_WEATHER/USE_WIKIPEDIA/USE_IFTTT harici herhangi bir bayrak) tanimlandiginda `WiFi.h` hic include edilmiyordu - cunku bu bayraklarin `USE_WIFI`'yi otomatik tanimladigi blok, `#include <WiFi.h>` satirindan SONRA geliyordu. v1.4.0'dan once bu, kosulsuz (artik kaldirilmis) bir `WiFi.h` include'u tarafindan maskeleniyordu. `WiFi.h` include'u artik butun bu bayraklardan SONRA, en sona alindi. (Gercek donanimda IoTBot<->MiniBot ESP-NOW testi sirasinda kesfedildi.)

## [1.4.0] - 2026-09-26
### Changed
- **Flash boyutu ciddi olcude kucultuldu** (bir egitim uygulamasinda 888KB -> 434KB, ~%51): `Stepper.h` artik sadece `USE_STEP_MOTOR` tanimliyken, `LittleFS.h` artik sadece `USE_FIREBASE` tanimliyken include ediliyor; onceden kosulsuz include edilen ikinci (fazladan) `WiFi.h` satiri kaldirildi (asagidaki `USE_WIFI` korumali kopyasi zaten yeterli). `moduleStepMotorMotion()` de artik `USE_STEP_MOTOR` tanimlanmadan kullanilamiyor - step motor kullanan sketch'ler `#include <IOTBOT.h>` satirindan ONCE `#define USE_STEP_MOTOR` eklemeli.
- `begin()` icindeki `WiFi.mode(WIFI_OFF)` cagrisi kaldirildi: Arduino-ESP32'de WiFi radyosu hicbir WiFi/ESPNOW/Bluetooth API'si cagrilmadan zaten baslatilmiyor, bu cagri sadece WiFi kutuphanesini WiFi kullanmayan sketch'lere de linkleyip boyutu sisiriyordu. **Donanim notu**: B1/B2 butonu ve joystick X ekseni ADC2 uzerinden okunuyor; bu degisiklik sonrasi donanimda dogrulanmali, sorun gorulursse blok geri eklenebilir.

## [1.3.1] - 2026-09-25
### Removed
- Kullanilmayan, yanlislikla "kart uzeri LED" sanilabilecek `LED_BUILTIN 1` tanimi kaldirildi (GPIO1 = ESP32'de varsayilan UART0 TXD hatti; hicbir yerde kullanilmiyordu). IOTBOT'ta modullerden bagimsiz sabit bir LED yok - gorunur LED'ler P1-P5 sinyal hatlarina paralel baglidir, bkz. `digitalWritePin`.

## [1.3.0] - 2026-09-25
### Added
- NTP time helpers: `ntpSync`, `ntpIsTimeValid`, `ntpGetEpoch`, `ntpGetDateTimeString`.
- CRC-protected EEPROM record helpers: `eepromCrc32`, `eepromWriteRecord`, `eepromReadRecord`.
- New advanced example: `IOTBOT_NTP_Time_Advanced_Example.ino` (TR/EN).
- New advanced example: `IOTBOT_OTA_WiFi_Remote_Info_Example.ino` (TR/EN).
- `lcdWriteCornerArrow`: bir hucreye capraz (↘) ok karakteri yazar; P6 gibi kartin bir kosesindeki soketin yerini gostermek icin.
- `lcdScrollText`: 20 kolonu asan bir metni kaydirarak gosterir.
- `moduleServoDetach`: servoya ayrilan LEDC kanalini/pini serbest birakir; ayni pin baska bir modulde (DC motor, step motor, akilli LED) kullanilmadan once cagrilmalidir.
- Yeni gelistirilmis ornek: `IOTBOT_Musteri_Karsilama_V2_Sensor_Kesif.cpp` - P1-P6 soketlerini, tum sensor/aktuator modullerini ve donanim turunu adim adim ogreten interaktif egitim akisi (encoder ile ileri, joystick butonu ile bir onceki adima geri).

### Fixed
- **Encoder okuma**: A/B pinlerinin Gray-kod gecisleri artik dogru sekilde takip ediliyor; onceki surum sadece tek bir pinin kenarina bakiyordu ve bazi donuslerde adim kacirabiliyor ya da ters sayabiliyordu. A/B pinleri de `INPUT_PULLUP` yapildi.
- **DC motor sola donme**: `moduleDCMotorGOCounterClockWise` fonksiyonundaki PWM/yon pin ataması duzeltildi - onceki surumde bu yonde motor hic donmuyordu (IO27 hep PWM aliyor, IO26 sadece HIGH/LOW oluyordu; simdi yon her zaman "diger" pin LOW tutularak PWM'in verildigi pinle belirleniyor).
- **B1/B2 butonu ve joystick X ekseni (ADC2) guvenilirligi**: WiFi/Bluetooth kullanmayan sketch'lerde `begin()` artik WiFi radyosunu kapatiyor; ESP32'nin bilinen ADC2 kisitlamasi (radyo acikken guvenilmez okuma) boylece ortadan kalkiyor.
- **Matris buton okumasi**: tek anlik ADC okumasi yerine 5 orneğin ortalamasi aliniyor; 4 ve 5 numarali tuslar arasindaki dar/asimetrik esik bandi genisletildi - komsu tuslarin birbirine karismasi onemli olcude azaldi.
- **Servo hareket araligi**: puls genisligi 1000-2000us'den (ARMBOT/CARBOT haric) 500-2500us'e genisletildi - bazi servolar kenar acilarda (0/180) net hareket uretmiyordu. Ayrica sinyal pini artik `GPIO_DRIVE_CAP_3` ile maksimum surus gucune ayarlaniyor (P1-P6 ortak hattindaki koruma diyotunun gerilim dususunu telafi eder).
- **Akilli LED bellek sizintisi**: `moduleSmartLEDPrepare` art arda cagrildiginda onceki `Adafruit_NeoPixel` nesnesi artik serbest birakiliyor.
- **LCD uzun metin tasmasi**: `lcdWriteMid` artik 20 kolonu asan satirlari kesiyor; HD44780'in DDRAM adresleme sarmasindan kaynakli ekran kalintisi onlendi.

## [1.2.5] - 2026-02-04
### Added
- OTA helpers: `otaBegin`, `otaHandle` (requires `USE_OTA`).
- New advanced example: `IOTBOT_OTA_Update_Example.ino` (TR/EN).

## [1.2.4] - 2025-12-20
### Added
- Extended EEPROM helpers: `eepromBegin/Commit/End`, byte/int32/uint32/float/string/bytes read-write and region clear.
- New advanced example: `IOTBOT_EEPROM_Advanced_Example.ino` (TR/EN).

### Changed
- EEPROM int (legacy) helpers now lazy-initialize EEPROM to reduce common runtime issues.

## [1.2.1] - 2025-03-09
### Fixed
- PlatformIO yeniden yayını için sürüm numarası artırıldı.

## [1.2.3] - 2025-12-18
### Added
- New `IOTBOT_Multi_Task_Example.ino` demonstrates the simplified `createLoopTask` APIs for ESP32-based multitasking flows.
- Updated `IOTBOT_Wikipedia_Example.ino` and adjacent advanced samples with clearer, bilingual instructions and timing comments so the HTTP helpers stay reliable.

## [1.2.2] - 2025-03-09
### Fixed
- `USE_WIKIPEDIA` bloğundaki yorum satırı kapatılarak `getWikipedia()` betimlemesi tekrar etkinleştirildi.

## [1.2.0] - 2025-03-09
### Added
- `triggerIFTTTEvent` helper for HTTPS Maker Webhook calls.
- New advanced example: `IOTBOT_IFTTT_Webhook_Example.ino`.

### Updated
- Documentation, keywords, and metadata to highlight the IFTTT workflow.

## [1.0.0] - 2025-03-04
### Added
- **Rebranding**: Transitioned from CODROB to CODLAI.
- Standardized library structure.
- Added `serialStart` and `serialWrite` wrappers.
- Added `buzzerPlay` alias.
- Updated examples to use library wrappers.
- Initial Release for PlatformIO and Arduino IDE.

---

# CODROB ERA (Legacy Models)

## [1.7.0] - 2025-02-28
### Added
- Tüm modüller için config dosyası kaldırıldı. Ortak kütüpahaneler devrede. 

## [1.6.7] - 2025-02-28
### Added
- Tüm modüller için config dosyası eklendi. 

## [1.6.6] - 2025-02-27
### Fixed
- BUZZER TONE DEVRE DIŞI BIRAKILDI, ANALOGWRİTE İLE GÜNCELLENDİ. SERVO MOTOR ÇAKIŞMASI ENGELLENDİ

## [1.6.5] - 2025-02-21
### Added
- Config dosyası eklendi. 

## [1.5.4] - 2025-02-20
### Fixed
- Kütüphna versiyonu arduıno ve vscode için güncellendi. 

## [1.5.3] - 2025-02-19
### Added
- LCD setcursor eklendi. 
- Trafik iışıkları için tekli modül eklendi. 
- Arduino uyumluluğu için library.properties eklendi.
- esphome/ESPAsyncWebServer-esphome yerine mathieucarbou/ESPAsyncWebServer eklendi. 
- Keywords listesi güncellendi. 
- Gerekli uygulamalara define eklendi. uygulamaya gore kütüphane aktifleşecek hale getirildi.

### Fixed
- CPP ve H dosyası arduıno ile uyumlu hale getirildi. 

## [1.5.2] - 2025-02-13
### Fixed
- Firebase ve Wifi örnek uygulamalrındaki eksiklikler düzeltildi. 

## [1.4.5] - 2025-02-11
### Fixed
- Firebase ve Wifi örnek uygulamalrındaki eksiklikler düzeltildi.  

## [1.4.0] - 2025-02-08
### Fixed 
- Örnek uygulamalarda türkçe karakterler kaldırıldı. 

## [1.3.0] - 2025-02-04
### Fixed 
- Firebase fonksiyonları düzeltildi. 
- Örnek uygulamalarda lcdmid fonksiyonları düzeltildi. 

## [1.2.3] - 2025-02-03

### Fixed 
- Firebase fonksiyonları düzeltildi. 

## [1.2.2] - 2025-02-03
### Added
- Firebase Fonskiyonları eklendi. 

### Fixed 
- Açıklamalar düzeltildi. 

## [1.2.1] - 2025-01-31
### Added
- RFID Kütüphanesi eklendi 
- DHT için Fahreneght kodlaarı eklendi. 
- Wifi Kütüphaneleri ve fonskiyonları ekleni 
- Local server fonksiyonaları eklendi. 
- EEPROM fonksiyonları ekledi.

### Fixed
- Servo motor ayarları optimize edildi. 
- Açıklamalar düzeltildi. 

## [1.2.0] - 2025-01-30
### Added
- Eksik olan tüm kütüphaneler eklendi, örnek uygulamalar güncellendi. 

### Fixed
- Servo ve IR okuyucu modüllerindeki buglar düzeltildi. 

## [1.0.7] - 2025-01-25
### Added
- Test kodları oluşturuldu:
  - LCD ekran testi.
  - Tüm butonlar için ayrı ayrı testler.
  - Motorlar için testler:
    - Servo motor.
    - DC motor.
    - Step motor.
  - Modüller için testler:
    - Mikrofon sensörü.
    - PIR hareket sensörü.
    - Matris buton sensörü.
    - NTC sıcaklık sensörü.
    - Trafik ışığı modülü.
  - Röle ve duman sensörü için test kodları.
- `keywords.txt` dosyası eklendi:
  - Arduino IDE desteği için fonksiyonlar eklendi.
- Tüm açıklamalar Türkçe ve İngilizce olarak güncellendi.

### Fixed
- IR kütüphanesiyle çakışma sorunları giderildi.
- Seri port kullanımındaki karışıklıklar IoTBot sınıfı üzerinden düzenlendi.

---

## [1.0.6] - 2025-01-20
### Added
- `IRremote` desteği eklendi.
- Kod yapısı ESP32'ye uygun optimize edildi.

### Fixed
- LCD başlatma sırasında oluşan uyumsuzluklar düzeltildi.

---

## [1.0.5] - 2025-01-15
### Added
- IoTBot'a DHT11 sıcaklık ve nem sensörü desteği eklendi.
- IoTBot için NeoPixel LED desteği sağlandı.

---

## [1.0.4] - 2025-01-10
### Added
- Yeni buton kontrol işlevleri eklendi.
- Buzzer test kodları düzenlendi.

### Fixed
- Encoder kontrol fonksiyonlarında iyileştirmeler yapıldı.

---

## [1.0.3] - 2025-01-05
### Added
- Potansiyometre okuma işlevi eklendi.
- Röle kontrolü için destek sağlandı.

---

## [1.0.2] - 2025-01-01
### Fixed
- Motor kontrol algoritmalarında düzeltmeler yapıldı.
- Trafik ışığı modül desteği optimize edildi.

---

## [1.0.1] - 2024-12-25
### Added
- IoTBot sınıfı ve temel sensör işlevleri eklendi.

---

## [1.0.0] - 2024-12-20
### Added
- İlk sürüm yayımlandı.

## [1.1.0] - 2025-11-26
### Added
- **Multi-Tasking Support (ESP32 Only)**:
  - `createTask`: Simplified task creation.
  - `createLoopTask`: Automatically loops task functions, ideal for block-based coding.
  - `taskDelay`: Simplified non-blocking delay for tasks.
- Updated `keywords.txt` and documentation.

## [1.1.1] - 2025-11-26
### Fixed
- Fixed syntax error in `relaytest` function (missing return type).

