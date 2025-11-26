# Changelog

# CODLAI ERA (New Models)

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

