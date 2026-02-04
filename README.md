# IoTBOT Library User Guide
This library is specially designed and produced by the CODLAI developer team to control the IoTBOT product.
![alt text](<images/1.png>)

## Using with Arduino IDE

### Installation

1. Open the Arduino IDE.
2. Go to "Sketch" -> "Include Library" -> "Manage Libraries..." from the menu bar.
3. Type "IoTBOT" in the search box.
4. Find the IoTBOT library and click the "Install" button to complete the installation.

## What is IoTBOT?
IoTBOT is an IoT-focused development board specifically designed for use in robotics coding courses. It comes with built-in basic level sensors required for IoT applications, eliminating the need for complex wiring and sensor connections.

### Processor
ESP32 - 240 MHz Tensilica Xtensa Dual-Core 32-bit LX6 microprocessor

### Battery
2000mAh Li-Po

### HMI
20X04 Programmable Characterized Blue LCD Screen

### Peripherals & I/O:
- 1 x Type-C Socket (Programming + Power)
- 5 x RJ45 Type Module Ports (Analog & Digital)
- 1 x RJ45 Type Motor Driver Port
- 1 x Programmable Relay Output
- 1 x Jumper Cable Type SPI, UART, I2C, 5V, 3V, GND Output
- 1 x RC522 RFID Communication Port

### Programming Languages
C++, MicroPython (Text and Block-Based)

### Wireless Connectivity
2.4 GHz WiFi + Bluetooth® + Bluetooth LE

### Built-in Sensors:
- 20X04 LCD Screen
- L293D Motor Driver
- 2-Axis Button Joystick
- Potentiometer
- Encoder
- Analog Button
- Digital Button
- LDR
- Relay
- Buzzer

### Additional Features:
- Parallel visual feedback LEDs for sensors
- Reset and power button
- Shock-resistant external case
- CODLAI Editor Support
- CODLAI Platform Support
- On-device information sections
- Programmable with open-source editors
- Documentation for developing IoT applications
- Persistent storage helpers (EEPROM): int16/int32/float/string/bytes via `eeprom*` functions
- CRC-protected EEPROM records (versioned): `eepromWriteRecord/eepromReadRecord`
- NTP time sync helpers: `ntpSync/ntpGetDateTimeString` (requires WiFi)
- OTA firmware updates: `otaBegin/otaHandle` (requires WiFi)

### Cloud Integrations
- Send instant alerts via Telegram bots with `sendTelegram()`.
- Connect directly to IFTTT services (Google Sheets, Discord, smart lights, etc.) using the new `triggerIFTTTEvent()` helper. See `examples/main/2-Advanced/IOTBOT_IFTTT_Webhook_Example.ino` for a ready-to-run sketch.

### Security:
AES and SSL/TLS hardware accelerators

### Certifications:
CE, ROSH, EMC

### Dependencies & Versions
- **Platform:** Espressif 32 (ESP32)
- **Framework:** Arduino
- **Libraries:**
  - ESP32Servo
  - LiquidCrystal_I2C
  - Encoder
  - Adafruit NeoPixel
  - Adafruit Unified Sensor
  - DHT sensor library
  - Stepper
  - IRremoteESP8266
  - MFRC522
  - ESPAsyncWebServer
  - Firebase Arduino Client Library for ESP8266 and ESP32
  - ESP Mail Client
  - ArduinoJson
- **Offline Libraries:**
  - `other_libraries.zip`: Contains all required library dependencies for offline installation.

### Video Guide:
[Watch on YouTube](https://www.youtube.com/watch?v=nx7cBQLKk_k&t=6s&ab_channel=CODLAI)

---

# IoTBOT Kütüphanesi Kullanım Kılavuzu
Bu kütüphane CODLAI geliştirici ekibi tarafından IoTBOT ürününü kontrol etmek için özel olarak tasarlanmış ve üretilmiştir.
![alt text](<images/1.png>)

## Arduino IDE ile Kullanım

### Kurulum

1. Arduino IDE'yi açın.
2. Menu çubuğundan "Sketch" -> "Include Library" -> "Manage Libraries..." seçeneğine gidin.
3. Arama kutusuna "IoTBOT" yazın.
4. IoTBOT kütüphanesini bulun ve "Install" düğmesine tıklayarak kurulumu tamamlayın.

## IoTBOT Nedir?
IoTBOT, özellikle robotik kodlama derslerinde kullanılmak üzere IoT odaklı geliştirilmiş bir karttır. IoT uygulamaları yapmak için gereken temel seviye sensörleri dahili olarak bulundurur ve böylece kullanıcıları karmaşık kablo ve sensör bağlantıları yapmaktan kurtarır.

### İşlemci
ESP32 - 240 MHz Tensilica Xtensa Çift Çekirdekli 32-bit LX6 mikroişlemci

### Batarya
2000mAh Li-Po

### HMI
20X04 Programlanabilir Karakterize Mavi LCD Ekran

### Çevresel Giriş/Çıkış:
- 1 x Type-C Soket (Programlama + Güç)
- 5 x RJ45 Tip Modülcük Portu (Analog & Dijital)
- 1 x RJ45 Tip Motor Sürücü Portu
- 1 x Programlanabilir Röle Çıkışı
- 1 x Jumper Kablo Tip SPI, UART, I2C, 5V, 3V, GND Çıkışı
- 1 x RC522 RFID Haberleşme Portu

### Programlama Dilleri
C++, MicroPython (Metin ve Blok Tabanlı)

### Kablosuz Bağlantı
2.4 GHz WiFi + Bluetooth® + Bluetooth LE

### Dahili Sensörler:
- 20X04 LCD Ekran
- L293D Motor Sürücü
- 2 Eksen Butonlu Joystick
- Potansiyometre
- Encoder
- Analog Buton
- Dijital Buton
- LDR
- Röle
- Buzzer

### Ek Özellikler:
- Sensörlere paralel görsel geri bildirim LED'leri
- Reset ve açma-kapama tuşu
- Darbelere karşı dayanıklı dış kabuk
- CODLAI Editor Desteği
- CODLAI Platform Desteği
- Cihaz üzerinde bilgilendirme bölümleri
- Açık kaynaklı editörler ile programlayabilme
- IoT uygulamalar yapmak için gerekli dokümantasyon
- Kalıcı hafıza yardımcıları (EEPROM): `eeprom*` fonksiyonları ile int16/int32/float/string/bytes
- CRC korumalı (versiyonlu) EEPROM record: `eepromWriteRecord/eepromReadRecord`
- NTP saat senkron yardımcıları: `ntpSync/ntpGetDateTimeString` (WiFi gerekir)
- OTA yazılım güncelleme: `otaBegin/otaHandle` (WiFi gerekir)

### Bulut Entegrasyonları
- `sendTelegram()` ile Telegram botlarına anlık bildirimler gönderebilirsiniz.
- Yeni `triggerIFTTTEvent()` fonksiyonu ile IFTTT servislerine (Google Sheets, Discord, akıllı ışıklar vb.) doğrudan bağlanabilirsiniz. Hazır örnek için `examples/main/2-Advanced/IOTBOT_IFTTT_Webhook_Example.ino` dosyasına göz atın.

### Güvenlik:
AES ve SSL/TLS için donanım hızlandırıcıları

### Sertifikalar:
CE, ROSH, EMC

### Video Kılavuzu:
[YouTube'da İzle](https://www.youtube.com/watch?v=nx7cBQLKk_k&t=6s&ab_channel=CODLAI)

---

# Library Structure & Contributing / Kütüphane Yapısı ve Katkıda Bulunma
This library follows a modular design pattern to ensure efficiency.
- **Configuration:** Use `IOTBOT_Config.h` to enable/disable specific modules (e.g., WiFi, Firebase).
- **Extension:** To add new features, define a new flag in the config file and wrap your code in `#if defined(...)` blocks.

Bu kütüphane verimliliği sağlamak için modüler bir tasarım deseni izler.
- **Yapılandırma:** Belirli modülleri (örn. WiFi, Firebase) etkinleştirmek/devre dışı bırakmak için `IOTBOT_Config.h` dosyasını kullanın.
- **Genişletme:** Yeni özellikler eklemek için yapılandırma dosyasında yeni bir bayrak tanımlayın ve kodunuzu `#if defined(...)` blokları içine alın.

# Katkıda Bulunma / Contributing
Katkıda bulunmak isterseniz, lütfen GitHub deposuna Pull Request gönderin. / If you'd like to contribute, please send a Pull Request to the GitHub repository.

# Lisans / License
Bu kütüphane 2024 Yılında Samed KAYA tarafından lisanslanmıştır. Detaylar için LICENSE dosyasına bakınız. / Copyright (c) 2024 Samed KAYA. All rights reserved.

See the LICENSE file for details.

![alt text](<images/2.png>)
![alt text](<images/3.png>)
![alt text](<images/4.png>)
![alt text](<images/5.png>)

