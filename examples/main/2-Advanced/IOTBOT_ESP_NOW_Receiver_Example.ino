/*
 * 📌 ESP-NOW ALICI (RECEIVER) ÖRNEĞİ
 * 📌 ESP-NOW RECEIVER EXAMPLE
 *
 * 📌 IOTBOT_Config.h dosyasında 'USE_ESPNOW' aktif edilmelidir.
 * 📌 'USE_ESPNOW' must be enabled in IOTBOT_Config.h.
 */
#define USE_ESPNOW
#include <IOTBOT.h>

IOTBOT iotbot;

// Gelen veri yapısı (Gönderici ile aynı olmalı)
// Incoming data structure (Must match the sender)
typedef struct struct_message {
  char msg[32];
  int value;
  float temp;
  bool status;
} struct_message;

struct_message incomingData;

// 📥 Veri Alındığında Çalışan Fonksiyon
// 📥 Function called when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingBytes, int len) {
  memcpy(&incomingData, incomingBytes, sizeof(incomingData));
  
  // Seri Port Çıktısı / Serial Output
  Serial.print("Bytes: "); Serial.println(len);
  Serial.print("Msg: "); Serial.println(incomingData.msg);
  Serial.print("Value: "); Serial.println(incomingData.value);

  // LCD Geri Bildirim / LCD Feedback
  iotbot.lcdClear();
  iotbot.lcdWriteCR(0, 0, "Veri Alindi!");
  iotbot.lcdWriteCR(0, 1, incomingData.msg);
  iotbot.lcdWriteCR(0, 2, "Deger: " + String(incomingData.value));
  iotbot.lcdWriteCR(0, 3, "Sicaklik: " + String(incomingData.temp));
  
  // Sesli Uyarı / Buzzer Alert
  iotbot.buzzerPlayTone(1500, 100);
  delay(50);
  iotbot.buzzerPlayTone(2000, 100);
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);

  iotbot.lcdShowLoading("Alici Baslatiliyor");
  iotbot.buzzerPlayTone(1000, 200);

  // ESP-NOW Başlat / Initialize ESP-NOW
  iotbot.initESPNow();

  // Alıcı Fonksiyonunu Kaydet / Register Receive Callback
  iotbot.registerOnRecv(OnDataRecv);
  
  // MAC Adresini Göster / Show MAC Address
  String myMac = WiFi.macAddress();
  Serial.println("Receiver MAC: " + myMac);
  
  iotbot.lcdClear();
  iotbot.lcdWriteCR(0, 0, "ALICI MODU (RX)");
  iotbot.lcdWriteCR(0, 1, "MAC Adresim:");
  iotbot.lcdWriteCR(0, 2, myMac.substring(12)); // Son kısmı göster / Show the last part
  iotbot.lcdWriteCR(0, 3, "Veri Bekleniyor...");
}

void loop() {
  // Alıcı modunda döngüde bir şey yapmaya gerek yoktur.
  // Her şey OnDataRecv fonksiyonunda gerçekleşir.
  // Nothing to do in loop for receiver mode.
  // Everything happens in OnDataRecv function.
}
