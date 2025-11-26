/*
 * 📌 ESP-NOW GÖNDERİCİ (SENDER) ÖRNEĞİ
 * 📌 ESP-NOW SENDER EXAMPLE
 *
 * 📌 IOTBOT_Config.h dosyasında 'USE_ESPNOW' aktif edilmelidir.
 * 📌 'USE_ESPNOW' must be enabled in IOTBOT_Config.h.
 */
#define USE_ESPNOW
#include <IOTBOT.h>

IOTBOT iotbot;

// 📡 ALICI MAC ADRESİ / RECEIVER MAC ADDRESS
// Tüm cihazlara göndermek için (Broadcast): {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
// To send to all devices (Broadcast): {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
// Özel bir cihaza göndermek için o cihazın MAC adresini yazın.
// To send to a specific device, write that device's MAC address.
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Gönderilecek veri yapısı
// Structure of data to be sent
typedef struct struct_message {
  char msg[32];
  int value;
  float temp;
  bool status;
} struct_message;

struct_message myData;

// 📤 Veri Gönderildiğinde Çalışan Fonksiyon
// 📤 Function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
  
  if (status == ESP_NOW_SEND_SUCCESS) {
    iotbot.lcdShowStatus("Gonderim", "Basarili", true);
    iotbot.buzzerPlayTone(2000, 100);
  } else {
    iotbot.lcdShowStatus("Gonderim", "Basarisiz", false);
    iotbot.buzzerPlayTone(500, 500);
  }
}

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);

  iotbot.lcdShowLoading("Gonderici Baslatiliyor");
  iotbot.buzzerPlayTone(1000, 200);

  // ESP-NOW Başlat / Initialize ESP-NOW
  iotbot.initESPNow();

  // Gönderim Durumu Fonksiyonunu Kaydet / Register Send Callback
  esp_now_register_send_cb(OnDataSent);
  
  iotbot.lcdShowStatus("Gonderici (TX)", "Hazir", true);
  delay(1000);
}

void loop() {
  // Verileri Hazırla / Prepare Data
  strcpy(myData.msg, "Merhaba IOTBOT");
  myData.value = random(0, 100);
  myData.temp = 24.5;
  myData.status = true;

  // LCD Bilgi / LCD Info
  iotbot.lcdClear();
  iotbot.lcdWriteCR(0, 0, "Gonderilen Veri:");
  iotbot.lcdWriteCR(0, 1, myData.msg);
  iotbot.lcdWriteCR(0, 2, "Deger: " + String(myData.value));
  iotbot.lcdWriteCR(0, 3, "Gonderiliyor...");

  // Veriyi Gönder / Send Data
  iotbot.sendESPNow(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  delay(3000); // 3 saniye bekle / Wait 3 seconds
}
