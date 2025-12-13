/*
 * IOTBOT IFTTT Webhook Example / IOTBOT IFTTT Webhook Örneği
 *
 * This advanced example shows how to trigger IFTTT Webhooks whenever the IoTBOT boots
 * or when Button 1 is pressed. Customize the WiFi credentials, IFTTT event name,
 * and webhook key before uploading the sketch.
 *
 * Bu gelişmiş örnek, IoTBOT açıldığında veya 1. butona basıldığında IFTTT Webhook
 * çağrıları oluşturmayı gösterir. Kodu yüklemeden önce WiFi bilgilerini, IFTTT
 * olay adını ve webhook anahtarını güncellemeyi unutmayın.
 *
 * How to get your IFTTT credentials / IFTTT bilgilerini nasıl alırsınız:
 * 1. Go to https://ifttt.com/create and pick Webhooks -> "Receive a web request" as the trigger, then enter
 *    an Event Name (use the same text as `iftttEventName`).
 * 2. Choose any target service for the "Then" action (Google Sheets, Discord, Gmail, etc.) and finish the applet.
 * 3. Visit https://ifttt.com/maker_webhooks , click "Documentation" and copy the key that appears in the
 *    sample URL; paste it into `iftttWebhookKey` below.
 * 4. Your Maker applet can read the JSON fields `value1`, `value2`, and `value3` as ingredients.
 *
 * NOTE / NOT: When `USE_IFTTT` (or Firebase) is enabled, the library automatically activates the Wi-Fi helpers.
 * You no longer need to add `#define USE_WIFI` to the sketch.
 */

#define USE_IFTTT   // Enable IFTTT helpers / IFTTT fonksiyonlarını açar
#include <IOTBOT.h>

IOTBOT iotbot;

// WiFi Credentials / WiFi Bilgileri
const char *ssid = "YOUR_WIFI_SSID";
const char *password = "YOUR_WIFI_PASSWORD";

// IFTTT Maker Webhooks data / IFTTT Maker Webhooks bilgileri
String iftttEventName = "YOUR_EVENT_NAME";   // Example: "iotbot_button"
String iftttWebhookKey = "YOUR_IFTTT_KEY";   // Find at https://ifttt.com/maker_webhooks

unsigned long lastTrigger = 0;
const unsigned long triggerInterval = 5000; // Minimum 5 seconds between button events

void setup() {
  iotbot.serialStart(115200);
  iotbot.begin();
  iotbot.playIntro();

  iotbot.lcdClear();
  iotbot.lcdWriteMid("IFTTT", "Webhook", "Connecting", "WiFi...");

  iotbot.wifiStartAndConnect(ssid, password);

  if (iotbot.wifiConnectionControl()) {
    iotbot.lcdClear();
    iotbot.lcdWriteMid("WiFi", "Connected", "Triggering", "Startup");

    // Send a boot notification so automations know we are online.
    String bootPayload = "{\"value1\":\"IoTBOT\",\"value2\":\"Boot\",\"value3\":\"System Online\"}";
    bool ok = iotbot.triggerIFTTTEvent(iftttEventName, iftttWebhookKey, bootPayload);
    Serial.println(ok ? "[IFTTT] Boot event sent" : "[IFTTT] Boot event failed");

    iotbot.lcdClear();
    iotbot.lcdWriteMid("Ready", "Press B1", "to send", "Webhook");
  } else {
    iotbot.lcdClear();
    iotbot.lcdWriteMid("WiFi", "Failed", "Check", "SSID/PASS");
  }
}

void loop() {
  bool buttonPressed = iotbot.button1Read();
  unsigned long now = millis();

  if (buttonPressed && (now - lastTrigger) > triggerInterval) {
    lastTrigger = now;

    int ldrValue = iotbot.ldrRead();
    int potValue = iotbot.potentiometerRead();

    String payload = "{\"value1\":\"Button 1\",\"value2\":\"Pressed\",\"value3\":\"LDR:";
    payload += String(ldrValue);
    payload += " POT:";
    payload += String(potValue);
    payload += "\"}";

    iotbot.lcdClear();
    iotbot.lcdWriteMid("Button 1", "Sending", "IFTTT", "Webhook");

    bool ok = iotbot.triggerIFTTTEvent(iftttEventName, iftttWebhookKey, payload);
    Serial.println(ok ? "[IFTTT] Button event delivered" : "[IFTTT] Button event failed");

    iotbot.lcdClear();
    iotbot.lcdWriteMid("Webhook", ok ? "Delivered" : "Failed", "Waiting", "...");
  }

  delay(50);
}
