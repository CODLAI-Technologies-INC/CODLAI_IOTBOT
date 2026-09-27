// TR: YENI NEOPIXEL EFEKTLERI. Bu ornek akilli LED (NeoPixel) icin yeni
// kolaylik fonksiyonlarini gosterir: tum seridi tek renge boyama
// (Fill), sondurme (Clear), parlaklik ayarlama (SetBrightness), yanip
// sondurme (Blink) ve "nefes alma" efekti (Breathe).
// EN: NEW NEOPIXEL EFFECTS. This example shows new convenience
// functions for the smart LED (NeoPixel): filling the whole strip with
// one color (Fill), turning it off (Clear), adjusting brightness
// (SetBrightness), blinking on/off (Blink), and a "breathing" effect
// (Breathe).
//
// Baglanti / Wiring: Akilli LED seridini P1-P5 soketlerinden BIRINE
// takin ve asagidaki LED_PIN degerini o soketin sinyaline gore
// ayarlayin. / Plug the smart LED strip into ONE of the P1-P5 sockets
// and set LED_PIN below to match that socket's signal.

#define USE_NEOPIXEL
#include <IOTBOT.h>

IOTBOT iotbot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

#define LED_PIN IO27 // Akilli LED'in bagli oldugu pin / Pin the smart LED is connected to
// Desteklenen pinler: IO25 - IO26 - IO27 - IO32 - IO33
// Supported pins: IO25 - IO26 - IO27 - IO32 - IO33

void setup() {
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.moduleSmartLEDPrepare(LED_PIN);
  iotbot.lcdWriteMid(turkish ? "NEOPIXEL EFEKTLERI" : "NEOPIXEL EFFECTS",
                      turkish ? "Baslatiliyor..." : "Starting...",
                      "", "");
}

void loop() {
  iotbot.serialWrite(turkish ? "Fill: Kirmizi" : "Fill: Red");
  iotbot.moduleSmartLEDFill(255, 0, 0);
  delay(1000);

  iotbot.serialWrite(turkish ? "Fill: Yesil" : "Fill: Green");
  iotbot.moduleSmartLEDFill(0, 255, 0);
  delay(1000);

  iotbot.serialWrite(turkish ? "Clear: Sondu" : "Clear: Off");
  iotbot.moduleSmartLEDClear();
  delay(1000);

  iotbot.serialWrite(turkish ? "Parlaklik: dusuk (mavi)" : "Brightness: low (blue)");
  iotbot.moduleSmartLEDSetBrightness(40);
  iotbot.moduleSmartLEDFill(0, 0, 255);
  delay(1000);

  iotbot.serialWrite(turkish ? "Parlaklik: yuksek (mavi)" : "Brightness: high (blue)");
  iotbot.moduleSmartLEDSetBrightness(255);
  delay(1000);

  iotbot.serialWrite(turkish ? "Blink: sari, 3 kez" : "Blink: yellow, 3 times");
  iotbot.moduleSmartLEDBlink(255, 255, 0, 3, 200);

  iotbot.serialWrite(turkish ? "Breathe: mor" : "Breathe: purple");
  iotbot.moduleSmartLEDBreathe(150, 0, 255, 2000);

  iotbot.moduleSmartLEDClear();
  delay(2000);
}
