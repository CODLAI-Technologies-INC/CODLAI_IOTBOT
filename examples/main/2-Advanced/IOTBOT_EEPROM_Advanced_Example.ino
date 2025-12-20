/*
  CODLAI IOTBOT - EEPROM Advanced Example

  TR:
  - Bu örnek, IOTBOT kütüphanesindeki EEPROM yardımcı fonksiyonlarının kullanımını gösterir.
  - IOTBOT (ESP32) üzerinde EEPROM emülasyonu flash üzerinde çalışır.
  - Adresleri planlayın ve sık yazmaktan kaçının.

  EN:
  - This example demonstrates EEPROM helper functions in the IOTBOT library.
  - On ESP32, EEPROM is an emulation layer on flash.
  - Plan addresses and avoid excessive writes.
*/

#include <IOTBOT.h>

IOTBOT iotbot;

void setup()
{
  iotbot.serialStart(115200);
  delay(200);

  // TR/EN: Initialize EEPROM (ESP32: choose a size suitable for your app)
  bool ok = iotbot.eepromBegin(1024);
  iotbot.serialWrite(ok ? "[EEPROM] Ready" : "[EEPROM] Begin failed");

  // TR/EN: Legacy int16
  iotbot.eepromWriteInt(0, 777);
  int v16 = iotbot.eepromReadInt(0);
  iotbot.serialWrite("v16 = ");
  iotbot.serialWrite(v16);

  // TR/EN: int32 + float
  iotbot.eepromWriteInt32(10, 987654321);
  int32_t v32 = iotbot.eepromReadInt32(10, -1);
  iotbot.serialWrite("v32 = ");
  iotbot.serialWrite((long)v32);

  iotbot.eepromWriteFloat(20, 3.14159f);
  float vf = iotbot.eepromReadFloat(20, -1.0f);
  iotbot.serialWrite("vf = ");
  iotbot.serialWrite(vf);

  // TR/EN: string
  iotbot.eepromWriteString(40, String("Merhaba IOTBOT / Hello IOTBOT"), 64);
  String s = iotbot.eepromReadString(40, 64);
  iotbot.serialWrite("str = ");
  iotbot.serialWrite(s);

  // TR/EN: Recommended (CRC + versioned record)
  struct ExampleConfig
  {
    uint32_t bootCount;
    float lastValue;
  };

  const int CONFIG_ADDR = 200;

  ExampleConfig cfgOut;
  cfgOut.bootCount = 1;
  cfgOut.lastValue = 3.14f;
  bool wrec = iotbot.eepromWriteRecord(CONFIG_ADDR, (const uint8_t *)&cfgOut, (uint16_t)sizeof(cfgOut), 1);
  iotbot.serialWrite(wrec ? "record write = OK" : "record write = FAIL");

  ExampleConfig cfgIn;
  uint16_t outLen = 0;
  uint16_t outVer = 0;
  bool rrec = iotbot.eepromReadRecord(CONFIG_ADDR, (uint8_t *)&cfgIn, (uint16_t)sizeof(cfgIn), &outLen, &outVer);

  if (rrec)
  {
    Serial.print("record ver=");
    Serial.print(outVer);
    Serial.print(" len=");
    Serial.print(outLen);
    Serial.print(" bootCount=");
    Serial.print((unsigned long)cfgIn.bootCount);
    Serial.print(" lastValue=");
    Serial.println(cfgIn.lastValue);
  }
  else
  {
    Serial.println("record read = FAIL (magic/len/crc mismatch)");
  }
}

void loop()
{
}
