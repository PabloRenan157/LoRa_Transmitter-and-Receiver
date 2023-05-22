// Receptor
#include <SoftwareSerial.h>
#include "EBYTE.h"

//Pinos do LoRa *****Arduino UNO*****
#define M0_LoRa 4
#define M1_LoRa 5
#define RX_LoRa 3
#define TX_LoRa 2
#define AUX_LoRa 6

struct DATA {
  int16_t accelX, accelY, accelZ;
  float altitude, pressure, temperature,aceleracao;
  double latitude, longitude;
  int Count;
};

DATA MyData;
unsigned long Last;

SoftwareSerial lora(RX_LoRa, TX_LoRa);
EBYTE LoRa(&lora, M0_LoRa, M1_LoRa, AUX_LoRa);

void setup() {
  Serial.begin(9600);
  LoRaInit();
  Last = millis();
}

void LoRaInit()
{
    //LoRa configurações
    lora.begin(9600);
    LoRa.init();
    LoRa.SetAirDataRate(ADR_1K);
    LoRa.SetAddress(1);
    LoRa.SetChannel(23);
    LoRa.SaveParameters(TEMPORARY);
    //LoRa.PrintParameters();
    LoRa.SetMode(MODE_NORMAL);
}

void loop() {
 // sendAck();
   if (Serial.available()>0) {
    if (Serial.read() == 'B') {
     LoRa.SendByte('B');
    }
  }
  Last = millis();
  while (!lora.available()) {
   // delay(1);
    if ((millis() - Last) > 650) {
      //sendAck();
      //delay(250);
      break;
    }
  }
  if (lora.available()) {
    if (receiveSensorData()) {
      Last = millis();
    }
  }
}

bool receiveSensorData() {
  DATA receivedData;
  if (LoRa.GetStruct(&receivedData, sizeof(receivedData))) {
    Serial.print(receivedData.Count);
    Serial.print(", ");
    Serial.print(receivedData.altitude);
    Serial.print(", ");
    Serial.print(receivedData.pressure);
    Serial.print(", ");
    Serial.print(receivedData.temperature);
    Serial.print(", ");
    Serial.print(receivedData.latitude, 6);
    Serial.print(", ");
    Serial.print(receivedData.longitude, 6);
    Serial.print(", ");
    Serial.print(receivedData.accelX);
    Serial.print(", ");
    Serial.print(receivedData.accelY);
    Serial.print(", ");
    Serial.print(receivedData.accelZ);
    Serial.print(", ");
    Serial.println(receivedData.aceleracao);
    return true;
  }
  return false;
}

void sendAck() {
  LoRa.SendByte('A');
  Serial.println("Ack sent---------");
}