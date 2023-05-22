//-------------------------------------Transmissor-------------------------------------

//Bibliotecas
#include <SoftwareSerial.h>
#include "EBYTE.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Pinos OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

//****Pinos para comunicação I2C*****
// Pinos SDA e SCL 
#define OLED_SDA 21
#define OLED_SCL 22
// Pinos do BMP280
#define BMP_SDA_PIN 20
#define BMP_SCL_PIN 21
// Pinos do MPU9250
#define MPU_SDA_PIN 20
#define MPU_SCL_PIN 21
#define MPU_ADDRESS 0x68


//Pinos do LoRa *****Arduino MEGA*****
#define M0_LoRa 4
#define M1_LoRa 5
#define RX_LoRa 3 // Vai no TXD do módulo
#define TX_LoRa 2 // Vai no RXD do módulo
#define AUX_LoRa 6

// Registradores do MPU-9250/6500
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

//Pino de CS do leitor SD 
#define pinoSS 53 //Arduino MEGA 53 Arduino Uno 10

// Pino do relé
#define RELAY_PIN 8

// Limite de aceleração para acionar o relé (ajuste conforme necessário)
#define ACELERACAO_LIMITE 10000 // Em metros por segundo ao quadrado (m/s²)

// Inicialize o objeto da tela OLED
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

// Inicialização dos objetos
Adafruit_BMP280 bmp;
TinyGPSPlus gps;
File dataFile;
SoftwareSerial lora(RX_LoRa, TX_LoRa);
EBYTE LoRa(&lora, M0_LoRa, M1_LoRa, AUX_LoRa);


// Objeto de comunicação serial para o GY-NEO6MV2

//Struct de dados de sensores
struct DATA {
  int16_t accelX, accelY, accelZ;
  float altitude, pressure, temperature,aceleracao;
  double latitude, longitude;
  int Count;
};
char test;
DATA MyData;
unsigned long Last;
unsigned long LastAck;

//-------------------------------------Função de configs iniciais-------------------------------------
void setup() {
//inicia a Serial
Serial.begin(9600);

//definições de pinos de entrada e saída
pinMode(pinoSS, OUTPUT);
pinMode(RELAY_PIN, OUTPUT);

//funcões de inicialização
DisplayInit();
LoRaInit();
initBMP();
initSD();
LastAck = 0;
}

//-------------------------------------Funções INIT----------------------------------------
void DisplayInit()
{
    // Inicialize a comunicação I2C
  display.begin(OLED_SDA, OLED_SCL);
  // Inicialize a tela OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) {
    Serial.println("Falha ao iniciar a tela OLED");
    while (true);
  }
  // Limpe a tela
  display.clearDisplay();
  // Defina o tamanho do texto
  display.setTextSize(1);
  // Defina a cor do texto (branco)
  display.setTextColor(SSD1306_WHITE);

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
void initSD()
{ 
  // Inicialização do cartão microSD
  if (SD.begin()) 
  { // Inicializa o SD Card
      Serial.println("SD Card pronto para uso."); // Imprime na tela
  }
  else 
  {
      Serial.println("Falha na inicialização do SD Card.");
      return;
  }
    Serial.println("Deu boa SD");
    // Abre o arquivo para escrita
    dataFile = SD.open("dados.txt", FILE_WRITE);

    if (dataFile) 
    {
        // Escreve o cabeçalho no arquivo
        dataFile.println("Altitude (m), Pressão (ATM), Temperatura (C), Latitude, Longitude, Aceleracao X, Aceleracao Y, Aceleracao Z, Aceleracao Total");
        dataFile.close();
    } 
    else 
    {
        Serial.println("Não foi possível abrir o arquivo para escrita!");
    }
}

void initBMP() { 
 bmp.begin(BMP_SDA_PIN , BMP_SCL_PIN );
  if (!bmp.begin(0x76)) {
    Serial.println("Não foi possível encontrar o BMP280. Verifique a conexão!");
    while (1);
  }
 Serial.println("Deu boa BMP");
}

//-------------------------------------Função principal-------------------------------------
void loop() 
{
  if (millis() - LastAck > 80) {
  // if (receiveAck())
  // {
      sendSensorData();
      LastAck = millis();
  // }
  //Se não houver confirmação recebida por 2 segundos, reenviar os dados
  }
}
//-------------------------------------Função que printa as informações na telinha-------------------------------------
void DislpayInfo(DATA Oled)
{
 // Limpe a tela
  display.clearDisplay();
 // Defina a posição do texto na tela (0, 0)
   display.setCursor(0, 0);
  // Imprima o texto na tela
  display.println("Send:     |");
   // Defina a posição do texto na tela (Coluna, Linha) 
   display.setCursor(30, 0);
     // Imprima o texto na tela
  display.print(Oled.Count);

  display.setCursor(65, 0);
  // Imprima o texto na tela
  display.println("Temp: ");
   // Defina a posição do texto na tela (Coluna, Linha) 
   display.setCursor(95, 0);
     // Imprima o texto na tela
  display.print(Oled.temperature);

 display.setCursor(0, 10);
  // Imprima o texto na tela
  display.println("Press:    |");
   // Defina a posição do texto na tela (Coluna, Linha) 
   display.setCursor(35, 10);
     // Imprima o texto na tela
  display.print(Oled.pressure);

 display.setCursor(65, 10);
  // Imprima o texto na tela
  display.println("Alt: ");
   // Defina a posição do texto na tela (Coluna, Linha) 
   display.setCursor(90, 10);
     // Imprima o texto na tela
  display.print(Oled.altitude);

  // Atualize a tela
  display.display();

}
//-------------------------------------Função de Envio de dados e salvamento no SD ---------------------------------------
void sendSensorData() 
{
  readBMP280(MyData.altitude, MyData.pressure, MyData.temperature);
  readGPS(MyData.latitude, MyData.longitude);
  readAccelerometerData (MyData.accelX, MyData.accelY, MyData.accelZ);
  MyData.aceleracao = sqrt(pow(MyData.accelX, 2) + pow(MyData.accelY, 2) + pow(MyData.accelZ, 2));

  LoRa.SendStruct(&MyData, sizeof(MyData));
  Serial.print("Sending: "); Serial.println(MyData.Count);
  DislpayInfo(MyData);

  MyData.Count ++;
  delay(100);
 // Abre o arquivo para escrita
  dataFile = SD.open("dados.txt", FILE_WRITE);

  if (dataFile) 
  {
    // Escreve os dados no arquivo
    dataFile.print(MyData.altitude);
    dataFile.print(", ");
    dataFile.print(MyData.pressure);
    dataFile.print(", ");
    dataFile.print(MyData.temperature);
    dataFile.print(", ");
    dataFile.print(MyData.latitude, 6);
    dataFile.print(", ");
    dataFile.print(MyData.longitude, 6);
    dataFile.print(", ");
    dataFile.print(MyData.accelX);
    dataFile.print(", ");
    dataFile.print(MyData.accelY);
    dataFile.print(", ");
    dataFile.print(MyData.accelZ);
    dataFile.print(", ");
    dataFile.println(MyData.aceleracao);
    dataFile.close();
  } 
  else 
  {
    Serial.println("Não foi possível abrir o arquivo para escrita!");
  }
}
//-------------------------------------Função que verfica se precisa enviar dados -------------------------------
bool receiveAck() 
{
if (lora.available()) 
  {
   test = (char)LoRa.GetByte();
    if(test == 'A')
    {
      Serial.println("Ack received----------------");
      return true;
    }
  }
return false;
}

//-------------------------------------SENSORES ----------------------------------------
void readGPS(double& latitude, double& longitude) {
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
      }
    }
  }
}
void readBMP280(float& altitude, float& pressure, float& temperature) 
{
  altitude = bmp.readAltitude(1013.25);
  pressure = (bmp.readPressure() / 100.0)/1013.25;
  temperature = bmp.readTemperature();
}
void readAccelerometerData(int16_t& accelX, int16_t& accelY, int16_t& accelZ) 
{
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) 
  {
    return false; // Erro na transmissão I2C
  }
  if (Wire.requestFrom(MPU_ADDRESS, 6) != 6) 
  {
    return false; // Erro na leitura dos registros
  }
  
  accelX = (Wire.read() << 8) | Wire.read();
  accelY = (Wire.read() << 8) | Wire.read();
  accelZ = (Wire.read() << 8) | Wire.read();
  
  return true;
}