#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "..\..\hygro_client_eeh\lib\Low-Power-master\LowPower.h"
#include "..\..\hygro_client_eeh\lib\RadioHead-master\RHReliableDatagram.h"
#include "..\..\hygro_client_eeh\lib\RadioHead-master\RH_ASK.h"

#define CLIENT_ADDRESS 2

#define SERVER_ADDRESS 1
#define ENABLERXTX 2

/*Satelit
   connetions:
   tx - 12
   rx - 11
   sda - A4
   scl - A5
*/
float iREF = 1.1; //internal reference cal factor

RH_ASK driver(4000);
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

float fReadVcc();

void setup() {
  Serial.begin(9600*2);
  //manager.setRetries(10);
  manager.setTimeout(500);
  if (!manager.init())
    Serial.println("init failed");
  pinMode(13, OUTPUT);
  Wire.begin();
  Serial.println("start");
  pinMode(ENABLERXTX, OUTPUT);
  pinMode(13, OUTPUT);
  analogReference(INTERNAL); 
  //pinMode(A1, INPUT);
  digitalWrite(ENABLERXTX, LOW);
  digitalWrite(13, LOW);
}

uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
bool state = 0;
uint16_t low_power_sleep = 0;

void loop() {
  unsigned char i2cResponse[5];
  int i = 0;
  Wire.beginTransmission(0x40); // transmit to device #0x40
  Wire.write((byte)0xE1);        // sends five bytes
  Wire.endTransmission(true);    // stop transmitting
  delay(30);
  Wire.requestFrom(0x40, 5, true);
  while (Wire.available())
    i2cResponse[i++] = Wire.read();
  unsigned int tb;
  tb = i2cResponse[1] << 8 | i2cResponse[2];
  double t = -46.85 + 175.72 * (tb / 65535.0);
  unsigned int rhb;
  rhb = i2cResponse[3] << 8 | i2cResponse[4];
  double rh = -6.0 + 125 * (rhb / 65535.0);

  double ah;
  ah = (6.112 * exp(17.67 * t / (t + 243.5)) * rh * 2.1674) / (273.15 + t);

  char data[24] = "";
  char temp[5] = "v";
  strcpy(data, temp);
  dtostrf(fReadVcc(), 4, 2, temp);
  strcat(data, temp);

  strcat(data, ",h");
  dtostrf(rh, 3, 1, temp);
  strcat(data, temp);

  strcat(data, ",t");
  dtostrf(t, 3, 1, temp);
  strcat(data, temp); 

  strcat(data, ",f");
  dtostrf(ah,3,2,temp);
  strcat(data,temp);

  digitalWrite(ENABLERXTX, HIGH);
  digitalWrite(13, HIGH);
  delay(100);
  Serial.println("Sending to ask_reliable_datagram_server");
  // Send a message to manager_server
  if (manager.sendtoWait((uint8_t *)data, sizeof(data), SERVER_ADDRESS))
  {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAckTimeout(buf, &len, 2000, &from))
    {
      Serial.print("got reply from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);
      if (state == 0) {
        digitalWrite(13, 1);
        state = 1;
      }
      else {
        digitalWrite(13, 0);
        state = 0;
      }
      Serial.println(state);
    }
    else
    {
      Serial.println("No reply, is ask_reliable_datagram_server running?");
    }
  }
  else{
    Serial.println("sendtoWait failed");
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    delay((analogRead(A1) & 0x03) << 7);
  }
  digitalWrite(ENABLERXTX, LOW);
  digitalWrite(13, LOW);
  delay(100);
  for(;low_power_sleep<2;low_power_sleep++){
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
  low_power_sleep = 0;
}

float fReadVcc() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(5); //delay for 5 milliseconds
  ADCSRA |= _BV(ADSC); // Start ADC conversion
  while (bit_is_set(ADCSRA, ADSC)); //wait until conversion is complete
  int result = ADCL; //get first half of result
  result |= ADCH << 8; //get rest of the result
  float batVolt = (iREF / result) * 1024; //Use the known iRef to calculate battery voltage + resistance
  return batVolt;
    // digitalWrite(ENABLE5, HIGH);
    // analogReference(INTERNAL); 
    // pinMode(A1, INPUT);
    // delay(100);
    // uint8_t batraw = analogRead(A1);
    // digitalWrite(ENABLE5, LOW);
    // float batVolt = map(batraw, 0,1024,0,1100)/100.0;
    // batVolt = batVolt;
    // Serial.println(batraw);
    // Serial.println(batVolt);
    // return batVolt;
}