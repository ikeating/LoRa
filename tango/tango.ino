// Feather32u4 rfm96_TX
// -*- mode: C++ -*-
//  **  tango  **  messaging client (transmitter)
//Keating

#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

#define CLIENT_ADDRESS 42
#define SERVER_ADDRESS 128

// for feather32u4 
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

#define LED 13

int sensorPin = 0; //the analog pin the TMP36's Vout (sense) pin is connected to
int reading = 0;

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

RHReliableDatagram manager(rf95, CLIENT_ADDRESS);

void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  pinMode(LED, OUTPUT);

  Serial.begin(115200);
  //while (!Serial) {
  //  delay(1);
  //}

  delay(100);

  Serial.println("Feather LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!manager.init())
    {
      Serial.println("LoRa radio init failed");
      Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
      while (1);
    }
  Serial.println("LoRa radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ))
    {
      Serial.println("setFrequency failed");
      while (1);
    }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

char radiopacket[60] = "Temp         Voltage      N0TSU ROMEO  # ";
char vBat[5] = {0};
char temp[10] = {0};

void loop()
{
  temp[9] = {0};
  vBat[4] = NULL;
  radiopacket[59] = NULL;
  delay(1000); // Wait 2.5 second between transmits, could also 'sleep' here!
  
  Serial.println("Transmitting..."); // Send a message to rf95_server
  
  //reading temperature and coverting it to string and storing it in an array
  reading = analogRead(sensorPin);
  float voltage = reading * 3.3;
  voltage /= 1024.0;
  float temperatureC = (voltage - 0.5) * 100 ;
  float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;
  dtostrf(temperatureF, 4, 2, temp);

  //Serial.print(temperatureF);
  radiopacket[6] = temp[0];
  radiopacket[7] = temp[1];
  radiopacket[8] = temp[2];
  radiopacket[9] = temp[3];
  radiopacket[10] = temp[4];

  //reading voltage and coverting it to string and storing it in an array
  #define VBATPIN A9
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;
  measuredvbat *= 3.3;
  measuredvbat /= 1024;
  Serial.print("VBat: ");
  Serial.println(measuredvbat);
  dtostrf(measuredvbat, 3, 2, vBat);

  radiopacket[21] = vBat[0];
  radiopacket[22] = vBat[1];
  radiopacket[23] = vBat[2];
  radiopacket[24] = vBat[3];

  itoa( packetnum++, radiopacket+40, 10);

  Serial.print("Sending ");
  Serial.println(radiopacket);
  Serial.println("Sending...");
      digitalWrite(LED, HIGH); delay(30);
      digitalWrite(LED, LOW);delay(30);
      digitalWrite(LED, HIGH); delay(30);
      digitalWrite(LED, LOW);delay(30);
      digitalWrite(LED, HIGH); delay(30);
      digitalWrite(LED, LOW);delay(30);
      digitalWrite(LED, HIGH); delay(30);
  //delay(10);
  manager.sendtoWait(radiopacket, sizeof(radiopacket), SERVER_ADDRESS);
  Serial.println("Waiting for packet to complete..."); 
  delay(10);
  manager.waitPacketSent();

  digitalWrite(LED, LOW);
  
  // Now wait for a reply
  Serial.println("Waiting for reply...");
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from;
      
  if (rf95.waitAvailableTimeout(1500)) // needed to increase timeout
    { 
      digitalWrite(LED, HIGH); delay(200);
      // Should be a reply message for us now   
      if (manager.recvfromAckTimeout(buf, &len, 2000, &from))
        {
          Serial.print("Got reply from : 0x");
          Serial.print(from, HEX);
          Serial.print(": ");
          Serial.println((char*)buf);
          Serial.print("RSSI: ");
          Serial.println(rf95.lastRssi(), DEC);
          digitalWrite(LED, LOW);  
        }
      else
        {
          Serial.println("Receive failed");
        }
    }
  else
    {
      Serial.println("No reply, is there a listener around?");
    }
}
