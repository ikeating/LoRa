// Feather32u4 rfm96_RX
// -*- mode: C++ -*-
//  **  romeo  **  messaging server (reciever)
//Keating

#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

#define CLIENT_ADDRESS 42
#define SERVER_ADDRESS 128

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// for Feather32u4 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

// for TFT Screen
#if defined (__AVR_ATmega32U4__) || defined(ARDUINO_SAMD_FEATHER_M0) || defined (__AVR_ATmega328P__) || \
    defined(ARDUINO_SAMD_ZERO) || defined(__SAMD51__) || defined(__SAM3X8E__) || defined(ARDUINO_NRF52840_FEATHER)
   #define STMPE_CS 6
   #define TFT_CS   9
   #define TFT_DC   10
   #define SD_CS    5
#endif

// Pallete - Where you assign names to colors you like
#define BACKCOLOR 0x001F 
#define PRINTCOL 0x07E0
#define CHARBACK 0x001F
// Color definitions
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

RHReliableDatagram manager(rf95, SERVER_ADDRESS);

// Blinky on receipt
#define LED 13

void xyGridLine (double xLength, double yLength, double xZero, double yZero, double numTicks)
{
   double llx = xZero;
   double lrx = xZero + (xLength/numTicks);


   double lly = yZero;
   double uly = yZero - (yLength/numTicks);

   for (int i = 0;i < numTicks;i++)
   {
   tft.drawLine(llx,lly,lrx,lly,0xFFFF);
   tft.drawLine(lrx,lly,lrx,lly+5, 0xFFFF);
   tft.drawLine(lrx,lly,lrx,lly-5, 0xFFFF);

   lrx=lrx+(xLength/numTicks);
   }

   for (int i = 0;i < numTicks;i++)
   {
   tft.drawLine(llx,lly,llx,uly, 0xFFFF);
   tft.drawLine(llx,uly,llx+5,uly, 0xFFFF);
   tft.drawLine(llx,uly,llx-5,uly, 0xFFFF);

   uly=uly-(yLength/numTicks);
   }
}

int numTimes = 0;

void setup()
{ 
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  /*while (!Serial) {
    delay(1);
  }*/
  delay(100);

  Serial.println("Feather LoRa RX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  //while (!rf95.init()) {
  while (!manager.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  //  setup gfx
  tft.begin();
  tft.fillScreen(BACKCOLOR);
  tft.setCursor(0,0);
  tft.setTextWrap(true);
  tft.setTextColor(PRINTCOL,CHARBACK);
  tft.setTextSize(3);

  xyGridLine (/*double xLength*/200, /*double yLength*/100, /*double xZero*/5, /*double yZero*/315, /*double numTicks*/10);

  //*********** Circle Tics ******************************

  for( int i = 0; i < 360; i++)
    {
      double radius = 55;//tic is 5 because the circle is 50
      double radian = (i * 0.0174532925)* 10;//one tic every 10 rad
      double dataX = radius * cos(radian);
      double dataY = radius * sin(radian);
      tft.drawLine( 120, 240, dataX+120, dataY+240, 0xFFE0);
    }
  //******************************************************
}
void loop()
{
  if (manager.available())
  {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    uint8_t from;
    //if (rf95.recv(buf, &len))
    if (manager.recvfromAck(buf,&len, &from))
    {
      digitalWrite(LED, HIGH); delay(100);
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.println("");
      Serial.print("got transmission from : 0x");
      Serial.print(from, HEX);
      Serial.println(": ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      digitalWrite(LED, LOW);
      //  gfx
      tft.setCursor(0,0);
      //tft.fillScreen(BACKCOLOR);
      tft.print("RSSI: "); tft.print(rf95.lastRssi(), DEC);
      tft.println("");
      tft.print("freq: ");
      tft.println(RF95_FREQ);
      tft.print("CLIENT: "); tft.print(CLIENT_ADDRESS);
      tft.println("");
      tft.println((char*)buf);
      tft.println(""); 

      //xyGridLine (/*double xLength*/200, /*double yLength*/100, /*double xZero*/5, /*double yZero*/315, /*double numTicks*/10);
      
      //numTimes++;
      double radius = 50;
      double radian = numTimes * 0.0174532925;

      double dataX = radius * cos(radian);
      double dataY = radius * sin(radian);

      double triX2 = (radius-15) * cos(radian+.2);
      double triY2 = (radius-15) * sin(radian+.2);

      double triX3 = (radius-15) * cos(radian-.2);
      double triY3 = (radius-15) * sin(radian-.2);

      //tft.println(numTimes);

      tft.fillCircle(120,240,50,0x001F);
      tft.drawLine( 120, 240, dataX+120, dataY+240, 0x07E0);
      tft.fillTriangle( dataX+120, dataY+240, triX2+120, triY2+240, triX3+120, triY3+240, 0xFFE0);
      tft.drawCircle(120,240,51,0x07E0);

      numTimes = numTimes + 10;
/*      
      int howMany = 200;
      for (int i = 0; i < howMany; i++ )
        {
          dataX = (i+5);//+ moves the begining over
          dataY = (20*sin((dataX-350)/10))+250;
          //dataY = -((i*i)/100)+315;
          tft.drawPixel(dataX,dataY, 0xFFFF);
        }     */
      // Send a reply
      digitalWrite(LED, HIGH); delay(30);
      digitalWrite(LED, LOW);delay(30);
      digitalWrite(LED, HIGH); delay(30);
      digitalWrite(LED, LOW);delay(30);
      digitalWrite(LED, HIGH); delay(30);
      digitalWrite(LED, LOW);delay(30);
      digitalWrite(LED, HIGH); delay(30);
      uint8_t data[] = "And hello Tango N0TSU";
      //rf95.send(data, sizeof(data));
      manager.sendtoWait(data, sizeof(data), CLIENT_ADDRESS);
      delay(10);
      manager.waitPacketSent();
      digitalWrite(LED, LOW);
      Serial.println("Sent a reply");

      Serial.println(rf95.printRegisters());
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
}
