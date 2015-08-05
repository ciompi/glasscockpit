

//c.FissionChips 19/6/2015//v1.1
#include <EEPROM.h>
#include <Button.h>
#include <SFE_BMP180.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Wire.h>
#include <SPI.h>
#include "EEPROMAnything.h"
// If using software SPI (the default case):
#define OLED_MOSI   10
#define OLED_CLK   9
#define OLED_DC    12
#define OLED_CS    11
#define OLED_RESET 13
#define BUTTON1_PIN     4

Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
extern uint8_t Font24x40[];

SFE_BMP180 pressure;
Button button1 = Button(BUTTON1_PIN,BUTTON_PULLDOWN);
boolean longPush = false;
int value, etat = 0;
double QNH, saveQNH;
double temperature, pression, altitude  = 0;
double  altitude2  = 0;
double baseAltitude, saveBaseAltitude = 0;
double lastValue = 0.0;
int eepromAddr = 10;
int encoderPin1 = 2;
int encoderPin2 = 3;
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
long lastencoderValue = 0;
int lastMSB = 0;
int lastLSB = 0;
#define MAX_SAMPLES 20
double samplesBuffer[MAX_SAMPLES];
int indexBfr = 0;
double averagePressure = 0;
boolean bufferReady = false;
int screen = 0; // numero d'ecran
#define NB_SCREENS 1 
boolean flag = false;

/* ------------------------------------ setup ------------------------------------------ */
void setup()   { 
// wdt_enable(WDTO_2S); 
  pinMode(encoderPin1, INPUT); 
  pinMode(encoderPin2, INPUT);  
  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

  button1.releaseHandler(handleButtonReleaseEvents);
  button1.holdHandler(handleButtonHoldEvents,60);

  display.begin(SSD1306_SWITCHCAPVCC);

  // init QNH
  EEPROM_readAnything(eepromAddr, QNH);
if ((QNH)>1050.00 || (QNH )<950.00)
 {
    QNH = 1013.25;
    EEPROM_writeAnything(eepromAddr, QNH);  // QNE
  }
  saveQNH = QNH;
  display.clearDisplay(); 
  display.display();
  delay(100);
  display.clearDisplay(); 
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
 if (!pressure.begin()) {
   display.println("Init fail !");
    display.println("Turn OFF");
   display.display();
   while(1); // Pause forever.
  }
 button1.isPressed();
  screen = 1;
}
/* ------------------------------------ loop ------------------------------------------ */
void loop() 
{
  char status;
  button1.isPressed();
  // get pressure and temperature and calculate altitude 
  status = pressure.startTemperature();
  if (status != 0) 
  {
  delay(status);
  status = pressure.getTemperature(temperature);
  if (status != 0) 
       {
  status = pressure.startPressure(0);
  if (status != 0) 
            {
  delay(status);
  status = pressure.getPressure(pression, temperature);
  if (status != 0) 
                {
  savePressureSample(pression);
  averagePressure = getPressureAverage();
  if (bufferReady) 
                      {
  altitude = pressure.altitude(averagePressure*100, QNH*100);
                      }
                }
            } 
        } 
     }

  if (etat == 0)
  {
   //  init baseAltitude
      if (baseAltitude == 0) { 
      baseAltitude =round (altitude);
      saveBaseAltitude = baseAltitude;
  }
    switch (screen) 
    {
    case 1: // Altitude
          detachInterrupt(0);
   detachInterrupt(1);
        if (QNH != saveQNH) 
        {
          saveQNH = QNH;
          EEPROM_writeAnything(eepromAddr, QNH);
          delay(5);
        } 
      if (lastValue != altitude) 
      {
        altitude=(altitude*3.2808);
        showScreen("ALTITUDE", altitude );
        lastValue = altitude;
        flag=0;
      }  
      break;
    }
  }
 else  
 { // Settings
    display.clearDisplay(); 
    display.setTextSize(3);
    display.setCursor(0,0);
    display.setCursor(0,1);
    display.print(QNH,0); 
    display.setCursor(0,33);
    if (flag == 1)
    {
    display.print(3.2808*baseAltitude,0);
    }
    else
     {
     display.print(3.2808*altitude,0); 
     baseAltitude =round (altitude);
     saveBaseAltitude = baseAltitude;  
     }
    display.setTextSize(2);
    display.print("'");
    display.display(); 
    delay(10);   
    attachInterrupt(0, updateEncoder, CHANGE); 
    attachInterrupt(1, updateEncoder, CHANGE);
  }     
} 

/* -------------------- functions --------------------  */

void updateEncoder()
{
  delay(50);
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  delay(2);
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit
  delay(2);
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
      {
       if ((QNH)>950.00) //end stop
       {
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)  baseAltitude--;//( baseAltitude-2);
   }
  if ((QNH)<1050.00)// end stop
  {
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) baseAltitude++;// (baseAltitude+2);
  }
  lastEncoded = encoded; //store this value for next time

          QNH = pressure.sealevel(pression, baseAltitude);
          flag=1;
      }
  }

// put data on screen
void showScreen(String label, double value) {
     display.clearDisplay(); 
  display.setCursor(0,0);
         display.setTextSize(2);
   display.println("ALT");
  drawFloatValueb(0, 20, value);
  display.display(); 
 }
 
// get pressure sample
void savePressureSample(float pressure) {
  if (indexBfr == MAX_SAMPLES)  {
    indexBfr = 0;
    bufferReady = true;  
  }
  samplesBuffer[indexBfr++] = pressure; 
}

// get average
float getPressureAverage() {
  double sum = 0;
  for (int i =0; i<MAX_SAMPLES; i++) {
    sum += samplesBuffer[i];
  }
  return sum/MAX_SAMPLES;
}

// release button
void handleButtonReleaseEvents(Button &btn) {
  //debugMsg = "Release";
  if (!longPush) {
 if (etat != 0 ) { 
   } 
  //  else { // Change screen
      screen++;
      if (screen > NB_SCREENS) screen = 1;
      lastValue = 0;
   // }
  }
  longPush = false;
}

// long push
void handleButtonHoldEvents(Button &btn) {
  //debugMsg = "Hold";
  longPush = true;
  //screen = 1;
  value = 0;
if (screen == 1 && ++etat > 1) {
    etat = 0;
}
}
// place character x, y
void drawCar(int sx, int sy, int num, uint8_t *font, int fw, int fh, int color) {
  byte row;
  for(int y=0; y<fh; y++) {
    for(int x=0; x<(fw/8); x++) {
      row = pgm_read_byte_near(font+x+y*(fw/8)+(fw/8)*fh*num);
      for(int i=0;i<8;i++) {
        if (bitRead(row, 7-i) == 1) display.drawPixel(sx+(x*8)+i, sy+y, color);
      }
    }
  }
}
  void drawdot(int sx, int sy, int h) {
  display.fillRect(sx, 36, 20, 6, WHITE);
}
//display big char x, y
void drawBigCar(int sx, int sy, int num) {

 
  drawCar(sx, sy, num, Font24x40, 24, 40, WHITE) ;

}
  //display number
void drawFloatValueb(int sx, int sy, double val) {
  char charBuf[8];
  {
    dtostrf(val,3, 1, charBuf); //turn the value into a single precision string

    int nbCar = strlen(charBuf); //number of characters returned gives length to buffer i guess
   
 {
           
      drawBigCar(sx+104, sy, charBuf[nbCar-3]- '0');      //placing the minus sign
      nbCar--;
      if((val<0)&&(val>-10))
      {
      drawdot (sx+78, sy+28,8); 
      return;
      } 
      if (--nbCar > 1) drawBigCar(sx+78, sy, charBuf[nbCar-2]- '0');
      if((val<-10)&&(val>-100))
      {
      drawdot (sx+52, sy+28 ,8); 
      return;
      }
      if (--nbCar > 1) drawBigCar(sx+52, sy, charBuf[nbCar-2]- '0');
      if((val<-100)&&(val>-1000))
      {
      drawdot (sx+26, sy+28,8);
      return; 
      }
      
      if (--nbCar > 1) drawBigCar(sx+26, sy, charBuf[nbCar-2]- '0');
      if (val<-1000)
      {
          drawdot (sx+0, sy+28,8);
      }
  }
 }
}


































