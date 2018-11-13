//
// Date and time functions using a PCF8523 RTC connected via I2C and Wire lib
//
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
//<debuuging defines>
#define   _DEBUG_ // for Serial1 debugging
//#undef    _DEBUG_
//</debuging defines>
#define _TIMESTAMP_PER_MINUTE_
//#undef  _TIMESTAMP_PER_MINUTE_
#define _TIMESTAMP_PER_POWERUP_
#undef _TIMESTAMP_PER_POWERUP_
#define RXEN            LOW
#define RXDIS           HIGH
#define TXEN            HIGH
#define TXDIS           LOW

//<pin defines>
#define RS485RXEN       2 // LOW to enable
#define RS485TXEN       3 // HIGH to enable
#define DEBUG_HEARTBEAT 9 // tracking millis() accuracy
#define B1OXYGEN        A0
#define B2OXYGEN        A3
#define NOTUSED         A2
#define TPOS            A1
#define ERRORLED_RED    5 // pwm mode
#define ERRORLED_GREEN  6 // pwm mode
#define HEAT_1          7 // digital input
#define HEAT_2          8 // digital input
//</pin defines>
//<led defines>
#define OFF             0
#define RED_ON          32
#define GREEN_ON        127
//</led defines>

RTC_PCF8523 rtc;

unsigned long startmillis = 0;
unsigned long currentmillis = 0;
//  used to timestamp output to file every minute
#ifdef  _TIMESTAMP_PER_MINUTE_
unsigned char loopcount = 0;
#endif
//  cleared first trip through the loop
//  used to time stamp the car being started
unsigned char powerup = 1; 

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

void setup () {

  // put your setup code here, to run once:
  //
  // first set adc reference to external to 
  analogReference(EXTERNAL);       // 2.5 Volt reference used
  // disable input buffers on ADC pins,
  // per datasheet page 43
  DIDR0 |= _BV(ADC3D) | _BV(ADC2D) | _BV(ADC1D) | _BV(ADC0D); //adc0 through 3

  // <set all unused pins to OUTPUT >  
  // digital pins
  int i;  
  const unsigned char unused_pins[1]{9};
  const size_t numunused = sizeof(unused_pins)/sizeof(unused_pins[0]);

    for(i=0;i<numunused;i++){
      digitalWrite(unused_pins[i],LOW);
      pinMode(unused_pins[i],OUTPUT);
      }
  // </set all unused pins to output and LOW>

  // <digital pins in use>
    
    
    pinMode (DEBUG_HEARTBEAT,OUTPUT);
    pinMode (RS485TXEN,OUTPUT);
    pinMode (RS485RXEN,OUTPUT);
    pinMode(HEAT_1,INPUT);
    pinMode(HEAT_2,INPUT);
  // </digital pins in use>
  
  //<pwm pins in use>
  analogWrite(ERRORLED_RED,OFF);
  analogWrite(ERRORLED_GREEN,OFF);
  //</pwm pins in use>

  //delay(20); // a short delay to let things stabilize
  #ifdef  _DEBUG_
  
  digitalWrite(RS485TXEN,TXEN);
  digitalWrite(RS485TXEN,RXDIS);
  Serial1.begin(57600);
  Serial1.print("Initializing SD card...");
  #endif
  // see if the card is present and can be initialized:
  if (!SD.begin(10)) {
    #ifdef  _DEBUG_
    Serial1.println("Card failed, or not present");
    // don't do anything more:
    #endif
    digitalWrite(ERRORLED_GREEN,OFF); // no card inserted
    digitalWrite(ERRORLED_RED,RED_ON);
    while (1); // hang till power down and card inserted
    
  }
  #ifdef _DEBUG_
  Serial1.println("card initialized.");
  #endif
  String  StartString = "";
  StartString += "Startup";
  digitalWrite(ERRORLED_RED,OFF); // all is well, get on with it
  digitalWrite(ERRORLED_GREEN,GREEN_ON);
  digitalWrite(DEBUG_HEARTBEAT,LOW);

 
  while (!Serial1) {
    delay(1);  // for Leonardo/Micro/Zero
    
  }

  Serial1.begin(57600);
  if (! rtc.begin()) {
    Serial1.println("Couldn't find RTC");
    while (1);
  }

  if (! rtc.initialized()) {
    Serial1.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  startmillis = millis();
  }

void loop () {
  
  int notused, tpos, b1oxygen, b2oxygen, temp;
    DateTime now = rtc.now();
   
   String dataString = "";
currentmillis = (millis()-startmillis);

#ifdef  _TIMESTAMP_PER_MINUTE_
unsigned long myunixtime = now.unixtime();
  if(loopcount ==0){
  dataString += String(myunixtime);
  dataString += ";\r\n";
  }

#endif

#ifdef  _TIMESTAMP_PER_POWERUP_ 
if(powerup >=0){
  unsigned long myunixtime = now.unixtime();
  dataString += String(myunixtime);
  dataString += ";\r\n";
  powerup = 0;
}
#endif
  
dataString += String(millis()-startmillis);
dataString += String(",");

 
  // <get us some heater info>
  dataString += String(digitalRead(HEAT_1));
  dataString += String(",");
  dataString += String(digitalRead(HEAT_2));
  dataString += String(",");
  temp = 0;
  // </get us some heater info>

  
  //<get us some o2 info (gain of 2(1))>
  temp += analogRead(B1OXYGEN);
  temp += analogRead(B1OXYGEN);
  temp += analogRead(B1OXYGEN);
  temp += analogRead(B1OXYGEN);
  temp = temp >> 2;  
  b1oxygen = map(temp,0,1023,0,1250);
  
  dataString += String(b1oxygen);
  dataString += String(",");
  //</get us some o2 info (gain of 2(1))>
  
  //<get us some o2 info (gain of 2(2))>
  temp = 0;
  // get 4 samples and then average them
  temp += analogRead(B2OXYGEN);
  temp += analogRead(B2OXYGEN);
  temp += analogRead(B2OXYGEN);
  temp += analogRead(B2OXYGEN);
  temp = temp >> 2;
  b2oxygen = map(temp,0,1023,0,1250);
  
  dataString += String(b2oxygen);
  dataString += String(",");
  //</get us some o2 info (gain of 2(2))>
  
  // <get us some throttle info (gain = 1/2(1))> 
  temp = 0;
  // get 4 samples and then average them
  temp += analogRead(TPOS);
  temp += analogRead(TPOS);
  temp += analogRead(TPOS);
  temp += analogRead(TPOS);
  temp = temp >> 2;    
  tpos = map(temp,0,1023,0,5000);
  
  dataString += String(tpos);
  dataString += String(",");
  // </get us some throttle info>
  
  // <get us some unused channel (gain = 1/2(2))>
  temp = 0;
  // get 4 samples and then average them
  temp += analogRead(NOTUSED);
  temp += analogRead(NOTUSED);
  temp += analogRead(NOTUSED);
  temp += analogRead(NOTUSED);
  temp = temp >> 2;   
  notused = map(temp,0,1023,0,5000);
  
  dataString += String(notused); // last channel no comma appended
// </get us some unused channel (gain = 1/2(2))>

  // <SD card setup>
 
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.csv", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
     #ifdef _DEBUG_
    // print to the Serial1 port too:
    Serial1.println(dataString);
     #endif
  }
  // if the file isn't open, pop up an error:
  else {
    #ifdef _DEBUG_
    Serial1.println("error opening datalog.csv");
    #endif
    analogWrite (ERRORLED_GREEN,GREEN_ON); // yellow if file error
    analogWrite (ERRORLED_RED,RED_ON);
  }
  
  //should give us ~10 hertz sample rate
  // with _DEBUG_ enabled
  /*
  #ifdef  _DEBUG_
    delay(40);
  #endif
  
  //should give us ~10 hertz sample rate
  // with _DEBUG_ disabled 
   
  #ifndef _DEBUG_ 
    delay (100);
  #endif  
  */

while (millis()-currentmillis < 100); // do every 100 millis aka 10 sample / sec.
#ifdef _TIMESTAMP_PER_MINUTE_ 
if(loopcount <=600){
  loopcount++; 
  }else {
    loopcount = 0; //reset each minute 
  // see beginning of loop for the usage of this}
  }
#endif  
digitalWrite(DEBUG_HEARTBEAT,!digitalRead(DEBUG_HEARTBEAT));

}    
