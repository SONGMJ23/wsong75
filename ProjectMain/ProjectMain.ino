//#include "Adafruit_ssd1306syp.h" //OLED lib
#include "src/MAX30105.h"
#include "src/spo2_algorithm.h"
#include "src/Adafruit_MPU6050.h"
#include "src/Adafruit_Sensor.h"
#include <SPI.h>
#include <Wire.h>

int a=0;
int lasta=0;
int lastb=0;
int LastTime=0;
int ThisTime;
bool BPMTiming=false;
bool BeatComplete=false;
int BPM=0;
#define UpperThreshold 560
#define LowerThreshold 530

MAX30105 particleSensor;
//Adafruit_ssd1306syp oled(9,8);

void setup()
{
  // Initialize the serial communication with the ECG sensor
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  pinMode(10, INPUT); // Setup for leads off detection LO +
  pinMode(11, INPUT); // Setup for leads off detection LO -
  
  // Initialize the serial communication with Pulse ox
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }
  
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  
  //oled.initialize();
}

void loop()
{
  //New ECG functioning block taking only 8% of memory or so.
  if ((digitalRead(10) == 1)||(digitalRead(11) == 1))
  {
    Serial.println('!'); // No current inputs!
  }
  else
  {
    // New version of HR (BPM) calculation
    if(a>127)
    {
      a=0;
      lasta=a;
    }
    
    ThisTime=millis();
    int value=analogRead(A0);
    int b=60-(value/16);
    lastb=b;
    lasta=a;

    if(value>UpperThreshold)
    {
      if(BeatComplete)
      {
        BPM=ThisTime-LastTime;
        BPM=int(60/(float(BPM)/1000));
        BPMTiming=false;
        BeatComplete=false;
      }
      if(BPMTiming==false)
      {
        LastTime=millis();
        BPMTiming=true;
      }
    }
    if((value<LowerThreshold)&(BPMTiming))
    {
      BeatComplete=true;
    }
    Serial.println(BPM);
    a++;
    
    //Simplified version of spO2 and gyro. Add your codes below.
    
    
    //Conditions for buzzing. You can add any condition you want, probably regarding the spO2 & gyro.
    if (BPM > 85)
    {
      tone(8,1000,250); //Pin 8, change 1000 and 250 to modify the melody.
    }
  }
  
  /*OLED display*/
  //oled.clear();
  //oled.setTextSize(2);
  //oled.setTextColor(WHITE);
  //oled.setCursor(0,0);
  //oled.println(BPM);
  //oled.update();
}
