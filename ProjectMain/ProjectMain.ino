#include "Adafruit_ssd1306syp.h" //OLED lib

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

//Adafruit_ssd1306syp oled(9,8);

void setup()
{
  // Initialize the serial communication with the ECG sensor
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  pinMode(10, INPUT); // Setup for leads off detection LO +
  pinMode(11, INPUT); // Setup for leads off detection LO -

  //oled.initialize();
}

void loop()
{
  if ((digitalRead(10) == 1)||(digitalRead(11) == 1))
  {
    Serial.println('!'); // No current inputs!
  }
  else
  {
    // ECG functioning block
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

    if (BPM > 85)
    {
      tone(8,1000,250);
    }
    
    a++;
  }
  //oled.clear();
  //oled.setTextSize(2);
  //oled.setTextColor(WHITE);
  //oled.setCursor(0,0);
  //oled.println(BPM);
  //oled.update();
}
