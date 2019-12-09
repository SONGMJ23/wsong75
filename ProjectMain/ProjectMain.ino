#include "src/MAX30105.h"
#include "src/spo2_algorithm.h"
#include "src/Adafruit_MPU6050.h"
#include "src/Adafruit_Sensor.h"
#include <SPI.h>
#include <Wire.h>
#include "src/SSD1306Ascii.h"
#include "src/SSD1306AsciiAvrI2c.h"

int LastTime=0;
int ThisTime;
int time_start;
unsigned long time_cycle;
bool BPMTiming=false;
bool BeatComplete=false;
int BPM=0;
bool state_0 = true;
bool state_1 = false;
bool state_2 = false;
bool state_3 = false;
float resp_rate=0;
float cycles=0;
#define UpperThreshold 560
#define LowerThreshold 530
float first_threshold = 6.8;
float second_threshold = 6.2;

MAX30105 particleSensor;
Adafruit_MPU6050 mpu1;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)

SSD1306AsciiAvrI2c oled;

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[50]; //infrared LED sensor data
uint16_t redBuffer[50];  //red LED sensor data
#else
uint32_t irBuffer[50]; //infrared LED sensor data
uint32_t redBuffer[50];  //red LED sensor data
#endif

const int MAX_BRIGHTNESS  = 255;
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid

byte pulseLED = 9; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

void setup()
{
  // Initialize the serial communication with the ECG sensor
  Serial.begin(115200); // initialize serial communication at 115200 bits per second: 

  oled.begin(&Adafruit128x64, 0x3C, OLED_RESET);
  oled.setFont(Adafruit5x7);
  oled.clear();
  oled.set2X();
  oled.println(F("Opiod\nOverdose\nMonitor!"));
  
  pinMode(10, INPUT); // Setup for leads off detection LO +
  pinMode(11, INPUT); // Setup for leads off detection LO -
  
  // Initialize the serial communication with MPU6050
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);
  Serial.println(F("Adafruit MPU6050 test!"));

  // Initialize the serial communication with gyro 1
  if (!mpu1.begin(MPU6050_I2CADDR_DEFAULT, &Wire, 0)) {
    Serial.println(F("Failed to find MPU6050 1 chip"));
    while (1) {
      delay(10);
    }
  }
  Serial.println(F("MPU6050 1 Found!"));

//  // Initialize the serial communication with gyro 2
//  if (!mpu2.begin(0x69, &Wire, 1)) {
//    Serial.println(F("Failed to find MPU6050 2 chip"));
//    while (1) {
//      delay(10);
//    }
//  }
//  Serial.println(F("MPU6050 2 Found!"));

  mpu1.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu1.setGyroRange(MPU6050_RANGE_500_DEG);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  Serial.read();

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  time_start = millis();

}

void loop()
{
  if ((digitalRead(10) == 1)||(digitalRead(11) == 1))
  {
    Serial.println('!'); // No current inputs!
  }
  else
  {
    for (int a = 0; a < 200; a++)
    {
      ThisTime=millis();
      int value=analogRead(A0);
      int b=60-(value/16);

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
      delay(10);
    }
  }

  //Pulse ox functioning block
  bufferLength = 50;
  //read the first 100 samples, and determine the signal range
  for (int i = 0 ; i < bufferLength ; i++)
  {
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
  }

  //calculate SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2);

//  //Continuously taking samples from MAX30102.  SpO2 are calculated every 1 second
//  //dumping the first 25 sets of samples in the memory and shift the last 25 sets of samples to the top
//  for (byte i = 25; i < 100; i++)
//  {
//    redBuffer[i - 25] = redBuffer[i];
//    irBuffer[i - 25] = irBuffer[i];
//  }
//
//  //take 25 sets of samples before calculating the heart rate.
//  for (byte i = 75; i < 100; i++)
//  {
//    while (particleSensor.available() == false) //do we have new data?
//      particleSensor.check(); //Check the sensor for new data
//
//    digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read
//
//    redBuffer[i] = particleSensor.getRed();
//    irBuffer[i] = particleSensor.getIR();
//    particleSensor.nextSample(); //We're finished with this sample so move to next sample
//
//    //send samples and calculation result to terminal program through UART
//    Serial.print(F("red="));
//    Serial.print(redBuffer[i], DEC);
//    Serial.print(F(", ir="));
//    Serial.print(irBuffer[i], DEC);
//
//    Serial.print(F(", SPO2="));
//    Serial.print(spo2, DEC);
//
//    Serial.print(F(", SPO2Valid="));
//    Serial.println(validSPO2, DEC);
//  }

//  //After gathering 25 new samples recalculate SP02
//  maxim_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2);

//    /* Take a new reading */
    mpu1.read();
//    /* Get new sensor events with the readings */
    sensors_event_t a1, g1, temp1;   
    /* Print out the values */
    for (int i = 0; i < 100; i++)
    {
      mpu1.getEvent(&a1, &g1, &temp1);
      float acc = sqrt(a1.acceleration.x * a1.acceleration.x + a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z);
      float ang = sqrt(g1.gyro.x * g1.gyro.x + g1.gyro.y * g1.gyro.y + g1.gyro.z * g1.gyro.z);
      Serial.println(ang);

      if (state_2 && ((millis() - time_cycle) > 500)) {
//        Serial.print(F("Time elapsed: "));
//        Serial.println(millis() - time_cycle);
        //Serial.println(F("BACK TO STATE 0"));
        state_2 = false;
        state_0 = true;
        time_cycle = 0;
      }
      if (state_0 && (ang >= 10.0)) {
        state_1 = true;
        state_0 = false;
        //Serial.println(F("Moved into state 1"));
        
      }
      else if (state_1 && ang <= first_threshold) {
        state_2 = true;
        state_1 = false;
        //Serial.println(F("Moved into state 2"));
        time_cycle = millis();
      }
      else if (state_2 && ang >= second_threshold) {
        state_3 = true;
        state_2 = false;
        //Serial.println(F("Moved into state 3"));
      }
      else if (state_3 && ang <= second_threshold) {
        state_0 = true;
        state_3 = false;
        
        Serial.println(F("Moved into state 0"));
        Serial.println(F("Cycle incremented"));
        cycles++;
      }
      delay(10);
    }
    resp_rate = (60000*cycles)/(millis() - time_start);


  if ((BPM > 85) || ((validSPO2 != 0) && (spo2 < 97)))
  {
    tone(8,1000,250);
  }
  
  /*OLED display*/
  oled.clear();
  oled.set2X();
  oled.print(F("BPM: "));
  oled.println(BPM);

  oled.set1X();
  oled.print(F("\n\n\nOxygen Sat: "));
  if (validSPO2 != 0)
  {
    oled.println(spo2, DEC);
  }
  oled.print(F("\nResp Rate: "));
  oled.println(resp_rate);
  delay(1);
}
