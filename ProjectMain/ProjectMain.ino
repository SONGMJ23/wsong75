#include "src/MAX30105.h"
#include "src/spo2_algorithm.h"
#include "src/Adafruit_MPU6050.h"
#include "src/Adafruit_Sensor.h"
#include <SPI.h>
#include <Wire.h>
#include "src/SSD1306Ascii.h"
#include "src/SSD1306AsciiAvrI2c.h"

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
Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)

SSD1306AsciiAvrI2c oled;

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif


const int MAX_BRIGHTNESS  = 255;
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int32_t resp_rate; //resp_value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid

byte pulseLED = 9; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read
double ax, ay, az, gx, gy, gz;
double ax_i, ay_i, az_i, gx_i, gy_i, gz_i;

double timeStep, time_now, timePrev;
double arx, ary, arz, axi, azi, a_total, grx, gry, grz, gsx, gsy, gsz, rx, ry, rz;
double vel, disp;

int counter = 0;
int i;

void setup()
{

  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
//  while (!Serial)
//    delay(10); 


  oled.begin(&Adafruit128x64, 0x3C, OLED_RESET);
  oled.setFont(Adafruit5x7);
  oled.clear();
  oled.set2X();
  oled.println(F("Opiod\nOverdose\nMonitor!"));

  // Initialize the serial communication with the ECG sensor
  pinMode(10, INPUT); // Setup for leads off detection LO +
  pinMode(11, INPUT); // Setup for leads off detection LO -
  
  // Initialize the serial communication with Pulse ox
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  Serial.println(F("Adafruit MPU6050 test!"));

  //// Initialize the serial communication with gyro
  if (!mpu1.begin(MPU6050_I2CADDR_DEFAULT, &Wire, 0)) {
    Serial.println(F("Failed to find MPU6050 1 chip"));
    while (1) {
      delay(10);
    }
  }
  Serial.println(F("MPU6050 1 Found!"));

//    //// Initialize the serial communication with gyro
//  if (!mpu2.begin(0x69, &Wire, 1)) {
//    Serial.println(F("Failed to find MPU6050 2 chip"));
//    while (1) {
//      delay(10);
//    }
//  }
//  Serial.println(F("MPU6050 2 Found!"));

  mpu1.setAccelerometerRange(MPU6050_RANGE_8_G);
  //mpu2.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu1.setGyroRange(MPU6050_RANGE_500_DEG);
  //mpu2.setGyroRange(MPU6050_RANGE_500_DEG);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  //while (Serial.available() == 0) ; //wait until user presses a key
  Serial.read();

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  i = 1;
  time_now = millis();
}

void loop()
{
//  //Pulse ox functioning block
//    bufferLength = 50; //buffer length of 100 stores 4 seconds of samples running at 25sps
//
//  //read the first 100 samples, and determine the signal range
//  for (byte i = 0 ; i < bufferLength ; i++)
//  {
//    while (particleSensor.available() == false) //do we have new data?
//      particleSensor.check(); //Check the sensor for new data
//
//    redBuffer[i] = particleSensor.getRed();
//    irBuffer[i] = particleSensor.getIR();
//    particleSensor.nextSample(); //We're finished with this sample so move to next sample
//    ecg_method();
//
//    Serial.print(F("red="));
//    Serial.print(redBuffer[i], DEC);
//    Serial.print(F(", ir="));
//    Serial.println(irBuffer[i], DEC);
//  }
//
//  //calculate SpO2 after first 100 samples (first 4 seconds of samples)
//  maxim_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2);
//  
//  //Continuously taking samples from MAX30102.  SpO2 are calculated every 1 second
//  //dumping the first 25 sets of samples in the memory and shift the last 25 sets of samples to the top
//  for (byte i = 25; i < 100; i++)
//  {
//    redBuffer[i - 25] = redBuffer[i];
//    irBuffer[i - 25] = irBuffer[i];
//    ecg_method();
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
//    ecg_method();
//  }
//
//  //After gathering 25 new samples recalculate SP02
//  maxim_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2);
      
  // Gyro functioning block
  // set up time for integration
//  timePrev = time_now;
//  time_now = millis();
//  timeStep = (time_now - timePrev) / 1000; // time-step in s
  /* Take a new reading */
  mpu1.read();
  mpu2.read();

  /* Get new sensor events with the readings */
  sensors_event_t a1, g1, temp1, a2, g2, temp2;
  mpu1.getEvent(&a1, &g1, &temp1);
  mpu2.getEvent(&a2, &g2, &temp2);

  /* Print out the values */
  Serial.print(F("Acceleration X Gyro 1: "));
  Serial.print(a1.acceleration.x);
  Serial.print(F(", Y: "));
  Serial.print(a1.acceleration.y);
  Serial.print(F(", Z: "));
  Serial.print(a1.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print(F("Rotation X Gyro 1: "));
  Serial.print(g1.gyro.x);
  Serial.print(F(", Y: "));
  Serial.print(g1.gyro.y);
  Serial.print(F(", Z: "));
  Serial.print(g1.gyro.z);
  Serial.println(F(" deg/s"));
//
//  /* Print out the values */
//  Serial.print(F("Acceleration X Gyro 2: "));
//  Serial.print(a2.acceleration.x);
//  Serial.print(F(", Y: "));
//  Serial.print(a2.acceleration.y);
//  Serial.print(F(", Z: "));
//  Serial.print(a2.acceleration.z);
//  Serial.println(F(" m/s^2"));
//
//  Serial.print(F("Rotation X Gyro 2: "));
//  Serial.print(g2.gyro.x);
//  Serial.print(F(", Y: "));
//  Serial.print(g2.gyro.y);
//  Serial.print(F(", Z: "));
//  Serial.print(g2.gyro.z);
//  Serial.println(F(" deg/s"));

  //initial acceleration and angular velocity
  if (i == 1){
  ax_i = a1.acceleration.x;
  ay_i = a1.acceleration.y;
  az_i = a1.acceleration.z;

  gx_i = g1.gyro.x;
  gy_i = g1.gyro.y;
  gz_i = g1.gyro.z;
  }

  ax = a1.acceleration.x;
  ay = a1.acceleration.y;
  az = a1.acceleration.z;

  gx = g1.gyro.x;
  gy = g1.gyro.y;
  gz = g1.gyro.z;

  //calculate accelerometer angles
  arx = (180/3.141592) * atan((ax - ax_i) / sqrt(square((ay- ay_i)) + square((az - az_i)))); 
  ary = (180/3.141592) * atan((ay- ay_i) / sqrt(square(ax - ax_i) + square((az - az_i))));
  arz = (180/3.141592) * atan(sqrt(square(ay- ay_i) + square(ax - ax_i)) / (az - az_i));

  // set initial values equal to accel values
  if (i == 1) {
    grx = arx;
    gry = ary;
    grz = arz;
  }
  // integrate to find the gyro angle
  else{
    grx = grx + (timeStep * (gx - gx_i));
    gry = gry + (timeStep * (gy - gy_i));
    grz = grz + (timeStep * (gz - gz_i));
  } 
  
  rx = (0.96 * arx) + (0.04 * (gx - gx_i));
  ry = (0.96 * ary) + (0.04 * (gy - gy_i));
  rz = (0.96 * arz) + (0.04 * (gz - gz_i));
  axi = arx - 9.81*cos(ry);
  azi = arz - 9.81*sin(ry);
  a_total = sqrt(square(axi) + square(azi));

  vel = time_now * a_total;
  disp = (time_now * vel)/100000000;
//  Serial.print("Displacement: ");
//  Serial.println(disp, DEC);
  

  
  
  
  

  
  
//  //Conditions for buzzing. You can add any condition you want, probably regarding the spO2 & gyro.
//  if (BPM > 85)
//  {
//    tone(8,1000,250); //Pin 8, change 1000 and 250 to modify the melody.
//  }
  
//  /*OLED display update*/
  oled.clear();
  oled.set2X();
  oled.print(F("\nBPM: "));
  oled.println(BPM);
  if (validSPO2 != 0){
    oled.set1X();
    oled.print(F("\nOxygen Sat: "));
    oled.println(spo2, DEC);
    oled.print(F("\nResp Rate: "));
    oled.println(resp_rate, DEC);
  }

  i++;

  delay(1000);
  
}

void ecg_method() {
  //New ECG functioning block taking only 8% of memory or so.
  if ((digitalRead(10) == 1)||(digitalRead(11) == 1))
  {
    Serial.println(F("No current inputs!")); // No current inputs!
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
    Serial.print(F("Analog value: "));
    Serial.println(value);
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
    Serial.print(F("BPM value: "));
     Serial.println(BPM);
    a++;
  }
}
