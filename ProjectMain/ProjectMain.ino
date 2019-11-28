#include "src/MAX30105.h"
#include "src/spo2_algorithm.h"

#include "src/Adafruit_MPU6050.h"
#include "src/Adafruit_Sensor.h"
#include <Wire.h>
#include "Adafruit_ssd1306syp.h" //See the "ssd1306" directory

MAX30105 particleSensor;
Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;
Adafruit_ssd1306syp oled(9,8);

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
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

int counter = 0;

void setup()
{
  // Initialize the serial communication with the ECG sensor
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  while (!Serial)
    delay(10); 

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
  Serial.println("MPU6050 1 Found!");

    //// Initialize the serial communication with gyro
  if (!mpu2.begin(0x69, &Wire, 1)) {
    Serial.println(F("Failed to find MPU6050 2 chip"));
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 2 Found!");

  mpu1.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu2.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu1.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu2.setGyroRange(MPU6050_RANGE_500_DEG);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }
  
  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  while (Serial.available() == 0) ; //wait until user presses a key
  Serial.read();

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  
  //OLED initialize
  oled.initialize();
}

void loop() 
{ 
  // Pulse ox functioning block
  bufferLength = 50; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  
  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  //dumping the first 25 sets of samples in the memory and shift the last 25 sets of samples to the top
  for (byte i = 12; i < 50; i++)
  {
    redBuffer[i - 12] = redBuffer[i];
    irBuffer[i - 12] = irBuffer[i];
  }

  //take 25 sets of samples before calculating the heart rate.
  for (byte i = 38; i < 50; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    //send samples and calculation result to terminal program through UART
    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.print(irBuffer[i], DEC);

    Serial.print(F(", HR="));
    Serial.print(heartRate, DEC);

    Serial.print(F(", HRvalid="));
    Serial.print(validHeartRate, DEC);

    Serial.print(F(", SPO2="));
    Serial.print(spo2, DEC);

    Serial.print(F(", SPO2Valid="));
    Serial.println(validSPO2, DEC);
  }

  //After gathering 25 new samples recalculate HR and SP02
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
      
  // Gyro functioning block
  /* Take a new reading */
  mpu1.read();
  mpu2.read();

  /* Get new sensor events with the readings */
  sensors_event_t a1, g1, temp1, a2, g2, temp2;
  mpu1.getEvent(&a1, &g1, &temp1);
  mpu2.getEvent(&a2, &g2, &temp2);

  /* Print out the values */
  Serial.print("Acceleration X Gyro 1: ");
  Serial.print(a1.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a1.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a1.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X Gyro 1: ");
  Serial.print(g2.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g2.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g2.gyro.z);
  Serial.println(" deg/s");

  Serial.print("Temperature Gyro 1: ");
  Serial.print(temp1.temperature);
  Serial.println(" degC");
  Serial.println("");

  /* Print out the values */
  Serial.print("Acceleration X Gyro 2: ");
  Serial.print(a2.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a2.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a2.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X Gyro 2: ");
  Serial.print(g2.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g2.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g2.gyro.z);
  Serial.println(" deg/s");

  Serial.print("Temperature Gyro 2: ");
  Serial.print(temp2.temperature);
  Serial.println(" degC");
  Serial.println("");
  
  //Buzzer & OLED display demo
  if (validSPO2 > 0.1)
  {
    tone(8,1000,250); //Buzzing
  }
  oled.setTextSize(2);
  oled.setTextColor(WHITE);
  oled.setCursor(0,0);
  oled.println("validSPO2:");
  oled.println(validSPO2);
  oled.update();
  //delay(20); // Interval of display
}
