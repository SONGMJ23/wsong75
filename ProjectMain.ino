#include "Project.h"
// We also have to include an "LCD.h" for LCD display depending on what LCD we use

Project project;

void setup()
{
  // Initialize the serial communication with the ECG sensor
  Serial.begin(9600);
  pinMode(10, INPUT); // Setup for leads off detection LO +
  pinMode(11, INPUT); // Setup for leads off detection LO -
  
  // Initialize the serial communication with Pulse ox
  ...
  
  // Initialize the serial communication with gyro
  ...
}

void loop() 
{
  // ECG functioning block
  if ((digitalRead(10) == 1)||(digitalRead(11) == 1))
  {
    Serial.println('!'); // No current inputs!
  }
  else
  {
    static unsigned char count;
    static unsigned int value[SAMPLE_NUMBER];
    value[count] = project.getECG(A0); //ECG values sampled by A0 foot
    count++;
    if (count >= SAMPLE_NUMBER)
    {
      count = 0; // Start a new sampling row
    }
    // ECG waveform's LCD display
    ...
  } 
  
  // Pulse ox functioning block
  ...
      
  // Gyro functioning block
  ...

  delay(20); // Interval of display
}
