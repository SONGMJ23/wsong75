#include "Project.h"

#define debug_ 1

uint16_t value[SAMPLE_NUMBER] = { 0 }; // Initialize the sampling point value.

uint16_t Project::getECG(uint8_t pin) 
{
  valueCount_ ++;
  if(valueCount_ >= SAMPLE_NUMBER)
  {
    valueCount_ = 0;
  }    
  value[valueCount_] = analogRead(pin); // Read the ECG analog from a certain pin
  return (value[valueCount_]);
}
