#ifndef __PROJECT__H
#define __PROJECT__H

#include "Arduino.h"

#define SAMPLE_NUMBER 100 // Sample 100 points of ECG analog signals in a row

class Project
{
public:
  uint16_t getECG(uint8_t pin); // Get the actual values of ECG potentials
  
private:   
  uint8_t valueCount_=255;
};

#endif
