#ifndef __PROJECT__H
#define __PROJECT__H

#include "Arduino.h"

#define SAMPLE_NUMBER 100

class Project
{
public:
  uint16_t getECG(uint8_t pin);
  
private:   
  uint8_t valueCount_=255;
};

#endif
