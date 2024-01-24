#ifndef ADXL345_IMU_BROADCASTER__ADXL345_H_
#define ADXL345_IMU_BROADCASTER__ADXL345_H_

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
extern "C" {
	#include <linux/i2c-dev.h>
	#include <i2c/smbus.h>
}

#include "adxl345_defines.h"

class ADXL345
{
public:

  ADXL345(){}
  ADXL345(std::string device, int8_t addr);
  ~ADXL345();

  void setup(std::string device, int8_t addr);
  void getAccel(float &x, float &y, float &z);
  
private:	  	
  int _f_dev;
  float _accel[3];
};

#endif // ADXL345_IMU_BROADCASTER__ADXL345_H_