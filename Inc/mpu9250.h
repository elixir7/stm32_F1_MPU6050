#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>

extern int mpuReady;

void mpuInit(void);
void readData(float *p);


#endif
