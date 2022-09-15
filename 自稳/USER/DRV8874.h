#include "stdio.h"
#include "stdint.h"	



void DRV8874_Init();
void nSLEEP_set(uint8_t state);
void EN_set(uint8_t state);
void PH_set(uint8_t state);
void nFAULT_set(uint8_t state);
void motor_control(uint8_t state,uint8_t direction,int speed);