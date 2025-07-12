#ifndef SYSCONFIG_H 
#define SYSCONFIG_H
#include <stdint.h>

void SysClockConfig (void);
void TIM6Config(void);
void delay_us (uint16_t us);
void TIM7Config(void);
void TIM7SetCounter(uint32_t value);
int TIM7GetCounter();

#endif