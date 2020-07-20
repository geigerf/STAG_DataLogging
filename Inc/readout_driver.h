#ifndef _READOUT_DRIVER_H_
#define _READOUT_DRIVER_H_

// include everything that is needed from main
#include "main.h"

// defines
// #define DATA_SIZE 1024 
#define P_EN_PIN P_EN_Pin
#define P_EN_GPIO_PORT P_EN_GPIO_Port

// function prototypes
void En_ReadoutCircuit();
void Dis_ReadoutCircuit();
HAL_StatusTypeDef Read_Frame(uint16_t *pDstBuffer); 

#endif /*__READOUT_DRIVER_H */