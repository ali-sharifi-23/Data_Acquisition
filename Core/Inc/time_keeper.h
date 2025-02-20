/*
 * time_keeper.h
 *
 *  Created on: Feb 19, 2025
 *      Author: ali
 */

#ifndef INC_TIME_KEEPER_H_
#define INC_TIME_KEEPER_H_
#include "stm32f7xx_hal.h"

void  timeManagerInit(TIM_HandleTypeDef *TIM);
void time_keeper_interrupt_routine(void);
uint64_t Tick(void);

#endif /* INC_TIME_KEEPER_H_ */
