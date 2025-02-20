#include "stm32f7xx_hal.h"

volatile uint64_t udp_tick = 0;
uint32_t tick_index = 23283;
TIM_HandleTypeDef *TIM;

void  timeManagerInit(TIM_HandleTypeDef *Timer)
{
	TIM = Timer;
	HAL_TIM_Base_Start_IT(TIM);
}

void time_keeper_interrupt_routine(void)
{
	tick_index++;
}

uint64_t Tick(void)
{
	return 10*(((uint64_t)(tick_index - 1) << 32) | ((uint64_t)TIM->Instance->CNT));
}
