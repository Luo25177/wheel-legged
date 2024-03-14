#pragma once

#include "misc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"

extern vu32 GlobalTimer;

void Tim2Init();
void Tim3Init();
