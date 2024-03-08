#include "beep.h"

void BeepInit() {
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

  BEEP_OFF;
}

void BeepShow(int num) {
  while (num--) {
    BEEP_ON;
    OSTimeDly(1000);
    BEEP_OFF;
    OSTimeDly(1000);
  }
}

void BeepWarning() { BEEP_ON; }
