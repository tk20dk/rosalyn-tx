#ifndef ROSALYN_TX_H__
#define ROSALYN_TX_H__

#include "main.h"
#include "system.h"
#include "sx1268.h"


extern "C" SPI_HandleTypeDef hspi1;

class TRosalynTx
{
  static uint32_t const MinPPM = 900;
  static uint32_t const MaxPPM = 2100;
  static uint32_t const NoOfPPMs = 8;

public:
  TRosalynTx();

  void Loop();
  void Setup();
  void RadioEvent( TRadioEvent const Event );
  void HAL_GPIO_EXTI_Callback( uint16_t const GPIO_Pin );
  void HAL_TIM_IC_CaptureCallback( TIM_HandleTypeDef *const htim );

private:
  bool Failsafe;
  bool RadioFlag;
  uint32_t Data3[ NoOfPPMs ];
  uint32_t Data4[ NoOfPPMs ];
  uint32_t Index3;
  uint32_t Index4;
  uint32_t OldIC3;
  uint32_t OldIC4;
  TSx1268 Radio;
};

#endif // ROSALYN_TX_H__
