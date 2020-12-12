#ifndef ROSALYN_TX_H__
#define ROSALYN_TX_H__

#include "nvdata.h"
#include "system.h"
#include "sx1268.h"
#include "aes-crypto.h"


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
  void TransmitPPM();
  void RadioEvent( TRadioEvent const Event );
  void HAL_GPIO_EXTI_Callback( uint16_t const GPIO_Pin );
  void HAL_TIM_IC_CaptureCallback( TIM_HandleTypeDef *const htim );

private:
  TNvData NvData;
  bool PpmFlag;
  bool RadioFlag;
  uint16_t Data3[ NoOfPPMs ];
  uint16_t Data4[ NoOfPPMs ];
  uint32_t Index3;
  uint32_t Index4;
  uint32_t OldIC3;
  uint32_t OldIC4;
  TSx1268 Radio;
  TAesCrypto AesCrypto;
};

#endif // ROSALYN_TX_H__
