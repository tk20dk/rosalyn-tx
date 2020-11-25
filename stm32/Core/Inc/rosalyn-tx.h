#ifndef ROSALYN_TX_H__
#define ROSALYN_TX_H__

#include "main.h"
#include "system.h"


class TRosalynTx
{
  static uint32_t const MinPPM = 900;
  static uint32_t const MaxPPM = 2100;
  static uint32_t const NoOfPPMs = 8;

public:
  TRosalynTx();

  void Loop();
  void Setup();
  void TIM_IC_CaptureCallback( TIM_HandleTypeDef *const htim );

private:
  bool Failsafe;
  uint32_t Data[ NoOfPPMs ];
  uint32_t Index;
  uint32_t OldIC4;
};

#endif // ROSALYN_TX_H__
