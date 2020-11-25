#include "rosalyn-tx.h"


TSystem System;
TRosalynTx RosalynTx;

void TRosalynTx::Loop()
{
  if(( HAL_GetTick() % 1000 ) == 0 )
  {
    UsbPrintf( "%4u %4u %4u %4u %4u %4u %4u %4u\n",
      Data[ 0 ], Data[ 1 ], Data[ 2 ], Data[ 3 ],
      Data[ 4 ], Data[ 5 ], Data[ 6 ], Data[ 7 ] );
  }
}

void TRosalynTx::Setup()
{
  UsbPrintf( "RosalynTX\n" );
}

void TRosalynTx::TIM_IC_CaptureCallback( TIM_HandleTypeDef *const htim )
{
  if( htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4 )
  {
    uint32_t const NewIC4 = HAL_TIM_ReadCapturedValue( htim, TIM_CHANNEL_4 );

    uint32_t Diff = 0;
    if( NewIC4 > OldIC4 )
    {
      Diff = NewIC4 - OldIC4;
    }
    else if( NewIC4 < OldIC4 )
    {
      Diff = (( 0xFFFFFFFF - OldIC4 ) + NewIC4 ) + 1;
    }

    Diff /= 48; // Convert to milliseconds
    OldIC4 = NewIC4;

    if(( Diff >= MinPPM ) && ( Diff <= MaxPPM ) && ( Index < NoOfPPMs ))
    {
      Data[ Index++ ] = Diff;
    }
    else
    {
      Index = 0;
    }
  }
}

TRosalynTx::TRosalynTx() :
  Failsafe( true ),
  Data(),
  Index( 0 ),
  OldIC4( 0 )
{
}

extern "C" void RosalynTxLoop()
{
  RosalynTx.Loop();
}

extern "C" void RosalynTxSetup()
{
  RosalynTx.Setup();
}

extern "C" void HAL_TIM_IC_CaptureCallback( TIM_HandleTypeDef *const htim )
{
  RosalynTx.TIM_IC_CaptureCallback( htim );
}
