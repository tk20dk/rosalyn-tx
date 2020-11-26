#include "rosalyn-tx.h"


TSystem System;
TRosalynTx RosalynTx;

void TRosalynTx::Loop()
{
  if(( HAL_GetTick() % 1000 ) == 0 )
  {
    UsbPrintf( "3: %4u %4u %4u %4u %4u %4u %4u %4u\n",
      Data3[ 0 ], Data3[ 1 ], Data3[ 2 ], Data3[ 3 ],
      Data3[ 4 ], Data3[ 5 ], Data3[ 6 ], Data3[ 7 ] );
    UsbPrintf( "4: %4u %4u %4u %4u %4u %4u %4u %4u\n",
      Data4[ 0 ], Data4[ 1 ], Data4[ 2 ], Data4[ 3 ],
      Data4[ 4 ], Data4[ 5 ], Data4[ 6 ], Data4[ 7 ] );
  }
}

void TRosalynTx::Setup()
{
  UsbPrintf( "RosalynTX\n" );
}

void TRosalynTx::TIM_IC_CaptureCallback( TIM_HandleTypeDef *const htim )
{
  if( htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3 )
  {
    uint32_t const NewIC3 = HAL_TIM_ReadCapturedValue( htim, TIM_CHANNEL_3 );

    uint32_t Diff3 = 0;
    if( NewIC3 > OldIC3 )
    {
      Diff3 = NewIC3 - OldIC3;
    }
    else if( NewIC3 < OldIC3 )
    {
      Diff3 = (( 0xFFFFFFFF - OldIC3 ) + NewIC3 ) + 1;
    }

    Diff3 /= 48; // Convert to milliseconds
    OldIC3 = NewIC3;

    if(( Diff3 >= MinPPM ) && ( Diff3 <= MaxPPM ) && ( Index3 < NoOfPPMs ))
    {
      Data3[ Index3++ ] = Diff3;
    }
    else
    {
      Index3 = 0;
    }
  }
  else if( htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4 )
  {
    uint32_t const NewIC4 = HAL_TIM_ReadCapturedValue( htim, TIM_CHANNEL_4 );

    uint32_t Diff4 = 0;
    if( NewIC4 > OldIC4 )
    {
      Diff4 = NewIC4 - OldIC4;
    }
    else if( NewIC4 < OldIC4 )
    {
      Diff4 = (( 0xFFFFFFFF - OldIC4 ) + NewIC4 ) + 1;
    }

    Diff4 /= 48; // Convert to milliseconds
    OldIC4 = NewIC4;

    if(( Diff4 >= MinPPM ) && ( Diff4 <= MaxPPM ) && ( Index4 < NoOfPPMs ))
    {
      Data4[ Index4++ ] = Diff4;
    }
    else
    {
      Index4 = 0;
    }
  }
}

TRosalynTx::TRosalynTx() :
  Failsafe( true ),
  Data3(),
  Data4(),
  Index3( 0 ),
  Index4( 0 ),
  OldIC3( 0 ),
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
