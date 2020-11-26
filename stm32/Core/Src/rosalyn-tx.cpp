#include "rosalyn-tx.h"


TSpi Spi( hspi1 );
TSystem System;
TRosalynTx RosalynTx;

void TRosalynTx::Loop()
{
  if( RadioFlag )
  {
    RadioFlag = false;
    Radio.Interrupt();
  }

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

void TRosalynTx::RadioEvent( TRadioEvent const Event )
{
  uint8_t Buffer[ 256 ];

  if( Event == TRadioEvent::RxDone )
  {
    auto const Snr = Radio.GetSnr();
    auto const Rssi = Radio.GetRssi();
    auto const Length = Radio.ReadPacket( Buffer, sizeof( Buffer ));

    UsbPrintf( "433 Rssi:%4d Snr:%3d.%u Len:%u Length error\n", Rssi, Snr / 10, abs(Snr) % 10, Length );

    Radio.Receive();
  }

  if( Event == TRadioEvent::TxDone )
  {
    Radio.Receive();
  }

  if( Event == TRadioEvent::Timeout )
  {
  }

  if( Event == TRadioEvent::CrcError )
  {
    auto const Snr = Radio.GetSnr();
    auto const Rssi = Radio.GetRssi();
    auto const Length = Radio.ReadPacket( Buffer, sizeof( Buffer ));

    UsbPrintf( "433 Rssi:%4d Snr:%3d.%u Len:%u CRC Error\n", Rssi, Snr / 10, abs(Snr) % 10, Length );
  }

  if( Event == TRadioEvent::NoCrc )
  {
    auto const Snr = Radio.GetSnr();
    auto const Rssi = Radio.GetRssi();
    auto const Length = Radio.ReadPacket( Buffer, sizeof( Buffer ));

    UsbPrintf( "433 Rssi:%4d Snr:%3d.%u Len:%u No CRC\n", Rssi, Snr / 10, abs(Snr) % 10, Length );
  }
}

void TRosalynTx::Setup()
{
  UsbPrintf( "RosalynTX\n" );

  if( Radio.Setup( System.Config.Modulation[ 2 ], System.Config.TxPower, System.Config.Channel ))
  {
    Radio.Receive();
  }
}

void TRosalynTx::HAL_GPIO_EXTI_Callback( uint16_t const GPIO_Pin )
{
  switch( GPIO_Pin )
  {
    case RADIO_DIO1_Pin:
    {
      RadioFlag = true;
    }
    break;

    default:
    {
    }
    break;
  }
}

void TRosalynTx::HAL_TIM_IC_CaptureCallback( TIM_HandleTypeDef *const htim )
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
  RadioFlag( false ),
  Data3(),
  Data4(),
  Index3( 0 ),
  Index4( 0 ),
  OldIC3( 0 ),
  OldIC4( 0 ),
  Radio(
    433050000,
	RADIO_NSS_GPIO_Port,
	RADIO_NSS_Pin,
	RADIO_NRST_GPIO_Port,
	RADIO_NRST_Pin,
	RADIO_BUSY_GPIO_Port,
	RADIO_BUSY_Pin,
	RADIO_RXEN_GPIO_Port,
	RADIO_RXEN_Pin,
	RADIO_TXEN_GPIO_Port,
	RADIO_TXEN_Pin,
    std::bind( &TRosalynTx::RadioEvent, this, std::placeholders::_1 ))
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

extern "C" void HAL_GPIO_EXTI_Callback( uint16_t const GPIO_Pin )
{
  RosalynTx.HAL_GPIO_EXTI_Callback( GPIO_Pin );
}

extern "C" void HAL_TIM_IC_CaptureCallback( TIM_HandleTypeDef *const htim )
{
  RosalynTx.HAL_TIM_IC_CaptureCallback( htim );
}
