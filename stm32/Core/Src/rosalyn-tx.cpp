#include "rosalyn-tx.h"
#include "sbus.h"


TSpi Spi( hspi1 );
TRosalynTx RosalynTx;

void TRosalynTx::Loop()
{
  if( RadioFlag )
  {
    RadioFlag = false;
    Radio.Interrupt();
  }

  if( PpmFlag )
  {
    PpmFlag = false;
    TransmitPPM();
  }

  if(( HAL_GetTick() % 1000 ) == 0 )
  {
    UsbPrintf( "4: %4u %4u %4u %4u %4u %4u %4u %4u\n",
      Data4[ 0 ], Data4[ 1 ], Data4[ 2 ], Data4[ 3 ],
      Data4[ 4 ], Data4[ 5 ], Data4[ 6 ], Data4[ 7 ] );
  }

  if(( HAL_GetTick() % 1000 ) == 0 )
  {
    HmiStatus( true );
    HAL_Delay( 5 );
    HmiStatus( false );
  }
}

void TRosalynTx::TransmitPPM()
{
  TSbusData SbusData;

  SbusData.SetPWM( Data4 );
  auto const SbusFrame = SbusData.Encode();

  uint8_t Buffer[ TSbusFrame::SbusFrameSize ];
  int32_t LenOut = 0;
  AesCrypto.EncryptCFB( SbusFrame.Buffer, TSbusFrame::SbusFrameSize, Buffer, LenOut );

  Radio.Transmit( Buffer, LenOut );
}

void TRosalynTx::RadioEvent( TRadioEvent const Event )
{
  uint8_t Buffer[ 256 ];

  if( Event == TRadioEvent::RxDone )
  {
    auto const Snr = Radio.GetSnr();
    auto const Rssi = Radio.GetRssi();
    auto const Length = Radio.ReadPacket( Buffer, sizeof( Buffer ));

    UsbPrintf( "Rssi:%4d Snr:%3d.%u Len:%u Length error\n", Rssi, Snr / 10, abs(Snr) % 10, Length );

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

    UsbPrintf( "Rssi:%4d Snr:%3d.%u Len:%u CRC Error\n", Rssi, Snr / 10, abs(Snr) % 10, Length );
  }

  if( Event == TRadioEvent::NoCrc )
  {
    auto const Snr = Radio.GetSnr();
    auto const Rssi = Radio.GetRssi();
    auto const Length = Radio.ReadPacket( Buffer, sizeof( Buffer ));

    UsbPrintf( "Rssi:%4d Snr:%3d.%u Len:%u No CRC\n", Rssi, Snr / 10, abs(Snr) % 10, Length );
  }
}

void TRosalynTx::Setup()
{
//  NvData.Setup();
  UsbPrintf( "RosalynTX\n" );
  HmiStatus( true );

  if( Radio.Setup( NvData.Modulation[ 2 ], NvData.TxPower, NvData.Channel ))
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
      HmiError( true );
    }
    break;
  }
}

void TRosalynTx::HAL_TIM_IC_CaptureCallback( TIM_HandleTypeDef *const htim )
{
  if( htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4 )
  {
    uint32_t const NewIC4 = HAL_TIM_ReadCapturedValue( htim, TIM_CHANNEL_4 );

    uint32_t Diff4 = 0;
    if( NewIC4 > OldIC4 )
    {
      Diff4 = NewIC4 - OldIC4;
    }
    else if( NewIC4 < OldIC4 )
    {
      Diff4 = (( 0xFFFFFFFFu - OldIC4 ) + NewIC4 ) + 1;
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
  else
  {
    HmiError( true );
  }
}

TRosalynTx::TRosalynTx() :
  NvData(),
  PpmFlag( false ),
  RadioFlag( false ),
  Data4(),
  Index4( 0 ),
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
    std::bind( &TRosalynTx::RadioEvent, this, std::placeholders::_1 )),
  AesCrypto( NvData.AesIV, NvData.AesKey )
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
