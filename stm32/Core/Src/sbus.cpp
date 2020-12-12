#include <sbus.h>


uint32_t TSbusData::Map( uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max )
{
  if( x < in_min ) x = in_min;
  if( x > in_max ) x = in_max;
  return ((( x - in_min ) * ( out_max - out_min )) / ( in_max - in_min )) + out_min;
}

void TSbusData::SetPWM( uint16_t const *const Data )
{
  Ch1 = Map( Data[ 0 ], 1000, 2000, 0, 2047 );
  Ch2 = Map( Data[ 1 ], 1000, 2000, 0, 2047 );
  Ch3 = Map( Data[ 2 ], 1000, 2000, 0, 2047 );
  Ch4 = Map( Data[ 3 ], 1000, 2000, 0, 2047 );
  Ch5 = Map( Data[ 4 ], 1000, 2000, 0, 2047 );
  Ch6 = Map( Data[ 5 ], 1000, 2000, 0, 2047 );
  Ch7 = Map( Data[ 6 ], 1000, 2000, 0, 2047 );
  Ch8 = Map( Data[ 7 ], 1000, 2000, 0, 2047 );
}


TSbusData TSbusSerial::Receive() const
{
  return TSbusData( RxFrame );
}

void TSbusSerial::Transmit( TSbusData const &SbusData )
{
  TxFrame = SbusData.Encode();

  TxState = SbusStateNull;
  LL_USART_EnableIT_TXE( USARTx );
}

void TSbusSerial::RxError()
{
  RxState = SbusStateNull;
  RxTimeout = false;
}

void TSbusSerial::TxEmpty()
{
  if( TxState == SbusFrameSize )
  {
	LL_USART_DisableIT_TXE( USARTx );
  }
  else
  {
    LL_USART_TransmitData8( USARTx, TxFrame[ TxState++ ] );
  }
}

void TSbusSerial::RxNotEmpty()
{
  auto const Data = LL_USART_ReceiveData8( USARTx );
  RxFrame[ RxState++ ] = Data;

  if( RxState == SbusStateSOF )
  {
    if(( Data != SbusSOF ) || ( RxTimeout == false ))
    {
      RxState = SbusStateNull;
    }
    RxTimeout = false;
  }
  else if( RxState == SbusStateEOF )
  {
    if(( Data == SbusEOF ) && ( RxTimeout == false ))
    {
      SerialFlag = true;
    }
    RxState = SbusStateNull;
    RxTimeout = false;
  }
}

void TSbusSerial::USART_IRQHandler()
{
  if( LL_USART_IsActiveFlag_RXNE( USARTx ) && LL_USART_IsEnabledIT_RXNE( USARTx ))
  {
    RxNotEmpty();
  }

  if(LL_USART_IsEnabledIT_TXE( USARTx ) && LL_USART_IsActiveFlag_TXE( USARTx ))
  {
    TxEmpty();
  }

  if( LL_USART_IsActiveFlag_PE( USARTx ))
  {
    LL_USART_ClearFlag_PE( USARTx );
    RxError();
    ++StatPE;
  }

  if( LL_USART_IsActiveFlag_FE( USARTx ))
  {
    LL_USART_ClearFlag_FE( USARTx );
    RxError();
    ++StatFE;
  }

  if( LL_USART_IsActiveFlag_NE( USARTx ))
  {
    LL_USART_ClearFlag_NE( USARTx );
    RxError();
    ++StatNE;
  }

  if( LL_USART_IsActiveFlag_ORE( USARTx ))
  {
    LL_USART_ClearFlag_ORE( USARTx );
    RxError();
    ++StatORE;
  }

  if( LL_USART_IsActiveFlag_RTO( USARTx ))
  {
    LL_USART_ClearFlag_RTO( USARTx );
    RxTimeout = true;
  }
}

void TSbusSerial::Setup()
{
  LL_USART_SetRxTimeout( USARTx, 400 ); // 4ms at 100000 Bit/s
  LL_USART_EnableRxTimeout( USARTx );
  LL_USART_EnableIT_RTO( USARTx );

  LL_USART_EnableIT_PE( USARTx );
  LL_USART_EnableIT_RXNE( USARTx );
  LL_USART_EnableIT_ERROR( USARTx );
}

TSbusSerial::TSbusSerial( USART_TypeDef *const USARTx, bool &SerialFlag ) :
  RxTimeout( false ),
  SerialFlag( SerialFlag ),
  RxState( 0 ),
  TxState( 0 ),
  RxFrame(),
  TxFrame(),
  StatPE(),
  StatFE(),
  StatNE(),
  StatORE(),
  USARTx( USARTx )
{
}
