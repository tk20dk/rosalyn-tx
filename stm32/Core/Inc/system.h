#ifndef SYSTEM_H__
#define SYSTEM_H__

#include <cstdlib>
#include <cstdarg>
#include "stm32f0xx_hal.h"
#include "usbd_cdc_if.h"


void UsbPrintf( char const *const Format, ... );
void UartPrintf( char const *const Format, ... );
char const* UsbGetLine();

inline void SetPin( GPIO_TypeDef *const Port, uint32_t const Pin )
{
  Port->BSRR = Pin;
}
inline void ResetPin( GPIO_TypeDef *const Port, uint32_t const Pin )
{
  Port->BSRR = ( Pin << 16U );
}
inline bool ReadPin( GPIO_TypeDef *const Port, uint32_t const Pin )
{
  return (( Port->IDR & Pin ) != 0U );
}

uint8_t const FLAG_CONFIG = 0x12;

class TSystem
{
  struct TConfig
  {
    TConfig() :
      Flag( FLAG_CONFIG ),
      Mode( 0 ),
      UnitId( 0 ),
      SensorDelay( 60 ),
      TxPower( 2 ),
      Channel( 38 ),
      CodingRate( 5 ),      // 5 to 8  => 4/5 to 4/8
      SpreadingFactor( 7 ), // 6 to 12 => 85 to 4096 chips/symbols
      SignalBandwidth( 9 )  // 0 to 9  => 7,8kHz to 500kHz
    {
    }

    uint8_t Flag;
    uint8_t Mode;
    uint8_t UnitId;
    uint8_t SensorDelay;
    int8_t  TxPower;
    uint8_t Channel;
    uint8_t CodingRate;
    uint8_t SpreadingFactor;
    uint8_t SignalBandwidth;
  };

  void UpdateConfig( void );

public:
  TConfig Config;
};
extern TSystem System;

#endif // SYSTEM_H__
