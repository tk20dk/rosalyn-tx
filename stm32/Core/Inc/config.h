#ifndef CONFIG_H__
#define CONFIG_H__

#include "sx1268-def.h"


struct TConfig
{
  static uint8_t const FlagConfig = 0x01;

  TConfig() :
    Flag( FlagConfig ),
    Mode( 0 ),
    UnitId( 0 ),
    TxPower( -3 ),  // -3dBm to 22dBm
    Channel( 38 ),
    Modulation
    {
      { LORA_BW_125, LORA_CR_4_5, LORA_SF12 }, // Low
      { LORA_BW_250, LORA_CR_4_5, LORA_SF8  }, // Medium
      { LORA_BW_500, LORA_CR_4_5, LORA_SF5  }  // High
    }
  {
  }

  uint8_t Flag;
  uint8_t Mode;
  uint8_t UnitId;
  int8_t  TxPower;
  uint8_t Channel;
  Modulation_t Modulation[ 3 ];
};


#endif // CONFIG_H__
