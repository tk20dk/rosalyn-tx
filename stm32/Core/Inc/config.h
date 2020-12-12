#ifndef CONFIG_H__
#define CONFIG_H__

#include <crypto.h>
#include "sx1268-def.h"


struct TConfig
{
  static uint8_t const FlagConfig = 0x01;

  TConfig() :
    Flag( FlagConfig ),
    Mode( 0 ),
    UnitId( 0 ),
    TxPower( -3 ), // -3dBm to 22dBm
    Channel( 38 ),
    Modulation
    {
      { LORA_BW_125, LORA_CR_4_5, LORA_SF12 }, // Low
      { LORA_BW_250, LORA_CR_4_5, LORA_SF8  }, // Medium
      { LORA_BW_500, LORA_CR_4_5, LORA_SF5  }  // High
    },
    AesIV
    {
      0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
      0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
    },
    AesKey
    {
      0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
      0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
    }
  {
  }

  uint8_t Flag;
  uint8_t Mode;
  uint8_t UnitId;
  int8_t  TxPower;
  uint8_t Channel;
  Modulation_t Modulation[ 3 ];
  uint8_t AesIV[ CRL_AES_BLOCK ];
  uint8_t AesKey[ CRL_AES128_KEY ];
};

#endif // CONFIG_H__
