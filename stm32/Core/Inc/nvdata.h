#ifndef NVDATA_H__
#define NVDATA_H__

#ifdef STM32F303xC
  #include "stm32f3xx_hal.h"
#elif defined ( STM32F070x6 ) || defined( STM32F072xB )
  #include "stm32f0xx_hal.h"
#endif
#include "config.h"


class TNvData :
  public TConfig
{
public:
  static uint32_t const NvSize = 2048;
  static uint32_t const NvBank1 = 0x0801F000;
  static uint32_t const NvBank2 = 0x0801F800;

  void Setup();
  void Update();

private:
  static void Program( uint32_t const Address, uint32_t const NbBytes, void const* const Data );
  static void PageErase( uint32_t const PageAddress, uint32_t const NbPages = 1 );
};

#endif // NVDATA_H__
