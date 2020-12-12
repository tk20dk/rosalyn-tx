#ifndef NVDATA_H__
#define NVDATA_H__

#include "main.h"
#include "config.h"


class TNvData :
  public TConfig
{
public:
  static uint32_t const NvSize = 2048;
  static uint32_t const NvBank1 = 0x0801F000;
  static uint32_t const NvBank2 = 0x0801F800;

  void Load();
  void Update();

private:
  static void FlashUnlock();
  static void FlashProgram( uint32_t const Address, uint32_t const NbBytes, void const* const Data );
  static void FlashPageErase( uint32_t const PageAddress, uint32_t const NbPages = 1 );
};

#endif // NVDATA_H__
