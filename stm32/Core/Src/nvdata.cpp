#include "nvdata.h"


TNvData::TNvData()
{
  Load();
}

void TNvData::Load()
{
  *this = *reinterpret_cast< TNvData* >( NvBank1 );
}

void TNvData::Update()
{
  PageErase( NvBank1 );
  Program( NvBank1, sizeof( *this ), this );
}

void TNvData::Program( uint32_t const Address, uint32_t const NbBytes, void const* const Data )
{
  HAL_FLASH_Unlock();

  uint32_t const* Data0 = reinterpret_cast< uint32_t const* >( Data );

  for( auto Index = 0u; Index < NbBytes; Index += 4 )
  {
    if( HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, Address + Index, *Data0++ ) != HAL_OK )
    {
    }
  }

  HAL_FLASH_Lock();
}

void TNvData::PageErase( uint32_t const PageAddress, uint32_t const NbPages )
{
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef EraseInitStruct;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = PageAddress;
  EraseInitStruct.NbPages = NbPages;

  uint32_t PageError;
  if( HAL_FLASHEx_Erase( &EraseInitStruct, &PageError ) != HAL_OK )
  {
  }

  HAL_FLASH_Lock();
}
