#include "nvdata.h"


void TNvData::Load()
{
  *this = *reinterpret_cast< TNvData* >( NvBank1 );
}

void TNvData::Update()
{
  FlashUnlock();
  FlashPageErase( NvBank1 );
  FlashProgram( NvBank1, sizeof( *this ), this );
}

void TNvData::FlashUnlock()
{
  /* (1) Wait till no operation is on going */
  /* (2) Check that the Flash is unlocked */
  /* (3) Perform unlock sequence */

  while(( FLASH->SR & FLASH_SR_BSY ) != 0 ) /* (1) */
  {
    /* For robust implementation, add here time-out management */
  }

  if(( FLASH->CR & FLASH_CR_LOCK ) != 0 ) /* (2) */
  {
    FLASH->KEYR = FLASH_KEY1; /* (3) */
    FLASH->KEYR = FLASH_KEY2;
  }
}

void TNvData::FlashProgram( uint32_t const Address, uint32_t const NbBytes, void const* const Data )
{
  /* (1) Set the PG bit in the FLASH_CR register to enable programming */
  /* (2) Perform the data write (half-word) at the desired address */
  /* (3) Wait until the BSY bit is reset in the FLASH_SR register */
  /* (4) Check the EOP flag in the FLASH_SR register */
  /* (5) clear it by software by writing it at 1 */
  /* (6) Reset the PG Bit to disable programming */

  FLASH->CR |= FLASH_CR_PG; /* (1) */

  uint16_t const* Data0 = reinterpret_cast< uint16_t const* >( Data );

  for( auto Index = 0u; Index < NbBytes; Index += 2 )
  {
    *reinterpret_cast< uint16_t* >( Address + Index ) = *Data0++; /* (2) */
    while(( FLASH->SR & FLASH_SR_BSY ) != 0 ) /* (3) */
    {
      /* For robust implementation, add here time-out management */
    }
    if(( FLASH->SR & FLASH_SR_EOP ) != 0 ) /* (4) */
    {
      FLASH->SR = FLASH_SR_EOP; /* (5) */
    }
    else
    {
      /* Manage the error cases */
    }
  }

  FLASH->CR &= ~FLASH_CR_PG; /* (6) */
}

void TNvData::FlashPageErase( uint32_t const PageAddress, uint32_t const NbPages )
{
  /* (1) Set the PER bit in the FLASH_CR register to enable page erasing */
  /* (2) Program the FLASH_AR register to select a page to erase */
  /* (3) Set the STRT bit in the FLASH_CR register to start the erasing */
  /* (4) Wait until the BSY bit is reset in the FLASH_SR register */
  /* (5) Check the EOP flag in the FLASH_SR register */
  /* (6) Clear EOP flag by software by writing EOP at 1 */
  /* (7) Reset the PER Bit to disable the page erase */

  FLASH->CR |= FLASH_CR_PER; /* (1) */

  for( auto Index = 0u; Index < NbPages; Index++ )
  {
    FLASH->AR = PageAddress + Index * NvSize; /* (2) */
    FLASH->CR |= FLASH_CR_STRT; /* (3) */
    while(( FLASH->SR & FLASH_SR_BSY ) != 0 ) /* (4) */
    {
      /* For robust implementation, add here time-out management */
    }
    if(( FLASH->SR & FLASH_SR_EOP ) != 0 ) /* (5) */
    {
      FLASH->SR = FLASH_SR_EOP; /* (6)*/
    }
    else
    {
      /* Manage the error cases */
    }
  }

  FLASH->CR &= ~FLASH_CR_PER; /* (7) */
}
