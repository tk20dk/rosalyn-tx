#ifndef SYSTEM_H__
#define SYSTEM_H__

#include "main.h"


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

//inline void HmiStatus( bool const Mode )
//{
//  HMI_STATUS_GPIO_Port->BSRR = Mode ? HMI_STATUS_Pin << 16 : HMI_STATUS_Pin;
//}

//inline void HmiError( bool const Mode )
//{
//  HMI_ERROR_GPIO_Port->BSRR = Mode ? HMI_ERROR_Pin << 16 : HMI_ERROR_Pin;
//}

inline void HmiStatusExt( bool const Mode )
{
  HMI_STATUS_EXT_GPIO_Port->BSRR = Mode ? HMI_STATUS_EXT_Pin << 16 : HMI_STATUS_EXT_Pin;
}

inline void HmiErrorExt( bool const Mode )
{
  HMI_ERROR_EXT_GPIO_Port->BSRR = Mode ? HMI_ERROR_EXT_Pin << 16 : HMI_ERROR_EXT_Pin;
}

#endif // SYSTEM_H__
