#ifndef AES_CRYPTO_H__
#define AES_CRYPTO_H__

#include <crypto.h>


class TAesCrypto
{
public:
  TAesCrypto( uint8_t const* const AesIV, uint8_t const* const AesKey );

  void TestCFB();
  int32_t EncryptCFB( uint8_t const* const Input, int32_t const InLen, uint8_t *const Output, int32_t &OutLen );
  int32_t DecryptCFB( uint8_t const* const Input, int32_t const InLen, uint8_t *const Output, int32_t &OutLen );

private:
  uint8_t IV[ CRL_AES_BLOCK ];
  uint8_t Key[ CRL_AES128_KEY ];
};


#endif // AES_CRYPTO_H__
