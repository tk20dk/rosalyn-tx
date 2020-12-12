#include "aes-crypto.h"
#include <memory.h>


TAesCrypto::TAesCrypto( uint8_t const* const IV, uint8_t const* const Key ) :
  IV(),
  Key()
{
  memcpy( this->IV, IV, sizeof( this->IV ));
  memcpy( this->Key, Key, sizeof( this->Key ));
}

void TAesCrypto::TestCFB()
{
  uint32_t const PLAINTEXT_LENGTH = 15;

  const uint8_t Plaintext[ PLAINTEXT_LENGTH ] =
  {
    0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
    0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17//, 0x2a
  };

  uint8_t const Expected_Ciphertext[ PLAINTEXT_LENGTH ] =
  {
    0x3b, 0x3f, 0xd9, 0x2e, 0xb7, 0x2d, 0xad, 0x20,
    0x33, 0x34, 0x49, 0xf8, 0xe8, 0x3c, 0xfb//, 0x4a
  };

  uint8_t OutputMessage[ PLAINTEXT_LENGTH ] = { 0 };
  int32_t OutputMessageLength = 0;
  int32_t Status = EncryptCFB(
    Plaintext,
    PLAINTEXT_LENGTH,
    OutputMessage,
    OutputMessageLength );

  if( Status == AES_SUCCESS )
  {
    if( memcmp( Expected_Ciphertext, OutputMessage, PLAINTEXT_LENGTH ) == 0 )
    {
      Status = 0;
    }
  }

  Status = DecryptCFB(
    Expected_Ciphertext,
    PLAINTEXT_LENGTH,
    OutputMessage,
    OutputMessageLength);

  if( Status == AES_SUCCESS )
  {
    if( memcmp( Plaintext, OutputMessage, PLAINTEXT_LENGTH ) == 0 )
    {
      Status = 0;
    }
  }
}

int32_t TAesCrypto::EncryptCFB( uint8_t const* const Input, int32_t const InLen, uint8_t *const Output, int32_t &OutLen )
{
  AESECBctx_stt AESctx = { 0 };
  AESctx.mFlags = E_SK_DEFAULT;
  AESctx.mIvSize = CRL_AES_BLOCK;
  AESctx.mKeySize = CRL_AES128_KEY;

  OutLen = 0;
  int32_t Status = AES_CFB_Encrypt_Init( &AESctx, Key, IV );
  if( Status == AES_SUCCESS )
  {
    int32_t Length = 0;
    Status = AES_CFB_Encrypt_Append( &AESctx, Input, InLen, Output, &Length );
    if( Status == AES_SUCCESS )
    {
      OutLen = Length;
      Status = AES_CFB_Encrypt_Finish( &AESctx, Output + OutLen, &Length );
      OutLen += Length;
    }
  }

  return Status;
}

int32_t TAesCrypto::DecryptCFB( uint8_t const* const Input, int32_t const InLen, uint8_t *const Output, int32_t &OutLen )
{
  AESECBctx_stt AESctx = { 0 };
  AESctx.mFlags = E_SK_DEFAULT;
  AESctx.mKeySize = CRL_AES128_KEY;
  AESctx.mIvSize = CRL_AES_BLOCK;

  OutLen = 0;
  int32_t Status = AES_CFB_Decrypt_Init( &AESctx, Key, IV );
  if( Status == AES_SUCCESS )
  {
    int32_t Length = 0;
    Status = AES_CFB_Decrypt_Append( &AESctx, Input, InLen, Output, &Length );
    if( Status == AES_SUCCESS )
    {
      OutLen = Length;
      Status = AES_CFB_Decrypt_Finish( &AESctx, Output + OutLen, &Length );
      OutLen += Length;
    }
  }

  return Status;
}
