//============================================================================
//
//    OPENROX   : File bch.h
//
//    Contents  : API of bch module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_BCH__
#define __OPENROX_BCH__

#include <system/memory/datatypes.h>

//! \ingroup Maths
//! \addtogroup BCH
//! \brief Bose-Chaudhuri-Hocquenghem cyclic error-correcting codes
//! @{

//! Encode 8 bits with 8 errors tolerancy on a 64 bit codeword
//! \param  [out]  output         A pointer to the value encoded
//! \param  [in ]  input          The value to encode
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_bch_c8_e8_encode(Rox_Ulint * output, Rox_Uchar input);

//! Encode 6 bits with 2 errors tolerancy on a 16 bit codeword
//! \param  [out]  output         A pointer to the value encoded
//! \param  [in ]  input          The value to encode
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_bch_c6_e2_encode(Rox_Ushort * output, Rox_Uchar input);

//! Decode 8 bits with 8 errors tolerancy on a 64 bit codeword
//! \param  [out]  output         A pointer to the value decoded
//! \param  [in ]  input          The value to decode
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_bch_c8_e8_decode(Rox_Uchar *output, Rox_Ulint input);

//! Decode 6 bits with 2 errors tolerancy on a 16 bit codeword
//! \param  [out]  output         A pointer to the value decoded
//! \param  [in ]  input          The value to decode
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_bch_c6_e2_decode(Rox_Uchar *output, Rox_Ushort input);

//! @} 

#endif
