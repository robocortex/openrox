//==============================================================================
//
//    OPENROX   : File fillval.h
//
//  	Contents  : API of fillval module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_FILLVAL__
#define __OPENROX_FILLVAL__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>
#include <generated/array2d_uchar.h>
#include <generated/array2d_sint.h>
#include <generated/array2d_uint.h>
#include <generated/array2d_sshort.h>

//! \ingroup Array2D_Double
//! \brief Fill array with the same scalar
//! \param [in] dest the left operand
//! \param [in] value the right operand
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_double_fillval(Rox_Array2D_Double dest, const Rox_Double value);

//! \ingroup Array2D_Float
//! \brief Fill array with the same scalar
//! \param [in] dest the left operand
//! \param [in] value the right operand
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_float_fillval(Rox_Array2D_Float dest, const Rox_Float value);

//! \ingroup Array2D_Sint
//! \brief Fill array with the same scalar
//! \param [in] dest the left operand
//! \param [in] value the right operand
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_sint_fillval(Rox_Array2D_Sint dest, const Rox_Sint value);

//! \ingroup Array2D_Uint
//! \brief Fill array with the same scalar
//! \param [in] dest the left operand
//! \param [in] value the right operand
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_uint_fillval(Rox_Array2D_Uint dest, const Rox_Uint value);

//! \ingroup Array2D_Uchar
//! \brief Fill array with the same scalar
//! \param [in] dest the left operand
//! \param [in] value the right operand
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_uchar_fillval(Rox_Array2D_Uchar dest, const Rox_Uchar value);

//! \ingroup Array2D_Sshort
//! \brief Fill array with the same scalar
//! \param [in] dest the left operand
//! \param [in] value the right operand
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_sshort_fillval(Rox_Array2D_Sshort dest, const Rox_Sshort value);

#endif
