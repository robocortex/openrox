//==============================================================================
//
//    OPENROX   : File substract.h
//
//  	Contents  : API of substract module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_SUBSTRACT__
#define __OPENROX_SUBSTRACT__

#include <generated/array2d_double.h>
#include <generated/array2d_uchar.h>
#include <generated/array2d_float.h>

//! \ingroup Matrix
//! \addtogroup Substract
//!   @{

//! Substract every cell of the array and take absolute value | one - two |
//! \param  [out]  res            The destination array
//! \param  [in ]  one            The left operand
//! \param  [in ]  two            The right operand
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uchar_substract ( 
   Rox_Array2D_Uchar res, 
   const Rox_Array2D_Uchar one, 
   const Rox_Array2D_Uchar two
);

//! Substract every cell of the array (A-B)
//! \param  [out]  res            The destination array
//! \param  [in ]  one            The left operand
//! \param  [in ]  two            The right operand
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_substract (
   Rox_Array2D_Double res, 
   const Rox_Array2D_Double one, 
   const Rox_Array2D_Double two
);

//! Substract every cell of the array (A-B)
//! \param  [out]  res            The destination array
//! \param  [in ]  one            The left operand
//! \param  [in ]  two            The right operand
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_float_substract (
   Rox_Array2D_Float res, 
   const Rox_Array2D_Float one, 
   const Rox_Array2D_Float two
);

//! Substract every cell of the array (A-B)
//! \param  [out]  res            The destination array
//! \param  [in ]  one            The left operand
//! \param  [in ]  two            The right operand
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_float_substract_uchar (
   Rox_Array2D_Float res, 
   const Rox_Array2D_Uchar one, 
   const Rox_Array2D_Uchar two
);

//! @} 

#endif
