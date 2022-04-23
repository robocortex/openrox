//==============================================================================
//
//    OPENROX   : File add.h
//
//    Contents  : API of add module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ADD__
#define __OPENROX_ADD__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>

//! \ingroup  Array2D
//! \defgroup Array2D_Double Array2D_Double
//! \brief Functions for array of type double.

//! \ingroup  Array2D
//! \defgroup Array2D_Float Array2D_Float
//! \brief Functions for array of type float.

//! \ingroup Array2D_Double
//! @{

//! Add two arrays and place result in a third array
//! \param  [out]  res            the result
//! \param  [in ]  one            the left operand
//! \param  [in ]  two            the right operand
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_add (
   Rox_Array2D_Double res, 
   const Rox_Array2D_Double one, 
   const Rox_Array2D_Double two
);

//! @} 

//! \ingroup Array2D_Float
//! @{

//! Add two arrays and place result in a third array
//! \param  [out]  res            the result
//! \param  [in ]  one            the left operand
//! \param  [in ]  two            the right operand
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_float_add (
   Rox_Array2D_Float res, 
   const Rox_Array2D_Float one, 
   const Rox_Array2D_Float two
);

//! @} 

#endif // __OPENROX_ADD__
