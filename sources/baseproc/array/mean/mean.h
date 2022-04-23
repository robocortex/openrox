//==============================================================================
//
//    OPENROX   : File mean.h
//
//    Contents  : API of mean module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MEAN__
#define __OPENROX_MEAN__

#include <generated/array2d_float.h>
#include <generated/array2d_double.h>

//! \ingroup Statistics
//! \addtogroup Mean
//! @{

//! \brief Mean every cell of the array (0.5*(A+B))
//! \param  [out]  res            The destination array
//! \param  [in ]  one            The left operand
//! \param  [in ]  two            The right operand
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_float_mean ( 
   Rox_Array2D_Float res, 
   const Rox_Array2D_Float one, 
   const Rox_Array2D_Float two
);

//! \brief Mean every cell of the array (0.5*(A+B))
//! \param  [out]  res            The destination array
//! \param  [in ]  one            The left operand
//! \param  [in ]  two            The right operand
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_mean ( 
   Rox_Array2D_Double res, 
   const Rox_Array2D_Double one, 
   const Rox_Array2D_Double two
);

//! @}

#endif // __OPENROX_MEAN__
