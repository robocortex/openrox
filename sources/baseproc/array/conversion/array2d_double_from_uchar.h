//==============================================================================
//
//    OPENROX   : File array2d_double_from_uchar.h
//
//  	Contents  : API of array2d_double_from_uchar module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_DOUBLE_FROM_UCHAR__
#define __OPENROX_DOUBLE_FROM_UCHAR__

#include <generated/array2d_uchar.h>
#include <generated/array2d_double.h>

//! \ingroup Array2D
//! \defgroup Data_Conversion Data Conversion
//! \brief Types (re)definititon.

//! \ingroup  Data_Conversion
//! \defgroup Uchar2Double Uchar2Double
//! \brief Conversion from Uchar to Double.

//! \addtogroup Uchar2Double
//! @{

//! Convert an array from uchar type to double type
//! \param  [out]  output         The converted array
//! \param  [in ]  input          The array to convert
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_from_uchar ( 
   Rox_Array2D_Double output, 
   const Rox_Array2D_Uchar input 
);

//! Convert an array from uchar type to double type and scale values to [0;1]
//! \param  [out]  output         The converted array
//! \param  [in ]  input          The array to convert
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_from_uchar_normalize (
   Rox_Array2D_Double output, 
   const Rox_Array2D_Uchar input
);

//! Convert an array from uchar type to double type and scale values to [0;1] using min max values
//! \param  [out]  output         The converted array
//! \param  [in ]  input          The array to convert
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_from_uchar_normalize_minmax (
   Rox_Array2D_Double output, 
   const Rox_Array2D_Uchar input
);

//! @} 

#endif
