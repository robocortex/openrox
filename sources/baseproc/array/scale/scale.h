//==============================================================================
//
//    OPENROX   : File scale.h
//
//    Contents  : API of scale module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SCALE__
#define __OPENROX_SCALE__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>

//! \ingroup Matrix
//! \addtogroup Scale
//! @{

//! Scale all elements of an array2d_double
//! \param  [out]  out            The destination array
//! \param  [in ]  inp            The source array
//! \param  [in ]  scale          The scale to apply (a scalar)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_scale ( 
	Rox_Array2D_Double out, 
	Rox_Array2D_Double inp, 
	Rox_Double scale
);

//! Scale all elements of an array2d_double : inpuout = scale*inpuout
//! \param  [out]  inpuout        The source array that become the destination array
//! \param  [in ]  scale          The scale to apply (a scalar)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_scale_inplace ( 
	Rox_Array2D_Double inpout, 
	const Rox_Double scale 
);

//! Scale a column of an array2d_double
//! \param [out]   out      The destination array
//! \param [in ]    inp      The source array
//! \param [in ]    scale    The scale to apply (a scalar)
//! \param [in ]    col      The column to scale    
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_double_scale_col ( 
	Rox_Array2D_Double out, 
	const Rox_Array2D_Double inp, 
	const Rox_Double scale, 
	const Rox_Sint col 
);

//! Scale a row of an array2d_double
//! \param [out]   out      The destination array
//! \param [in]    inp      The source array
//! \param [in]    scale    The scale to apply (a scalar)
//! \param [in]    row      The row to scale    
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_double_scale_row ( 
	Rox_Array2D_Double out, 
	const Rox_Array2D_Double inp, 
	const Rox_Double scale,
	const Rox_Sint row 
);

//! Scale all elements of an array
//! \param  [out]  out            The destination array
//! \param  [in ]  inp            The source array
//! \param  [in ]  scale          The scale to apply (a scalar)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_float_scale ( 
   Rox_Array2D_Float out, 
   const Rox_Array2D_Float inp, 
   const Rox_Float scale
);

ROX_API Rox_ErrorCode rox_array2d_float_scale_inplace ( 
	Rox_Array2D_Float inpout, 
	const Rox_Float scale 
);

//! @} 

#endif // __OPENROX_SCALE__
