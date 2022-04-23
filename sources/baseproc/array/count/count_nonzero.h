//==============================================================================
//
//    OPENROX   : File count_nonzero.h
//
//  	Contents  : API of count_nonzero module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_COUNT_NONZERO__
#define __OPENROX_COUNT_NONZERO__

#include <generated/array2d_uint.h>

//! \ingroup Basemaths
//! \addtogroup count_nonzero
//! @{

//! Compute the min and max of an array
//! \param  [out]  count 			 The number of nonzero entries
//! \param  [in ]  input 			 The input array
//! \return An error code
//! \todo 	To be tested
ROX_API Rox_ErrorCode rox_array2d_uint_count_nonzero ( Rox_Size * count, Rox_Array2D_Uint input );

//! To be completed
//! \param  [out]  out           To be completed
//! \param  [in ]  inp           To be completed
//! \return An error code
//! \todo   To be tested
ROX_API int rox_ansi_array_float_count_nonzero ( size_t * count, float * input, size_t input_size );


//! To be completed
//! \param  [out]  out           To be completed
//! \param  [in ]  inp           To be completed
//! \return An error code
//! \todo   To be tested
ROX_API int rox_ansi_array_uint_count_nonzero ( size_t * count, unsigned int * input, size_t input_size );

//! To be completed
//! \param  [out]  out           To be completed
//! \param  [in ]  inp           To be completed
//! \return An error code
//! \todo   To be tested
ROX_API int rox_ansi_array_uint_matrix_count_nonzero_step_ogl  ( size_t * count, unsigned int * input_data, size_t input_rows, size_t input_cols, size_t step );

//! @}

#endif // __OPENROX_COUNT_NONZERO__
