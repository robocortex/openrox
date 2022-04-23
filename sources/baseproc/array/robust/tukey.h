//==============================================================================
//
//    OPENROX   : File tukey.h
//
//    Contents  : API of tukey module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TUKEY__
#define __OPENROX_TUKEY__

#include <generated/array2d_double.h>

//! \ingroup Optimization
//! \defgroup Robust 

//! \addtogroup Robust
//! @{

//! Compute the weights of the first row of the given array using huber function.
//! Please take care of the input values range (if they are too small, this may not work correctly), simply scale the values a priori.
//! \param [out] weights the weigth for each element of the input first row.
//! \param [out] workbuffer1 an array of the same size than input.
//! \param [out] workbuffer2 an array of the same size than input.
//! \param [in] input the array to compute weigths on
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_array2d_double_tukey(Rox_Array2D_Double weights, Rox_Array2D_Double workbuffer1, Rox_Array2D_Double workbuffer2, Rox_Array2D_Double input);

//! Compute the weights of the first row of the given array using huber function.
//! Please take care of the input values range (if they are too small, this may not work correctly), simply scale the values a priori.
//! \param [out] weights the weigth for each element of the input first row.
//! \param [out] workbuffer1 an array of the same size than input.
//! \param [out] workbuffer2 an array of the same size than input.
//! \param [in] input the array to compute weigths on
//! \param [in] sigma_minimal minimal value sigma can goes to
//! \param [in] sigma_maximal maximal value sigma can goes to
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_array2d_double_tukey_bounded(Rox_Array2D_Double weights, Rox_Array2D_Double workbuffer1, Rox_Array2D_Double workbuffer2, Rox_Array2D_Double input, Rox_Double sigma_minimal, Rox_Double sigma_maximal);

//! @} 

#endif

