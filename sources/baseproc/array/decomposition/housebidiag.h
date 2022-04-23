//==============================================================================
//
//    OPENROX   : File housebidiag.h
//
//    Contents  : API of housebidiag module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_HOUSE_BIDIAG__
#define __OPENROX_HOUSE_BIDIAG__

#include <generated/array2d_double.h>

//! \ingroup  Linalg
//! \defgroup Householder Householder
//! \brief Hoseholder decomposition.

//! \addtogroup Householder
//! @{

//! Householder bidiagonalization (See google for more explanation ;))
//! \param  []  U                output matrix U
//! \param  []  S                output matrix S
//! \param  []  V                output matrix V
//! \param  []  e                output matrix e
//! \param  []  ptr_eps          pointer to the estimated epsilon
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_householder_bidiagonalization (
   Rox_Array2D_Double U, 
   Rox_Array2D_Double S, 
   Rox_Array2D_Double V, 
   Rox_Array2D_Double e, 
   Rox_Double * ptr_eps
);

//! @} 

#endif
