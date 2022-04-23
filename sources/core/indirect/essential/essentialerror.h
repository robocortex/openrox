//==============================================================================
//
//    OPENROX   : File essentialerror.h
//
//    Contents  : API of essentialerror module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ESSENTIAL_ERROR__
#define __OPENROX_ESSENTIAL_ERROR__

#include <generated/array2d_double.h>
#include <generated/dynvec_point2d_float.h>
#include <generated/dynvec_double.h>

//! \ingroup Geometry
//! \addtogroup Essential
//! @{

//! Given an Essential matrix, compute the per point symmetric geometric error
//! \param  []  vecerrors        error vector
//! \param  []  E                essential matrix
//! \param  []  ref              reference points
//! \param  []  cur              current points
//! \return An error code
ROX_API Rox_ErrorCode rox_essential_geometric_error(Rox_Array2D_Double vecerrors, Rox_Array2D_Double E, Rox_DynVec_Point2D_Float ref, Rox_DynVec_Point2D_Float cur);

//! @} 

#endif
