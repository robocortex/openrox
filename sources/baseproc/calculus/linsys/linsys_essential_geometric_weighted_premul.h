//==============================================================================
//
//    OPENROX   : File linsys_essential_geometric_weighted_premul.h
//
//    Contents  : API of essential_geometric_weighted_premul module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINSYS_ESSENTIALGEOMETRIC_PREMUL__
#define __OPENROX_LINSYS_ESSENTIALGEOMETRIC_PREMUL__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>
#include <generated/array2d_uint.h>
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/maths/linalg/matrix.h>

//! \addtogroup Jacobians
//! @{

//! Jacobian for essential matrix optimization refinement
//! \param  [out]  LtL            The result hessian
//! \param  [out]  Lte            The result J'*d vector
//! \param  [in ]  error          The input measured error
//! \param  [in ]  weight         The robust weight per point
//! \param  [in ]  pose           The currently estimated pose
//! \param  [in ]  E              The estimated essential matrix
//! \param  [in ]  refs           The reference points to use for minimization
//! \param  [in ]  curs           The current points to use for minimization
//! \param  [in ]  nb_points      The number of points to use for minimization
//! \param  [in ]  unused_translation_id which translation parameter (0-2) will be ignored and used for scaling
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_jacobian_essential_geometric_weighted_premul (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Array2D_Double error, 
   const Rox_Array2D_Double weight, 
   const Rox_Array2D_Double pose, 
   const Rox_Array2D_Double E, 
   const Rox_Point2D_Float  refs, 
   const Rox_Point2D_Float  curs, 
   const Rox_Uint nb_points, 
   const Rox_Uint unused_translation_id
);

//! @}

#endif
