//==============================================================================
//
//    OPENROX   : File intmat_se3_so3z_r2.h
//
//    Contents  : API of intmat_se3_so3z_r2
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_INTMAT_SE3_SO3Z_R2__
#define __OPENROX_INTMAT_SE3_SO3Z_R2__

#include <generated/dynvec_point3d_double.h>

#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/maths/linalg/matse3.h>

//! \ingroup Interaction Matrices
//! \defgroup specifics
//! \brief Jacobians relative to the SE3 group.

//! \addtogroup specifics
//! @{

//! Compute the interaction matrices 
//! dq / dt = Lo * vo + Lb * vb 
//! \param  [out]  Lo             The resulting interaction matrix (size(Lo) = 2n x 6)
//! \param  [out]  Lb             The resulting interaction matrix (size(Lo) = 2n x 3)
//! \param  [in ]  cTo            The error vector for the b points
//! \param  [in ]  mo             The 3D model points in meters
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_interaction_matse3_matso3z_r2_point2d_nor (
   Rox_Matrix Lo,
   Rox_Matrix Lb,
   const Rox_MatSE3 cTo,
   const Rox_DynVec_Point3D_Double mo
);

//! @}

#endif
