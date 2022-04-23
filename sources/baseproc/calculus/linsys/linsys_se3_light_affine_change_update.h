//==============================================================================
//
//    OPENROX   : File linsys_se3_light_affine_change_update.h
//
//    Contents  : API of linsys_se3_light_affine_premul_left module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINSYS_SE3_LIGHT_AFFINE_CHANGE_UPDATE__
#define __OPENROX_LINSYS_SE3_LIGHT_AFFINE_CHANGE_UPDATE__

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matrix.h>

//! \ingroup Jacobians
//! \brief 
ROX_API Rox_ErrorCode rox_linsys_se3_light_affine_change_update_from_right_to_left (
   Rox_Matrix LtL, 
   Rox_Matrix Lte,  
   const Rox_MatSE3 pose
);

#endif
