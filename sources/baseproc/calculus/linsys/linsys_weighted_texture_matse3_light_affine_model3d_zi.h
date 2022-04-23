//==============================================================================
//
//    OPENROX   : File linsys_weighted_texture_matse3_light_affine_model3d_zi.h
//
//    Contents  : API of linsys_weighted_texture_matse3_light_affine_model3d_zi module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINSYS_WEIGHTED_TEXTURE_MATSE3_LIGHT_AFFINE_MODEL3D_ZI__
#define __OPENROX_LINSYS_WEIGHTED_TEXTURE_MATSE3_LIGHT_AFFINE_MODEL3D_ZI__

#include <generated/array2d_float.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Interactions
//! \addtogroup Interaction_MatSE3
//! @{

//! \brief Weighted Linear System A = L'*W^2*L and b = L'*W^2*e
ROX_API Rox_ErrorCode rox_linsys_weighted_texture_matse3_light_affine_model3d_zi (
   Rox_Matrix LtL, 
   Rox_Matrix Lte,
   const Rox_MatUT3 K,
   const Rox_MatSE3 T,
   const Rox_Array2D_Float Zi, 
   const Rox_Array2D_Float Ziu, 
   const Rox_Array2D_Float Ziv, 
   const Rox_Array2D_Float Iu, 
   const Rox_Array2D_Float Iv, 
   const Rox_Array2D_Float Id, 
   const Rox_Array2D_Float Ia, 
   const Rox_Array2D_Float weight,
   const Rox_Imask Im
);

//! @} 

#endif // __OPENROX_LINSYS_WEIGHTED_TEXTURE_MATSE3_LIGHT_AFFINE_MODEL3D_ZI__
