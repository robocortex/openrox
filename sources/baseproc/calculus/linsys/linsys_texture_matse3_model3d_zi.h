//==============================================================================
//
//    OPENROX   : File linsys_texture_matse3_model3d_zi.h
//
//    Contents  : API of linsys_texture_matse3_model3d_zi module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINSYS_TEXTURE_MATSE3_MODEL3D_ZI__
#define __OPENROX_LINSYS_TEXTURE_MATSE3_MODEL3D_ZI__

#include <generated/array2d_float.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Interactions
//! \addtogroup Interaction_MatSE3
//! @{

//! \brief Linear System A = L'*L and b = L'*e
//! \brief L is the interaction matrix of the image texture 
//! \brief e is the error of the image texture Ir-Iw 
//! \param  [in ]  K              The intrinsic parameters K = Kc = Kr
//! \param  [in ]  T              The pose cTr 
ROX_API Rox_ErrorCode rox_linsys_texture_matse3_model3d_zi (
   Rox_Matrix LtL, 
   Rox_Matrix Lte,
   const Rox_Array2D_Float Iu, 
   const Rox_Array2D_Float Iv, 
   const Rox_Array2D_Float Id, 
   const Rox_Array2D_Float zir, 
   const Rox_Array2D_Float ziur, 
   const Rox_Array2D_Float zivr, 
   const Rox_MatUT3 K,
   const Rox_MatSE3 T,
   const Rox_Imask mask
);

//! @} 

#endif // __OPENROX_LINSYS_TEXTURE_MATSE3_MODEL3D_ZI__
