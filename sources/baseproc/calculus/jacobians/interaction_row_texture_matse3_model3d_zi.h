//==============================================================================
//
//    OPENROX   : File interaction_row_texture_matse3_model3d_zi.h
//
//    Contents  : API of interaction_row_texture_matse3_model3d_zi module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_INTERACTION_ROW_TEXTURE_MATSE3_MODEL3D_ZI__
#define __OPENROX_INTERACTION_ROW_TEXTURE_MATSE3_MODEL3D_ZI__

#include <generated/array2d_float.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Interactions
//! \addtogroup Interaction_MatSE3
//! @{

//! \brief Interaction row with respect to SE3
//! \remark 3 dof translation first, then 3 dof rotation
//! \param  [out]  Lk             row of the interaction matrix
//! \param  [in ]  u              u coordinates of the point (in pixels)
//! \param  [in ]  v              v coordinates of the point (in pixels)
//! \param  [in ]  Iu             image gradient along u axis
//! \param  [in ]  Iv             image gradient along v axis
//! \param  [in ]  K              camera intrinsic parameters
//! \param  [in ]  tc             translation in the current frame
//! \param  [in ]  zi             inverse depth at pixel (u,v)
//! \param  [in ]  ziu            gradient of inverse depth at pixel (u,v) along u axis
//! \param  [in ]  ziv            gradient of inverse depth at pixel (u,v) along v axis
ROX_API Rox_ErrorCode rox_interaction_row_texture_matse3_model3d_zi ( 
   Rox_Double L_row[6], 
   const Rox_Double ur,
   const Rox_Double vr, 
   const Rox_Double Iu,
   const Rox_Double Iv, 
   const Rox_Double zi,  
   const Rox_Double ziu,
   const Rox_Double ziv,  
   const Rox_MatUT3 K, 
   const Rox_Matrix tc
);

//! @} 

#endif // __OPENROX_INTERACTION_ROW_TEXTURE_MATSE3_MODEL3D_ZI__
