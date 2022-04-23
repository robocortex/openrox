//==============================================================================
//
//    OPENROX   : File ansi_linsys_texture_matse3_light_affine_model3d_zi.h
//
//    Contents  : API of ansi_linsys_texture_matse3_light_affine_model3d_zi module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ANSI_LINSYS_TEXTURE_MATSE3_LIGHT_AFFINE_MODEL3D_ZI__
#define __OPENROX_ANSI_LINSYS_TEXTURE_MATSE3_LIGHT_AFFINE_MODEL3D_ZI__

int rox_ansi_linsys_texture_matse3_light_affine_model3d_zi (
   double ** LtL_data, 
   double ** Lte_data,
   double ** K_data,
   double ** tau_data,
   float ** Zi_data, 
   float ** Ziu_data, 
   float ** Ziv_data, 
   float ** Iu_data, 
   float ** Iv_data, 
   float ** Id_data, 
   float ** Ia_data, 
   unsigned int ** Im_data,
   int rows,
   int cols
);

#endif // __OPENROX_ANSI_LINSYS_TEXTURE_MATSE3_LIGHT_AFFINE_MODEL3D_ZI__
