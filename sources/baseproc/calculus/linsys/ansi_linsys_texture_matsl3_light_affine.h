//==============================================================================
//
//    OPENROX   : File ansi_linsys_texture_matsl3_light_affine.h
//
//    Contents  : API of ansi_linsys_texture_matsl3_light_affine module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

int rox_ansi_linsys_texture_matsl3_light_affine (   
   double ** LtL_data, 
   double *  Lte_data,
   float ** Iu_data, 
   float ** Iv_data, 
   float ** Id_data, 
   float ** Ia_data, 
   unsigned int ** Im_data,
   int rows,
   int cols
);