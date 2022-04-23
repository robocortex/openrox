//==============================================================================
//
//    OPENROX   : File ansi_linsys_se3_z1_light_affine_premul.h
//
//    Contents  : API of se3z1_light_affine_premul module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

int rox_ansi_linsys_se3_light_affine_premul (
   double ** LtL_data, 
   double *  Lte_data,
   double ** K_data,
   double ** T_data,
   float ** Iu_data, 
   float ** Iv_data, 
   float ** Id_data, 
   float ** Ia_data, 
   unsigned int ** Im_data,
   int rows,
   int cols
);