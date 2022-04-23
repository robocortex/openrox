//============================================================================
//
//    OPENROX   : File ansi_basegradient.h
//
//    Contents  : API of basegradient module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

int rox_ansi_array2d_float_basegradient (
   float ** Iu_data,
   float ** Iv_data,
   unsigned int ** Gm_data,
   float ** I_data,
   unsigned int ** Im_data,
   int rows,
   int cols
);

int rox_ansi_array2d_float_basegradient_nomask (
   float ** Iu_data,
   float ** Iv_data,
   float ** I_data,
   int rows,
   int cols
);