//==============================================================================
//
//    OPENROX   : File ansi_algse3.h
//
//    Contents  : API of ansi algse3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_algse3.h"


int rox_ansi_array_float_algse3_set_velocity ( float * alg_data, float * vec_data )
{
   int error = 0;

   alg_data[0*4+0] = 0;
   alg_data[0*4+1] = -vec_data[5];
   alg_data[0*4+2] =  vec_data[4];
   alg_data[0*4+3] =  vec_data[0];

   alg_data[1*4+0] =  vec_data[5];
   alg_data[1*4+1] = 0;
   alg_data[1*4+2] = -vec_data[3];
   alg_data[1*4+3] =  vec_data[1];

   alg_data[2*4+0] = -vec_data[4];
   alg_data[2*4+1] =  vec_data[3];
   alg_data[2*4+2] = 0;
   alg_data[2*4+3] =  vec_data[2];

   alg_data[3*4+0] = 0;
   alg_data[3*4+1] = 0;
   alg_data[3*4+2] = 0;
   alg_data[3*4+3] = 0;

   return error;
}

int rox_ansi_array_double_algse3_set_velocity ( double * alg_data, double * vec_data )
{
   int error = 0;

   alg_data[0*4+0] = 0;
   alg_data[0*4+1] = -vec_data[5];
   alg_data[0*4+2] =  vec_data[4];
   alg_data[0*4+3] =  vec_data[0];

   alg_data[1*4+0] =  vec_data[5];
   alg_data[1*4+1] = 0;
   alg_data[1*4+2] = -vec_data[3];
   alg_data[1*4+3] =  vec_data[1];

   alg_data[2*4+0] = -vec_data[4];
   alg_data[2*4+1] =  vec_data[3];
   alg_data[2*4+2] = 0;
   alg_data[2*4+3] =  vec_data[2];

   alg_data[3*4+0] = 0;
   alg_data[3*4+1] = 0;
   alg_data[3*4+2] = 0;
   alg_data[3*4+3] = 0;

   return error;
}

int rox_ansi_array2d_double_algse3_set_velocity ( double ** alg_data, double ** vec_data )
{
   int error = 0;

   alg_data[0][0] = 0;
   alg_data[0][1] = -vec_data[5][0];
   alg_data[0][2] =  vec_data[4][0];
   alg_data[0][3] =  vec_data[0][0];

   alg_data[1][0] =  vec_data[5][0];
   alg_data[1][1] = 0;
   alg_data[1][2] = -vec_data[3][0];
   alg_data[1][3] =  vec_data[1][0];

   alg_data[2][0] = -vec_data[4][0];
   alg_data[2][1] =  vec_data[3][0];
   alg_data[2][2] = 0;
   alg_data[2][3] =  vec_data[2][0];

   alg_data[3][0] = 0;
   alg_data[3][1] = 0;
   alg_data[3][2] = 0;
   alg_data[3][3] = 0;

   return error;
}

int rox_ansi_array_float_algse3_check_velocity_convergence ( int * convergence, float *v, const float vt_conv_thresh, const float vr_conv_thresh )
{
   int error = 0;

   float norm_vt = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
   float norm_vr = v[3]*v[3] + v[4]*v[4] + v[5]*v[5];

   if (( norm_vt < vt_conv_thresh ) && ( norm_vr < vr_conv_thresh ))
   {
      *convergence = 1;
   }
   else
   {
      *convergence = 0;
   }

   return error;
}
