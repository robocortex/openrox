//==============================================================================
//
//    OPENROX   : File ansi_matse3.c
//
//    Contents  : Implementation of ansi matse3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_matse3.h"

#include <math.h>
#include <float.h>
#include <baseproc/array/multiply/ansi_mulmatmat.h>
#include <baseproc/maths/linalg/generators/ansi_algse3.h>
#include <baseproc/array/scale/ansi_scale.h>
#include <baseproc/array/copy/ansi_array_float_copy.h>


int rox_ansi_matse3_mulmatmat ( double * T, double * T1, double * T2 )
{
   int error = 0;

   // Multiplication imposing the constraint T[12] = 0; T[13] = 0; T[14] = 0; T[15] = 1;
   T[ 0] = T1[0]*T2[0] + T1[1]*T2[4] + T1[ 2]*T2[8]; T[1] = T1[0]*T2[1] + T1[1]*T2[5] + T1[ 2]*T2[9]; T[ 2] = T1[0]*T2[2] + T1[1]*T2[6] + T1[ 2]*T2[10];
   T[ 4] = T1[4]*T2[0] + T1[5]*T2[4] + T1[ 6]*T2[8]; T[5] = T1[4]*T2[1] + T1[5]*T2[5] + T1[ 6]*T2[9]; T[ 6] = T1[4]*T2[2] + T1[5]*T2[6] + T1[ 6]*T2[10];
   T[ 8] = T1[8]*T2[0] + T1[9]*T2[4] + T1[10]*T2[8]; T[9] = T1[8]*T2[1] + T1[9]*T2[5] + T1[10]*T2[9]; T[10] = T1[8]*T2[2] + T1[9]*T2[6] + T1[10]*T2[10];
   T[12] = 0;                                                                              T[13] = 0;                                         T[14] = 0;

   T[ 3] = T1[ 3] + T1[0]*T2[3] + T1[1]*T2[7] + T1[ 2]*T2[11];
   T[ 7] = T1[ 7] + T1[4]*T2[3] + T1[5]*T2[7] + T1[ 6]*T2[11];
   T[11] = T1[11] + T1[8]*T2[3] + T1[9]*T2[7] + T1[10]*T2[11];
   T[15] = 1;

   return error;
}

int rox_ansi_matse3_float_mulmatmat ( float * T, float * T1, float * T2 )
{
   int error = 0;

   // Multiplication imposing the constraint T[12] = 0; T[13] = 0; T[14] = 0; T[15] = 1;
   T[ 0] = T1[0]*T2[0] + T1[1]*T2[4] + T1[ 2]*T2[8]; T[1] = T1[0]*T2[1] + T1[1]*T2[5] + T1[ 2]*T2[9]; T[ 2] = T1[0]*T2[2] + T1[1]*T2[6] + T1[ 2]*T2[10];
   T[ 4] = T1[4]*T2[0] + T1[5]*T2[4] + T1[ 6]*T2[8]; T[5] = T1[4]*T2[1] + T1[5]*T2[5] + T1[ 6]*T2[9]; T[ 6] = T1[4]*T2[2] + T1[5]*T2[6] + T1[ 6]*T2[10];
   T[ 8] = T1[8]*T2[0] + T1[9]*T2[4] + T1[10]*T2[8]; T[9] = T1[8]*T2[1] + T1[9]*T2[5] + T1[10]*T2[9]; T[10] = T1[8]*T2[2] + T1[9]*T2[6] + T1[10]*T2[10];
   T[12] = 0;                                                                              T[13] = 0;                                         T[14] = 0;

   T[ 3] = T1[ 3] + T1[0]*T2[3] + T1[1]*T2[7] + T1[ 2]*T2[11];
   T[ 7] = T1[ 7] + T1[4]*T2[3] + T1[5]*T2[7] + T1[ 6]*T2[11];
   T[11] = T1[11] + T1[8]*T2[3] + T1[9]*T2[7] + T1[10]*T2[11];
   T[15] = 1;

   return error;
}

int rox_ansi_matse3_float_inv ( float * Ti, float * T )
{
   int error = 0;

   float tx = T[ 3];
   float ty = T[ 7];
   float tz = T[11];

   // R^-1 = R^T
   Ti[ 0] = T[ 0];
   Ti[ 1] = T[ 4];
   Ti[ 2] = T[ 8];
   Ti[ 4] = T[ 1];
   Ti[ 5] = T[ 5];
   Ti[ 6] = T[ 9];
   Ti[ 8] = T[ 2];
   Ti[ 9] = T[ 6];
   Ti[10] = T[10];

   // t^-1 = -R^T*t
   Ti[ 3] = -Ti[0] * tx - Ti[1] * ty - Ti[ 2] * tz;
   Ti[ 7] = -Ti[4] * tx - Ti[5] * ty - Ti[ 6] * tz;
   Ti[11] = -Ti[8] * tx - Ti[9] * ty - Ti[10] * tz;

   // Last row
   Ti[12] = 0;
   Ti[13] = 0;
   Ti[14] = 0;
   Ti[15] = 1;

   return error;
}


int rox_ansi_array_float_matse3_update_left ( float * pose, float * vector )
{
   int error = 0;
   float algebra[4*4];
   float update[4*4];

   error = rox_ansi_array_float_algse3_set_velocity ( algebra, vector );
   if(error)
   { goto function_terminate; }

   error = rox_ansi_array_float_scale_inplace ( algebra, 16, -1.0 );
   if(error)
   { goto function_terminate; }

   error = rox_ansi_array_float_matse3_exponential_algse3 ( update, algebra );
   if(error)
   { goto function_terminate; }

   error = rox_ansi_array_float_mulmatmat ( algebra, 4, 4, update, pose, 4 );
   if(error)
   { goto function_terminate; }

   error = rox_ansi_array_float_copy ( pose, algebra, 16 );
   if(error)
   { goto function_terminate; }

function_terminate:
   return error;
}

int rox_ansi_array_float_matse3_exponential_algse3 ( float * matse3_data, float * algse3_data )
{
   int error = 0;

   float th = 0.0;
   float ux = 0.0;
   float uy = 0.0;
   float uz = 1.0;

   float vx = 0.0;
   float vy = 0.0;
   float vz = 0.0;

   float th_inv = 0.0;

   float sin_th = 0.0;
   float cos_th = 1.0;

   // sin_th/th
   float sin_th_inv = 1.0;

   // (1.0-cos_th)/th
   float cos_th_inv = 0.0;

   // A = [0 -uz*th +uy*th vx; +uz*th 0 -ux*th vy; -uy*th +ux*th 0 vz; 0 0 0 0]
   // R = I + sin_th*[u] + (1-cos_th)*[u]^2 ;
   // Q = I + (1-cos_th)*[u]/th + (1-sin_th/th)*[u]^2 ;
   // t = Q*v;
   // T = [R t; 0 0 0 1];

   vx = algse3_data[3];
   vy = algse3_data[7];
   vz = algse3_data[11];

   th = sqrt(algse3_data[6] * algse3_data[6] + algse3_data[1] * algse3_data[1] + algse3_data[8] * algse3_data[8]);

   // avoid division through 0
   if (th < DBL_EPSILON)
   {
      // limiting values l'Hospital
      sin_th_inv = 1.0;
      cos_th_inv = 0.0;

      sin_th = 0.0;
      cos_th = 1.0;
  }
   else
   {
      sin_th = sin(th);
      cos_th = cos(th);

      th_inv = 1.0 / th;
      sin_th_inv = sin_th * th_inv;
      cos_th_inv = (1.0 - cos_th) * th_inv;

      // Axis of rotation such that ux^2+uy^2+uz^2-1=0
      ux = algse3_data[9] * th_inv;
      uy = algse3_data[2] * th_inv;
      uz = algse3_data[4] * th_inv;
   }

   // Rotation matrix
   matse3_data[0] = 1.0 + (1.0 - cos_th) * (ux * ux - 1.0); // 1.0 + (1.0 - cos_th) * (-uz * uz - uy * uy);
   matse3_data[1] = -sin_th * uz + (1.0 - cos_th) * uy * ux;
   matse3_data[2] =  sin_th * uy + (1.0 - cos_th) * uz * ux;

   matse3_data[4] =  sin_th * uz + (1.0 - cos_th) * uy * ux;
   matse3_data[5] = 1.0 + (1.0 - cos_th) * (uy * uy - 1.0); // 1.0 + (1.0 - cos_th) * (-uz * uz - ux * ux);
   matse3_data[6] = -sin_th * ux + (1.0 - cos_th) * uz * uy;

   matse3_data[8] = -sin_th * uy + (1.0 - cos_th) * uz * ux;
   matse3_data[9] =  sin_th * ux + (1.0 - cos_th) * uz * uy;
   matse3_data[10] = 1.0 + (1.0 - cos_th) * (uz * uz - 1.0); // 1.0 + (1.0 - cos_th) * (-uy * uy - ux * ux);

   //matse3[3] = (1.0 + (1.0 - sin_th_inv) * (-uz * uz - uy * uy)) * vx + (-cos_th_inv * uz + (1.0 - sin_th_inv) * uy * ux) * vy + (cos_th_inv * uy + (1.0 - sin_th_inv) * uz * ux) * vz;
   //matse3[7] = (cos_th_inv * uz + (1.0 - sin_th_inv) * uy * ux) * vx + (1.0 + (1.0 - sin_th_inv) * (-uz * uz - ux * ux)) * vy + (-cos_th_inv * ux + (1.0 - sin_th_inv) * uz * uy) * vz;
   //matse3[11] = (-cos_th_inv * uy + (1.0 - sin_th_inv) * uz * ux) * vx + (cos_th_inv * ux + (1.0 - sin_th_inv) * uz * uy) * vy + (1.0 + (1.0 - sin_th_inv) * (-uy * uy - ux * ux)) * vz;

   // Translation vector
   matse3_data[3] = (1.0 + (1.0 - sin_th_inv) * (ux * ux - 1.0)) * vx + (-cos_th_inv * uz + (1.0 - sin_th_inv) * uy * ux) * vy + (cos_th_inv * uy + (1.0 - sin_th_inv) * uz * ux) * vz;
   matse3_data[7] = (cos_th_inv * uz + (1.0 - sin_th_inv) * uy * ux) * vx + (1.0 + (1.0 - sin_th_inv) * (uy * uy - 1.0)) * vy + (-cos_th_inv * ux + (1.0 - sin_th_inv) * uz * uy) * vz;
   matse3_data[11] = (-cos_th_inv * uy + (1.0 - sin_th_inv) * uz * ux) * vx + (cos_th_inv * ux + (1.0 - sin_th_inv) * uz * uy) * vy + (1.0 + (1.0 - sin_th_inv) * (uz * uz - 1.0)) * vz;

   // Fourth row
   matse3_data[12] = 0.0;
   matse3_data[13] = 0.0;
   matse3_data[14] = 0.0;
   matse3_data[15] = 1.0;

   return error;
}

int rox_ansi_array_double_matse3_exponential_algse3 ( double * matse3_data, double * algse3_data )
{
   int error = 0;

   double th = 0.0;
   double ux = 0.0;
   double uy = 0.0;
   double uz = 1.0;

   double vx = 0.0;
   double vy = 0.0;
   double vz = 0.0;

   double th_inv = 0.0;

   double sin_th = 0.0;
   double cos_th = 1.0;

   // sin_th/th
   double sin_th_inv = 1.0;

   // (1.0-cos_th)/th
   double cos_th_inv = 0.0;

   // A = [0 -uz*th +uy*th vx; +uz*th 0 -ux*th vy; -uy*th +ux*th 0 vz; 0 0 0 0]
   // R = I + sin_th*[u] + (1-cos_th)*[u]^2 ;
   // Q = I + (1-cos_th)*[u]/th + (1-sin_th/th)*[u]^2 ;
   // t = Q*v;
   // T = [R t; 0 0 0 1];

   vx = algse3_data[3];
   vy = algse3_data[7];
   vz = algse3_data[11];

   th = sqrt(algse3_data[6] * algse3_data[6] + algse3_data[1] * algse3_data[1] + algse3_data[8] * algse3_data[8]);

   // avoid division through 0
   if (th < DBL_EPSILON)
   {
      // limiting values l'Hospital
      sin_th_inv = 1.0;
      cos_th_inv = 0.0;

      sin_th = 0.0;
      cos_th = 1.0;
  }
   else
   {
      sin_th = sin(th);
      cos_th = cos(th);

      th_inv = 1.0 / th;
      sin_th_inv = sin_th * th_inv;
      cos_th_inv = (1.0 - cos_th) * th_inv;

      // Axis of rotation such that ux^2+uy^2+uz^2-1=0
      ux = algse3_data[9] * th_inv;
      uy = algse3_data[2] * th_inv;
      uz = algse3_data[4] * th_inv;
   }

   // Rotation matrix
   matse3_data[0] = 1.0 + (1.0 - cos_th) * (ux * ux - 1.0); // 1.0 + (1.0 - cos_th) * (-uz * uz - uy * uy);
   matse3_data[1] = -sin_th * uz + (1.0 - cos_th) * uy * ux;
   matse3_data[2] =  sin_th * uy + (1.0 - cos_th) * uz * ux;

   matse3_data[4] =  sin_th * uz + (1.0 - cos_th) * uy * ux;
   matse3_data[5] = 1.0 + (1.0 - cos_th) * (uy * uy - 1.0); // 1.0 + (1.0 - cos_th) * (-uz * uz - ux * ux);
   matse3_data[6] = -sin_th * ux + (1.0 - cos_th) * uz * uy;

   matse3_data[8] = -sin_th * uy + (1.0 - cos_th) * uz * ux;
   matse3_data[9] =  sin_th * ux + (1.0 - cos_th) * uz * uy;
   matse3_data[10] = 1.0 + (1.0 - cos_th) * (uz * uz - 1.0); // 1.0 + (1.0 - cos_th) * (-uy * uy - ux * ux);

   //matse3[3] = (1.0 + (1.0 - sin_th_inv) * (-uz * uz - uy * uy)) * vx + (-cos_th_inv * uz + (1.0 - sin_th_inv) * uy * ux) * vy + (cos_th_inv * uy + (1.0 - sin_th_inv) * uz * ux) * vz;
   //matse3[7] = (cos_th_inv * uz + (1.0 - sin_th_inv) * uy * ux) * vx + (1.0 + (1.0 - sin_th_inv) * (-uz * uz - ux * ux)) * vy + (-cos_th_inv * ux + (1.0 - sin_th_inv) * uz * uy) * vz;
   //matse3[11] = (-cos_th_inv * uy + (1.0 - sin_th_inv) * uz * ux) * vx + (cos_th_inv * ux + (1.0 - sin_th_inv) * uz * uy) * vy + (1.0 + (1.0 - sin_th_inv) * (-uy * uy - ux * ux)) * vz;

   // Translation vector
   matse3_data[3] = (1.0 + (1.0 - sin_th_inv) * (ux * ux - 1.0)) * vx + (-cos_th_inv * uz + (1.0 - sin_th_inv) * uy * ux) * vy + (cos_th_inv * uy + (1.0 - sin_th_inv) * uz * ux) * vz;
   matse3_data[7] = (cos_th_inv * uz + (1.0 - sin_th_inv) * uy * ux) * vx + (1.0 + (1.0 - sin_th_inv) * (uy * uy - 1.0)) * vy + (-cos_th_inv * ux + (1.0 - sin_th_inv) * uz * uy) * vz;
   matse3_data[11] = (-cos_th_inv * uy + (1.0 - sin_th_inv) * uz * ux) * vx + (cos_th_inv * ux + (1.0 - sin_th_inv) * uz * uy) * vy + (1.0 + (1.0 - sin_th_inv) * (uz * uz - 1.0)) * vz;

   // Fourth row
   matse3_data[12] = 0.0;
   matse3_data[13] = 0.0;
   matse3_data[14] = 0.0;
   matse3_data[15] = 1.0;

   return error;
}

int rox_ansi_matse3_exponential_algse3 ( double ** matse3_data, double ** algse3_data )
{
   int error = 0;

   double th = 0.0;
   double ux = 0.0;
   double uy = 0.0;
   double uz = 1.0;

   double vx = 0.0;
   double vy = 0.0;
   double vz = 0.0;

   double th_inv = 0.0;

   // limiting values l'Hospital

   double sin_th = 0.0;
   double cos_th = 1.0;

   // sin_th/th
   double sin_th_inv = 1.0;

   // (1.0-cos_th)/th
   double cos_th_inv = 0.0;

   // A = [0 -uz*th +uy*th vx; +uz*th 0 -ux*th vy; -uy*th +ux*th 0 vz; 0 0 0 0]
   // R = I + sin_th*[u] + (1-cos_th)*[u]^2 ;
   // Q = I + (1-cos_th)*[u]/th + (1-sin_th/th)*[u]^2 ;
   // t = Q*v;
   // T = [R t; 0 0 0 1];

   vx = algse3_data[0][3];
   vy = algse3_data[1][3];
   vz = algse3_data[2][3];

   th = sqrt(algse3_data[1][2] * algse3_data[1][2] + algse3_data[0][1] * algse3_data[0][1] + algse3_data[2][0] * algse3_data[2][0]);

   // avoid division through 0
   if (th > DBL_EPSILON)
   {
      sin_th = sin(th);
      cos_th = cos(th);

      th_inv = 1.0 / th;
      sin_th_inv = sin_th * th_inv;
      cos_th_inv = (1.0 - cos_th) * th_inv;

      // Axis of rotation such that ux^2+uy^2+uz^2-1=0
      ux = algse3_data[2][1] * th_inv;
      uy = algse3_data[0][2] * th_inv;
      uz = algse3_data[1][0] * th_inv;
   }

   // Rotation matrix
   matse3_data[0][0] = 1.0 + (1.0 - cos_th) * (ux * ux - 1.0); // 1.0 + (1.0 - cos_th) * (-uz * uz - uy * uy);
   matse3_data[0][1] = -sin_th * uz + (1.0 - cos_th) * uy * ux;
   matse3_data[0][2] =  sin_th * uy + (1.0 - cos_th) * uz * ux;

   matse3_data[1][0] =  sin_th * uz + (1.0 - cos_th) * uy * ux;
   matse3_data[1][1] = 1.0 + (1.0 - cos_th) * (uy * uy - 1.0); // 1.0 + (1.0 - cos_th) * (-uz * uz - ux * ux);
   matse3_data[1][2] = -sin_th * ux + (1.0 - cos_th) * uz * uy;

   matse3_data[2][0] = -sin_th * uy + (1.0 - cos_th) * uz * ux;
   matse3_data[2][1] =  sin_th * ux + (1.0 - cos_th) * uz * uy;
   matse3_data[2][2] = 1.0 + (1.0 - cos_th) * (uz * uz - 1.0); // 1.0 + (1.0 - cos_th) * (-uy * uy - ux * ux);

   //matse3[0][3] = (1.0 + (1.0 - sin_th_inv) * (-uz * uz - uy * uy)) * vx + (-cos_th_inv * uz + (1.0 - sin_th_inv) * uy * ux) * vy + (cos_th_inv * uy + (1.0 - sin_th_inv) * uz * ux) * vz;
   //matse3[1][3] = (cos_th_inv * uz + (1.0 - sin_th_inv) * uy * ux) * vx + (1.0 + (1.0 - sin_th_inv) * (-uz * uz - ux * ux)) * vy + (-cos_th_inv * ux + (1.0 - sin_th_inv) * uz * uy) * vz;
   //matse3[2][3] = (-cos_th_inv * uy + (1.0 - sin_th_inv) * uz * ux) * vx + (cos_th_inv * ux + (1.0 - sin_th_inv) * uz * uy) * vy + (1.0 + (1.0 - sin_th_inv) * (-uy * uy - ux * ux)) * vz;

   // Translation vector
   matse3_data[0][3] = (1.0 + (1.0 - sin_th_inv) * (ux * ux - 1.0)) * vx + (-cos_th_inv * uz + (1.0 - sin_th_inv) * uy * ux) * vy + (cos_th_inv * uy + (1.0 - sin_th_inv) * uz * ux) * vz;
   matse3_data[1][3] = (cos_th_inv * uz + (1.0 - sin_th_inv) * uy * ux) * vx + (1.0 + (1.0 - sin_th_inv) * (uy * uy - 1.0)) * vy + (-cos_th_inv * ux + (1.0 - sin_th_inv) * uz * uy) * vz;
   matse3_data[2][3] = (-cos_th_inv * uy + (1.0 - sin_th_inv) * uz * ux) * vx + (cos_th_inv * ux + (1.0 - sin_th_inv) * uz * uy) * vy + (1.0 + (1.0 - sin_th_inv) * (uz * uz - 1.0)) * vz;

   // Fourth row
   matse3_data[3][0] = 0.0;
   matse3_data[3][1] = 0.0;
   matse3_data[3][2] = 0.0;
   matse3_data[3][3] = 1.0;

   return error;
}


int rox_ansi_matse3_generate_centered_at_fixed_distance ( double * cTo_centered, double distance, double phi, double theta, double gamma )
{
   int error = 0;

   double norm = 0;

   double r1[3];
   double r2[3];
   double r3[3];
   double v[3];

   float sg = sin(gamma);
   float cg = cos(gamma);

   // Compute last column of the rotation matrix
   r3[0] = sin(theta)*cos(phi); 
   r3[1] = sin(theta)*sin(phi); 
   r3[2] = -cos(theta);

   if ((r3[0] <= r3[1]) && (r3[0] <= r3[2]))
   {
      v[0] = 1.0; v[1] = 0.0; v[2] = 0.0; 
   }
   if ((r3[1] <= r3[0]) && (r3[1] <= r3[2]))
   {
      v[0] = 0.0; v[1] = 1.0; v[2] = 0.0; 
   }
   if ((r3[2] <= r3[0]) && (r3[2] <= r3[1]))
   {
      v[0] = 0.0; v[1] = 0.0; v[2] = 1.0; 
   }

   // r2 = skew(r3)*v; 
   r2[0] = r3[1]*v[2] - r3[2]*v[1];
   r2[1] = r3[2]*v[0] - r3[0]*v[2];
   r2[2] = r3[0]*v[1] - r3[1]*v[0];

   norm = sqrt(r2[0]*r2[0]+r2[1]*r2[1]+r2[2]*r2[2]);
   
   // r2=r2/norm(r2);
   r2[0] = r2[0] / norm; 
   r2[1] = r2[1] / norm; 
   r2[2] = r2[2] / norm; 

   // r1 = skew(r2)*r3;
   r1[0] = r2[1]*r3[2] - r2[2]*r3[1]; 
   r1[1] = r2[2]*r3[0] - r2[0]*r3[2]; 
   r1[2] = r2[0]*r3[1] - r2[1]*r3[0]; 

   // Compute rotation
   cTo_centered[ 0] = cg*r1[0]-sg*r2[0]; 
   cTo_centered[ 1] = cg*r1[1]-sg*r2[1]; 
   cTo_centered[ 2] = cg*r1[2]-sg*r2[2]; 

   cTo_centered[ 4] = cg*r2[0]+sg*r1[0]; 
   cTo_centered[ 5] = cg*r2[1]+sg*r1[1]; 
   cTo_centered[ 6] = cg*r2[2]+sg*r1[2]; 

   // Compute translation
   cTo_centered[ 3] = 0.0;
   cTo_centered[ 7] = 0.0;
   cTo_centered[11] = distance;

   cTo_centered[12] = 0.0;
   cTo_centered[13] = 0.0;
   cTo_centered[14] = 0.0;
   cTo_centered[15] = 1.0;

//function_terminate:
   return error;
}

int rox_ansi_matse3_float_generate_moving_camera_centered_at_fixed_distance ( float * oTc_centered, float * oTc, float distance, float rx, float ry, float rz )
{
   int error = 0;

   float pTc[16];
   float oTp[16];

   // float cTp[16];

   // Compute the pivot frame Fp relative to the camera frame Fc
   // cTp[ 0] = 1; cTp[ 1] = 0; cTp[ 2] = 0; cTp[ 3] = 0;
   // cTp[ 4] = 0; cTp[ 5] = 1; cTp[ 6] = 0; cTp[ 7] = 0;
   // cTp[ 8] = 0; cTp[ 9] = 0; cTp[10] = 1; cTp[11] = +distance;
   // cTp[12] = 0; cTp[13] = 0; cTp[14] = 0; cTp[15] = 1;

   // Compute the the camera frame Fc relative to pivot frame Fp 
   // pTc[ 0] = 1; pTc[ 1] = 0; pTc[ 2] = 0; pTc[ 3] = 0;
   // pTc[ 4] = 0; pTc[ 5] = 1; pTc[ 6] = 0; pTc[ 7] = 0;
   // pTc[ 8] = 0; pTc[ 9] = 0; pTc[10] = 1; pTc[11] = -distance;
   // pTc[12] = 0; pTc[13] = 0; pTc[14] = 0; pTc[15] = 1;

   // pTp * pTc
   pTc[ 0] =                           cos(ry)*cos(rz); pTc[ 1] =                          -cos(ry)*sin(rz); pTc[ 2] =          sin(ry); pTc[ 3] = -distance*sin(ry)        ;
   pTc[ 4] = cos(rx)*sin(rz) + cos(rz)*sin(rx)*sin(ry); pTc[ 5] = cos(rx)*cos(rz) - sin(rx)*sin(ry)*sin(rz); pTc[ 6] = -cos(ry)*sin(rx); pTc[ 7] = +distance*cos(ry)*sin(rx);
   pTc[ 8] = sin(rx)*sin(rz) - cos(rx)*cos(rz)*sin(ry); pTc[ 9] = cos(rz)*sin(rx) + cos(rx)*sin(ry)*sin(rz); pTc[10] =  cos(rx)*cos(ry); pTc[11] = -distance*cos(rx)*cos(ry);
   pTc[12] =                                         0; pTc[13] =                                         0; pTc[14] =                0; pTc[15] =                         1;

   // Compute the pivot frame Fp relative to the object frame Fo
   // oTp = oTc * cTp;

   oTp[ 0] = oTc[0]; oTp[ 1] = oTc[1]; oTp[ 2] = oTc[ 2]; oTp[ 3] = oTc[ 3] + distance * oTc[ 2];
   oTp[ 4] = oTc[4]; oTp[ 5] = oTc[5]; oTp[ 6] = oTc[ 6]; oTp[ 7] = oTc[ 7] + distance * oTc[ 6];
   oTp[ 8] = oTc[8]; oTp[ 9] = oTc[9]; oTp[10] = oTc[10]; oTp[11] = oTc[11] + distance * oTc[10];
   oTp[12] =      0; oTp[13] =      0; oTp[14] =       0; oTp[15] =                            1;

   // Compute the new frame 
   // oTc_centered = oTp * pTc;

   error = rox_ansi_matse3_float_mulmatmat ( oTc_centered, oTp, pTc );

// function_terminate:
   return error;
}

int rox_ansi_matse3_float_generate_moving_object_constained_axes ( float * cTo_constrained, float * cTo, float tra[3], float rot[3] )
{
   int error = 0;

   float oTp[16];

   float ux = 0.0;
   float uy = 0.0;
   float uz = 1.0;

   float angle = sqrt( rot[0] * rot[0] + rot[1] * rot[1] + rot[2] * rot[2] );
   if (fabs(angle) > FLT_EPSILON)
   { 
      ux = rot[0]/angle;
      uy = rot[1]/angle;
      uz = rot[2]/angle;
  }

   float cosa = cos(angle);
   float sina = sin(angle);

   // Compute the pivot frame Fp relative to the object frame Fo
   
   oTp[12] =    0.0; 
   oTp[13] =    0.0; 
   oTp[14] =    0.0; 
   oTp[15] =    1.0;

   // Set rotation matrix
   oTp[ 0] = 1.0 - uz * uz + uz * uz * cosa - uy * uy + uy * uy * cosa;
   oTp[ 1] = -sina * uz + uy * ux - uy * ux * cosa;
   oTp[ 2] =  sina * uy + uz * ux - uz * ux * cosa;
   oTp[ 4] =  sina * uz + uy * ux - uy * ux * cosa;
   oTp[ 5] = 1.0 - uz * uz + uz * uz * cosa - ux * ux + ux * ux * cosa;
   oTp[ 6] = -sina * ux + uz * uy - uz * uy * cosa;
   oTp[ 8] = -sina * uy + uz * ux - uz * ux * cosa;
   oTp[ 9] =  sina * ux + uz * uy - uz * uy * cosa;
   oTp[10] = 1.0 - uy * uy + uy * uy * cosa - ux * ux + ux * ux * cosa;

   // Set translation vector
   oTp[ 3] = tra[0];
   oTp[ 7] = tra[1];
   oTp[11] = tra[2];

   // Compute the new frame 
   // cTo_constrained = oTp * pTc;
   error = rox_ansi_matse3_float_mulmatmat ( cTo_constrained, cTo, oTp );

// function_terminate:
   return error;
}
