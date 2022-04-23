//============================================================================
//
//    OPENROX   : File gaussian_noise.c
//
//    Contents  : Implementation of gaussian_noise module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "gaussian_noise.h"

#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>

// Temporary implementation of Gaussian random generator
// See http://people.sc.fsu.edu/~jburkardt/f77_src/ziggurat/ziggurat.html
// This is a fast gaussian rng.

unsigned long int shr3 (unsigned long int *jsr)
{
   unsigned long int value;

   value = *jsr;

   *jsr = ( *jsr ^ ( *jsr <<   13 ) );
   *jsr = ( *jsr ^ ( *jsr >>   17 ) );
   *jsr = ( *jsr ^ ( *jsr <<    5 ) );

   value = value + *jsr;

   return value;
}

float r4_uni(unsigned long int *jsr)
{
   unsigned long int jsr_input;
   Rox_Float value;

   jsr_input = *jsr;

   *jsr = ( *jsr ^ ( *jsr <<   13 ) );
   *jsr = ( *jsr ^ ( *jsr >>   17 ) );
   *jsr = ( *jsr ^ ( *jsr <<    5 ) );

   value = (Rox_Float) fmod ( 0.5 + ( Rox_Float ) ( jsr_input + *jsr ) / 65536.0 / 65536.0, 1.0 );

   return value;
}

Rox_Float r4_nor (unsigned long int *jsr, Rox_Sint kn[128], Rox_Float fn[128], Rox_Float wn[128])
{
   int hz;
   int iz;
   const Rox_Float r = 3.442620f;
   Rox_Float value;
   Rox_Float x;
   Rox_Float y;

   hz = shr3 ( jsr );
   iz = ( hz & 127 );

   if ( abs ( hz ) < kn[iz] )
   {
      value = ( Rox_Float ) ( hz ) * wn[iz];
   }
   else
   {
      for ( ; ; )
      {
         if ( iz == 0 )
         {
            for ( ; ; )
            {
               x = (Rox_Float)(-0.2904764 * log ( r4_uni ( jsr ) ));
               y = (Rox_Float)( -log ( r4_uni ( jsr ) ));
               if ( x * x <= y + y )
               {
                  break;
               }
            }

            if ( hz <= 0 )
            {
               value = - r - x;
            }
            else
            {
               value = + r + x;
            }
            break;
         }

         x = ( Rox_Float ) ( hz ) * wn[iz];

         if ( fn[iz] + r4_uni ( jsr ) * ( fn[iz-1] - fn[iz] ) < exp ( - 0.5 * x * x ) )
         {
            value = x;
            break;
         }

         hz = shr3 ( jsr );
         iz = ( hz & 127 );

         if ( abs ( hz ) < kn[iz] )
         {
            value = ( Rox_Float ) ( hz ) * wn[iz];
            break;
         }
      }
   }

   return value;
}

void r4_nor_setup(int kn[128], Rox_Float fn[128], Rox_Float wn[128])
{
   Rox_Double dn = 3.442619855899;
   const Rox_Double m1 = 2147483648.0;
   Rox_Double q;
   Rox_Double tn = 3.442619855899;
   const Rox_Double vn = 9.91256303526217E-03;

   q = vn / exp ( - 0.5 * dn * dn );

   kn[0] = ( int ) ( ( dn / q ) * m1 );
   kn[1] = 0;

   wn[0] = ( Rox_Float ) ( q / m1 );
   wn[127] = ( Rox_Float ) ( dn / m1 );

   fn[0] = 1.0;
   fn[127] = ( Rox_Float ) ( exp ( - 0.5 * dn * dn ) );

   for ( int i = 126; 1 <= i; i-- )
   {
      dn = sqrt ( - 2.0 * log ( vn / dn + exp ( - 0.5 * dn * dn ) ) );
      kn[i+1] = ( int ) ( ( dn / tn ) * m1 );
      tn = dn;
      fn[i] = ( Rox_Float ) ( exp ( - 0.5 * dn * dn ) );
      wn[i] = ( Rox_Float ) ( dn / m1 );
   }

   return;
}

// Gaussian noise image generator

Rox_ErrorCode rox_array2d_uchar_gaussian_noise(Rox_Image dst, Rox_Image src, Rox_Double sigma)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   unsigned long int seed = 123456789;
   Rox_Float fn[128];
   int kn[128];
   Rox_Float wn[128];
   Rox_Float value;

   if (!dst || !src) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, src); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_check_size(dst, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** ds = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &ds, src);
   Rox_Uchar ** dd = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &dd, dst);

   r4_nor_setup(kn, fn, wn);

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         value = (Rox_Float) (r4_nor(&seed, kn, fn, wn) * sigma + (Rox_Float) ds[i][j]);

         if (value < 0.0) value = 0.0;
         else if (value > 255.0) value = 255.0;

         dd[i][j] = (Rox_Uchar) value;
      }
   }

function_terminate:
   return error;
}
