//==============================================================================
//
//    OPENROX   : File roxrgba_to_roxlab.c
//
//    Contents  : Implementation of roxrgba_to_roxlab module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "roxrgba_to_roxlab.h"

#include <baseproc/maths/maths_macros.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_roxrgba_to_roxlab(Rox_Array2D_Uint dest, const Rox_Array2D_Uint source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Float X, Y, Z, fX, fY, fZ, R, G, B, L, a, b;

   if (!source || !dest) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint width = 0, height = 0;
   error = rox_array2d_uint_get_size(&height, &width, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(dest, height, width); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Uint ** ds = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &ds, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** dd = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &dd, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < height; i++)
   {
      Rox_Uchar * rd = (Rox_Uchar*)dd[i];
      Rox_Uchar * rs = (Rox_Uchar*)ds[i];

      for ( Rox_Sint j = 0; j < width; j++)
      {
         R = rs[0];
         G = rs[1];
         B = rs[2];

         R = R / 255.0f;
         G = G / 255.0f;
         B = B / 255.0f;

         X = 0.412453f * R + 0.357580f * G + 0.180423f * B;
         Y = 0.212671f * R + 0.715160f * G + 0.072169f * B;
         Z = 0.019334f * R + 0.119193f * G + 0.950227f * B;

         X = X / 0.950456f;
         Z = Z / 1.088754f;

         if (Y > 0.008856)
         {
            fY = (Rox_Float) pow((double)Y, 1.0/3.0);
            L = (Rox_Float)((int)(116.0*fY - 16.0 + 0.5));
         }
         else
         {
            fY = (Rox_Float)(7.787*Y + 16.0/116.0);
            L = (Rox_Float)((int)(903.3*Y + 0.5));
         }

         if (X > 0.008856) fX = (Rox_Float) pow((double)X, 1.0/3.0);
         else fX = (Rox_Float)(7.787*X + 16.0/116.0);

         if (Z > 0.008856) fZ = (Rox_Float) pow((double)Z, 1.0/3.0);
         else fZ = (Rox_Float)(7.787*Z + 16.0/116.0);

         a = (Rox_Float)((int)(500.0*(fX - fY) + 0.5));
         b = (Rox_Float)((int)(200.0*(fY - fZ) + 0.5));

         rd[0] = (Rox_Uchar) L;
         rd[1] = (Rox_Uchar) a;
         rd[2] = (Rox_Uchar) b;

         rd += 4;
         rs += 4;
      }
   }

function_terminate:
   return error;
}
