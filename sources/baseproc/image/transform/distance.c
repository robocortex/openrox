//==============================================================================
//
//    OPENROX   : File distance.c
//
//    Contents  : Implementation of distance module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "distance.h"
#include <generated/array2d_sint.h>
#include <baseproc/geometry/point/point2d.h>

#include <inout/system/errors_print.h>

#define F_MEIJSTER(A,B) ((A-B)*(A-B)) + ROW[B]*ROW[B]
#define SEP_MEIJSTER(I,U) (U*U - I*I + ROW[U]*ROW[U] - ROW[I]*ROW[I])/(2 * (U-I))

Rox_ErrorCode rox_array2d_uchar_distancetransform(Rox_Array2D_Sshort distancemap, Rox_Array2D_Point2D_Sshort closestpoints, Rox_Image source)
{
   //TODO ? not sure if we should gain something, but computation is in Sint where we have Sshort in output
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Point2D_Sshort H = NULL;
   Rox_Array2D_Sint G = NULL, T = NULL, S = NULL;
   
   if (!distancemap || !closestpoints || !source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, source); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_sshort_check_size(distancemap, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_point2d_sshort_check_size(closestpoints, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_point2d_sshort_new(&H, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_sint_new(&G, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_sint_new(&T, cols, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_sint_new(&S, cols, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** di = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &di, source);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Sshort ** dd = NULL;
   error = rox_array2d_sshort_get_data_pointer_to_pointer( &dd, distancemap);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint ** dg = NULL;
   error = rox_array2d_sint_get_data_pointer_to_pointer( &dg, G);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint *  dt = NULL; 
   error = rox_array2d_sint_get_data_pointer ( &dt, T );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint *  ds = NULL; 
   error = rox_array2d_sint_get_data_pointer ( &ds, S );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point2D_Sshort * dp = NULL; 
   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer ( &dp, closestpoints);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point2D_Sshort * dh = NULL; 
   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer ( &dh, H);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint max = 1 + rows + cols;

   // First pass
   for (Rox_Sint x = 0; x < cols; x++)
   {
      if (di[0][x])
      {
         dg[0][x] = 0;
         dh[0][x].u = x;
         dh[0][x].v = 0;
      }
      else
      {
         dh[0][x].u = -1;
         dh[0][x].v = -1;
         dg[0][x] = max;
      }

      for (Rox_Sint y = 1; y < rows; y++)
      {
         if (di[y][x])
         {
            dh[y][x].u = x;
            dh[y][x].v = y;
            dg[y][x] = 0;
         }
         else
         {
            dh[y][x] = dh[y - 1][x];
            dg[y][x] = 1 + dg[y - 1][x];
         }
      }

      for (Rox_Sint y = rows - 2; y >= 0; y--)
      {
         if (dg[y + 1][x] < dg[y][x])
         {
            dh[y][x] = dh[y + 1][x];
            dg[y][x] = 1 + dg[y + 1][x];
         }
      }
   }

   // Second pass
   for (Rox_Sint y = 0; y < rows; y++)
   {
      Rox_Sint * ROW = dg[y];
      Rox_Sint q = 0;

      ds[0] = 0;
      dt[0] = 0;

      for (Rox_Sint x = 1; x < cols; x++)
      {
         while (q >= 0 && F_MEIJSTER(dt[q], ds[q]) > F_MEIJSTER(dt[q], x))
         {
            q = q - 1;
         }

         if (q  < 0)
         {
            q = 0;
            ds[0] = x;
         }
         else
         {
            Rox_Sint w = 1 + SEP_MEIJSTER(ds[q],x);
            if (w < cols)
            {
               q = q + 1;
               ds[q] = x;
               dt[q] = w;
            }
         }
      }
      
      for (Rox_Sint x = cols - 1; x >= 0; x--)
      {
         dd[y][x] = F_MEIJSTER(x, ds[q]);

         dp[y][x] = dh[y][ds[q]];

         if (x == dt[q])
         {
            q--;
         }
      }
   }

function_terminate:
   rox_array2d_sint_del(&G);
   rox_array2d_sint_del(&T);
   rox_array2d_sint_del(&S);
   rox_array2d_point2d_sshort_del(&H);
   return error;
}
