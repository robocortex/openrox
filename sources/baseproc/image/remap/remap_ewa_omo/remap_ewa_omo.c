//============================================================================
//
//    OPENROX   : File remap_ewa_omo.c
//
//    Contents  : Implementation of remap_ewa_omo module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "remap_ewa_omo.h"

#include <baseproc/maths/maths_macros.h>
#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d_struct.h>

#include <inout/system/errors_print.h>

Rox_Double computeLanczos(Rox_Double x)
{
   Rox_Double xoverrad = x * 0.5;
   if (x == 0.0) return 1.0;
   return (sin(x)/x) * (sin(xoverrad)/xoverrad);
}

Rox_Double computeTriangle(Rox_Double x)
{
   return 1.0 - fabs(x);
}

Rox_ErrorCode rox_remap_ewa_omo_uchar (
   Rox_Image output, 
   Rox_Array2D_Uint output_mask_output, 
   Rox_Image input, 
   Rox_MeshGrid2D_Float grid
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double fk, fl;
   Rox_Double cx, cy;
   Rox_Sint icx, icy;
   Rox_Double dudx, dudy, dvdx, dvdy;
   Rox_Double A,B,C,F, detQuad;
   Rox_Double trace, discriminant, l1,l2,ls;
   Rox_Double majorlength, minorlength, eccentricity;
   Rox_Double eigvec1x, eigvec1y, normeigvec1;
   Rox_Double eigvec2x, eigvec2y, normeigvec2;
   Rox_Double majoraxisx, majoraxisy, normaxis;
   Rox_Double minoraxisx, minoraxisy;
   Rox_Double minx, miny, maxx, maxy;
   Rox_Double x, weight, totalweight, num, val;
   Rox_Double tabWeight[1024];
   const Rox_Double max_eccentricity = 8.0;

   if (!output || !output_mask_output || !input || !grid) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, output); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint incols = 0, inrows = 0;
   error = rox_array2d_uchar_get_size(&inrows, &incols, input); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(output_mask_output, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_check_size(grid, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** out = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &out, output);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Uint ** out_masko = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &out_masko, output_mask_output);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Uchar ** in = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &in, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** grid_u_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_u_data, grid->u );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** grid_v_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_v_data, grid->v );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Build weight for gaussian
   for (Rox_Sint k = 0; k < 1024; k++)
   {
      weight = ((Rox_Double)k) / 1023.0;
      tabWeight[k] = exp(-2.0 * weight);
   }

   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         out[i][j] = 0;
         out_masko[i][j] = 0;
         if ((i == (rows - 1)) || (j == (cols - 1))) continue;

         // Elipse center
         cx = grid_u_data[i][j];
         cy = grid_v_data[i][j];

         icx = (Rox_Sint) cx;
         icy = (Rox_Sint) cy;

         if (icx < 0 || icy < 0) continue;
         if (icx >= incols || icy >= inrows) continue;

         // Derivatives of texture space wrt screen space
         dudx = grid_u_data[i][j+1] - cx;
         dudy = grid_u_data[i+1][j] - cx;
         dvdx = grid_v_data[i][j+1] - cy;
         dvdy = grid_v_data[i+1][j] - cy;

         // [x y] = J^-1 * [u v]
         // x^2 * y^2 = 1
         // Collect u^2, v^2 and u*v give the coefficients :
         A = dvdx * dvdx + dvdy * dvdy + 1.0;
         B = -dudy * dvdy - dudx * dvdx;
         C = dudx * dudx + dudy * dudy + 1.0;
         F = 1.0 / (A*C-B*B);
         A *= F;
         B *= F;
         C *= F;

         // Compute eigen values, solve (det(A-lambda*I)
         // Note that length of an axis = 1/sqrt(eigenvalue)
         trace = A + C;
         detQuad = A*C - B*B;
         discriminant = trace*trace - 4.0 * detQuad;
         if (discriminant < DBL_EPSILON) discriminant = 0.0;
         l1 = 0.5 * (trace + sqrt(discriminant));
         l2 = 0.5 * (trace - sqrt(discriminant));

         // Make sure the values are sorted in ascending order
         if (l2 < l1)
         {
            ls = l2;
            l2 = l1;
            l1 = ls;
         }

         majorlength = 1.0 / sqrt(l1);
         minorlength = 1.0 / sqrt(l2);
         eccentricity = majorlength / minorlength;

         if (eccentricity > max_eccentricity)
         {
            // Scale the minor axis to make eccentricity == max_eccenttricity

            // Compute possible eigen vectors
            eigvec1x = B;
            eigvec1y = l1 - A;
            eigvec2x = l1 - C;
            eigvec2y = B;

            // Pick the vector with greatest length, normalize to get a unit vector
            normeigvec1 = eigvec1x * eigvec1x + eigvec1y * eigvec1y;
            normeigvec2 = eigvec2x * eigvec2x + eigvec2y * eigvec2y;
            if (normeigvec1 < normeigvec2)
            {
               normaxis = 1.0 / sqrt(normeigvec2);
               majoraxisx = eigvec2x * normaxis;
               majoraxisy = eigvec2y * normaxis;
            }
            else
            {
               normaxis = 1.0 / sqrt(normeigvec1);
               majoraxisx = eigvec1x * normaxis;
               majoraxisy = eigvec1y * normaxis;
            }

            // Minor axis is the orthogonal complement
            minoraxisx = -majoraxisy;
            minoraxisy = majoraxisx;
            minorlength = eccentricity / max_eccentricity;

            // Scale
            minoraxisx *= minorlength;
            minoraxisy *= minorlength;
            majoraxisx *= majorlength;
            majoraxisy *= majorlength;

            // Update elipse quadratic form
            A = majoraxisy * majoraxisy + minoraxisy * minoraxisy + 1.0;
            B = -majoraxisy * majoraxisx - minoraxisy * minoraxisx;
            C = majoraxisx * majoraxisx + minoraxisx * minoraxisx + 1.0;
            F = 1.0 / (A*C-B*B);
            A *= F;
            B *= F;
            C *= F;
         }

         // Boudning box (gradient of elipse is the tangent)
         detQuad = A*C-B*B;
         maxx = sqrt(C * detQuad) / detQuad;
         minx = - maxx;
         maxy = sqrt(A * detQuad) / detQuad;
         miny = -maxy;

         // Align on pixel
         minx += cx;
         maxx += cx;
         miny += cy;
         maxy += cy;

         // iminx = ceil(minx);
         // imaxx = floor(maxx);
         // iminy = ceil(minx);
         // imaxy = floor(maxy);

         // Loop and sum
         num = 0;
         totalweight = 0;
         for (Rox_Sint k = (Rox_Sint) miny; k <= (Rox_Sint)maxy; k++)
         {
            fk = -cy + (Rox_Double)k;
            for (Rox_Sint l = (Rox_Sint) minx; l <= (Rox_Sint)maxx; l++)
            {
               fl = -cx + (Rox_Double)l;

               x = A * fl * fl + C * fk * fk + 2.0 * B * fl * fk;

               if (x < 1.0 && k >= 0 && l >= 0 && k < inrows - 1 && l < incols - 1)
               {
                  weight = tabWeight[ROX_MIN(1023, (int)(x * 1024.0))];
                  totalweight += weight;
                  num += weight * (Rox_Double)in[k][l];
               }
            }
         }

         // Clamp
         val = num / totalweight;
         if (val < 0) val = 0;
         if (val > 255.0) val = 255.0;
         out[i][j] = (Rox_Uchar) val;
      }
   }

function_terminate:
   return error;
}
