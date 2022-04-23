//==============================================================================
//
//    OPENROX   : File dog.c
//
//    Contents  : Implementation of dog module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "dog.h"

#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/solve/symm3x3solve.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/rectangle/rectangle_struct.h>

#include <inout/system/errors_print.h>

#define SRAID_IMG_BORDER 5
#define SRAID_MAX_INTERPOLATION_STEPS 5

Rox_ErrorCode rox_dogdetector_interpolate(Rox_Dog_Feature feat, Rox_Array2D_Float_Collection dogspace, int i, int j, int scale, Rox_Array2D_Double sol, Rox_Array2D_Double H, Rox_Array2D_Double iH, Rox_Array2D_Double diff, Rox_Float contrast_threshold, Rox_Float curvature_adv_threshold, Rox_Uint nbintervals)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   int iter;
   Rox_Double xi, xj, xscale;
   int dogspacesize;
   Rox_Array2D_Float curdog, prevdog, nextdog;
   Rox_Float **dcd, **dpd, **dnd;
   Rox_Sint cols, rows;
   Rox_Double dx, dy, ds, cv, dxx, dyy, dss, dxy, dxs, dys, upc, contrast, tr, det, edgeness;
   Rox_Double **dH, **dsol, **ddiff;

   error = rox_array2d_double_get_data_pointer_to_pointer(&dH, H);

   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer(&dsol,sol);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer(&ddiff,diff);
   ROX_ERROR_CHECK_TERMINATE ( error );

   dogspacesize = rox_array2d_float_collection_get_count(dogspace);

   prevdog = rox_array2d_float_collection_get(dogspace, scale - 1);
   curdog = rox_array2d_float_collection_get(dogspace, scale);
   nextdog = rox_array2d_float_collection_get(dogspace, scale + 1);
   // !!! Check for input is done outside for speed !!!

   iter = 0;
   while (iter < SRAID_MAX_INTERPOLATION_STEPS)
   {
      error = rox_array2d_float_get_data_pointer_to_pointer(&dpd, prevdog);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_float_get_data_pointer_to_pointer(&dcd, curdog);

      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_float_get_data_pointer_to_pointer(&dnd, nextdog);
      ROX_ERROR_CHECK_TERMINATE ( error );


      error = rox_array2d_float_get_size(&rows, &cols, curdog); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // current value
      cv = dcd[i][j];

      // first order
      dx = .5 * (dcd[i][j + 1] - dcd[i][j - 1]);
      dy = .5 * (dcd[i + 1][j] - dcd[i - 1][j]);
      ds = .5 * (dnd[i][j] - dpd[i][j]);
      ddiff[0][0] = dx;
      ddiff[1][0] = dy;
      ddiff[2][0] = ds;

      // Second horder
      dxx = dcd[i][j + 1] + dcd[i][j - 1] - 2.0 * cv;
      dyy = dcd[i + 1][j] + dcd[i - 1][j] - 2.0 * cv;
      dss = dnd[i][j] + dpd[i][j] - 2.0 * cv;
      dxy = 0.25 * (dcd[i + 1][j + 1] - dcd[i + 1][j - 1] - dcd[i - 1][j + 1] + dcd[i - 1][j - 1]);
      dxs = 0.25 * (dnd[i][j + 1] - dnd[i][j - 1] - dpd[i][j + 1] + dpd[i][j - 1]);
      dys = 0.25 * (dnd[i + 1][j] - dnd[i - 1][j] - dpd[i + 1][j] + dpd[i - 1][j]);
      dH[0][0] = dxx; dH[0][1] = dxy; dH[0][2] = dxs;
      dH[1][0] = dxy; dH[1][1] = dyy; dH[1][2] = dys;
      dH[2][0] = dxs; dH[2][1] = dys; dH[2][2] = dss;

      // Minimize

      error = rox_array2d_double_symm3x3_solve(sol, H, diff); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      xj = -dsol[0][0];
      xi = -dsol[1][0];
      xscale = -dsol[2][0];

      // Check for convergence
      if (fabs(xi) < 0.5 && fabs(xj) < 0.5 && fabs(xscale) < 0.5) break;

      // Update params
      i += ROX_ROUND_DOUBLE(xi);
      j += ROX_ROUND_DOUBLE(xj);
      scale += ROX_ROUND_DOUBLE(xscale);

      // Check if new value is outside allowed range
      if (scale < 1 || scale >= dogspacesize - 1) {error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; goto function_terminate; } // ROX_ERROR_CHECK_TERMINATE(error)}
      if (i < SRAID_IMG_BORDER || j < SRAID_IMG_BORDER) {error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; goto function_terminate; } // ROX_ERROR_CHECK_TERMINATE(error)}
      if (i >= rows - SRAID_IMG_BORDER || j >= cols - SRAID_IMG_BORDER) {error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; goto function_terminate; } // ROX_ERROR_CHECK_TERMINATE(error)}

      // Update dogspace pointers
      prevdog = rox_array2d_float_collection_get(dogspace, scale - 1);
      curdog = rox_array2d_float_collection_get(dogspace, scale);
      nextdog = rox_array2d_float_collection_get(dogspace, scale + 1);

      iter++;
   }

   if (iter >= SRAID_MAX_INTERPOLATION_STEPS) {error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; goto function_terminate;}

   // first order for last increment
   dx = .5 * (dcd[i][j + 1] - dcd[i][j - 1]);
   dy = .5 * (dcd[i + 1][j] - dcd[i - 1][j]);
   ds = .5 * (dnd[i][j] - dpd[i][j]);

   // Interpolate contrast
   upc = .5 * (dx * xj + dy * xi + ds * xscale);
   contrast = dcd[i][j] + upc;

   // Check contrast validity
   if (fabs(contrast) < (contrast_threshold / ((double) nbintervals))) {error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE(error)}

   // Check edgeness

   dxx = dcd[i][j + 1] + dcd[i][j - 1] - 2.0 * cv;
   dyy = dcd[i + 1][j] + dcd[i - 1][j] - 2.0 * cv;
   dxy = 0.25 * (dcd[i + 1][j + 1] - dcd[i + 1][j - 1] - dcd[i - 1][j + 1] + dcd[i - 1][j - 1]);
   tr = dxx + dyy;
   det = dxx * dyy - dxy * dxy;
   if (det <= DBL_EPSILON) {error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; goto function_terminate; } // ROX_ERROR_CHECK_TERMINATE(error)}
   edgeness = tr * tr / det;

   if (edgeness >= curvature_adv_threshold) {error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; goto function_terminate; } // ROX_ERROR_CHECK_TERMINATE(error)}

   feat->i = i;
   feat->j = j;
   feat->lvl = scale;
   feat->flvl = (float) xscale;
   feat->x = (float) (xj + j);
   feat->y = (float) (xi + i);

function_terminate:
   return error;
}

Rox_ErrorCode rox_dogdetector_process(Rox_Dog_Feature * features, Rox_Uint * countFeatures, Rox_Array2D_Float_Collection dogspace, Rox_Rect_Sint image_bounds, Rox_Float contrast_threshold, Rox_Float curvature_threshold, Rox_Float preliminary_threshold, Rox_Uint nbintervals, Rox_Uint octave, Rox_Float sigma)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint i, j, slice, isminimal;
   Rox_Uint countslice;
   Rox_Float **dp, **dn, **dc;
   Rox_Float cur;
   Rox_Array2D_Float curdog, prevdog, nextdog;
   Rox_Array2D_Double H, iH, diff, sol;
   Rox_Uint count, allocated;
   Rox_Dog_Feature_Struct curfeat, *lfeats;
   Rox_Float curvature_adv_threshold;
   Rox_Float scale, finter;
   Rox_Sint left, right, top, bottom;


   if (!features) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!countFeatures) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   countslice = rox_array2d_float_collection_get_count(dogspace);
   if (countslice < 3) { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   scale = (Rox_Float)(ROX_POWF(2.0, (int)octave));

   curdog = rox_array2d_float_collection_get(dogspace, 1);

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_float_get_size(&rows, &cols, curdog); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   left = image_bounds->x;
   if (left < SRAID_IMG_BORDER) left = SRAID_IMG_BORDER;
   top = image_bounds->y;
   if (top < SRAID_IMG_BORDER) top = SRAID_IMG_BORDER;

   bottom = image_bounds->y + image_bounds->height - 1;
   if (bottom >= rows - SRAID_IMG_BORDER) bottom = rows - SRAID_IMG_BORDER - 1;
   if (bottom < top) bottom = top + 1;
   right = image_bounds->x + image_bounds->width - 1;
   if (right >= cols - SRAID_IMG_BORDER) right = cols - SRAID_IMG_BORDER - 1;
   if (right < left) right = left + 1;

   error = rox_array2d_double_new(&H, 3, 3);

   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&iH, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&diff, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&sol, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   curvature_adv_threshold = (Rox_Float) ((curvature_threshold + 1.0) * (curvature_threshold + 1.0) / curvature_threshold);

   count = 0;

   allocated = 100;
   lfeats = (Rox_Dog_Feature ) rox_memory_allocate(sizeof(Rox_Dog_Feature_Struct), allocated);
   if (!lfeats)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (slice = 1; slice < countslice - 1; slice++)
   {
      prevdog = rox_array2d_float_collection_get(dogspace, slice - 1);
      curdog = rox_array2d_float_collection_get(dogspace, slice);
      nextdog = rox_array2d_float_collection_get(dogspace, slice + 1);


      error = rox_array2d_float_get_data_pointer_to_pointer(&dp, prevdog); ROX_ERROR_CHECK_TERMINATE ( error ); 
      error = rox_array2d_float_get_data_pointer_to_pointer(&dc, curdog); ROX_ERROR_CHECK_TERMINATE ( error ); 
      error = rox_array2d_float_get_data_pointer_to_pointer(&dn, nextdog); ROX_ERROR_CHECK_TERMINATE ( error ); 

      for (i = (Rox_Uint) top; i <= (Rox_Uint)bottom; i++)
      {
         for (j = (Rox_Uint) left; j <= (Rox_Uint)right; j++)
         {
            cur = dc[i][j];

            // Preliminary test
            if (fabs(cur) <= preliminary_threshold) continue;

            // Check neighboorhood for extrema validity
            if (cur > 0.0)
            {
               if (cur < dp[i - 1][j - 1]) continue;
               if (cur < dp[i - 1][j]) continue;
               if (cur < dp[i - 1][j + 1]) continue;
               if (cur < dp[i][j - 1]) continue;
               if (cur < dp[i][j]) continue;
               if (cur < dp[i][j + 1]) continue;
               if (cur < dp[i + 1][j - 1]) continue;
               if (cur < dp[i + 1][j]) continue;
               if (cur < dp[i + 1][j + 1]) continue;

               if (cur < dn[i - 1][j - 1]) continue;
               if (cur < dn[i - 1][j]) continue;
               if (cur < dn[i - 1][j + 1]) continue;
               if (cur < dn[i][j - 1]) continue;
               if (cur < dn[i][j]) continue;
               if (cur < dn[i][j + 1]) continue;
               if (cur < dn[i + 1][j - 1]) continue;
               if (cur < dn[i + 1][j]) continue;
               if (cur < dn[i + 1][j + 1]) continue;

               if (cur < dc[i - 1][j - 1]) continue;
               if (cur < dc[i - 1][j]) continue;
               if (cur < dc[i - 1][j + 1]) continue;
               if (cur < dc[i][j - 1]) continue;
               if (cur < dc[i][j + 1]) continue;
               if (cur < dc[i + 1][j - 1]) continue;
               if (cur < dc[i + 1][j]) continue;
               if (cur < dc[i + 1][j + 1]) continue;

               isminimal = 1;
            }
            else
            {
               if (cur > dp[i - 1][j - 1]) continue;
               if (cur > dp[i - 1][j]) continue;
               if (cur > dp[i - 1][j + 1]) continue;
               if (cur > dp[i][j - 1]) continue;
               if (cur > dp[i][j]) continue;
               if (cur > dp[i][j + 1]) continue;
               if (cur > dp[i + 1][j - 1]) continue;
               if (cur > dp[i + 1][j]) continue;
               if (cur > dp[i + 1][j + 1]) continue;

               if (cur > dn[i - 1][j - 1]) continue;
               if (cur > dn[i - 1][j]) continue;
               if (cur > dn[i - 1][j + 1]) continue;
               if (cur > dn[i][j - 1]) continue;
               if (cur > dn[i][j]) continue;
               if (cur > dn[i][j + 1]) continue;
               if (cur > dn[i + 1][j - 1]) continue;
               if (cur > dn[i + 1][j]) continue;
               if (cur > dn[i + 1][j + 1]) continue;

               if (cur > dc[i - 1][j - 1]) continue;
               if (cur > dc[i - 1][j]) continue;
               if (cur > dc[i - 1][j + 1]) continue;
               if (cur > dc[i][j - 1]) continue;
               if (cur > dc[i][j + 1]) continue;
               if (cur > dc[i + 1][j - 1]) continue;
               if (cur > dc[i + 1][j]) continue;
               if (cur > dc[i + 1][j + 1]) continue;

               isminimal = 0;
            }

            // Interpolation of local maxima for current feature in the 3D space of image+scale
            error = rox_dogdetector_interpolate(&curfeat, dogspace, i, j, slice, sol, H, iH, diff, contrast_threshold, curvature_adv_threshold, nbintervals);
            if (error) continue;

            // Update feature given current octave
            curfeat.x *= scale;
            curfeat.y *= scale;
            curfeat.octave = octave;
            finter = (Rox_Float) (curfeat.flvl + curfeat.lvl);
            curfeat.scale = (float) (sigma *  ROX_POWF(2.0f, ((float) octave) + ((float) finter) / ((float) nbintervals)));
            curfeat.octave_scale = (float) (sigma *  ROX_POWF(2.0f, ((float) finter) / ((float) nbintervals)));
            curfeat.isminimal = isminimal;

            lfeats[count] = curfeat;

            // Increase memory if needed
            count++;
            if (count >= allocated)
            {
               allocated += 100;
               lfeats = (Rox_Dog_Feature )rox_memory_reallocate(lfeats, sizeof(Rox_Dog_Feature_Struct), allocated);
               if (!lfeats)
               {
                  error = ROX_ERROR_NULL_POINTER;
                  ROX_ERROR_CHECK_TERMINATE(error)
               }
            }
         }
      }
   }

   *countFeatures = count;
   *features = lfeats;

   // Do not delete the following error setting unless you know what you do
   error = ROX_ERROR_NONE;

function_terminate:
   rox_array2d_double_del(&H);
   rox_array2d_double_del(&iH);
   rox_array2d_double_del(&diff);
   rox_array2d_double_del(&sol);

   return error;
}

Rox_ErrorCode rox_dogspace_create(Rox_Array2D_Float_Collection *dogspace, Rox_Array2D_Float_Collection scalespace)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float_Collection retspace = NULL;


   if (!dogspace) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!scalespace) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint size = rox_array2d_float_collection_get_count(scalespace);
   if (size < 2) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   Rox_Array2D_Float cur = rox_array2d_float_collection_get(scalespace, 0);

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, cur);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_collection_new(&retspace, size - 1, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Uint i = 0; i < size - 1; i++)
   {
      Rox_Array2D_Float res = rox_array2d_float_collection_get(retspace, i);
      Rox_Array2D_Float cur = rox_array2d_float_collection_get(scalespace, i);
      Rox_Array2D_Float next = rox_array2d_float_collection_get(scalespace, i + 1);

      error = rox_array2d_float_substract(res, next, cur);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   *dogspace = retspace;

function_terminate:
   return error;
}
