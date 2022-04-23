//==============================================================================
//
//    OPENROX   : File optimalcalib.c
//
//    Contents  : Implementation of optimalcalib module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "optimalcalib.h"

#include <float.h>

#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/transforms/distortion/point2d_undistort.h>
#include <inout/system/errors_print.h>

#define NB_SAMPLES_PER_SIDE 10

Rox_ErrorCode rox_calibration_optimalview(Rox_Array2D_Double newcalib, Rox_Array2D_Double calib, Rox_Array2D_Double distortion, Rox_Uint image_width, Rox_Uint image_height)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double sampling;
   Rox_Double stepx, stepy;
   Rox_Double minu, maxu, minv, maxv;
   Rox_Double newwidth, newheight;
   Rox_Double shiftx, shifty, scalex, scaley;

   Rox_Double ** dnk = NULL;

   Rox_Point2D_Double_Struct left[NB_SAMPLES_PER_SIDE];
   Rox_Point2D_Double_Struct right[NB_SAMPLES_PER_SIDE];
   Rox_Point2D_Double_Struct top[NB_SAMPLES_PER_SIDE];
   Rox_Point2D_Double_Struct bottom[NB_SAMPLES_PER_SIDE];

   Rox_Point2D_Double_Struct uleft[NB_SAMPLES_PER_SIDE];
   Rox_Point2D_Double_Struct uright[NB_SAMPLES_PER_SIDE];
   Rox_Point2D_Double_Struct utop[NB_SAMPLES_PER_SIDE];
   Rox_Point2D_Double_Struct ubottom[NB_SAMPLES_PER_SIDE];

   if (!newcalib || !calib || !distortion) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_array2d_double_check_size(newcalib, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(calib, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(distortion, 5, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer(&dnk, newcalib);
   ROX_ERROR_CHECK_TERMINATE ( error );

   sampling = ((double)NB_SAMPLES_PER_SIDE) - 1.0;
   stepx = ((double)(image_width - 1)) / sampling;
   stepy = ((double)(image_height - 1)) / sampling;

   // Sample points around image domain rectangle
   for ( Rox_Sint i = 0; i < NB_SAMPLES_PER_SIDE; i++)
   {
      left[i].u = 0;
      left[i].v = stepy * (double)i;
      right[i].u = image_width - 1;
      right[i].v = stepy * (double)i;
      top[i].u = stepx * (double)i;
      top[i].v = 0;
      bottom[i].u = stepx * (double)i;
      bottom[i].v = image_height - 1;
   }

   // Undistort points
   for ( Rox_Sint i = 0; i < NB_SAMPLES_PER_SIDE; i++)
   {
      error = rox_point2d_double_undistort(&uleft[i], &left[i], calib, distortion); 
      // ROX_ERROR_CHECK_TERMINATE ( error );
      if (error) break;
      error = rox_point2d_double_undistort(&uright[i], &right[i], calib, distortion); 
      // ROX_ERROR_CHECK_TERMINATE ( error );
      if (error) break;
      error = rox_point2d_double_undistort(&utop[i], &top[i], calib, distortion); 
      // ROX_ERROR_CHECK_TERMINATE ( error );
      if (error) break;
      error = rox_point2d_double_undistort(&ubottom[i], &bottom[i], calib, distortion); 
      // ROX_ERROR_CHECK_TERMINATE ( error );
      if (error) break;
   }

   if (error)
   {
      // copy the calib into newcalib
      error = rox_array2d_double_copy(newcalib, calib);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
   // Get bounding box
   minu = DBL_MAX;
   minv = DBL_MAX;
   maxu = -DBL_MAX;
   maxv = -DBL_MAX;
   for ( Rox_Sint i = 0; i < NB_SAMPLES_PER_SIDE; i++)
   {
      if ( uleft[i].u   < minu) minu = uleft[i].u;
      if ( uright[i].u  < minu) minu = uright[i].u;
      if ( utop[i].u    < minu) minu = utop[i].u;
      if ( ubottom[i].u < minu) minu = ubottom[i].u;

      if ( uleft[i].u   > maxu) maxu = uleft[i].u;
      if ( uright[i].u  > maxu) maxu = uright[i].u;
      if ( utop[i].u    > maxu) maxu = utop[i].u;
      if ( ubottom[i].u > maxu) maxu = ubottom[i].u;

      if ( uleft[i].v   < minv) minv = uleft[i].v;
      if ( uright[i].v  < minv) minv = uright[i].v;
      if ( utop[i].v    < minv) minv = utop[i].v;
      if ( ubottom[i].v < minv) minv = ubottom[i].v;

      if ( uleft[i].v   > maxv) maxv = uleft[i].v;
      if ( uright[i].v  > maxv) maxv = uright[i].v;
      if ( utop[i].v    > maxv) maxv = utop[i].v;
      if ( ubottom[i].v > maxv) maxv = ubottom[i].v;
   }

   newwidth = maxu  - minu;
   newheight = maxv - minv;

   scalex = ((double)image_width - 1) / newwidth;
   scaley = ((double)image_height - 1) / newheight;
   shiftx = - scalex * minu;
   shifty = - scaley * minv;

   dnk[0][0] = scalex; dnk[0][1] = 0.0; dnk[0][2] = shiftx;
   dnk[1][0] = 0.0; dnk[1][1] = scaley; dnk[1][2] = shifty;
   dnk[2][0] = 0.0; dnk[2][1] = 0.0; dnk[2][2] = 1.0;
   }

   // TODO: Temporary modification to have the same camera parameters K for the distorted image and the undistorted image
   error = rox_array2d_double_copy(newcalib, calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
