//==============================================================================
//
//    OPENROX   : File sl3virtualview.c
//
//    Contents  : Implementation of sl3virtualview module
//
//    Author(s) : R&D department leaded by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "sl3virtualview.h"
#include "sl3from4points.h"

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/point/point2d_struct.h>
#include <float.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_virtualview_anisotropic_scaling (
   Rox_Sint * new_width,
   Rox_Sint * new_height,
   Rox_MatSL3 homography,
   Rox_Double scalex,
   Rox_Double scaley,
   Rox_Sint source_width,
   Rox_Sint source_height
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!new_width || !new_height || !homography) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(homography, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dh = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dh, homography);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double nh = ((Rox_Double) source_height) / scaley;
   Rox_Double nw = ((Rox_Double) source_width) / scalex;

   *new_width = (Rox_Sint)(nw + 0.5);
   *new_height = (Rox_Sint)(nh + 0.5);

   dh[0][0] = scalex; dh[0][1] = 0; dh[0][2] = 0.5 * (scalex - 1.0);
   dh[1][0] = 0; dh[1][1] = scaley; dh[1][2] = 0.5 * (scaley - 1.0);
   dh[2][0] = 0; dh[2][1] = 0; dh[2][2] = 1;

function_terminate:
   return error;
}

Rox_ErrorCode rox_virtualview_skew(Rox_Sint *new_width, Rox_Sint *new_height, Rox_Array2D_Double homography, Rox_Double skew, Rox_Sint source_width, Rox_Sint source_height)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double nw, nh;
   Rox_Double scale;
   Rox_Bool positive;
   Rox_Point2D_Float_Struct source[4];
   Rox_Point2D_Float_Struct dest[4];

   if ( !new_width || !new_height || !homography ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(homography, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   positive = 0;
   if (skew > 0.0) positive = 1;
   skew = fabs(skew);

   nw = source_width;
   nh = sqrt(source_height * source_height - skew * skew * source_width * source_width) / (1.0 + skew);

   scale = 1.0 / (1.0 + skew);
   skew *= scale;

   source[0].v = 0;
   source[1].v = 0;
   source[2].v = (Rox_Float) (nh - 1.0);
   source[3].v = (Rox_Float) (nh - 1.0);

   dest[0].v = 0;
   dest[1].v = 0;
   dest[2].v = (Rox_Float) (source_height - 1.0f);
   dest[3].v = (Rox_Float) (source_height - 1.0f);

   dest[0].u = 0.0;
   dest[1].u = (Rox_Float)(source_width - 1.0);
   dest[2].u = 0.0;
   dest[3].u = (Rox_Float)(source_width - 1.0);

   if (positive)
   {
      source[0].u = (Rox_Float) (skew * nw);
      source[1].u = (Rox_Float) (nw - 1.0f);
      source[2].u = 0.0;
      source[3].u = (Rox_Float) (nw * (1.0f - skew));
   }
   else
   {
      source[0].u = 0;
      source[1].u = (Rox_Float) (nw * (1.0f - skew));
      source[2].u = (Rox_Float) (skew * nw);
      source[3].u = (Rox_Float) (nw - 1.0f);
   }

   error = rox_matsl3_from_4_points_float(homography, source, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *new_width = (Rox_Sint)(nw + 0.5);
   *new_height = (Rox_Sint)(nh + 0.5);

function_terminate:
   return error;
}
