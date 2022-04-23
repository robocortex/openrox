//==============================================================================
//
//    OPENROX   : File draw_checkercorner_detector_corners.c
//
//    Contents  : Implementation of draw_checkercorner_detector_corners module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "draw_checkercorner_detector_corners.h"
#include "draw_points.h"
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

ROX_API Rox_ErrorCode rox_checkercorner_detector_draw_corners (
   Rox_Image_RGBA image_rgba,
   Rox_CheckerCorner_Detector checkercorner_detector,
   Rox_Uint color
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!image_rgba) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!checkercorner_detector) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint c = 0; c < checkercorner_detector->corners->used; c++)
   {
      // access corners
      Rox_Point2D_Double_Struct point2d = checkercorner_detector->corners->data[c].coords;
      rox_image_rgba_draw_2d_points(image_rgba, &point2d, 1, color); // color =  ROX_MAKERGBA(255,0,0,255)
   }

function_terminate:
   return error;
}
