//==============================================================================
//
//    OPENROX   : File gftt.c
//
//    Contents  : Implementation of gftt module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "gftt.h"

#include <baseproc/array/nonmax/nonmax.h>

#include <core/features/detectors/segment/segmentpoint_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_gftt_detector(Rox_DynVec_Segment_Point corners, Rox_Array2D_Float response, Rox_Uint radius, Rox_Float threshold)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float nonmax = NULL;

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, response);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (radius > 0)
   {
      error = rox_array2d_float_new(&nonmax, rows, cols); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_float_nonmax(nonmax, response, radius); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      nonmax = response;
   }

   rox_dynvec_segment_point_reset(corners);

   Rox_Float ** dnm = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer(&dnm, nonmax);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         Rox_Segment_Point_Struct toadd;

         if (dnm[i][j] < threshold) continue;

         toadd.i = i;
         toadd.j = j;
         toadd.response = dnm[i][j];

         rox_dynvec_segment_point_append(corners, &toadd);
      }
   }


function_terminate:
   if (radius > 0)
   {
      rox_array2d_float_del(&nonmax);
   }

   return error;
}
