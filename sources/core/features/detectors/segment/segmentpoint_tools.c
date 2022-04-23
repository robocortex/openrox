//==============================================================================
//
//    OPENROX   : File segmentpoint_tools.c
//
//    Contents  : API of segmentpoint_tools module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "segmentpoint_tools.h"

#include <core/features/detectors/corners/shicorner.h>
#include <core/features/detectors/corners/shicorner9x9.h>

#include <generated/dynvec_sint_struct.h>
#include <generated/dynvec_segment_point_struct.h>

#include <inout/system/errors_print.h>
#include <inout/system/print.h>

Rox_ErrorCode rox_segment_points_make_lut(Rox_DynVec_Sint lut, Rox_DynVec_Segment_Point input, Rox_Uint image_count_rows)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!lut || !input)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   // Initialize lut
   error = rox_dynvec_sint_usecells(lut, image_count_rows);
   ROX_ERROR_CHECK_TERMINATE ( error );
   for ( Rox_Uint id = 0; id < image_count_rows; id++) lut->data[id] = -1;

   // Loop through points
   for (Rox_Uint id = 0; id < input->used; id++)
   {
      Rox_Uint curi = input->data[id].i;
      if (curi >= lut->used)
      { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE(error); }
      if (lut->data[curi] < 0) lut->data[curi] = id;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_segment_points_compute_shi_score(Rox_DynVec_Segment_Point points, Rox_Image image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   const Rox_Sint radius = 4;
   Rox_Uint bounds = radius + 1; // +1 because of gradient

   if (!image || !points)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   Rox_Uchar ** data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer(&data, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint width = 0, height = 0;
   error = rox_array2d_uchar_get_size(&height, &width, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint wrad = width - bounds;
   Rox_Uint hrad = height - bounds;

   for (Rox_Uint id = 0; id < points->used; id++)
   {
      Rox_Uint i = points->data[id].i;
      Rox_Uint j = points->data[id].j;

      points->data[id].response = 0;

      // Test bounds
      if (i < bounds) continue;
      if (j < bounds) continue;
      if (i >= hrad) continue;
      if (j >= wrad) continue;

      // Shi corner test for this point
      rox_shicorner_test(&points->data[id].response, data, i, j, 4);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_segment_points_compute_shi_score9x9 (
   Rox_DynVec_Segment_Point points,
   Rox_Image image
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   const Rox_Sint radius = 4;
   Rox_Uint bounds = radius + 1; // +1 because of gradient

   if (!image || !points)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uchar ** data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer(&data, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint width = 0, height = 0;
   error = rox_array2d_uchar_get_size(&height, &width, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint wrad = width - bounds;
   Rox_Uint  hrad = height - bounds;

   for (Rox_Uint id = 0; id < points->used; id++)
   {
      Rox_Uint i = points->data[id].i;
      Rox_Uint j = points->data[id].j;

      points->data[id].response = 0;

      // Test bounds
      if (i < bounds) continue;
      if (j < bounds) continue;
      if (i >= hrad) continue;
      if (j >= wrad) continue;

      // Shi corner test for this point
      rox_shicorner9x9_test(&points->data[id].response, data, i, j);
   }

function_terminate:
   return error;
}

Rox_Sint comparePointsResponseDecreasing(const void * one, const void *two)
{
   Rox_Sint result = 0;

   Rox_Segment_Point pone = ( Rox_Segment_Point ) one;
   Rox_Segment_Point ptwo = ( Rox_Segment_Point ) two;

   if (pone->response > ptwo->response)
   { result = -1; goto function_terminate; }
   else if (pone->response < ptwo->response)
   { result = 1; goto function_terminate;}

function_terminate:
   return result;
}

Rox_ErrorCode rox_segment_points_sort_response(Rox_DynVec_Segment_Point points)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!points)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (points->used <= 1)
   { error = ROX_ERROR_NONE; goto function_terminate; }

   qsort(points->data, points->used, sizeof(struct Rox_Segment_Point_Struct), comparePointsResponseDecreasing);

function_terminate:
   return error;
}

Rox_ErrorCode rox_segment_points_print ( Rox_DynVec_Segment_Point points )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!points)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint id = 0; id < points->used; id++)
   {
      Rox_Uint level = points->data[id].level;
      Rox_Uint i = points->data[id].i;
      Rox_Uint j = points->data[id].j;
      Rox_Uint score = points->data[id].score;
      Rox_Float response = points->data[id].response;
      Rox_Float ori = points->data[id].ori;

      rox_log("level = %d\n",level);
      rox_log("(i,j) = (%d, %d)\n",i,j);
      rox_log("score = %d\n",score);
      rox_log("response = %f\n",response);
      rox_log("ori = %f\n",ori);
   }

function_terminate:
   return error;
}
