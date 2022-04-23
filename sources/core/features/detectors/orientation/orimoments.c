//==============================================================================
//
//    OPENROX   : File orimoments.c
//
//    Contents  : Implementation of orimoments module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "orimoments.h"
#include <baseproc/maths/maths_macros.h>

#include <generated/dynvec_segment_point_struct.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_segment_points_compute_orientation_moments(Rox_DynVec_Segment_Point points, Rox_Image source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   const Rox_Sint radius = 4;
   
   if (!points || !source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint cols = 0, rows = 0; 
   error = rox_array2d_uchar_get_size(&rows, &cols, source); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer(&data, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint id = 0; id < points->used; id++)
   {
      Rox_Sint i = points->data[id].i;
      Rox_Sint j = points->data[id].j;

      points->data[id].ori = 0;
      if (i < radius || j < radius) continue;
      if (i >= rows - radius || j >= cols - radius) continue;

      Rox_Sint m01 = 0;
      Rox_Sint m10 = 0;
      for (Rox_Sint y = -radius; y <= radius; y++)
      {
         Rox_Sint ry = y + i;
         for (Rox_Sint x = -radius; x <= radius; x++)
         {
            Rox_Sint rx = x + j;
            Rox_Sint pix = data[ry][rx];

            m10 += x * pix;
            m01 += y * pix;
         }
      }

      points->data[id].ori = (Rox_Float) atan2((double)m01, (double)m10);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_compute_orientation_moments(Rox_Double * theta, Rox_Image source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   
   if (!theta || !source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uchar ** data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer(&data, source);
   ROX_ERROR_CHECK_TERMINATE ( error );


   Rox_Sint cols = 0, rows = 0; 
   error = rox_array2d_uchar_get_size(&rows, &cols, source); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   if (cols != rows) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint radius = cols / 2;
   // Handle the case when the center of the image is not an integer
   if (cols % 2 == 0)
   {
     radius--;
   }
   Rox_Uint hh = radius;
   Rox_Uint hw = radius;

   Rox_Sint i = hh;
   Rox_Sint j = hw;

   Rox_Sint m01 = 0;
   Rox_Sint m10 = 0;
   for (Rox_Sint y = -radius; y <= radius; y++)
   {
      Rox_Sint ry = y + i;
      for (Rox_Sint x = -radius; x <= radius; x++)
      {
         Rox_Sint rx = x + j;
         Rox_Sint pix = data[ry][rx];

         m10 += x * pix;
         m01 += y * pix;
      }
   }

   *theta = atan2((double)m01, (double)m10);

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_compute_orientation_secondmoments(Rox_Double * theta, Rox_Image source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   
   if (!theta || !source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0; 
   error = rox_array2d_uchar_get_size(&rows, &cols, source); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer(&data, source);
   ROX_ERROR_CHECK_TERMINATE ( error );


   if (cols != rows) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint radius = cols / 2;
   // Handle the case when the center of the image is not an integer
   if (cols % 2 == 0)
   {
     radius--;
   }
   Rox_Uint hh = radius;
   Rox_Uint hw = radius;

   Rox_Sint i = hh;
   Rox_Sint j = hw;

      
   Rox_Slint m00 = 0;
   Rox_Slint m02 = 0;
   Rox_Slint m20 = 0;
   Rox_Slint m11 = 0;
   Rox_Slint sumx = 0;
   Rox_Slint sumy = 0;

   Rox_Sint count = 0;

   for (Rox_Sint y = -radius; y <= radius; y++)
   {
      Rox_Sint ry = y + i;
      for (Rox_Sint x = -radius; x <= radius; x++)
      {
         Rox_Sint rx = x + j;

         Rox_Sint pix = data[ry][rx]; 
                  
         m00 = m00 + (Rox_Slint)pix;
         m11 = m11 + (Rox_Slint)x * y * pix;
         m02 = m02 + (Rox_Slint)y * y * pix;
         m20 = m20 + (Rox_Slint)x * x * pix;

         sumx += x;
         sumy += y;
         count++;
      }
    }

    Rox_Double meanx = (double)sumx / (double)count;
    Rox_Double meany = (double)sumy / (double)count;

    Rox_Double mp20 = (double)m20 / (double)m00;
    Rox_Double mp02 = (double)m02 / (double)m00;
    Rox_Double mp11 = (double)m11 / (double)m00;

    mp20 -= meanx * meanx;
    mp02 -= meany * meany;
    mp11 -= meanx * meany;

    *theta = 0.5 * atan2(2.0 * mp11, mp20 - mp02);

function_terminate:
   return error;
}