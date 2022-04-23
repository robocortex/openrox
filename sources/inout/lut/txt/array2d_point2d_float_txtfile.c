//==============================================================================
//
//    OPENROX   : File array2d_point2d_float_txtfile.c
//
//    Contents  : Implementation of array2d_point2d_float_txtfile module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "array2d_point2d_float_txtfile.h"
#include <stdio.h>
#include <baseproc/geometry/point/point2d.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_point2d_float_save_txt (
   const char * filename, 
   Rox_Array2D_Point2D_Float lut
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * f = NULL;

   if ( !filename || !lut ) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   f = fopen(filename, "w");
   if ( !f ) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   
   error = rox_array2d_point2d_float_get_size(&rows, &cols, lut); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point2D_Float * pts = NULL;
   rox_array2d_point2d_float_get_data_pointer_to_pointer ( &pts, lut );

   // Write lut size 
   fprintf(f, "%d %d\n", rows, cols);

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++ )
      {
         fprintf(f, "%f %f\n", pts[i][j].u, pts[i][j].v);
      }
   }

function_terminate:
   if(f) fclose(f);
   return error;
}

Rox_ErrorCode rox_array2d_point2d_float_read_txt(Rox_Array2D_Point2D_Float lut, const char *filename)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * f = NULL;
   Rox_Sint cols, rows;

   if(!filename || !lut) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   f = fopen(filename, "r");
   if(!f) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error );}

   Rox_Point2D_Float * pts = NULL;
   rox_array2d_point2d_float_get_data_pointer_to_pointer ( &pts, lut);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Read lut size 
   if(fscanf(f, "%d %d\n", &rows, &cols) < 0)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Check lut size 
   error = rox_array2d_point2d_float_check_size(lut, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         if (fscanf(f, "%f %f\n", &pts[i][j].u, &pts[i][j].v) < 0)
         { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }
      }
   }

function_terminate:   
   if(f) fclose(f);
   return error;
}

Rox_ErrorCode rox_array2d_point2d_float_new_txt(Rox_Array2D_Point2D_Float * lut, char * filename)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Point2D_Float ret = NULL;
   FILE * f = NULL;
   Rox_Sint cols, rows;

   if(!filename || !lut) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error );}
   *lut = NULL;

   f = fopen(filename, "r");
   if (!f) 
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Read lut size 
   if (fscanf(f, "%d %d\n", &rows, &cols) < 0)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // create lut object 
   error = rox_array2d_point2d_float_new(&ret, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point2D_Float * pts = NULL;
   rox_array2d_point2d_float_get_data_pointer_to_pointer ( &pts, ret);

   // Read points 
   for(Rox_Sint i = 0; i < rows; i++)
   {
      for(Rox_Sint j = 0; j < cols; j++)
      {
         if(fscanf(f, "%f %f\n", &pts[i][j].u, &pts[i][j].v) < 0)
         {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error );}
      }
   }

   *lut = ret;

function_terminate:  
   if(error) rox_array2d_point2d_float_del(&ret);
   if(f) fclose(f);
   return error;
}
