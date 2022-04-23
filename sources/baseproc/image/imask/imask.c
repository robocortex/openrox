//==============================================================================
//
//    OPENROX   : File imask.c
//
//    Contents  : Implementation of imask module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "imask.h"

#include <baseproc/maths/maths_macros.h>

#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/band/band.h>
#include <baseproc/array/bnot/bnot.h>
#include <baseproc/maths/maths_macros.h>

#include <inout/mask/pgm/mask_pgmfile.h>
#include <baseproc/image/imask/fill/set_data.h>
#include <baseproc/image/imask/fill/set_border.h>
#include <baseproc/image/imask/fill/set_ellipse.h>
#include <baseproc/image/imask/fill/set_polygon.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_imask_new ( Rox_Imask * mask, const Rox_Sint cols, const Rox_Sint rows )
{
    return rox_array2d_uint_new(mask, rows, cols);
}

Rox_ErrorCode rox_imask_del(Rox_Imask *mask)
{
    return rox_array2d_uint_del(mask);
}

Rox_ErrorCode rox_imask_new_read_pgm(Rox_Imask *mask, const char *filename)
{
    return rox_array2d_uint_mask_new_pgm(mask, filename);
}

Rox_ErrorCode rox_imask_read_pgm(Rox_Imask mask, const char *filename)
{
    return rox_array2d_uint_mask_read_pgm ( mask, filename );
}

Rox_ErrorCode rox_imask_save_pgm(const char *filename, Rox_Imask mask)
{
    return rox_array2d_uint_mask_save_pgm(filename, mask);
}

Rox_ErrorCode rox_imask_get_data_pointer_to_pointer ( Rox_Uint ***data, Rox_Imask mask )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!mask)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uint_get_data_pointer_to_pointer(data, mask);
    
function_terminate:
   return error;
}

Rox_ErrorCode rox_imask_get_cols(Rox_Sint * cols, Rox_Imask mask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if(!cols || !mask) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uint_get_cols(cols, mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_imask_get_rows ( Rox_Sint * rows, const Rox_Imask mask )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
    
   if(!rows || !mask) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uint_get_rows(rows, mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_imask_get_size ( Rox_Sint * rows, Rox_Sint * cols, const Rox_Imask mask )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
    
   if ( !rows || !cols || !mask ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uint_get_size ( rows, cols, mask );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_imask_set_zero ( Rox_Imask mask )
{
   return rox_array2d_uint_fillval(mask, 0);
}

Rox_ErrorCode rox_imask_set_ones ( Rox_Imask mask )
{
   return rox_array2d_uint_fillval ( mask, ~0 );
}

Rox_ErrorCode rox_imask_set_and ( Rox_Imask result, const Rox_Imask input_1, const Rox_Imask input_2)
{
   return rox_array2d_uint_band(result, input_1, input_2);
}

Rox_ErrorCode rox_imask_set_data (Rox_Imask mask, Rox_Uint * data, const Rox_Sint bytesPerRow)
{
   return rox_array2d_uint_set_data (mask, data, bytesPerRow);
}

Rox_ErrorCode rox_imask_set_border ( Rox_Imask mask, const Rox_Sint size )
{
   return rox_array2d_uint_set_border  (mask, size);
}

Rox_ErrorCode rox_imask_set_centered_ellipse ( Rox_Imask mask )
{
   return rox_array2d_uint_set_centered_ellipse ( mask );
}

Rox_ErrorCode rox_imask_new_ellipse ( Rox_Imask * mask, const Rox_Ellipse2D ellipse2d )
{
   return rox_array2d_uint_new_ellipse ( mask, ellipse2d );
}

Rox_ErrorCode rox_imask_new_polygon ( Rox_Imask * mask, const Rox_Point2D_Double points_list, Rox_Sint const nb_points )
{
   return rox_array2d_uint_new_polygon ( mask, points_list, nb_points );
}

Rox_ErrorCode rox_imask_set_centered_circle ( 
   Rox_Imask mask, const Rox_Double radius, const Rox_Double mask_sizex )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double    radius_pix = 0.0;
   Rox_Double    radius2_pix = 0.0;
   Rox_Sint      mask_cols = 0, mask_rows = 0;
   Rox_Sint      i_first = 0, j_first=0, i_last=0, j_last=0;
   Rox_Double    di = 0.0, dj = 0.0;
   Rox_Imask     mask_copy = NULL;

   // Check inputs
   if ( NULL == mask )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( 0.0 >= radius || 0.0 >= mask_sizex )
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Get mask width and height
   error = rox_imask_get_cols( &mask_cols,  mask ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_imask_get_rows( &mask_rows, mask ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Make a copy of the mask, in case something goes wrong
   error = rox_imask_new( &mask_copy, mask_cols, mask_rows );
   if (error)
   { 
      rox_imask_del( &mask_copy );
      return error;
   }

   error = rox_array2d_uint_copy( mask_copy, mask );
   if (error)
   {
      rox_imask_del( &mask_copy );
      return error;
   }

   // Re-initialize mask
   error = rox_imask_set_ones( mask );
   ROX_ERROR_CHECK_TERMINATE(error)

   // Image center computation
   Rox_Double cu = ( (Rox_Double) (mask_cols -1) ) / 2.0;
   Rox_Double cv = ( (Rox_Double) (mask_rows -1) ) / 2.0;

   // Convert radius to pixels
   radius_pix  = ceil( radius*( (Rox_Double) mask_cols )/mask_sizex );
   radius2_pix = radius_pix*radius_pix;

   // Compute bounds
   i_first = ROX_MAX( (Rox_Sint) floor(cu - radius_pix), 0             );
   j_first = ROX_MAX( (Rox_Sint) floor(cv - radius_pix), 0             );
   i_last  = ROX_MIN( (Rox_Sint)  ceil(cu + radius_pix), mask_cols - 1 );
   j_last  = ROX_MIN( (Rox_Sint)  ceil(cv + radius_pix), mask_cols - 1 );

   // Set mask
   for ( Rox_Sint i = i_first; i <= i_last; i++ )
   {
      for ( Rox_Sint j = j_first; j <= j_last; j++ )
      {
         di = fabs( ((Rox_Double) i) - cu ); // I assume the compiler will optimize this
         dj = fabs( ((Rox_Double) j) - cv );
         if ( radius2_pix >= (di*di + dj*dj) ) // and that too
         {
            error = rox_array2d_uint_set_value( mask, i, j, 0 );
            ROX_ERROR_CHECK_TERMINATE(error)
         }
      }
   }

function_terminate:
   if (error)
   {
      // restore mask from copy
      rox_array2d_uint_copy( mask, mask_copy );
   }

   // delete copy
   rox_imask_del( &mask_copy );

   return error;
}

Rox_ErrorCode rox_imask_set_not ( Rox_Imask mask )
{
   return rox_array2d_uint_bnot(mask, mask);
}

Rox_ErrorCode rox_imask_count_valid ( Rox_Sint * valid_pixels, const Rox_Imask mask )
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Check inputs
   if ( mask == NULL || valid_pixels == NULL )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint mask_cols = 0, mask_rows = 0;

   // Get mask width and height
   error = rox_imask_get_cols( &mask_cols,  mask ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_imask_get_rows( &mask_rows, mask ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** data = NULL; 
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &data, mask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   *valid_pixels = 0;
   for (Rox_Sint r = 0; r < mask_rows; r++)
   {
      for (Rox_Sint c = 0; c < mask_cols; c++)
      {
         if ( data[r][c] != 0 )
         {
            *valid_pixels += 1;
         }
      }
   }

function_terminate:

   return error;
}

Rox_ErrorCode rox_imask_new_copy_roi ( 
   Rox_Imask * dest, 
   const Rox_Rect_Sint roi, 
   const Rox_Imask source 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !dest )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !source || !roi )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_imask_new ( dest, roi->width, roi->height );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** dest_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &dest_data, * dest );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** source_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &source_data, source );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint v = 0; v < roi->height; v++ )
   {
      for ( Rox_Sint u = 0; u < roi->width ; u++ )
      {
         dest_data[v][u] =  source_data[v+roi->y][u+roi->x];
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_imask_check_size ( const Rox_Imask imask, const Rox_Sint rows, const Rox_Sint cols )
{
   return rox_array2d_uint_check_size ( imask, rows, cols );
}


Rox_ErrorCode rox_imask_get_roi ( Rox_Rect_Sint_Struct * roi, const Rox_Imask imask )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !roi ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !imask )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint ** imask_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &imask_data, imask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0;
   Rox_Sint rows = 0;
   error = rox_array2d_uint_get_size ( &rows, &cols, imask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint min_u = cols;
   Rox_Sint max_u = 0;
   Rox_Sint min_v = rows;
   Rox_Sint max_v = 0;

   // Find the window
   for ( Rox_Sint v = 0; v < rows; v++ )
   {
      for ( Rox_Sint u = 0; u < cols ; u++ )
      {
         if ( imask_data[v][u] > 0 )
         {
            if ( u < min_u )
            {
               min_u = u;
            }
            if ( u > max_u )
            {
               max_u = u;
            }     
            if ( v < min_v )
            {
               min_v = v;
            }
            if ( v > max_v )
            {
               max_v = v;
            }         
         }
         
      }
   }

   // Extract the bounding box
   Rox_Sint rows_imask = max_v-min_v+1;
   Rox_Sint cols_imask = max_u-min_u+1;
   
   roi->x = min_u;
   roi->y = min_v; 
   roi->width = cols_imask;
   roi->height = rows_imask;

function_terminate:
   return error;
}

Rox_ErrorCode rox_imask_set_from_image ( Rox_Imask imask, const Rox_Image image, const Rox_Sint grays_threshold )
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Check inputs
   if ( image == NULL)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Check output
   if ( imask == NULL)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint imask_cols = 0, imask_rows = 0;

   // Get mask cols and rows
   error = rox_imask_get_size ( &imask_rows, &imask_cols, imask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** imask_data = NULL; 
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &imask_data, imask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** image_data = NULL; 
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &image_data, image );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint r = 0; r < imask_rows; r++)
   {
      for (Rox_Sint c = 0; c < imask_cols; c++)
      {
         if ( image_data[r][c] < grays_threshold )
         {
            imask_data[r][c] = 1;
         }
         else
         {
            imask_data[r][c] = 0;
         }
      }
   }

function_terminate:

   return error;
}