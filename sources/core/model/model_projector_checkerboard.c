//==============================================================================
//
//    OPENROX   : File model_projector_checkerboard.c
//
//    Contents  : Implementation of model_projector_checkerboard module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "model_projector_checkerboard.h"

#include <baseproc/array/fill/fillval.h>
#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/point/point2d.h>

#include <system/memory/memory.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_model_projector_checkerboard_new ( Rox_Model_Projector_CheckerBoard * model )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Model_Projector_CheckerBoard ret = NULL;
   
   if( !model ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *model = NULL;

   ret = (Rox_Model_Projector_CheckerBoard)rox_memory_allocate(sizeof(*ret), 1);
   if( !ret ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->cols         = 0;
   ret->rows         = 0;

   ret->space_width  = 0;
   ret->space_height = 0;

   ret->image_width  = 0;
   ret->image_height = 0;

   ret->center_u     = 0;
   ret->center_v     = 0;

   ret->elements     = 0;

   *model = ret;

function_terminate:
   return error;
}

Rox_ErrorCode rox_model_projector_checkerboard_del ( Rox_Model_Projector_CheckerBoard * model )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Model_Projector_CheckerBoard todel;

   if( !model ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *model;
   *model = NULL;

   if(!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (todel->elements) rox_memory_delete(todel->elements);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_model_projector_checkerboard_set_template_decentered (
   Rox_Model_Projector_CheckerBoard model,
   const Rox_Sint cols,
   const Rox_Sint rows,
   const Rox_Double space_width,
   const Rox_Double space_height,
   const Rox_Sint image_width,
   const Rox_Sint image_height,
   const Rox_Double center_u,
   const Rox_Double center_v
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double image_center_u = 0.0, image_center_v = 0.0;
   Rox_Double center_du = 0.0,      center_dv = 0.0;
   Rox_Double border_du = 0.0,      border_dv = 0.0;

   if(!model) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (  ( !cols )                   || ( !rows )                   ||
         ( 0 >= space_width )        || ( 0 >= space_height )       ||
         ( !image_width )            || ( !image_height )           ||
         ( 0 >= center_u )           || ( 0 >= center_v )           ||
         ( image_width <= center_u ) || ( image_height <= center_v ) )
   {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error );}

   model->elements = (Rox_Point2D_Double ) rox_memory_allocate(sizeof(Rox_Point2D_Double_Struct), cols * rows );

   if (!model->elements) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   image_center_u = (Rox_Double) ( (image_width -1.0)  / 2.0);
   image_center_v = (Rox_Double) ( (image_height -1.0) / 2.0);
   center_du      = center_u - image_center_u;
   center_dv      = center_v - image_center_v;
   border_du      = ( image_width  - (cols - 1)*space_width  ) / 2.0;
   border_dv      = ( image_height - (rows - 1)*space_height ) / 2.0;

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         model->elements[i * cols + j].u = (Rox_Double) (space_width  * j) + border_du + center_du - 0.5;
         model->elements[i * cols + j].v = (Rox_Double) (space_height * i) + border_dv + center_dv - 0.5;
      }
   }

   model->cols         = cols;
   model->rows         = rows;

   model->space_width  = space_width;
   model->space_height = space_height;

   model->image_width  = image_width;
   model->image_height = image_height;

   model->center_u     = center_u;
   model->center_v     = center_v;

function_terminate:
   return error;
}

Rox_ErrorCode rox_model_projector_checkerboard_set_template (
   Rox_Model_Projector_CheckerBoard model,
   const Rox_Sint cols,
   const Rox_Sint rows,
   const Rox_Double space_width,
   const Rox_Double space_height,
   const Rox_Sint image_width,
   const Rox_Sint image_height 
)
{
  Rox_ErrorCode error = ROX_ERROR_NONE;

  if ( !model )
  { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

  error = rox_model_projector_checkerboard_set_template_decentered ( model, cols, rows, space_width, space_height, image_width, image_height, ((Rox_Double)(image_width-1.0))/2.0, ((Rox_Double)(image_height-1.0))/2.0 );
  ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_model_projector_checkerboard_generate_image (
   Rox_Model_Projector_CheckerBoard model,
   Rox_Image image
)
{
   // Local variables
   Rox_ErrorCode error=ROX_ERROR_NONE;
   Rox_Uint      sr=0, sc=0;
   Rox_Uint      r_start=0, r_stop=0, c_start=0, c_stop=0, r_center=0, c_center=0;
   Rox_Double    border_du=0.0, border_dv=0.0;
   Rox_Double    image_center_u=0.0, image_center_v=0.0;
   Rox_Double    center_du=0.0, center_dv=0.0;
   Rox_Uint      radius=0;
   Rox_Uint      sc_start=0, sc_stop=0, sr_start=0, sr_stop=0;

   // Check inputs

   if (!model || !image ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0, cols =0;
   error = rox_array2d_uchar_get_size(&rows, &cols, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if ((cols != model->image_width) || (rows != model->image_height))
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Helpers
   image_center_u = ( (Rox_Double) model->image_width - 1.0 ) / 2.0;
   image_center_v = ( (Rox_Double) model->image_height - 1.0) / 2.0;

   center_du = model->center_u - image_center_u;
   center_dv = model->center_v - image_center_v;

   border_du = ( model->image_width  - (model->cols+1)*model->space_width  ) / 2.0;
   border_dv = ( model->image_height - (model->rows+1)*model->space_height ) / 2.0;

   radius = (Rox_Uint) (ROX_MIN( model->space_width, model->space_height )/4.0);

   // Fill image
   error = rox_array2d_uchar_fillval( image, (Rox_Uchar) 255 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint r = 0; r <= model->rows; r++ )
   for ( Rox_Sint c = 0; c <= model->cols; c++ )
   {
      // Boundaries of current square
      r_start = (Rox_Uint) ( (model->space_height *   r  ) + border_dv + center_dv );
      r_stop  = (Rox_Uint) ( (model->space_height * (r+1)) + border_dv + center_dv );
      c_start = (Rox_Uint) ( (model->space_width  *   c  ) + border_du + center_du );
      c_stop  = (Rox_Uint) ( (model->space_width  * (c+1)) + border_du + center_du );

      // Draw circles
      if ( ( (2==c) && (1==r) ) || ( (1==c) && (2==r) ) )
      {
         r_center = ( r_start + r_stop ) / 2;
         c_center = ( c_start + c_stop ) / 2;

         // Compute bounds
         sc_start = (Rox_Uint) ROX_MAX( c_center-radius, 0 );
         sc_stop  = (Rox_Uint) ROX_MIN((Rox_Sint) (c_center+radius), model->image_width  - 1);
         sr_start = (Rox_Uint) ROX_MAX( r_center-radius, 0 );
         sr_stop  = (Rox_Uint) ROX_MIN((Rox_Sint) (r_center+radius), model->image_height - 1);

         for ( sr = sr_start; sr < sr_stop; sr++ )
         for ( sc = sc_start; sc < sc_stop; sc++ )
         {
            Rox_Double dist = (r_center-sr)*(r_center-sr) + (c_center-sc)*(c_center-sc);
            if ( dist < radius*radius )
            {
               error = rox_array2d_uchar_set_value( image, sr, sc, (Rox_Uchar) 0 );
               ROX_ERROR_CHECK_TERMINATE ( error );
            }
         }
      }

      // Drop one square out of two
      if ( ( r % 2 ) != ( c % 2 ) )
         continue;

      // Fill squares
      sc_start = (Rox_Uint) ROX_MAX(c_start, 0);
      sc_stop  = (Rox_Uint) ROX_MIN((Rox_Sint) c_stop, model->image_width - 1);
      sr_start = (Rox_Uint) ROX_MAX(r_start, 0);
      sr_stop  = (Rox_Uint) ROX_MIN((Rox_Sint) r_stop, model->image_height - 1);
      for (sr = sr_start; sr < sr_stop; sr++)
      for (sc = sc_start; sc < sc_stop; sc++)
      {
         error = rox_array2d_uchar_set_value(image, sr, sc, (Rox_Uchar)0);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
   }

function_terminate:
   return error;
}
