//==============================================================================
//
//    OPENROX   : File ident_checkerboard.c
//
//    Contents  : Implementation of ident_checkerboard module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ident_checkerboard.h"

#include <baseproc/image/image.h>
#include <system/memory/memory.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_ident_checkerboard_new(Rox_Ident_CheckerBoard * ident)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ident_CheckerBoard ret = NULL;

   if (!ident) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *ident = NULL;

   ret = (Rox_Ident_CheckerBoard) rox_memory_allocate( sizeof(*ret), 1 );

   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->detector = 0;
   ret->checkerdetector = 0;
   ret->width = 0;
   ret->height = 0;

   error = rox_checkerboard_detector_new( &ret->checkerdetector );
   ROX_ERROR_CHECK_TERMINATE( error )

   *ident = ret;

function_terminate:
   if ( error ) rox_ident_checkerboard_del( &ret );

   return error;
}


Rox_ErrorCode rox_ident_checkerboard_del(Rox_Ident_CheckerBoard * ident)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ident_CheckerBoard todel = NULL;


   if ( !ident ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *ident;
   *ident = NULL;


   if ( !todel ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_checkerboard_detector_del( &todel->checkerdetector );
   rox_checkercorner_detector_del( &todel->detector );
   rox_memory_delete( todel );

function_terminate:
   return error;
}


Rox_ErrorCode rox_ident_checkerboard_set_model ( Rox_Ident_CheckerBoard ident_checkerboard, const Rox_Model_CheckerBoard model )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !ident_checkerboard || !model )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   ident_checkerboard->width = model->width;
   ident_checkerboard->height = model->height;

function_terminate:
   return error;
}


Rox_ErrorCode rox_ident_checkerboard_make (
  Rox_Point2D_Double detected_corners,
  const Rox_Ident_CheckerBoard ident_checkerboard,
  const Rox_Image image
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint detected = 0;
   Rox_Uint kernel_blur_levels = 1;
   Rox_Uint score_blur_levels = 6;

   if ( !detected_corners || !ident_checkerboard || !image )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   if ( !ident_checkerboard->detector )
   {
       Rox_Sint cols = 0, rows = 0;
       error = rox_image_get_size( &rows, &cols, image );
       ROX_ERROR_CHECK_TERMINATE( error );

       error = rox_checkercorner_detector_new ( &ident_checkerboard->detector, cols, rows, kernel_blur_levels, score_blur_levels );
       ROX_ERROR_CHECK_TERMINATE( error );
   }
   
   // Detection of corners
   error = rox_checkercorner_detector_process( ident_checkerboard->detector, image );
   ROX_ERROR_CHECK_TERMINATE( error );

   // Detection of checkerboard
   error = rox_checkerboard_detector_process ( ident_checkerboard->checkerdetector, ident_checkerboard->detector );
   ROX_ERROR_CHECK_TERMINATE( error );

#if 0
   for ( idchecker = 0; idchecker < ident_checkerboard->checkerdetector->checkerboards->used; idchecker++ )
   {
        Rox_CheckerBoard checker = ident_checkerboard->checkerdetector->checkerboards->data[idchecker];
        Rox_Sint width  = 0;
        Rox_Sint height = 0;
        error = rox_array2d_point2d_double_get_size( &height, &width, checker->points );
        ROX_ERROR_CHECK_TERMINATE( error );

        if ( width * height != ident_checkerboard->width * ident_checkerboard->height ) continue;

        error = rox_checkerboard_check_order( checker, image );
        ROX_ERROR_CHECK_TERMINATE( error );

        detected = 1;
   }

   if ( !detected ) {error = ROX_ERROR_TEMPLATE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE( error )}

   // Get points
   for ( idchecker = 0; idchecker < ident_checkerboard->checkerdetector->checkerboards->used; idchecker++ )
   {
      Rox_CheckerBoard checker = ident_checkerboard->checkerdetector->checkerboards->data[idchecker];
      Rox_Point2D_Double ** dp = NULL;
      error = rox_array2d_point2d_double_get_data_pointer_to_pointer ( &dp, checker->points );

      if ( checker->energy > -10 ) continue;

      // Set extracted points
      for ( i = 0; i < ident_checkerboard->height; i++ )
      {
          for ( j = 0; j < ident_checkerboard->width; j++ )
          {
             detected_corners[i*ident_checkerboard->width + j].u = dp[i][j].u;
             detected_corners[i*ident_checkerboard->width + j].v = dp[i][j].v;
          }
      }
   }
#endif

   for ( Rox_Uint idchecker = 0; idchecker < ident_checkerboard->checkerdetector->checkerboards->used; idchecker++ )
   {
      Rox_CheckerBoard   checker = ident_checkerboard->checkerdetector->checkerboards->data[idchecker];
      Rox_Point2D_Double_Struct ** dp   = NULL;
      Rox_Sint width = 0, height = 0;

      error = rox_array2d_point2d_double_get_size( &height, &width, checker->points );
      ROX_ERROR_CHECK_CONTINUE( error );

      // We cannot yet test if dimensions are correct
      // but we can check if the correct number of corner were detected
      if ( width * height != ident_checkerboard->width * ident_checkerboard->height )
      { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_CONTINUE( error ) }

      // Re-order detected points using the circles coding
      error = rox_checkerboard_check_order( checker, image );
      ROX_ERROR_CHECK_TERMINATE( error );

      // Update variables
      checker = ident_checkerboard->checkerdetector->checkerboards->data[idchecker];

      error = rox_array2d_point2d_double_get_size( &height, &width, checker->points );
      ROX_ERROR_CHECK_TERMINATE( error );

      // We can now check if dimensions are correct
      if ( width != ident_checkerboard->width || height != ident_checkerboard->height )
      { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_CONTINUE( error ) }

      if ( checker->energy > -10 )
      { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_CONTINUE( error ) }

      error = rox_array2d_point2d_double_get_data_pointer_to_pointer ( &dp, checker->points );
      ROX_ERROR_CHECK_CONTINUE( error );

      detected = 1;

      // Set extracted points
      for ( Rox_Sint i = 0; i < ident_checkerboard->height; i++ )
      {
         for ( Rox_Sint j = 0; j < ident_checkerboard->width ; j++ )
         {
            detected_corners[i*ident_checkerboard->width + j].u = dp[i][j].u;
            detected_corners[i*ident_checkerboard->width + j].v = dp[i][j].v;
         }
      }
   }

   if ( !detected )
   { error = ROX_ERROR_TEMPLATE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

function_terminate:
   return error;
}
