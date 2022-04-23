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

#include "ident_checkerboard_projector.h"

#include <system/memory/memory.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_ident_checkerboard_projector_new(Rox_Ident_CheckerBoard_Projector * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ident_CheckerBoard_Projector ret = NULL;

   ret = (Rox_Ident_CheckerBoard_Projector) rox_memory_allocate(sizeof(*ret), 1);

   if(!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->actual_detector = 0;
   error = rox_ident_checkerboard_new( &(ret->actual_detector) );
   ROX_ERROR_CHECK_TERMINATE ( error );

   *obj = ret;

function_terminate:
   if(error) rox_ident_checkerboard_projector_del(&ret);

   return error;
}

Rox_ErrorCode rox_ident_checkerboard_projector_del(Rox_Ident_CheckerBoard_Projector * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ident_CheckerBoard_Projector todel = NULL;

   if ( !obj ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   if ( !todel ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_ident_checkerboard_del( &todel->actual_detector );
   rox_memory_delete( todel );

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_checkerboard_projector_set_model(Rox_Ident_CheckerBoard_Projector obj, Rox_Model_Projector_CheckerBoard model )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Model_CheckerBoard  actual_model = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!model) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_model_checkerboard_new( &actual_model );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (!actual_model)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_model_checkerboard_set_template(actual_model, model->cols, model->rows, model->space_width, model->space_height);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ident_checkerboard_set_model( obj->actual_detector, actual_model );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_model_checkerboard_del( &actual_model );

   return error;
}

Rox_ErrorCode rox_ident_checkerboard_projector_make(
   Rox_Point2D_Double  detected_corners, 
   Rox_Ident_CheckerBoard_Projector obj, 
   Rox_Image image)
{
   return rox_ident_checkerboard_make( detected_corners, obj->actual_detector, image );
}
