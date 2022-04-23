//==============================================================================
//
//    OPENROX   : File model_checkerboard.c
//
//    Contents  : Implementation of model_checkerboard module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "model_checkerboard.h"

#include <generated/dynvec_point3d_double_struct.h>

#include <system/memory/memory.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_model_checkerboard_new ( Rox_Model_CheckerBoard * model )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Model_CheckerBoard ret = NULL;

   if (!model) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *model = NULL;

   ret = (Rox_Model_CheckerBoard) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   ret->sizx = 0;
   ret->sizy = 0;
   ret->width = 0;
   ret->height = 0;
   
   // Allocate 3D points of the checkerboard 
   error = rox_dynvec_point3d_double_new(&ret->points3D, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *model = ret;

function_terminate:
   return error;
}

Rox_ErrorCode rox_model_checkerboard_del(Rox_Model_CheckerBoard * model)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Model_CheckerBoard todel = NULL;

   if (!model) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = * model;
   *model = NULL;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_point3d_double_del(&todel->points3D);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_model_checkerboard_set_template (
   Rox_Model_CheckerBoard model, 
   const Rox_Sint width, 
   const Rox_Sint height, 
   const Rox_Double sizex, 
   const Rox_Double sizey
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!model)          
   { error = ROX_ERROR_NULL_POINTER;  ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (sizex  <= 0.0)   
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (sizey  <= 0.0)   
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (width  <= 1)     
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (height <= 1)     
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   model->sizx = sizex;
   model->sizy = sizey;
   model->width = width;
   model->height = height;
   
   // Reset the dynamic vector
   rox_dynvec_point3d_double_reset(model->points3D);
   
   // Define the model 
   for ( Rox_Sint i = 0; i < model->height; i++ )
   {
      for ( Rox_Sint j = 0; j < model->width; j++ )
      {
         Rox_Point3D_Double_Struct point3D;
         point3D.X = model->sizx * j;
         point3D.Y = model->sizy * i;
         point3D.Z = 0.0;
         rox_dynvec_point3d_double_append(model->points3D, &point3D);
      }
   }

function_terminate:
   return error;
}
