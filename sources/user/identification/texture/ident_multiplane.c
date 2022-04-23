//==============================================================================
//
//    OPENROX   : File ident_multiplane.c
//
//    Contents  : Implementation of ident_multiplane module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ident_multiplane.h"

#include <system/memory/memory.h>

#include <baseproc/array/conversion/array2d_float_from_uchar.h>
#include <baseproc/array/multiply/mulmatmat.h>

#include <core/identification/templateident_se3.h>

#include <inout/system/errors_print.h>

#include <user/sensor/camera/camera_struct.h>

Rox_ErrorCode rox_ident_multi_plane_new ( Rox_Ident_Multi_Plane *ident )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ident_Multi_Plane ret = NULL;

   if ( !ident ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }
   *ident = NULL;

   ret = (Rox_Ident_Multi_Plane)rox_memory_allocate(sizeof(*ret), 1);
   if(!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   error = rox_template_ident_se3_new(&ret->identifier);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_matse3_new ( &ret->pose );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_matut3_new ( &ret->calib_camera );
   ROX_ERROR_CHECK_TERMINATE(error)

   *ident = ret;

function_terminate:
   // Delete only if an error occurs
   if(error) rox_ident_multi_plane_del(&ret);

   return error;
}

Rox_ErrorCode rox_ident_multi_plane_del ( Rox_Ident_Multi_Plane * ident )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ident_Multi_Plane todel = NULL;

   if ( !ident ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *ident;
   *ident = NULL;

   if ( !todel )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_template_ident_se3_del(&todel->identifier);
   rox_matse3_del(&todel->pose);
   rox_matut3_del(&todel->calib_camera);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_multi_plane_set_model (
   Rox_Ident_Multi_Plane ident,
   Rox_Model_Multi_Plane model
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float fimg = NULL;

   if (!ident || !model) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_template_ident_se3_reset ( ident->identifier ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint idtemplate = 0; idtemplate < model->planes->used; idtemplate++)
   {
      Rox_MatUT3 Ktemplate = model->planes->data[idtemplate]->calibration_template;
      Rox_Image img = model->planes->data[idtemplate]->image_template;
      Rox_Sint width = 0, height = 0;

      error = rox_image_get_size ( &height, &width, img );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_float_new ( &fimg, height, width ); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_float_from_uchar_normalize ( fimg, img ); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_template_ident_se3_add_model_affine ( ident->identifier, fimg, Ktemplate, 0 ); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      rox_array2d_float_del(&fimg);
   }

   error = rox_template_ident_se3_compile ( ident->identifier );
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   rox_array2d_float_del ( &fimg );
   return error;
}

Rox_ErrorCode rox_ident_multi_plane_make (
   Rox_Ident_Multi_Plane ident,
   Rox_Model_Multi_Plane model,
   Rox_Camera camera
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float norm_cur = NULL;
   Rox_Array2D_Double ident_pose = NULL;

   Rox_Uint id = 0;

   if (!model || !ident || !camera) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0, cols = 0;
   error = rox_image_get_size(&rows, &cols, camera->image);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&norm_cur, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_from_uchar_normalize(norm_cur, camera->image); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_new ( &ident_pose ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_template_ident_se3_make(ident->identifier, norm_cur, camera->calib_camera, 0); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_template_ident_se3_get_next_best_pose(ident_pose, &id, ident->identifier, camera->calib_camera);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (!error)
   {
      Rox_Model_Single_Plane mp = model->planes->data[id];
      Rox_MatSE3 refpose = mp->z0_T_o;
      rox_matse3_mulmatmat(ident->pose, ident_pose, refpose);
   }

function_terminate:
   rox_array2d_float_del(&norm_cur);
   rox_matse3_del(&ident_pose);
   return error;
}

Rox_ErrorCode rox_ident_multi_plane_get_pose (
   Rox_MatSE3 pose,
   Rox_Ident_Multi_Plane ident
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!ident || !pose) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_copy ( pose, ident->pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
