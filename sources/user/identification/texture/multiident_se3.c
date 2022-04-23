//==============================================================================
//
//    OPENROX   : File multiident_se3.c
//
//    Contents  : Implementation of multiident_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "multiident_se3.h"

#include <system/memory/memory.h>

#include <baseproc/array/conversion/array2d_float_from_uchar.h>
#include <baseproc/geometry/transforms/transform_tools.h>

#include <core/identification/templateident_se3.h>

//#include <core/model/model_single_plane_struct.h>
#include <core/model/model_single_plane_struct.h>

#include <inout/system/errors_print.h>

#include <user/sensor/camera/camera_struct.h>

Rox_ErrorCode rox_multi_ident_se3_new(Rox_Multi_Ident_SE3 *ident)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Multi_Ident_SE3 ret = NULL;
   
   if(!ident) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   
   *ident = NULL;

   ret = (Rox_Multi_Ident_SE3) rox_memory_allocate(sizeof(*ret), 1);
   if(!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->identifier = NULL;
   ret->calib_camera = NULL;

   error = rox_template_ident_se3_new(&ret->identifier);
	ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matut3_new ( &ret->calib_camera ); 
	ROX_ERROR_CHECK_TERMINATE ( error );

   *ident = ret;

function_terminate:
   // Delete only if an error occurs
   if(error) rox_multi_ident_se3_del(&ret);

   return error;
}

Rox_ErrorCode rox_multi_ident_se3_del(Rox_Multi_Ident_SE3 *ident)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Multi_Ident_SE3 todel = NULL;

   if(!ident) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   todel = *ident;
   *ident = NULL;

   if(!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   rox_template_ident_se3_del(&todel->identifier);
   rox_matut3_del(&todel->calib_camera);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_multi_ident_se3_add_model(Rox_Multi_Ident_SE3 ident, Rox_Model_Single_Plane model)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float norm_model = NULL;
   Rox_MatUT3 calib_template = NULL;

   if (!ident || !model) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_image_get_size(&rows, &cols, model->image_template);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&norm_model, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_from_uchar_normalize(norm_model, model->image_template); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_new ( &calib_template ); 
   ROX_ERROR_CHECK_TERMINATE(error)

   // Sohuld be 
   // Rox_Double sizex = model->sizex
   // Rox_Double sizey = model->sizey
   Rox_Double sizex = model->vertices_ref[1].X - model->vertices_ref[0].X;
   Rox_Double sizey = model->vertices_ref[2].Y - model->vertices_ref[1].Y;
   error = rox_transformtools_build_calibration_matrix_for_template(calib_template, cols, rows, sizex, sizey);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_template_ident_se3_add_model_affine(ident->identifier, norm_model, calib_template, 0); 
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   rox_array2d_float_del(&norm_model);
   rox_matut3_del(&calib_template);
   return error;
}

Rox_ErrorCode rox_multi_ident_se3_compile(Rox_Multi_Ident_SE3 obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
    
   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   error = rox_template_ident_se3_compile(obj->identifier);
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_multi_ident_se3_make(Rox_Multi_Ident_SE3 ident, Rox_Camera camera)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float norm_cur = NULL;

   if(!ident || !camera) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matut3_copy(ident->calib_camera, camera->calib_camera); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Sint rows = 0, cols =0;
   error = rox_image_get_size(&rows, &cols, camera->image);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&norm_cur, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_float_from_uchar_normalize(norm_cur, camera->image); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_template_ident_se3_make(ident->identifier, norm_cur, ident->calib_camera, 0); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_float_del(&norm_cur);
   return error;
}

Rox_ErrorCode rox_multi_ident_se3_get_pose(Rox_MatSE3 pose, Rox_Uint *id, Rox_Multi_Ident_SE3 obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if(!pose || !id || !obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_template_ident_se3_get_next_best_pose(pose, id, obj->identifier, obj->calib_camera);
    
function_terminate:
   return error;
}
