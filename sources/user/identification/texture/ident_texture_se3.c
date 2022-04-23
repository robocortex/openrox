//==============================================================================
//
//    OPENROX   : File ident_texture_se3.c
//
//    Contents  : Implementation of ident_texture_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ident_texture_se3.h"
#include "ident_texture_se3_struct.h"

#include <system/memory/memory.h>
#include <core/model/model_single_plane_struct.h>

#include <core/identification/templateident_se3.h>
#include <core/identification/templateident_struct.h>
#include <core/identification/templateident_se3_struct.h>

#include <baseproc/array/conversion/array2d_float_from_uchar.h>
#include <baseproc/geometry/transforms/transform_tools.h>

#include <inout/system/errors_print.h>

#include <user/sensor/camera/camera_struct.h>

Rox_ErrorCode rox_ident_texture_se3_new ( Rox_Ident_Texture_SE3 * ident )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ident_Texture_SE3 ret = NULL;

   if ( !ident ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *ident = NULL;

   ret = (Rox_Ident_Texture_SE3) rox_memory_allocate(sizeof(*ret), 1);
   if(!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_template_ident_se3_new(&ret->template_ident_se3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *ident = ret;

function_terminate:
   if(error) rox_ident_texture_se3_del(&ret);

   return error;
}

Rox_ErrorCode rox_ident_texture_se3_del(Rox_Ident_Texture_SE3 *ident)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ident_Texture_SE3 todel;

   if ( !ident ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *ident;
   *ident = NULL;

   if ( !todel ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_template_ident_se3_del(&todel->template_ident_se3);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_texture_se3_set_model(Rox_Ident_Texture_SE3 ident, Rox_Model_Single_Plane model)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float norm_model = NULL;
   Rox_MatUT3 calib_template = NULL;

   if (!ident || !model) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_image_get_size ( &rows, &cols, model->image_template );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new ( &norm_model, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_float_from_uchar_normalize ( norm_model, model->image_template ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matut3_new ( &calib_template ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Sohuld be 
   // Rox_Double sizex = model->sizex
   // Rox_Double sizey = model->sizey
   Rox_Double sizex = model->vertices_ref[1].X - model->vertices_ref[0].X;
   Rox_Double sizey = model->vertices_ref[2].Y - model->vertices_ref[1].Y;
   error = rox_transformtools_build_calibration_matrix_for_template ( calib_template, cols, rows, sizex, sizey);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_template_ident_se3_reset ( ident->template_ident_se3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Commented affine model to go faster when detecting 3D structure
   error = rox_template_ident_se3_add_model_affine ( ident->template_ident_se3, norm_model, calib_template, 0); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   //error = rox_template_ident_se3_add_model(ident->template_ident_se3, norm_model, calib_template, 0); 
   //ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_template_ident_se3_compile(ident->template_ident_se3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_float_del(&norm_model);
   rox_matut3_del(&calib_template);
   return error;
}

Rox_ErrorCode rox_ident_texture_se3_set_features(Rox_Ident_Texture_SE3 obj, Rox_DynVec_SRAID_Feature sraid_ref_reloaded, Rox_DynVec_Point3D_Double points_map_ref_reloaded)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Template_Ident toadd = NULL;

   error = rox_template_ident_se3_reset(obj->template_ident_se3); 
   ROX_ERROR_CHECK_TERMINATE ( error );  

   if (!obj || !sraid_ref_reloaded || !points_map_ref_reloaded) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   error = rox_template_ident_new(&toadd);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_sraiddesc_clone(toadd->reference_features_subsets->data[0], sraid_ref_reloaded);   
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // No affine features
   for (Rox_Sint idaffine = 0; idaffine < 12; idaffine++)
   {
      rox_dynvec_sraiddesc_reset(toadd->reference_features_subsets->data[idaffine + 1]);
   }
   
   error = rox_objset_template_ident_append(obj->template_ident_se3->matcher->idents, toadd);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_new(&obj->template_ident_se3->reference_points_meters_extracted, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_clone(obj->template_ident_se3->reference_points_meters_extracted, points_map_ref_reloaded);   
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_texture_se3_make(Rox_Sint * is_identified, Rox_MatSE3 pose, Rox_Ident_Texture_SE3 ident, Rox_Camera camera)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float norm_cur = NULL;
   Rox_Uint id = 0;

   if (!pose || !ident || !camera) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *is_identified = 0;

   Rox_Sint cols = 0, rows = 0;
   error = rox_image_get_size(&rows, &cols, camera->image);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&norm_cur, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Normalize image passing from uchar to float
   error = rox_array2d_float_from_uchar_normalize(norm_cur, camera->image); 
   ROX_ERROR_CHECK_TERMINATE ( error );
    
   // Identify template 
   error = rox_template_ident_se3_make(ident->template_ident_se3, norm_cur, camera->calib_camera, 0); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Get the best pose from all possible  templates 
   error = rox_template_ident_se3_get_next_best_pose(pose, &id, ident->template_ident_se3, camera->calib_camera); 
   if(!error)
   {
     *is_identified = 1;
   }
   if(error == ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE) {error = ROX_ERROR_NONE;}
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_float_del(&norm_cur);
   return error;
}

Rox_ErrorCode rox_ident_texture_se3_getresult(Rox_Sint * is_identified, Rox_MatSE3 pose, Rox_Ident_Texture_SE3 ident_texture_se3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // Rox_Uint id = 0;

   if (!ident_texture_se3 || !is_identified || !pose) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
   // if (id > ident_texture_se3->photoframes->used) {error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error );}

   // Get the best pose from all possible  templates 
   // error = rox_template_ident_se3_get_next_best_pose(pose, &id, ident_texture_se3->template_ident_se3, camera->calib_camera); 
   ROX_ERROR_CHECK_TERMINATE(error)
   if (error)
   {
      *is_identified = 1;
      // error = rox_matse3_copy(pose, ident_texture_se3->photoframes->data[id]->pose);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}
