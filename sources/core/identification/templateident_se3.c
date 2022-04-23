//==============================================================================
//
//    OPENROX   : File templateident_se3.c
//
//    Contents  : Implementation of templateident_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "templateident_struct.h"
#include "templateident_se3.h"
#include "templateident_se3_struct.h"

#include <baseproc/geometry/point/point3d_from_template.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/transforms/matsl3/sl3virtualview.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
#include <baseproc/image/remap/remap_bilinear_float_to_float/remap_bilinear_float_to_float.h>
#include <baseproc/array/fill/fillval.h>

#include <core/indirect/homography/ransacsl3.h>
#include <core/indirect/euclidean/ransacse3.h>
#include <core/indirect/euclidean/vvspointsse3.h>

#include <inout/system/errors_print.h>

#define IDENT_PLANE 

Rox_ErrorCode rox_template_ident_se3_new(Rox_Template_Ident_SE3 *obj) 
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Template_Ident_SE3 ret = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   ret = (Rox_Template_Ident_SE3) rox_memory_allocate(sizeof(struct Rox_Template_Ident_SE3_Struct), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   ret->matcher = NULL;
   ret->current_points_ransac = NULL;
   ret->reference_points_meters_ransac = NULL;
   ret->reference_points_meters_matched = NULL;

   error = rox_multi_ident_new(&ret->matcher); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_float_new(&ret->current_points_ransac, 100); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_dynvec_point3d_float_new(&ret->reference_points_meters_ransac, 100); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_dynvec_point3d_float_new(&ret->reference_points_meters_matched, 100); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   *obj = ret;

function_terminate:
   if (error) rox_template_ident_se3_del(&ret);
   return error;
}

Rox_ErrorCode rox_template_ident_se3_del(Rox_Template_Ident_SE3 *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Template_Ident_SE3 todel = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   todel = *obj;
   *obj = NULL;

   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   // Delete point lists
   rox_multi_ident_del(&todel->matcher);

   rox_dynvec_point2d_float_del(&todel->current_points_ransac);
   rox_dynvec_point3d_float_del(&todel->reference_points_meters_ransac);
   rox_dynvec_point3d_float_del(&todel->reference_points_meters_matched);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_template_ident_se3_reset(Rox_Template_Ident_SE3 obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
    
   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   // Delete point lists
   error = rox_multi_ident_reset(obj->matcher);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
  return error;
}

Rox_ErrorCode rox_template_ident_se3_add_model(Rox_Template_Ident_SE3 obj, Rox_Array2D_Float model_image, Rox_Array2D_Double calib_template, Rox_Uint dbl_image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!obj || !model_image) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   error = rox_multi_ident_add_template(obj->matcher, model_image, calib_template, 0, dbl_image); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
  return error;
}

Rox_ErrorCode rox_template_ident_se3_add_model_affine(Rox_Template_Ident_SE3 obj, Rox_Array2D_Float model_image, Rox_Array2D_Double calib_template, Rox_Uint dbl_image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !model_image) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   error = rox_multi_ident_add_template(obj->matcher, model_image, calib_template, 1, dbl_image); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_template_ident_se3_compile(Rox_Template_Ident_SE3 obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   error = rox_multi_ident_compile(obj->matcher);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_template_ident_se3_make(Rox_Template_Ident_SE3 obj, Rox_Array2D_Float current_image, Rox_Array2D_Double calib_camera, Rox_Uint dbl_image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!obj || !current_image || !calib_camera) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // TODO: undefine IDENT_PLANE for 3D matching
   #ifdef IDENT_PLANE
      error = rox_multi_ident_make(obj->matcher, current_image, dbl_image); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   #else
      error = rox_multi_ident_make_features(obj->matcher, current_image, dbl_image, obj->reference_points_meters_matched, obj->reference_points_meters_extracted); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   #endif

function_terminate:
   return error;
}

Rox_ErrorCode rox_template_ident_se3_get_next_best_pose(Rox_Array2D_Double pose, Rox_Uint * id, Rox_Template_Ident_SE3 template_ident_se3, Rox_Array2D_Double calib_camera)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Template_Ident tmp = NULL;

   if (!template_ident_se3 || !id || !pose || !calib_camera) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_multi_ident_find_next_best(id, template_ident_se3->matcher); 
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   tmp = template_ident_se3->matcher->idents->data[*id];
   
   #ifdef IDENT_PLANE
      // Using Z = 0 prior and computed calibration, compute 3D points in template frame 
      error = rox_points3d_float_from_template ( template_ident_se3->reference_points_meters_matched, tmp->reference_points_matched, tmp->calib_input );
      ROX_ERROR_CHECK_TERMINATE ( error );
   #endif

   // Ransac pose estimation 
   error = rox_ransac_p3p(pose, template_ident_se3->current_points_ransac, template_ident_se3->reference_points_meters_ransac, tmp->current_points_matched, template_ident_se3->reference_points_meters_matched, calib_camera);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Refine pose estimation
   error = rox_points_float_refine_pose_vvs(pose, calib_camera, template_ident_se3->current_points_ransac, template_ident_se3->reference_points_meters_ransac, 15);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
