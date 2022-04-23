//==============================================================================
//
//    OPENROX   : File calibration_stereo_checkerboard_se3.c
//
//    Contents  : Implementation of calibration_stereo_checkerboard_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "calibration_stereo_checkerboard_se3.h"

#include <baseproc/geometry/transforms/matsl3/sl3fromNpoints.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_calibration_stereo_checkerboard_se3_new(
  Rox_Calibration_Stereo_CheckerBoard_SE3 * obj, 
  Rox_Model_CheckerBoard model)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Calibration_Stereo_CheckerBoard_SE3 ret = NULL;
   
    if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
    *obj = NULL;

    ret = (Rox_Calibration_Stereo_CheckerBoard_SE3) rox_memory_allocate(sizeof(*ret), 1);
    if(!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

    // Set pointers to NULL 
    ret->left = 0;
    ret->right = 0;
    ret->stereo = 0;
    ret->Gl = 0;
    ret->Gr = 0;
    ret->checkerdetector = 0;
    ret->ref_pts_3D = 0;
    ret->ref_pts_2D = 0;
    ret->left_detected_corners = 0;
    ret->right_detected_corners = 0;

    ret->nbpts = model->width * model->height;

    ret->ref_pts_3D = (Rox_Point3D_Double )rox_memory_allocate(sizeof(Rox_Point3D_Double_Struct), ret->nbpts);
    if(!ret->ref_pts_3D) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    ret->ref_pts_2D = (Rox_Point2D_Double )rox_memory_allocate(sizeof(Rox_Point2D_Double_Struct), ret->nbpts);
    if(!ret->ref_pts_2D) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    ret->left_detected_corners = (Rox_Point2D_Double )rox_memory_allocate(sizeof(Rox_Point2D_Double_Struct), ret->nbpts);
    if(!ret->left_detected_corners) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    ret->right_detected_corners = (Rox_Point2D_Double )rox_memory_allocate(sizeof(Rox_Point2D_Double_Struct), ret->nbpts);
    if(!ret->right_detected_corners) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    error = rox_calibration_mono_perspective_new(&ret->left);              
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_calibration_mono_perspective_new(&ret->right);             
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_calibration_stereo_perspective_se3_new(&ret->stereo);      
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_matsl3_new ( &ret->Gl );                        
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_matsl3_new ( &ret->Gr );                        
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_ident_checkerboard_new(&ret->checkerdetector);             
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_ident_checkerboard_set_model(ret->checkerdetector, model); 
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    // Set 3D model 
    for ( Rox_Sint i = 0; i < model->height; i++)
    {
        for ( Rox_Sint j = 0; j < model->width; j++)
        {
            ret->ref_pts_3D[i*model->width+j].X = model->sizx * j;
            ret->ref_pts_3D[i*model->width+j].Y = model->sizy * i;
            ret->ref_pts_3D[i*model->width+j].Z = 0.0;

            ret->ref_pts_2D[i*model->width+j].u = model->sizx * j;
            ret->ref_pts_2D[i*model->width+j].v = model->sizy * i;
        }
    }

    error = rox_calibration_mono_perspective_set_model_points(ret->left, ret->ref_pts_3D, ret->nbpts); 
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_calibration_mono_perspective_set_model_points(ret->right, ret->ref_pts_3D, ret->nbpts); 
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_calibration_stereo_perspective_se3_set_model_points(ret->stereo, ret->ref_pts_3D, ret->nbpts); 
    ROX_ERROR_CHECK_TERMINATE ( error );

    *obj = ret;

function_terminate:
    // Delete only if an error occurs
    if(error) rox_calibration_stereo_checkerboard_se3_del(&ret);

    return error;
}

Rox_ErrorCode rox_calibration_stereo_checkerboard_se3_del(
  Rox_Calibration_Stereo_CheckerBoard_SE3 * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Calibration_Stereo_CheckerBoard_SE3 todel;

    if (!obj) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

    todel = *obj;
    *obj = 0;

    if (!todel) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

    rox_calibration_mono_perspective_del(&todel->left);
    rox_calibration_mono_perspective_del(&todel->right);
    rox_calibration_stereo_perspective_se3_del(&todel->stereo);
    rox_matsl3_del ( &todel->Gl );
    rox_matsl3_del ( &todel->Gr );
    rox_ident_checkerboard_del(&todel->checkerdetector);

    rox_memory_delete(todel->left_detected_corners);
    rox_memory_delete(todel->right_detected_corners);
    rox_memory_delete(todel->ref_pts_3D);
    rox_memory_delete(todel->ref_pts_2D);
    rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_stereo_checkerboard_se3_add_left_image(
  Rox_Calibration_Stereo_CheckerBoard_SE3 obj, 
  Rox_Image left)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;
    
    if(!obj || !left)
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    // Left detection 
    error = rox_ident_checkerboard_make(obj->left_detected_corners, obj->checkerdetector, left); 
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_calibration_mono_perspective_add_current_points(obj->left, obj->left_detected_corners, obj->nbpts); 
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_matsl3_from_n_points_double(obj->Gl, obj->ref_pts_2D, obj->left_detected_corners, obj->nbpts);
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_calibration_mono_perspective_add_homography(obj->left, obj->Gl); 
    ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_stereo_checkerboard_se3_add_right_image(
  Rox_Calibration_Stereo_CheckerBoard_SE3 obj, 
  Rox_Image right)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    if(!obj || !right) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    // Right detection 
    error = rox_ident_checkerboard_make(obj->right_detected_corners, obj->checkerdetector, right); 
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_calibration_mono_perspective_add_current_points(obj->right, obj->right_detected_corners, obj->nbpts); 
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_matsl3_from_n_points_double(obj->Gr, obj->ref_pts_2D, obj->right_detected_corners, obj->nbpts); 
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_calibration_mono_perspective_add_homography(obj->right, obj->Gr); 
    ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_stereo_checkerboard_se3_add_stereo_images(
  Rox_Calibration_Stereo_CheckerBoard_SE3 obj, 
  const Rox_Image left, 
  const Rox_Image right
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!obj || !left || !right)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_image_match_size(left, right);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint width = 0, height = 0;
   error = rox_image_get_size(&height, &width, left);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Left detection 
   error = rox_ident_checkerboard_make(obj->left_detected_corners, obj->checkerdetector, left); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matsl3_from_n_points_double(obj->Gl, obj->ref_pts_2D, obj->left_detected_corners, obj->nbpts); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Right detection 
   error = rox_ident_checkerboard_make(obj->right_detected_corners, obj->checkerdetector, right); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matsl3_from_n_points_double(obj->Gr, obj->ref_pts_2D, obj->right_detected_corners, obj->nbpts); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_stereo_perspective_se3_add_current_homographies(obj->stereo, obj->Gl, obj->Gr); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_calibration_stereo_perspective_se3_add_current_points(obj->stereo, obj->left_detected_corners, obj->right_detected_corners, obj->nbpts); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set Kl and Kr 
   error = rox_matut3_build_calibration_matrix ( obj->left->K , 1.0, 1.0, (width-1.0)/2.0, (height-1.0)/2.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matut3_build_calibration_matrix ( obj->right->K, 1.0, 1.0, (width-1.0)/2.0, (height-1.0)/2.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_stereo_checkerboard_se3_make(
  Rox_Calibration_Stereo_CheckerBoard_SE3 obj
)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    if(!obj) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    // Left calibration 
    error = rox_calibration_mono_perspective_compute_parameters(obj->left, 3);
	  ROX_ERROR_CHECK_TERMINATE ( error );

    // Right calibration 
    error = rox_calibration_mono_perspective_compute_parameters(obj->right, 3);
	  ROX_ERROR_CHECK_TERMINATE ( error );

    // Set intrinsic parameters 
    error = rox_calibration_stereo_perspective_se3_set_intrinsics(obj->stereo, obj->left->K, obj->right->K);
	  ROX_ERROR_CHECK_TERMINATE ( error );

    // Stereo calibration 
    error = rox_calibration_stereo_perspective_se3_make(obj->stereo);
	  ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_stereo_checkerboard_se3_get_results(
  Rox_MatUT3 Kl, 
  Rox_MatUT3 Kr, 
  Rox_MatSE3 pose, 
  const Rox_Calibration_Stereo_CheckerBoard_SE3 obj
)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    if(!Kl || !Kr || !pose || !obj) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    error = rox_calibration_mono_perspective_get_intrinsics(Kl, obj->left);
	  ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_calibration_mono_perspective_get_intrinsics(Kr, obj->right);
	  ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_calibration_stereo_perspective_se3_get_results(pose, obj->stereo);
	  ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
