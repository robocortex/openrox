//==============================================================================
//
//    OPENROX   : File calibration_stereo_checkerboard.c
//
//    Contents  : Implementation of calibration_stereo_checkerboard module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "calibration_stereo_checkerboard.h"

#include <baseproc/geometry/transforms/matsl3/sl3fromNpoints.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_calibration_stereo_checkerboard_new (
   Rox_Calibration_Stereo_CheckerBoard * calibration_stereo_checkerBoard, 
   const Rox_Model_CheckerBoard model
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Calibration_Stereo_CheckerBoard ret = NULL;

   if(!calibration_stereo_checkerBoard ||!model) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *calibration_stereo_checkerBoard = NULL;

   ret = (Rox_Calibration_Stereo_CheckerBoard)rox_memory_allocate(sizeof(*ret), 1);
   if(!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Set pointers to NULL 
   ret->stereo_calib = 0;
   ret->Gl = 0;
   ret->Gr = 0;
   ret->checkerdetector = 0;
   ret->ref_pts_3D = 0;
   ret->ref_pts_2D = 0;
   ret->left_detected_corners = 0;
   ret->right_detected_corners = 0;

   ret->nbpts = model->width * model->height;

   ret->ref_pts_3D = (Rox_Point3D_Double_Struct *) rox_memory_allocate(sizeof(Rox_Point3D_Double_Struct ), ret->nbpts);
   if (!ret->ref_pts_3D) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->ref_pts_2D = (Rox_Point2D_Double_Struct *)rox_memory_allocate(sizeof(Rox_Point2D_Double_Struct ), ret->nbpts);
   if (!ret->ref_pts_2D) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->left_detected_corners = (Rox_Point2D_Double_Struct *)rox_memory_allocate(sizeof(Rox_Point2D_Double_Struct), ret->nbpts);
   if (!ret->left_detected_corners) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->right_detected_corners = (Rox_Point2D_Double_Struct *)rox_memory_allocate(sizeof(Rox_Point2D_Double_Struct), ret->nbpts);
   if (!ret->right_detected_corners) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_calibration_stereo_perspective_new(&ret->stereo_calib);
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

   error = rox_calibration_stereo_perspective_set_model_points(ret->stereo_calib, ret->ref_pts_3D, ret->nbpts); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   *calibration_stereo_checkerBoard = ret;

function_terminate:
   // Delete only if an error occurs
   if(error) rox_calibration_stereo_checkerboard_del(&ret);
   return error;
}

Rox_ErrorCode rox_calibration_stereo_checkerboard_del(
   Rox_Calibration_Stereo_CheckerBoard * calibration_stereo_checkerBoard
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Calibration_Stereo_CheckerBoard todel = NULL;

   if (!calibration_stereo_checkerBoard) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *calibration_stereo_checkerBoard;
   *calibration_stereo_checkerBoard = 0;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_calibration_stereo_perspective_del(&todel->stereo_calib);
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

Rox_ErrorCode rox_calibration_stereo_checkerboard_add_images(
   Rox_Calibration_Stereo_CheckerBoard calibration_stereo_checkerBoard, 
   const Rox_Image left, 
   const Rox_Image right
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!calibration_stereo_checkerBoard || !left || !right) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_image_match_size(left, right);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint width = 0, height = 0;
   error = rox_image_get_size(&height, &width, left);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Left detection 
   error = rox_ident_checkerboard_make(calibration_stereo_checkerBoard->left_detected_corners, calibration_stereo_checkerBoard->checkerdetector, left); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matsl3_from_n_points_double(calibration_stereo_checkerBoard->Gl, calibration_stereo_checkerBoard->ref_pts_2D, calibration_stereo_checkerBoard->left_detected_corners, calibration_stereo_checkerBoard->nbpts); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Right detection 
   error = rox_ident_checkerboard_make(calibration_stereo_checkerBoard->right_detected_corners, calibration_stereo_checkerBoard->checkerdetector, right); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matsl3_from_n_points_double(calibration_stereo_checkerBoard->Gr, calibration_stereo_checkerBoard->ref_pts_2D, calibration_stereo_checkerBoard->right_detected_corners, calibration_stereo_checkerBoard->nbpts); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_stereo_perspective_add_current_homographies(calibration_stereo_checkerBoard->stereo_calib, calibration_stereo_checkerBoard->Gl, calibration_stereo_checkerBoard->Gr); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_calibration_stereo_perspective_add_current_points(calibration_stereo_checkerBoard->stereo_calib, calibration_stereo_checkerBoard->left_detected_corners, calibration_stereo_checkerBoard->right_detected_corners, calibration_stereo_checkerBoard->nbpts); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set resolution 
   error = rox_calibration_stereo_perspective_set_resolution(calibration_stereo_checkerBoard->stereo_calib, width, height); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_stereo_checkerboard_make (
   Rox_Calibration_Stereo_CheckerBoard calibration_stereo_checkerBoard
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if(!calibration_stereo_checkerBoard) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_calibration_stereo_perspective_make(calibration_stereo_checkerBoard->stereo_calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_stereo_checkerboard_get_results (
   Rox_MatUT3 Kl, 
   Rox_MatUT3 Kr, 
   Rox_MatSE3 pose, 
   const Rox_Calibration_Stereo_CheckerBoard calibration_stereo_checkerBoard
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
    
   if(!Kl || !Kr || !pose || !calibration_stereo_checkerBoard) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_calibration_stereo_perspective_get_results(Kl, Kr, pose, calibration_stereo_checkerBoard->stereo_calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
