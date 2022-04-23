//==============================================================================
//
//    OPENROX   : File camera_calibration_checkerboard.c
//
//    Contents  : Implementation of camera_calibration_checkerboard module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "camera_calibration_checkerboard.h"

#include <system/memory/memory.h>

#include <baseproc/array/fill/fillunit.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/geometry/point/point2d_matsl3_transform.h>
#include <baseproc/geometry/transforms/matsl3/sl3fromNpoints.h>

#include <core/calibration/mono/calibration_perspective.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_camera_calibration_checkerboard_new (
   Rox_Camera_Calibration_Checkerboard * obj,
   const Rox_Model_CheckerBoard model )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Camera_Calibration_Checkerboard ret = NULL;
   
   if ( !obj || !model ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *obj = NULL;

   if ( !model->height || !model->width ) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   ret = ( Rox_Camera_Calibration_Checkerboard ) rox_memory_allocate( sizeof( *ret ), 1 );
   if ( !ret ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Set to NULL all intern pointers
   ret->calib           = NULL;
   ret->intrinsics      = NULL;
   ret->G               = NULL;
   ret->ref_pts2D       = NULL;
   ret->ref_pts3D       = NULL;
   ret->detected_pts    = NULL;
   ret->checkerdetector = NULL;

   ret->nbpts = model->height * model->width;

   ret->ref_pts2D = ( Rox_Point2D_Double  ) rox_memory_allocate( sizeof( Rox_Point2D_Double_Struct ), ret->nbpts );
   if ( !ret->ref_pts2D )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->detected_pts = ( Rox_Point2D_Double  ) rox_memory_allocate( sizeof( Rox_Point2D_Double_Struct ), ret->nbpts );
   if ( !ret->detected_pts )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->ref_pts3D = ( Rox_Point3D_Double  ) rox_memory_allocate( sizeof( Rox_Point3D_Double_Struct ), ret->nbpts );
   if ( !ret->ref_pts3D )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_calibration_mono_perspective_new( &ret->calib );             
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matsl3_new( &ret->G );                         
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matut3_new( &ret->intrinsics );                
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_ident_checkerboard_new( &ret->checkerdetector );             
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_ident_checkerboard_set_model( ret->checkerdetector, model ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Define the model
   for (Rox_Sint i = 0; i < model->height; i++ )
   {
       for (Rox_Sint j = 0; j < model->width; j++ )
       {

           //ret->ref_pts3D[i*model->width+j].X = model->sizx * j;
           //ret->ref_pts3D[i*model->width+j].Y = model->sizy * i;
           //ret->ref_pts3D[i*model->width+j].Z = 0.0;

           ret->ref_pts3D[i*model->width+j].X = model->points3D->data[i*model->width+j].X;
           ret->ref_pts3D[i*model->width+j].Y = model->points3D->data[i*model->width+j].Y;
           ret->ref_pts3D[i*model->width+j].Z = model->points3D->data[i*model->width+j].Z;

           // vector with fake points to compute intermodel homography supposing Z = 0
           ret->ref_pts2D[i*model->width+j].u = model->points3D->data[i*model->width+j].X;
           ret->ref_pts2D[i*model->width+j].v = model->points3D->data[i*model->width+j].Y;
       }
   }

   error = rox_calibration_mono_perspective_set_model_points( ret->calib, ret->ref_pts3D, ret->nbpts );
   ROX_ERROR_CHECK_TERMINATE ( error );

   *obj = ret;

function_terminate:
   // Delete only if an error occurs
   if (error) rox_camera_calibration_checkerboard_del( &ret );

   return error;
}


Rox_ErrorCode rox_camera_calibration_checkerboard_del ( Rox_Camera_Calibration_Checkerboard * obj )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // Delete the camera calibration structure
   Rox_Camera_Calibration_Checkerboard todel = NULL;

   if ( !obj )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = 0;

   if ( !todel ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_matsl3_del( &todel->G );
   rox_matut3_del( &todel->intrinsics );
   rox_calibration_mono_perspective_del( &todel->calib );
   rox_ident_checkerboard_del( &todel->checkerdetector );

   rox_memory_delete( todel->ref_pts2D );
   rox_memory_delete( todel->detected_pts );
   rox_memory_delete( todel->ref_pts3D );
   rox_memory_delete( todel );

function_terminate:
   return error;
}

Rox_ErrorCode rox_camera_calibration_checkerboard_add_image (
   Rox_Camera_Calibration_Checkerboard obj,
   const Rox_Image image
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !obj || !image )    
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_ident_checkerboard_make( obj->detected_pts, obj->checkerdetector, image );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // If you modify the following lines you'll probably want to modify
   // rox_camera_calibration_checkerboard_add_points as well
   error = rox_matsl3_from_n_points_double( obj->G, obj->ref_pts2D, obj->detected_pts, obj->nbpts );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_mono_perspective_add_current_points( obj->calib, obj->detected_pts, obj->nbpts );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_mono_perspective_add_homography( obj->calib, obj->G );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_get_size(&obj->image_height, &obj->image_width, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_camera_calibration_checkerboard_add_points(
   Rox_Camera_Calibration_Checkerboard obj,
   const Rox_Sint n_points,
   const Rox_Point2D_Double  points,
   const Rox_Sint image_width,
   const Rox_Sint image_height)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !obj || !points ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if ( n_points != obj->nbpts )        
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Copy input points to object points
   for (Rox_Sint i = 0; i < n_points; i++ )
   {
      obj->detected_pts[i].u = points[i].u;
      obj->detected_pts[i].v = points[i].v;
   }

   // If you modify the following lines you'll probably want to modify
   // rox_camera_calibration_checkerboard_add_image as well
   error = rox_matsl3_from_n_points_double( obj->G, obj->ref_pts2D, obj->detected_pts, obj->nbpts );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_mono_perspective_add_current_points( obj->calib, obj->detected_pts, obj->nbpts );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_mono_perspective_add_homography( obj->calib, obj->G );
   ROX_ERROR_CHECK_TERMINATE ( error );

   obj->image_width  = image_width;
   obj->image_height = image_height;

function_terminate:
   return error;
}


Rox_ErrorCode rox_camera_calibration_checkerboard_make (
   Rox_Camera_Calibration_Checkerboard obj,
   const Rox_Sint method
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj)       
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (method > 5) 
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Initialize the principal point to the image center
   error = rox_matut3_build_calibration_matrix ( obj->intrinsics, 1.0, 1.0, (obj->image_width-1.0)/2.0, (obj->image_height-1.0)/2.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_mono_perspective_set_intrinsics( obj->calib, obj->intrinsics );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_mono_perspective_compute_parameters ( obj->calib, method );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // update intrinsic parameters
   error = rox_calibration_mono_perspective_get_intrinsics ( obj->intrinsics, obj->calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_camera_calibration_checkerboard_get_intrinsics (
   Rox_MatUT3 intrinsics,
   const Rox_Camera_Calibration_Checkerboard obj
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!intrinsics || !obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matut3_copy ( intrinsics, obj->intrinsics );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_camera_calibration_checkerboard_get_pose(
   Rox_MatSE3 pose,
   const Rox_Camera_Calibration_Checkerboard calibration,
   const Rox_Sint index
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!pose || !calibration)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_ObjSet_Array2D_Double poses = calibration->calib->poses;

   if ((Rox_Uint) index >= poses->used)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_copy ( pose, calibration->calib->poses->data[index] );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_camera_calibration_checkerboard_get_mean_error(
   Rox_Double * mean,
   const Rox_Camera_Calibration_Checkerboard calibration
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !mean || !calibration )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_calibration_mono_perspective_get_mean_error ( mean, calibration->calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
  return error;
}
