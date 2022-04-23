//==============================================================================
//
//    OPENROX   : File camera_calibration.c
//
//    Contents  : Implementation of camera calibration module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "camera_calibration.h"

#include <system/memory/memory.h>

#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/geometry/point/point2d_matsl3_transform.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/rectangle/rectangle.h>

//#include <core/model/model_single_plane_struct.h>
#include <core/model/model_single_plane_struct.h>

#include <inout/system/errors_print.h>

#include <user/tracking/tracking.h>
#include <user/tracking/tracking_params.h>

Rox_ErrorCode rox_camera_calibration_new ( 
   Rox_Camera_Calibration * calibration, 
   const Rox_Model_Single_Plane model_single_plane 
)
{
   Rox_ErrorCode          error = ROX_ERROR_NONE;
   Rox_Sint               cols = 0, rows = 0;
   Rox_Point3D_Double_Struct     model_pts[4];
   Rox_Point2D_Double_Struct     ref_pts[4];
   Rox_Camera_Calibration ret = NULL;
   Rox_Tracking_Params    params = NULL;
   Rox_Double               size_x = 0.0;
   Rox_Double               size_y = 0.0;
   Rox_Image      model = NULL;
   
   if (!calibration || !model_single_plane)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   size_x = model_single_plane->sizex;
   size_y = model_single_plane->sizey;
   model  = model_single_plane->image_template;

   if ( size_x <= 0.0 || size_y <= 0.0 ) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); } 
   
   *calibration = NULL;

   ret = ( Rox_Camera_Calibration ) rox_memory_allocate( sizeof( *ret ), 1 );
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Set to NULL all intern pointers 
   ret->calib                = NULL;
   ret->tracker              = NULL;
   ret->identifier           = NULL;
   ret->template_calibration = NULL;
   ret->intrinsics           = NULL;
   ret->G                    = NULL;
   ret->width                = 0;
   ret->height               = 0;

   // Tracking params 
   error = rox_tracking_params_new(&params); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Model 
   error = rox_image_get_size(&rows, &cols, model);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // write a function to build the image coordinates
   ref_pts[0].u =     -0.5; ref_pts[0].v =     -0.5;
   ref_pts[1].u = cols-0.5; ref_pts[1].v =     -0.5;
   ref_pts[2].u = cols-0.5; ref_pts[2].v = rows-0.5;
   ref_pts[3].u =     -0.5; ref_pts[3].v = rows-0.5;

   // Build G_rm 
   error = rox_matut3_new( &ret->template_calibration );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_transformtools_build_calibration_matrix_for_template( ret->template_calibration, ( Rox_Double ) cols, ( Rox_Double ) rows, size_x, size_y );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matsl3_new( &ret->G );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_new( &ret->intrinsics );
   ROX_ERROR_CHECK_TERMINATE ( error );

  // Identification 
   error = rox_ident_texture_sl3_new( &ret->identifier );             
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ident_texture_sl3_set_model( ret->identifier, model ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Tracking 
   error = rox_tracking_new( &ret->tracker, params, model );          
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_tracking_set_score_thresh( ret->tracker, 0.9 );        
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_tracking_set_miter( ret->tracker, 12 );                
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Calibration object 
   error = rox_calibration_mono_perspective_new( &ret->calib );       
   ROX_ERROR_CHECK_TERMINATE(error)

   // model_pts[0].X = -size_x / 2.0; model_pts[0].Y = -size_y / 2.0; model_pts[0].Z = 0;
   // model_pts[1].X =  size_x / 2.0; model_pts[1].Y = -size_y / 2.0; model_pts[1].Z = 0;
   // model_pts[2].X =  size_x / 2.0; model_pts[2].Y =  size_y / 2.0; model_pts[2].Z = 0;
   // model_pts[3].X = -size_x / 2.0; model_pts[3].Y =  size_y / 2.0; model_pts[3].Z = 0;

   error = rox_rectangle3d_create_centered_plane_xright_ydown ( model_pts, size_x, size_y );
   ROX_ERROR_CHECK_TERMINATE ( error );


   error = rox_calibration_mono_perspective_set_model_points( ret->calib, model_pts, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_double_new( &ret->ref_pts,   10 );          
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_dynvec_point2d_double_append( ret->ref_pts, &ref_pts[0] ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_dynvec_point2d_double_append( ret->ref_pts, &ref_pts[1] ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_dynvec_point2d_double_append( ret->ref_pts, &ref_pts[2] ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_dynvec_point2d_double_append( ret->ref_pts, &ref_pts[3] ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   *calibration = ret;

function_terminate:
   // Delete onli if an error occurs
   if (error) rox_camera_calibration_del( &ret );

   rox_tracking_params_del( &params );
   return error;
}

Rox_ErrorCode rox_camera_calibration_del ( Rox_Camera_Calibration * calibration )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Delete the camera calibration strcuture 
   Rox_Camera_Calibration todel = NULL;

   if (!calibration)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   todel = *calibration;
   *calibration = 0;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_matut3_del( &todel->intrinsics );
   rox_matsl3_del( &todel->G );
   rox_matut3_del( &todel->template_calibration );
   rox_tracking_del ( &todel->tracker );
   rox_ident_texture_sl3_del( &todel->identifier );
   rox_calibration_mono_perspective_del( &todel->calib );
   rox_dynvec_point2d_double_del( &todel->ref_pts );

   rox_memory_delete(todel);
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_camera_calibration_add_image ( 
   Rox_Camera_Calibration calibration, 
   const Rox_Image image
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_MatSL3 H = NULL;
   Rox_Point2D_Double_Struct cur_pts[4];
   Rox_Sint is_identified = 0;

   if (!calibration || !image) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Get homography matrix 
   H = calibration->tracker->homography;

   // First image? 
   if ( calibration->calib->homographies->used == 0 )
   {
      error = rox_image_get_size(&calibration->height, &calibration->width, image);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Check the image resolution 
   error = rox_image_check_size( image, calibration->height, calibration->width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Identify the 2D model image in the current calibration image 
   error = rox_ident_texture_sl3_make(&is_identified, H, calibration->identifier, image );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Refine the homography 
   error = rox_tracking_make( calibration->tracker, image );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // If you modify the following lines you'll probably want to modify
   // rox_camera_calibration_add_homography as well

   // Compute 2D->3D homography 
   error = rox_array2d_double_mulmatmat( calibration->G, H, calibration->template_calibration );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Warp model coordinates 
   error = rox_point2d_double_homography( cur_pts, calibration->ref_pts->data, H, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Add measure 
   error = rox_calibration_mono_perspective_add_measure( calibration->calib, calibration->G, cur_pts, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_camera_calibration_add_homography (
   Rox_Camera_Calibration calibration, 
   Rox_MatSL3 homography, 
   Rox_Sint image_width, 
   Rox_Sint image_height
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_MatSL3 H = NULL;
   Rox_Point2D_Double_Struct cur_pts[4];

   if (!calibration || !homography)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // First image? 
   if ( calibration->calib->homographies->used == 0 )
   {
      calibration->width  = image_width;
      calibration->height = image_height;
   }

   // Check the image resolution 
   if ( ( image_width != calibration->width ) || ( image_height != calibration->height ) )
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Set tracker homography matrix 
   error = rox_tracking_set_homography( calibration->tracker, homography );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // replace with rox_tracking_get_homography ??? 
   H = calibration->tracker->homography;

   // If you modify the following lines you'll probably want to modify
   // rox_camera_calibration_add_image as well

   // Compute 2D->3D homography 
   error = rox_array2d_double_mulmatmat( calibration->G, H, calibration->template_calibration );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Warp model coordinates 
   error = rox_point2d_double_homography( cur_pts, calibration->ref_pts->data, H, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Add measure 
   error = rox_calibration_mono_perspective_add_measure( calibration->calib, calibration->G, cur_pts, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_camera_calibration_make ( 
   Rox_Camera_Calibration calibration, 
   const Rox_Sint method 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!calibration) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (method > 5)
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Initialize the principal point to the image center 
   error = rox_matut3_build_calibration_matrix ( calibration->intrinsics, 1.0, 1.0, (calibration->width-1.0)/2.0, (calibration->height-1.0)/2.0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_mono_perspective_set_intrinsics( calibration->calib, calibration->intrinsics );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_calibration_mono_perspective_compute_parameters( calibration->calib, method );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // update intrinsic parameters 
   error = rox_calibration_mono_perspective_get_intrinsics( calibration->intrinsics, calibration->calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_camera_calibration_get_intrinsics ( 
   Rox_MatUT3 intrinsics, 
   const Rox_Camera_Calibration calibration 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
  
   if (!intrinsics || !calibration) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_matut3_copy( intrinsics, calibration->intrinsics );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_camera_calibration_get_mean_error ( 
   Rox_Double * mean, 
   const Rox_Camera_Calibration calibration
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!mean || !calibration)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_calibration_mono_perspective_get_mean_error ( mean, calibration->calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
