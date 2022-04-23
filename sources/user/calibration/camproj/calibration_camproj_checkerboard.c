//==============================================================================
//
//    OPENROX   : File calibration_camproj_checkerboard.c
//
//    Contents  : Implementation of calibration camproj checkerboard module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "calibration_camproj_checkerboard.h"

#include <system/memory/memory.h>
#include <baseproc/geometry/transforms/matsl3/sl3fromNpoints.h>
#include <inout/geometry/point/point2d_print.h>
#include <inout/system/errors_print.h>

#ifdef ROX_USES_OPENMP
   #include <omp.h>
#endif

#define SAMPLING_OFFSET 0.5

Rox_ErrorCode rox_calibration_camproj_checkerboard_new (
   Rox_Calibration_CamProj_CheckerBoard * obj,
   const Rox_Model_CheckerBoard print_model,
   const Rox_Model_Projector_CheckerBoard proj_model
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Calibration_CamProj_CheckerBoard ret = NULL;

   if ( !obj || !print_model || !proj_model )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Calibration_CamProj_CheckerBoard)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   // Set pointers to NULL
   ret->camproj_calib          = NULL;
   ret->Gc                     = NULL;
   ret->print_detector         = NULL;
   ret->proj_detector          = NULL;
   ret->cam_refs3D             = NULL;
   ret->cam_slicedRefs         = NULL;
   ret->proj_refs2D            = NULL;
   ret->print_detected_corners = NULL;
   ret->proj_detected_corners  = NULL;
   ret->proj_width             = proj_model->image_width;
   ret->proj_height            = proj_model->image_height;
   ret->inverse_sampling       = 1.0;

   ret->print_npts = print_model->width * print_model->height;
   ret->proj_npts  = proj_model->cols   * proj_model->rows;

   ret->cam_refs3D = (Rox_Point3D_Double ) rox_memory_allocate(sizeof(Rox_Point3D_Double_Struct), ret->print_npts);
   if (!ret->cam_refs3D)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   ret->cam_slicedRefs = ( Rox_Point2D_Double ) rox_memory_allocate(sizeof(Rox_Point2D_Double_Struct), ret->print_npts);
   if (!ret->cam_slicedRefs)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   ret->proj_refs2D = ( Rox_Point2D_Double ) rox_memory_allocate(sizeof(Rox_Point2D_Double_Struct), ret->proj_npts);
   if (!ret->proj_refs2D)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   ret->print_detected_corners = (Rox_Point2D_Double )rox_memory_allocate(sizeof(Rox_Point2D_Double_Struct), ret->print_npts);
   if (!ret->print_detected_corners)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   ret->proj_detected_corners = (Rox_Point2D_Double )rox_memory_allocate(sizeof(Rox_Point2D_Double_Struct), ret->proj_npts);
   if (!ret->proj_detected_corners)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_calibration_camproj_perspective_new(&ret->camproj_calib);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_matsl3_new ( &ret->Gc );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ident_checkerboard_new ( &ret->print_detector);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ident_checkerboard_set_model ( ret->print_detector, print_model );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ident_checkerboard_projector_new ( &ret->proj_detector );             
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ident_checkerboard_projector_set_model ( ret->proj_detector, proj_model );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set printed model
   for(Rox_Sint i = 0; i < print_model->height; i++)
      for(Rox_Sint j = 0; j < print_model->width; j++)
      {
         ret->cam_refs3D[i*print_model->width+j].X = print_model->sizx * j;
         ret->cam_refs3D[i*print_model->width+j].Y = print_model->sizy * i;
         ret->cam_refs3D[i*print_model->width+j].Z = 0.0;

         ret->cam_slicedRefs[i*print_model->width+j].u = print_model->sizx * j;
         ret->cam_slicedRefs[i*print_model->width+j].v = print_model->sizy * i;
      }

   error = rox_calibration_camproj_perspective_set_cam_model_points ( ret->camproj_calib, ret->cam_refs3D, ret->print_npts );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set projected model
   for ( Rox_Sint i = 0; i < ret->proj_npts; i++ )
   {
      ret->proj_refs2D[i].u = proj_model->elements[i].u;
      ret->proj_refs2D[i].v = proj_model->elements[i].v;
   }

   error = rox_calibration_camproj_perspective_set_proj_model_points ( ret->camproj_calib, ret->proj_refs2D, ret->proj_npts );
   ROX_ERROR_CHECK_TERMINATE ( error );

   *obj = ret;

function_terminate:
   // Delete only if an error occurs
   if (error) rox_calibration_camproj_checkerboard_del ( &ret );
   return error;
}

Rox_ErrorCode rox_calibration_camproj_checkerboard_del ( Rox_Calibration_CamProj_CheckerBoard * obj )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Calibration_CamProj_CheckerBoard todel = NULL;

   if ( !obj ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   if ( !todel ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_calibration_camproj_perspective_del(&todel->camproj_calib);
   rox_matsl3_del(&todel->Gc);
   rox_ident_checkerboard_del(&todel->print_detector);
   rox_ident_checkerboard_projector_del(&todel->proj_detector);

   rox_memory_delete(todel->print_detected_corners);
   rox_memory_delete(todel->proj_detected_corners);
   rox_memory_delete(todel->cam_refs3D);
   rox_memory_delete(todel->cam_slicedRefs);
   rox_memory_delete(todel->proj_refs2D);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_checkerboard_add_images (
   Rox_Calibration_CamProj_CheckerBoard obj, 
   const Rox_Image print, 
   const Rox_Image proj
)
{
   Rox_ErrorCode errors[2];
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint      width = 0, height = 0;

   if ( !obj )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !print || !proj )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_image_match_size ( print, proj );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_get_size ( &height, &width, print );
   ROX_ERROR_CHECK_TERMINATE ( error );

   #ifdef ROX_USES_OPENMP
   #pragma omp parallel num_threads(2)
   #endif
   {
      Rox_Sint i = 0;
      #ifdef ROX_USES_OPENMP
      #pragma omp for private(i)
      #endif
      for ( i = 0; i < 2; i++ )
      {
         // Try print detection
         if ( i%2 )
            errors[i] = rox_ident_checkerboard_make ( obj->print_detected_corners, obj->print_detector, print );
         // Try proj detection
         else
            errors[i] = rox_ident_checkerboard_projector_make ( obj->proj_detected_corners, obj->proj_detector, proj );
      }
   }

   if (errors[0]) { error = errors[0]; goto function_terminate; }
   if (errors[1]) { error = errors[1]; goto function_terminate; }

   // Correct sampling
   if ( ( obj->inverse_sampling > 0.0 ) && ( obj->inverse_sampling  < 1.0 ) )
   {
      for ( Rox_Sint i=0; i<obj->print_npts; ++i )
      {
         obj->print_detected_corners[i].u *= obj->inverse_sampling;
         obj->print_detected_corners[i].v *= obj->inverse_sampling;

         obj->print_detected_corners[i].u += SAMPLING_OFFSET * obj->inverse_sampling;
         obj->print_detected_corners[i].v += SAMPLING_OFFSET * obj->inverse_sampling;
      }

      for (Rox_Sint i=0; i<obj->proj_npts; ++i)
      {
         obj->proj_detected_corners[i].u *= obj->inverse_sampling;
         obj->proj_detected_corners[i].v *= obj->inverse_sampling;

         obj->proj_detected_corners[i].u += SAMPLING_OFFSET * obj->inverse_sampling;
         obj->proj_detected_corners[i].v += SAMPLING_OFFSET * obj->inverse_sampling;
      }
   }

   // Try compute camera homography
   error = rox_matsl3_from_n_points_double ( obj->Gc, obj->cam_slicedRefs, obj->print_detected_corners, obj->print_npts);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set resolutions
   error = rox_calibration_camproj_perspective_set_cam_resolution ( obj->camproj_calib, width, height);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_camproj_perspective_set_proj_resolution ( obj->camproj_calib, obj->proj_width, obj->proj_height );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // If successful:

   // Add homography and points
   error = rox_calibration_camproj_perspective_add_current_cam_homography ( obj->camproj_calib, obj->Gc );
   ROX_ERROR_CHECK_TERMINATE(error);

   error = rox_calibration_camproj_perspective_add_current_points ( obj->camproj_calib, obj->print_detected_corners, obj->print_npts, obj->proj_detected_corners, obj->proj_npts );
   ROX_ERROR_CHECK_TERMINATE(error);

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_checkerboard_add_points (
   Rox_Calibration_CamProj_CheckerBoard obj,
   const Rox_Point2D_Double print_obs2D,
   const Rox_Sint           print_npts,
   const Rox_Point2D_Double proj_obs2D,
   const Rox_Sint           proj_npts,
   const Rox_Sint           cam_width,
   const Rox_Sint           cam_height
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint      width = 0, height = 0;
   Rox_Double    sampling_bias = 0.0;
   Rox_Double    sampling_gain = 1.0;


   if ( !obj ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if ( print_npts != obj->print_npts ||  proj_npts != obj->proj_npts ) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Correct sampling
   if ( ( obj->inverse_sampling > 0.0) && ( obj->inverse_sampling < 1.0) )
   {
      sampling_bias = SAMPLING_OFFSET;
      sampling_gain = obj->inverse_sampling;
   }

   width  = (Rox_Sint) (cam_width  * sampling_gain);
   height = (Rox_Sint) (cam_height * sampling_gain);

   for (Rox_Sint i=0; i<obj->print_npts; ++i)
   {
      obj->print_detected_corners[i].u = ( print_obs2D[i].u + sampling_bias ) * sampling_gain;
      obj->print_detected_corners[i].v = ( print_obs2D[i].v + sampling_bias ) * sampling_gain;
   }
   for (Rox_Sint i=0; i<obj->proj_npts; ++i)
   {
      obj->proj_detected_corners[i].u = ( proj_obs2D[i].u + sampling_bias ) * sampling_gain;
      obj->proj_detected_corners[i].v = ( proj_obs2D[i].v + sampling_bias ) * sampling_gain;
   }

   // Try compute camera homography
   error = rox_matsl3_from_n_points_double(obj->Gc, obj->cam_slicedRefs, obj->print_detected_corners, obj->print_npts);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Set resolutions
   error = rox_calibration_camproj_perspective_set_cam_resolution(obj->camproj_calib, width, height);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_calibration_camproj_perspective_set_proj_resolution(obj->camproj_calib, obj->proj_width, obj->proj_height);
   ROX_ERROR_CHECK_TERMINATE(error)

   // If successful:
   // Add homography and points
   error = rox_calibration_camproj_perspective_add_current_cam_homography( obj->camproj_calib, obj->Gc);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_calibration_camproj_perspective_add_current_points(obj->camproj_calib, obj->print_detected_corners, obj->print_npts, obj->proj_detected_corners, obj->proj_npts);
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_checkerboard_make(Rox_Calibration_CamProj_CheckerBoard obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_calibration_camproj_perspective_make ( obj->camproj_calib );
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_checkerboard_check_linear_results(
   Rox_Calibration_CamProj_CheckerBoard  obj,
   Rox_Double * error_cam,
   Rox_Double * error_proj,
   const Rox_Sint  index
)
{
   return rox_calibration_camproj_perspective_check_linear_results ( obj->camproj_calib, error_cam, error_proj, index);
}

Rox_ErrorCode rox_calibration_camproj_checkerboard_check_fine_results(
   Rox_Calibration_CamProj_CheckerBoard obj,
   Rox_Double * error_cam,
   Rox_Double * mean_error_proj_fwd,
   Rox_Double * median_error_proj_fwd,
   Rox_Double * mean_error_proj_bwd,
   Rox_Double * median_error_proj_bwd,
   const Rox_Sint     index
)
{
   return rox_calibration_camproj_perspective_check_refined_results (
      obj->camproj_calib,
      error_cam,
      mean_error_proj_fwd, median_error_proj_fwd,
      mean_error_proj_bwd, median_error_proj_bwd,
      index 
   );
}

Rox_ErrorCode rox_calibration_camproj_checkerboard_get_raw_intrinsics (
   Rox_Calibration_CamProj_CheckerBoard obj,
   const Rox_MatUT3 Kc,
   const Rox_MatUT3 Kp
)
{
   return rox_calibration_camproj_perspective_get_linear_intrinsics ( obj->camproj_calib, Kc, Kp );
}

Rox_ErrorCode rox_calibration_camproj_checkerboard_get_fine_intrinsics (
   Rox_Calibration_CamProj_CheckerBoard obj,
   Rox_MatUT3 Kc,
   Rox_MatUT3 Kp
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!Kc || !Kp || !obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_calibration_camproj_perspective_get_refined_intrinsics ( obj->camproj_calib, Kc, Kp );
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_checkerboard_get_raw_pTc (
   Rox_Calibration_CamProj_CheckerBoard obj,
   const Rox_MatSE3 pTc,
   const Rox_Sint index
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !pTc || !obj ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_calibration_camproj_perspective_get_indexed_linear_pTc ( obj->camproj_calib, pTc, index );
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_checkerboard_get_fine_pTc (
   Rox_Calibration_CamProj_CheckerBoard obj,
   const Rox_MatSE3 pTc
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !pTc || !obj ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   error = rox_calibration_camproj_perspective_get_refined_pTc( obj->camproj_calib, pTc );
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_checkerboard_set_camera_intrinsics (
   Rox_Calibration_CamProj_CheckerBoard obj,
   Rox_MatUT3 Kin
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !obj )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !Kin ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_calibration_camproj_perspective_set_camera_intrinsics( obj->camproj_calib, Kin );
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_checkerboard_set_images_inverse_sampling (
   Rox_Calibration_CamProj_CheckerBoard obj,
   const Rox_Double inverse_sampling
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (( inverse_sampling < 0.0 ) || ( inverse_sampling > 1.0 ))
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   obj->inverse_sampling = inverse_sampling;

function_terminate:
   return error;
}
