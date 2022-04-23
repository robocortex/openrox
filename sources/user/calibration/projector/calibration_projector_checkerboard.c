//==============================================================================
//
//    OPENROX   : File calibration_projector_checkerboard.c
//
//    Contents  : Implementation of calibration_projector_checkerboard module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "calibration_projector_checkerboard.h"

#include <system/memory/memory.h>

#include <core/calibration/projector/calibration_perspective.h>

#include <baseproc/array/fill/fillunit.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/geometry/point/point2d_matsl3_transform.h>
#include <baseproc/geometry/transforms/matsl3/sl3fromNpoints.h>

#include <inout/system/errors_print.h>


Rox_ErrorCode rox_calibration_projector_checkerboard_new (
   Rox_Calibration_Projector_CheckerBoard * obj,
   const Rox_Model_Projector_CheckerBoard model
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Calibration_Projector_CheckerBoard ret = NULL;

   if ( !obj || !model)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   if (!model->rows || !model->cols || !model->image_width || !model->image_height)
   {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !model->elements )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret = (Rox_Calibration_Projector_CheckerBoard) rox_memory_allocate(sizeof(*ret), 1);
   if(!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Set to NULL all intern pointers
   ret->calib           = NULL;
   ret->intrinsics      = NULL;
   ret->G               = NULL;
   ret->ref_pts2D       = NULL;
   ret->checkerdetector = NULL;

   ret->image_width     = model->image_width;
   ret->image_height    = model->image_height;

   ret->nbpts           = model->rows * model->cols ;

   ret->ref_pts2D = (Rox_Point2D_Double ) rox_memory_allocate(sizeof(Rox_Point2D_Double_Struct), ret->nbpts);

   if (!ret->ref_pts2D)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   error = rox_calibration_projector_perspective_new(&ret->calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matsl3_new ( &ret->G );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_new ( &ret->intrinsics );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Copy the reference points
   for ( Rox_Sint i = 0; i < ret->nbpts; i++)
   {
      ret->ref_pts2D[i].u = model->elements[i].u;
      ret->ref_pts2D[i].v = model->elements[i].v;
   }

   error = rox_calibration_projector_perspective_set_model_points(ret->calib, ret->ref_pts2D, ret->nbpts);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *obj = ret;

function_terminate:
   if(error) rox_calibration_projector_checkerboard_del(&ret);

   return error;
}

Rox_ErrorCode rox_calibration_projector_checkerboard_del (
   Rox_Calibration_Projector_CheckerBoard *obj
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // Delete the projector calibration structure
   Rox_Calibration_Projector_CheckerBoard todel = NULL;

   if(!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_matsl3_del(&todel->G);
   rox_matut3_del(&todel->intrinsics);
   rox_calibration_projector_perspective_del(&todel->calib);
   ROX_ERROR_CHECK(rox_ident_checkerboard_del(&todel->checkerdetector));

   rox_memory_delete(todel->ref_pts2D);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

// TODO: rename the function ??? add_image but we add points
Rox_ErrorCode rox_calibration_projector_checkerboard_add_image (
   Rox_Calibration_Projector_CheckerBoard obj,
   Rox_Point2D_Double  obs2D,
   const Rox_Sint nobs
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !obs2D)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( nobs != obj->nbpts )
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matsl3_from_n_points_double(obj->G, obs2D, obj->ref_pts2D, obj->nbpts);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_projector_perspective_add_current_points(obj->calib, obs2D, obj->nbpts);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_projector_perspective_add_homography(obj->calib, obj->G);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_projector_checkerboard_make (
   Rox_Calibration_Projector_CheckerBoard obj,
   const Rox_Sint method
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if(method > 5)
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Initialize the principal point to the image center
   error = rox_matut3_build_calibration_matrix ( obj->intrinsics, 1.0, 1.0, (obj->image_width-1.0)/2.0, (obj->image_height-1.0)/2.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_projector_perspective_set_intrinsics(obj->calib, obj->intrinsics);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_projector_perspective_compute_parameters(obj->calib, method);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // update intrinsic parameters
   error = rox_calibration_projector_perspective_get_intrinsics(obj->intrinsics, obj->calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_projector_checkerboard_get_intrinsics (
   Rox_MatUT3 intrinsics,
   const Rox_Calibration_Projector_CheckerBoard obj
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if(!intrinsics || !obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_matut3_copy(intrinsics, obj->intrinsics);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_projector_checkerboard_get_pose (
   Rox_MatSE3 pose,
   const Rox_Calibration_Projector_CheckerBoard calibration,
   const Rox_Sint index
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_ObjSet_Array2D_Double poses;

   if (!pose || !calibration)
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   poses = calibration->calib->poses;
   if (index >= (Rox_Sint) poses->used)
   {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error );}

   error = rox_matse3_copy(pose, calibration->calib->poses->data[index]);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
