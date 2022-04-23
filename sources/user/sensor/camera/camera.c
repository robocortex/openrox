//==============================================================================
//
//    OPENROX   : File camera.c
//
//    Contents  : Implementation of camera module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "camera.h"
#include "camera_struct.h"

#include <system/memory/memory.h>

#include <baseproc/array/conversion/array2d_uchar_from_float.h>
#include <baseproc/array/conversion/array2d_float_from_uchar.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/pixelgrid/warp_grid_distortion.h>
#include <baseproc/image/remap/remap_bilinear_float_to_float/remap_bilinear_float_to_float.h>
#include <baseproc/image/imask/imask.h>
#include <baseproc/geometry/calibration/optimalcalib.h>

#include <inout/image/pgm/pgmfile.h>
#include <inout/system/errors_print.h>


Rox_ErrorCode rox_camera_new ( Rox_Camera * camera, const Rox_Sint cols, const Rox_Sint rows)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Camera ret = NULL;
   // default intrinsics parameters
   Rox_Double fu = 1000.0, fv = 1000.0;
   Rox_Double cu = (Rox_Double) (cols-1) / 2.0, cv = (Rox_Double) (rows-1) / 2.0;

   if ( !camera )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *camera = NULL;

   if ( cols < 1 || rows < 1 )
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret = (Rox_Camera) rox_memory_allocate ( sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->image        = NULL;
   ret->calib_camera = NULL;
   ret->pose         = NULL;
   ret->grid_undistort = NULL;

   error = rox_image_new(&ret->image, cols, rows);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_new ( &ret->calib_camera );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_new ( &ret->pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_new ( &ret->grid_undistort, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_transformtools_build_calibration_matrix ( ret->calib_camera, fu, fv, cu, cv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   *camera = ret;

function_terminate:
   if(error) rox_camera_del(&ret);
   return error;
}

Rox_ErrorCode rox_camera_del ( Rox_Camera * camera )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Camera todel = NULL;

   if (!camera)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *camera;
   *camera = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_image_del(&todel->image);
   rox_matut3_del(&todel->calib_camera);
   rox_matse3_del(&todel->pose);
   rox_meshgrid2d_float_del(&todel->grid_undistort);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_camera_new_read_pgm(Rox_Camera * camera, const char * filename)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Camera ret = NULL;

   if (!camera || !filename)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *camera = NULL;

   ret = (Rox_Camera) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->image = NULL;
   ret->calib_camera = NULL;
   ret->pose = NULL;
   ret->grid_undistort = NULL;

   error = rox_image_new_read_pgm(&ret->image, filename);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_image_get_size(&rows, &cols, ret->image);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_new ( &ret->calib_camera );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matsl3_new ( &ret->pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_new(&ret->grid_undistort, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // default intrinsics parameters
   Rox_Double fu = 1000.0, fv = 1000.0, cu = (Rox_Double) (cols-1) / 2.0, cv = (Rox_Double) (rows-1) / 2.0;

   error = rox_transformtools_build_calibration_matrix(ret->calib_camera, fu, fv, cu, cv);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *camera = ret;

function_terminate:
   if (error) rox_camera_del(&ret);

   return error;
}

Rox_ErrorCode rox_camera_read_pgm(Rox_Camera camera, const char * filename)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!camera || !filename)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_image_read_pgm(camera->image, filename);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;

}

Rox_ErrorCode rox_camera_save_pgm ( const char * filename, const Rox_Camera camera )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!camera || !filename)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_image_save_pgm(filename, camera->image);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_camera_set_pinhole_params ( Rox_Camera camera, const Rox_Double fu, const Rox_Double fv, const Rox_Double cu, const Rox_Double cv )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!camera)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_transformtools_build_calibration_matrix(camera->calib_camera, fu, fv, cu, cv);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_camera_set_intrinsic_parameters ( Rox_Camera camera, const Rox_MatUT3 Kc )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !camera )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if ( !Kc )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matut3_copy ( camera->calib_camera, Kc );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_camera_set_extrinsic_parameters ( Rox_Camera camera, const Rox_MatSE3 cTo )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !camera )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if ( !cTo )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_copy ( camera->pose, cTo );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_camera_set_image ( Rox_Camera camera, const Rox_Image image )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !camera || !image )
   { error =  ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_image_copy ( camera->image, image );
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   return error;
}


Rox_ErrorCode rox_camera_set_grays_image_buffer(Rox_Camera camera, const Rox_Uchar * grays_image_buffer)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!camera || !grays_image_buffer)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_set_buffer_no_stride(camera->image, grays_image_buffer);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_camera_set_image_data(Rox_Camera camera, const Rox_Uchar * data, const Rox_Sint bytesPerRow, const enum Rox_Image_Format format)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!camera || !data)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_image_set_data(camera->image, data, bytesPerRow, format);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_camera_get_image_data(Rox_Uchar * data, Rox_Camera camera)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!camera || !data)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_image_get_data(data, camera->image);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_camera_get_image_rowsptr(Rox_Uchar *** rowsptr, const Rox_Camera camera)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!rowsptr) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_get_data_pointer_to_pointer(rowsptr, camera->image);

function_terminate:
   return error;
}

Rox_ErrorCode rox_camera_set_params_undistort (
   Rox_Camera camera, 
   const Rox_MatUT3 K, 
   const Rox_Array2D_Double radial, 
   const Rox_Array2D_Double tangential
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double dist = NULL;
   Rox_MatUT3 calib = NULL, caliboptim = NULL;

   if (!camera || !K || !radial || !tangential)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size ( tangential, 2, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( radial, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_check_size ( K );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_del ( &camera->grid_undistort );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_image_get_size ( &rows, &cols, camera->image );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dok = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dok, camera->calib_camera );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dK  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dK, K );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dT  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dT, tangential );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dR  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dR, radial );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&dist, 5, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_matut3_new ( &calib );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_matut3_new ( &caliboptim );
   ROX_ERROR_CHECK_TERMINATE(error)

   Rox_Double ** ddist  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &ddist, dist );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dcalib = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dcalib, calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dcaliboptim = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dcaliboptim, caliboptim );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ddist[0][0] = dR[0][0];
   ddist[1][0] = dR[1][0];

   ddist[2][0] = dT[0][0];
   ddist[3][0] = dT[1][0];
   ddist[4][0] = dR[2][0];

   // Copy K into calib
   dcalib[0][0] = dK[0][0];
   dcalib[0][1] = dK[0][1];
   dcalib[0][2] = dK[0][2];
   dcalib[1][0] = dK[1][0];
   dcalib[1][1] = dK[1][1];
   dcalib[1][2] = dK[1][2];
   dcalib[2][0] = dK[2][0];
   dcalib[2][1] = dK[2][1];
   dcalib[2][2] = dK[2][2];

   // Compute the optimal view for the undistorted image
   error = rox_calibration_optimalview ( caliboptim, calib, dist, cols, rows );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_new ( &camera->grid_undistort, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute the distortion map
   error = rox_warp_grid_distortion_float ( camera->grid_undistort, caliboptim, calib, dist);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Copy caliboptim into camera->calib_camera
   dok[0][0] = dcaliboptim[0][0];
   dok[0][1] = dcaliboptim[0][1];
   dok[0][2] = dcaliboptim[0][2];
   dok[1][0] = dcaliboptim[1][0];
   dok[1][1] = dcaliboptim[1][1];
   dok[1][2] = dcaliboptim[1][2];
   dok[2][0] = dcaliboptim[2][0];
   dok[2][1] = dcaliboptim[2][1];
   dok[2][2] = dcaliboptim[2][2];

function_terminate:
   rox_array2d_double_del(&dist);
   rox_matut3_del(&calib);
   rox_matut3_del(&caliboptim);
   return error;
}


Rox_ErrorCode rox_camera_undistort_image(Rox_Camera camera)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float source = NULL;
   Rox_Array2D_Float undist = NULL;
   Rox_Imask maski = NULL, omasko = NULL, omaski = NULL;

   if (!camera)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_image_get_size(&rows, &cols, camera->image);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_match_size((Rox_Array2D) camera->image, (Rox_Array2D) camera->grid_undistort);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&source, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&undist, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_imask_new(&maski, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_imask_new(&omaski, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_imask_new(&omasko, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_fillval(maski, ~0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_fillval(omaski, ~0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // uchar to float conversion
   error = rox_array2d_float_from_uchar(source, camera->image);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // warp source image using distortion map in camera->grid_undistort
   error = rox_remap_bilinear_float_to_float(undist, omasko, omaski, source, maski, camera->grid_undistort);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Convert the undistorted image into the camera image
   error = rox_array2d_uchar_from_float(camera->image, undist);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_float_del(&source);
   rox_array2d_float_del(&undist);
   rox_imask_del(&omaski);
   rox_imask_del(&omasko);
   rox_imask_del(&maski);
   return error;
}


Rox_ErrorCode rox_camera_get_intrinsic_parameters(Rox_MatUT3 calib, const Rox_Camera camera)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!camera || !calib)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matut3_copy(calib, camera->calib_camera);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_camera_get_intrinsic_parameters_pointer ( Rox_MatUT3 * calib, const Rox_Camera camera)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !camera || !calib )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *calib = camera->calib_camera;

function_terminate:
   return error;
}


Rox_ErrorCode rox_camera_get_rows(Rox_Sint * rows, const Rox_Camera camera)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!camera || !rows)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_image_get_rows(rows, camera->image);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_camera_get_cols(Rox_Sint *cols, const Rox_Camera camera)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!camera || !cols)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_image_get_cols(cols, camera->image);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_camera_get_size(Rox_Sint * rows, Rox_Sint * cols, const Rox_Camera camera)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!camera || !cols || !rows)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_image_get_cols(cols, camera->image);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_image_get_rows(rows, camera->image);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
