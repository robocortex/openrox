//==============================================================================
//
//    OPENROX   : File photoframe.c
//
//    Contents  : Implementation of photoframe module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "photoframe.h"
#include "photoframe_struct.h"

#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_point3d_float.h>
#include <generated/dynvec_point3d_double.h>
#include <generated/dynvec_point3d_double_struct.h>

#include <baseproc/geometry/point/dynvec_point2d_tools.h>
#include <baseproc/array/crosscor/zncrosscor.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/rotate/rotate90.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d.h>
#include <baseproc/geometry/point/point2d_tools.h>
#include <baseproc/geometry/point/point3d_tools.h>
#include <baseproc/geometry/point/dynvec_point3d_tools.h>
#include <baseproc/geometry/rectangle/rectangle.h>

#include <baseproc/image/remap/remap_bilinear_nomask_uchar_to_uchar/remap_bilinear_nomask_uchar_to_uchar.h>
#include <baseproc/maths/maths_macros.h>

#include <core/features/detectors/orientation/orimoments.h>
#include <core/indirect/euclidean/vvspointsse3.h>
#include <core/model/model_single_plane.h>
#include <core/model/model_single_plane_struct.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>
#include <inout/image/pgm/pgmfile.h>
#include <inout/numeric/array2d_print.h>
#include <inout/geometry/point/point2d_print.h>

#include <user/odometry/plane/odometry_singleplane_sparse.h>

Rox_ErrorCode rox_transformtools_build_pose_intermodel_optimal ( 
   Rox_MatSE3 pose, 
   Rox_Matrix 
   homography, 
   Rox_Matrix calibration, 
   Rox_Double rect_size_x, 
   Rox_Double rect_size_y
);

#if 1

Rox_ErrorCode rox_photoframe_new (
   Rox_PhotoFrame *photoframe, 
   Rox_Image reference, 
   Rox_Uint border_width
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_PhotoFrame ret = NULL;

   if (!photoframe || !reference)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret = (Rox_PhotoFrame) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Init internal variables
   ret->is_detected    = 0;
   ret->Hinsideframe   = NULL;
   ret->Hfinal         = NULL;
   ret->resH           = NULL;
   ret->inplaneH[0]    = NULL;
   ret->inplaneH[1]    = NULL;
   ret->inplaneH[2]    = NULL;
   ret->inplaneH[3]    = NULL;
   ret->templates[0]   = NULL;
   ret->templates[1]   = NULL;
   ret->templates[2]   = NULL;
   ret->templates[3]   = NULL;
   ret->grid            = NULL;
   ret->dest           = NULL;
   ret->calib_template = NULL;
   ret->pose           = NULL;
   ret->G              = NULL;
   ret->score          = 0.0f;
   ret->has_mask       = 0;
   ret->masks[0]       = NULL;
   ret->masks[1]       = NULL;
   ret->masks[2]       = NULL;
   ret->masks[3]       = NULL;
   ret->score_threshold = 0.9;

   // Reference must be square shaped
   Rox_Sint width  = 0, height = 0;
   error = rox_array2d_uchar_get_size(&height, &width, reference);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (width != height)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_uchar_new(&ret->templates[0], height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_new(&ret->templates[1], height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_new(&ret->templates[2], height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_new(&ret->templates[3], height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_copy(ret->templates[0], reference);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_rotate90(ret->templates[1], ret->templates[0]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_rotate90(ret->templates[2], ret->templates[1]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_rotate90(ret->templates[3], ret->templates[2]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_compute_orientation_moments(&ret->theta[0], ret->templates[0]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_compute_orientation_moments(&ret->theta[1], ret->templates[1]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_compute_orientation_moments(&ret->theta[2], ret->templates[2]);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_uchar_compute_orientation_moments(&ret->theta[3], ret->templates[3]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_array2d_point2d_sshort_new(&ret->grid, height, width); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_meshgrid2d_float_new(&ret->grid, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_new(&ret->dest, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_new ( &ret->calib_template );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_new ( &ret->pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matsl3_new ( &ret->G );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Hinsideframe, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Hfinal, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->resH, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->inplaneH[0], 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->inplaneH[1], 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->inplaneH[2], 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->inplaneH[3], 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(ret->resH);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Input homography Hi will map a 1*1 pixel square P1 to the contour of the photoframe PR(W,H)
   // PR = Hi * P1.
   // We want to replace P1 with the reference image square PS
   // P1 =  [1.0 / W 0 1; 0 1.0 / H 0; 0 0 1] * PR
   // Next we want to crop outside the marker borders (borderwidth) using another homography Hc to create PC
   // PR = Hc * PS

   Rox_Double rwidth  = (double)(width ) / (double)(width  + border_width * 2);
   Rox_Double rheight = (double)(height) / (double)(height + border_width * 2);

   Rox_Double ** dh = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dh, ret->Hinsideframe);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // We have border_width + 0.5 since the top left corner will be at 
   // coordinates (-border_width - 0.5, -border_width - 0.5)
   dh[0][0] = rwidth/width; dh[0][1] = 0             ; dh[0][2] = rwidth  * ((double) border_width + 0.5)/(double) width ;
   dh[1][0] = 0           ; dh[1][1] = rheight/height; dh[1][2] = rheight * ((double) border_width + 0.5)/(double) height;
   dh[2][0] = 0           ; dh[2][1] = 0             ; dh[2][2] = 1;

   // In plane rotation for each possible quarter
   // Rotation rz = 0
   error  = rox_array2d_double_fillunit(ret->inplaneH[0]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Rotation rz = pi/2
   error  = rox_transformtools_imagerotation(ret->inplaneH[1], width, height,       ROX_PI_2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Rotation rz = pi
   error  = rox_transformtools_imagerotation(ret->inplaneH[2], width, height,       ROX_PI  );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Rotation rz = 3*pi/2
   error  = rox_transformtools_imagerotation(ret->inplaneH[3], width, height, 3.0 * ROX_PI_2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *photoframe = ret;

function_terminate:
   if (error) rox_photoframe_del(&ret);
   return error;
}

#endif

Rox_ErrorCode rox_photoframe_new_model (
   Rox_PhotoFrame * photoframe,
   const Rox_Model_Single_Plane model_single_plane,
   const Rox_Sint border_width
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_PhotoFrame ret = NULL;

   if (!photoframe || !model_single_plane)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret = (Rox_PhotoFrame) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Init internal variables
   ret->is_detected    = 0;
   ret->Hinsideframe   = NULL;
   ret->Hfinal         = NULL;
   ret->resH           = NULL;
   ret->inplaneH[0]    = NULL;
   ret->inplaneH[1]    = NULL;
   ret->inplaneH[2]    = NULL;
   ret->inplaneH[3]    = NULL;
   ret->templates[0]   = NULL;
   ret->templates[1]   = NULL;
   ret->templates[2]   = NULL;
   ret->templates[3]   = NULL;
   ret->grid            = NULL;
   ret->dest           = NULL;
   ret->calib_template = NULL;
   ret->pose           = NULL;
   ret->G              = NULL;
   ret->score          = 0.0f;
   ret->has_mask       = 0;
   ret->masks[0]       = NULL;
   ret->masks[1]       = NULL;
   ret->masks[2]       = NULL;
   ret->masks[3]       = NULL;
   ret->score_threshold = 0.9;

   // Reference must be square shaped
   Rox_Sint width  = 0, height = 0;
   error = rox_array2d_uchar_get_size ( &height, &width, model_single_plane->image_template );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (width != height)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_uchar_new(&ret->templates[0], height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_new(&ret->templates[1], height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_new(&ret->templates[2], height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_new(&ret->templates[3], height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_copy ( ret->templates[0], model_single_plane->image_template );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_rotate90(ret->templates[1], ret->templates[0]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_rotate90(ret->templates[2], ret->templates[1]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_rotate90(ret->templates[3], ret->templates[2]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_compute_orientation_moments(&ret->theta[0], ret->templates[0]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_compute_orientation_moments(&ret->theta[1], ret->templates[1]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_compute_orientation_moments(&ret->theta[2], ret->templates[2]);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_uchar_compute_orientation_moments(&ret->theta[3], ret->templates[3]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_new(&ret->grid, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_new(&ret->dest, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_new ( &ret->calib_template );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Should be 
   // ret->width_meters  = model_single_plane->sizex 
   // ret->height_meters = model_single_plane->sizey

   ret->width_meters = model_single_plane->vertices_ref[1].X - model_single_plane->vertices_ref[0].X;
   ret->height_meters = model_single_plane->vertices_ref[2].Y - model_single_plane->vertices_ref[1].Y;

   error = rox_transformtools_build_calibration_matrix_for_template ( ret->calib_template, width, height, ret->width_meters, ret->height_meters );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_new ( &ret->pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matsl3_new ( &ret->G );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Hinsideframe, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Hfinal, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->resH, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->inplaneH[0], 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->inplaneH[1], 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->inplaneH[2], 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->inplaneH[3], 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(ret->resH);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Input homography Hi will map a 1*1 pixel square P1 to the contour of the photoframe PR(W,H)
   // PR = Hi * P1.
   // We want to replace P1 with the reference image square PS
   // P1 =  [1.0 / W 0 1; 0 1.0 / H 0; 0 0 1] * PR
   // Next we want to crop outside the marker borders (borderwidth) using another homography Hc to create PC
   // PR = Hc * PS

   Rox_Double rwidth  = (double)(width ) / (double)(width  + border_width * 2);
   Rox_Double rheight = (double)(height) / (double)(height + border_width * 2);

   Rox_Double ** dh = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dh, ret->Hinsideframe);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // We have border_width + 0.5 since the top left corner will be at 
   // coordinates (-border_width - 0.5, -border_width - 0.5)
   dh[0][0] = rwidth/width; dh[0][1] = 0             ; dh[0][2] = rwidth  * ((double) border_width + 0.5)/(double) width ;
   dh[1][0] = 0           ; dh[1][1] = rheight/height; dh[1][2] = rheight * ((double) border_width + 0.5)/(double) height;
   dh[2][0] = 0           ; dh[2][1] = 0             ; dh[2][2] = 1;

   // In plane rotation for each possible quarter
   // Rotation rz = 0
   error  = rox_array2d_double_fillunit(ret->inplaneH[0]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Rotation rz = pi/2
   error  = rox_transformtools_imagerotation ( ret->inplaneH[1], width, height,       ROX_PI_2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Rotation rz = pi
   error  = rox_transformtools_imagerotation ( ret->inplaneH[2], width, height,       ROX_PI  );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Rotation rz = 3*pi/2
   error  = rox_transformtools_imagerotation ( ret->inplaneH[3], width, height, 3.0 * ROX_PI_2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *photoframe = ret;

function_terminate:
   if (error) rox_photoframe_del(&ret);
   return error;
}

Rox_ErrorCode rox_photoframe_del(Rox_PhotoFrame *photoframe)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_PhotoFrame todel = NULL;

   if (!photoframe) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   todel = *photoframe;
   *photoframe = NULL;

   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   // rox_array2d_point2d_sshort_del(&todel->grid);
   ROX_ERROR_CHECK(rox_meshgrid2d_float_del(&todel->grid));
   ROX_ERROR_CHECK(rox_array2d_uchar_del (&todel->dest          ));
   ROX_ERROR_CHECK(rox_array2d_double_del(&todel->Hinsideframe  ));
   ROX_ERROR_CHECK(rox_array2d_double_del(&todel->Hfinal        ));
   ROX_ERROR_CHECK(rox_array2d_double_del(&todel->resH          ));
   ROX_ERROR_CHECK(rox_array2d_double_del(&todel->inplaneH[0]   ));
   ROX_ERROR_CHECK(rox_array2d_double_del(&todel->inplaneH[1]   ));
   ROX_ERROR_CHECK(rox_array2d_double_del(&todel->inplaneH[2]   ));
   ROX_ERROR_CHECK(rox_array2d_double_del(&todel->inplaneH[3]   ));
   ROX_ERROR_CHECK(rox_array2d_uchar_del (&todel->templates[0]  ));
   ROX_ERROR_CHECK(rox_array2d_uchar_del (&todel->templates[1]  ));
   ROX_ERROR_CHECK(rox_array2d_uchar_del (&todel->templates[2]  ));
   ROX_ERROR_CHECK(rox_array2d_uchar_del (&todel->templates[3]  ));
   ROX_ERROR_CHECK(rox_array2d_double_del(&todel->calib_template));
   ROX_ERROR_CHECK(rox_array2d_double_del(&todel->pose          ));
   ROX_ERROR_CHECK(rox_array2d_double_del(&todel->G             ));

   // The mask is maybe not allocated, test if allocated before deleting
   if(todel->masks[0]) ROX_ERROR_CHECK(rox_array2d_uint_del(&todel->masks[0]));
   if(todel->masks[1]) ROX_ERROR_CHECK(rox_array2d_uint_del(&todel->masks[1]));
   if(todel->masks[2]) ROX_ERROR_CHECK(rox_array2d_uint_del(&todel->masks[2]));
   if(todel->masks[3]) ROX_ERROR_CHECK(rox_array2d_uint_del(&todel->masks[3]));

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_photoframe_set_score_threshold( Rox_PhotoFrame photoframe, Rox_Double score_threshold )
{
   Rox_ErrorCode  error = ROX_ERROR_NONE;

   if (!photoframe)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (score_threshold < 0.0 || score_threshold > 1.0)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   photoframe->score_threshold = score_threshold;

function_terminate:
   return error;
}


Rox_ErrorCode rox_photoframe_set_mask( Rox_PhotoFrame photoframe, Rox_Imask mask )
{
   Rox_ErrorCode  error = ROX_ERROR_NONE;

   if (!photoframe || !mask || !photoframe->templates[0] )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Must be the same size as the reference
   Rox_Sint width = 0, height = 0;
   error = rox_array2d_uint_get_size ( &height, &width, mask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint template_width = 0;
   error = rox_array2d_uchar_get_cols(&template_width, photoframe->templates[0]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if ( width != height || width != template_width )
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !photoframe->templates[0] ) rox_array2d_uint_del( &photoframe->masks[0] );
   if ( !photoframe->templates[1] ) rox_array2d_uint_del( &photoframe->masks[1] );
   if ( !photoframe->templates[2] ) rox_array2d_uint_del( &photoframe->masks[2] );
   if ( !photoframe->templates[3] ) rox_array2d_uint_del( &photoframe->masks[3] );

   error = rox_array2d_uint_new(&photoframe->masks[0], height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new(&photoframe->masks[1], height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new(&photoframe->masks[2], height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new(&photoframe->masks[3], height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_copy(photoframe->masks[0], mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_rotate90(photoframe->masks[1], photoframe->masks[0]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_rotate90(photoframe->masks[2], photoframe->masks[1]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_rotate90(photoframe->masks[3], photoframe->masks[2]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   photoframe->has_mask = 1;

function_terminate:
   if (error)
   {
      photoframe->has_mask = 0;
      rox_array2d_uint_del( &photoframe->masks[0] );
      rox_array2d_uint_del( &photoframe->masks[1] );
      rox_array2d_uint_del( &photoframe->masks[2] );
      rox_array2d_uint_del( &photoframe->masks[3] );
      photoframe->masks[0] = NULL;
      photoframe->masks[1] = NULL;
      photoframe->masks[2] = NULL;
      photoframe->masks[3] = NULL;
   }

   return error;
}

Rox_ErrorCode rox_photoframe_reset(Rox_PhotoFrame photoframe)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!photoframe) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   photoframe->is_detected = 0;

function_terminate:

   return error;
}

Rox_ErrorCode rox_photoframe_make_with_fast_orientation_test (
   Rox_PhotoFrame photoframe, 
   Rox_Image image, 
   Rox_Matrix H
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint idclosestangle = 0;
   Rox_Double score = 0.0, ltheta = 0.0, dtheta = 0.0, mind = 0.0;

   if (!photoframe || !image || !H)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // If this photoframe already detected for another quad, ...
   if (photoframe->is_detected) 
   { error = ROX_ERROR_NONE; goto function_terminate; }

   error = rox_array2d_double_check_size(H, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(photoframe->Hfinal, H, photoframe->Hinsideframe);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //  Warp input image to photoframe->dest
   error = rox_warp_grid_sl3_float(photoframe->grid, photoframe->Hfinal);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_remap_bilinear_nomask_uchar_to_uchar(photoframe->dest, image, photoframe->grid);
   ROX_ERROR_CHECK_TERMINATE ( error );

   idclosestangle = 0;
   mind = DBL_MAX;

   // Compute orientation of current warped image using moments
   error = rox_array2d_uchar_compute_orientation_moments(&ltheta, photoframe->dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //  Find most similar patch among the four possible orientations
   for (Rox_Sint idcmp = 0; idcmp < 4; idcmp++)
   {
      dtheta = ltheta - photoframe->theta[idcmp];
      dtheta = ROX_MIN((2 * ROX_PI) - fabs(dtheta), fabs(dtheta));
      if (dtheta < mind)
      {
         mind = dtheta;
         idclosestangle = idcmp;
      }
   }

   if (photoframe->has_mask)
   {
      error = rox_array2d_uchar_zncc_normalizedscore(&score, photoframe->dest, photoframe->templates[idclosestangle], photoframe->masks[idclosestangle]);
   }
   else
   {
      error = rox_array2d_uchar_zncc_nomask_normalizedscore(&score, photoframe->dest, photoframe->templates[idclosestangle]);
   }
   ROX_ERROR_CHECK_TERMINATE ( error );

   photoframe->is_detected = 0;

   // Test if score is above a threshold
   // If score is more than SCORE_THRESHOLD the texture has been identified
   if ( score > photoframe->score_threshold )
   {
      error = rox_array2d_double_mulmatmat(photoframe->resH, photoframe->Hfinal, photoframe->inplaneH[idclosestangle]);
      ROX_ERROR_CHECK_TERMINATE ( error );
      photoframe->is_detected = 1;
   }

   if ( score > photoframe->score )
   {
      photoframe->score = score;
   }

function_terminate:
   return error;
}

// static int nbwarp = 0;
Rox_ErrorCode rox_photoframe_make_with_full_orientation_test (
   Rox_PhotoFrame photoframe, 
   Rox_Image image, 
   Rox_Array2D_Double H
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint idclosestangle;
   Rox_Double score, maxscore;

   if (!photoframe || !image || !H)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // If this photoframe already detected for another quad, ...
   if (photoframe->is_detected) 
   { error = ROX_ERROR_NONE; goto function_terminate; }

   error = rox_array2d_double_check_size(H, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(photoframe->Hfinal, H, photoframe->Hinsideframe);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Warp input image to photoframe->dest
   error = rox_warp_grid_sl3_float(photoframe->grid, photoframe->Hfinal);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Do not use mask to warp
   rox_array2d_uchar_fillval(photoframe->dest, 0);
   error = rox_remap_bilinear_nomask_uchar_to_uchar(photoframe->dest, image, photoframe->grid);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Find the best orientation
   idclosestangle = 0;
   maxscore = -10.0;

   // Find most similar patch among the four possible orientations
   for (Rox_Sint idcmp = 0; idcmp < 4; idcmp++)
   {
      if ( photoframe->has_mask )
      {
         error = rox_array2d_uchar_zncc_normalizedscore( &score, photoframe->dest, photoframe->templates[idcmp], photoframe->masks[idcmp] );
      }
      else
      {
         error = rox_array2d_uchar_zncc_nomask_normalizedscore( &score, photoframe->dest, photoframe->templates[idcmp] );
      }
      ROX_ERROR_CHECK_TERMINATE ( error );

      if (score > maxscore)
      {
         maxscore = score;
         idclosestangle = idcmp;
      }
   }

   photoframe->is_detected = 0;

   // Test if SCORE is above a threshold
   // If score is more than SCORE_THRESHOLD the tecture has been identified
   if ( maxscore > photoframe->score_threshold )
   {
      error = rox_array2d_double_mulmatmat(photoframe->resH, photoframe->Hfinal, photoframe->inplaneH[idclosestangle]);
      ROX_ERROR_CHECK_TERMINATE(error)

      photoframe->is_detected = 1;
   }

   if ( maxscore > photoframe->score )
   {
      photoframe->score = maxscore;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_photoframe_make_se3 (
   Rox_PhotoFrame photoframe, 
   Rox_Image image, 
   Rox_MatSL3 H, //  
   Rox_MatUT3 calibration, 
   Rox_Uint orientation_method
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!photoframe || !image || !H || !calibration)
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   if (orientation_method > 1)
   {error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error );}

   if (orientation_method == 0)
   {
      // test all four possible orientations with score based on ZNCC
      error = rox_photoframe_make_with_full_orientation_test(photoframe, image, H);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      // Compute global orientation
      error = rox_photoframe_make_with_fast_orientation_test(photoframe, image, H);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // calib_template is the model-to-image homography that allow to project the rectangle into the basic template frame
   // 
   // G is the model-to-image homography that allow to project the rectangle into the current image
   error = rox_array2d_double_mulmatmat(photoframe->G, photoframe->resH, photoframe->calib_template);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Replace rox_transformtools_build_pose_intermodel with rox_transformtools_build_pose_intermodel_optimal
   if(1)
   {
      error = rox_transformtools_build_pose_intermodel_optimal ( photoframe->pose, photoframe->G, calibration, photoframe->width_meters, photoframe->height_meters);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      error = rox_transformtools_build_pose_intermodel ( photoframe->pose, photoframe->G, calibration );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   function_terminate:
   return error;
}

// Input is the model-to-image homography that allow to project the rectangle into the image
Rox_ErrorCode rox_transformtools_build_pose_intermodel_optimal (
   Rox_MatSE3 pose, 
   Rox_Matrix homography, 
   Rox_Matrix calibration, 
   Rox_Double rect_size_x, 
   Rox_Double rect_size_y
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double maxdist_prefilter = 100.0;

   Rox_Point2D_Double_Struct points2D[4];
   Rox_Point3D_Double_Struct points3D[4];

   Rox_DynVec_Point2D_Double points2D_dynvec = NULL;
   Rox_DynVec_Point3D_Double points3D_dynvec = NULL;
   
   Rox_MatSE3 pose_init = NULL;

   error = rox_matse3_new ( &pose_init );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_double_new ( &points2D_dynvec, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_new ( &points3D_dynvec, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Ceate 3D model of the rectangle
   error = rox_rectangle3d_create_centered_plane_xright_ydown ( points3D, rect_size_x, rect_size_y );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute the image points using model-to-image homography
   error = rox_point2d_double_intermodel_projection ( points2D, homography, points3D, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error) ;

   error = rox_dynvec_point2d_double_append_vector_point2d_double ( points2D_dynvec, points2D, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_append_vector_point3d_double ( points3D_dynvec, points3D, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Extract pose from model-to-image homography. Model is a rectangle of size (rect_size_x, rect_size_y)
   //error = rox_odometry_singleplane_sparse ( pose_init, calibration, points2D, rect_size_x, rect_size_y );
   //ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_transformtools_build_pose_intermodel ( pose_init, homography, calibration );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Copy the init pose
   error = rox_array2d_double_copy ( pose, pose_init );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Refine the pose with VVS
   error = rox_points_double_refine_pose_vvs ( pose, calibration, points2D_dynvec, points3D_dynvec, maxdist_prefilter );
   if ( error )
   {
      error = rox_array2d_double_copy ( pose, pose_init );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }


function_terminate:
   rox_dynvec_point2d_double_del ( &points2D_dynvec );
   rox_dynvec_point3d_double_del ( &points3D_dynvec );
   rox_matse3_del(&pose_init);
   return error;
}
