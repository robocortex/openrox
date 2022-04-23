//==============================================================================
//
//    OPENROX   : File test_odometry_plane.cpp
//
//    Contents  : Tests for odometry_plane.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== INCLUDED HEADERS   =====================================================

#include <openrox_tests.hpp>

extern "C"
{
   #include <baseproc/image/image.h>
   #include <baseproc/image/imask/imask.h>   
   #include <baseproc/array/conversion/array2d_float_from_uchar.h>
   #include <baseproc/array/fill/fillval.h>
   #include <baseproc/geometry/transforms/transform_tools.h>
	#include <core/odometry/plane/odometry_plane.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN(odometry_plane)

// A (0.2 x 0.2) m square seen at T = [ 1,0,0,0; 0,1,0,0; 0,0,1,5; 0,0,0,1 ]
// m = rox_geometry_rectangle_model(0.2,0.2);
// K = [2560, 0, 255.5; 0, 2560, 255.5; 0, 0, 1];
// T = eye(4);T(3,4)=1;

// A (1.0 x 1.0) m square seen at T = [ 1,0,0,0; 0,1,0,0; 0,0,1,1; 0,0,0,1 ]
// m = rox_geometry_rectangle_model(0.1,0.1);
// K = [512, 0, 255.5; 0, 512, 255.5; 0, 0, 1];
// T = eye(4);T(3,4)=1;
// [p, z] = rox_camera_perspective_projection(K, T, m);

//#define IMG_REF_M2D_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D000.pgm"
//#define IMG_CUR_M2D_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D001.pgm"

#define IMG_REF_M2D_PATH ROX_DATA_HOME"/regression_tests/openrox/plane_trans/image_plane3D_trans_000.pgm"
#define IMG_CUR_M2D_PATH ROX_DATA_HOME"/regression_tests/openrox/plane_trans/image_plane3D_trans_001.pgm"

#define FU_M2D 512.0
#define FV_M2D 512.0
#define CU_M2D 255.5
#define CV_M2D 255.5

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================


ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_odometry_plane_new_del )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_plane_set_get_pose)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_odometry_plane_make )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double tra[3] = {0.0, 0.0, 1.0};

   Rox_Sint max_iters = 10;
   Rox_Odometry_Plane odometry_plane = NULL;

   Rox_MatSE3 cTo = NULL;
   error = rox_matse3_new ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_set_translation ( cTo, tra );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Image Ir_uchar = NULL;
   error = rox_image_new_read_pgm ( &Ir_uchar, IMG_REF_M2D_PATH );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Image Ic_uchar = NULL;
   error = rox_image_new_read_pgm ( &Ic_uchar, IMG_REF_M2D_PATH );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Sint width = 0, height = 0;
   error = rox_image_get_size ( &height, &width, Ir_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Imask Im = NULL;
   error = rox_imask_new ( &Im, width, height );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_set_ones ( Im );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float depth = NULL;
   error = rox_array2d_float_new ( &depth, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_fillval ( depth, 1.0 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Ir = NULL; 
   error = rox_array2d_float_new ( &Ir, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_from_uchar ( Ir, Ir_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Ic = NULL; 
   error = rox_array2d_float_new ( &Ic, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_from_uchar ( Ic, Ic_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_MatUT3 Kc = NULL;
   error = rox_matut3_new ( &Kc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_calibration_matrix ( Kc, FU_M2D, FV_M2D, CU_M2D, CV_M2D );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_MatUT3 Kr = NULL;
   error = rox_matut3_new ( &Kr );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_calibration_matrix ( Kr, FU_M2D, FV_M2D, CU_M2D, CV_M2D );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_PatchPlane patch_plane = NULL;
   error = rox_patchplane_new ( &patch_plane, height, width);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_patchplane_set_reference ( patch_plane, Ir, Im );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_plane_new ( &odometry_plane );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_plane_set_pose ( odometry_plane, cTo, Kc, Kr );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_plane_make ( odometry_plane, patch_plane, Ic, max_iters);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_plane_get_pose ( cTo, odometry_plane );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matse3_print(cTo);

   error = rox_odometry_plane_del ( &odometry_plane );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error =  rox_patchplane_del ( &patch_plane );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}

ROX_TEST_SUITE_END()
