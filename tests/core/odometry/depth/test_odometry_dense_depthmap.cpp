//==============================================================================
//
//    OPENROX   : File test_odometry_dense_depthmap.cpp
//
//    Contents  : Tests for odometry_dense_depthmap.c
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
   #include <float.h>
   #include <baseproc/image/image.h>
   #include <baseproc/image/imask/imask.h>
   #include <baseproc/geometry/transforms/transform_tools.h>
   #include <baseproc/array/conversion/array2d_float_from_uchar.h>
   #include <baseproc/array/fill/fillzero.h>
   #include <baseproc/array/fill/fillval.h>
   #include <baseproc/array/error/l2_error.h>

	#include <core/odometry/depth/odometry_dense_depthmap.h>

   #include <inout/numeric/array2d_save.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN ( odometry_dense_depthmap )

// #define DEPTH

// ---
#define IMG_M3D_PATH ROX_DATA_HOME"/regression_tests/openrox/odometry_dense_depthmap/switch_lumia_seq1_img%04d.pgm"
#define Zm_PATH      ROX_DATA_HOME"/regression_tests/openrox/odometry_dense_depthmap/rendering/Zm.pgm"
#define Z_PATH       ROX_DATA_HOME"/regression_tests/openrox/odometry_dense_depthmap/rendering/Z.txt"
#define Zi_PATH      ROX_DATA_HOME"/regression_tests/openrox/odometry_dense_depthmap/rendering/Zi.txt"

#define FU_M3D 1415
#define FV_M3D 1431
#define CU_M3D 954
#define CV_M3D 559

// ---

// A (0.2 x 0.2) m square seen at T = [ 1,0,0,0; 0,1,0,0; 0,0,1,5; 0,0,0,1 ]
// m = rox_geometry_rectangle_model(0.2, 0.2);
// K = [2560, 0, 255.5; 0, 2560, 255.5; 0, 0, 1];
// T = eye(4);T(3,4)=1;
// [p, z] = rox_camera_perspective_projection(K, T, m);
//#define IMG_REF_M2D_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D000.pgm"
//#define IMG_CUR_M2D_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D001.pgm"

#define IMG_REF_M2D_PATH ROX_DATA_HOME"/regression_tests/openrox/plane_trans/image_plane3D_trans_000.pgm"
#define IMG_CUR_M2D_PATH ROX_DATA_HOME"/regression_tests/openrox/plane_trans/image_plane3D_trans_003.pgm"

#define FU_M2D 2560.0
#define FV_M2D 2560.0
#define CU_M2D 255.5
#define CV_M2D 255.5

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_dense_depthmap_new_del)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint width = 1920, height = 1080;
   Rox_Odometry_Dense_DepthMap odometry_dense_depthmap = NULL;

   error = rox_odometry_dense_depthmap_new ( &odometry_dense_depthmap, width, height );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_dense_depthmap_del ( &odometry_dense_depthmap );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_dense_depthmap_set_calibration)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE("This test has not been implemented yet !!! \n");

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_dense_depthmap_get_pose)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE("This test has not been implemented yet !!! \n");

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_dense_depthmap_set_reference)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE("This test has not been implemented yet !!! \n");

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_dense_depthmap_prepare)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE("This test has not been implemented yet !!! \n");

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_odometry_dense_depthmap_track_model2d )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double err_tra = DBL_MAX;
   Rox_Double err_rot = DBL_MAX;
   Rox_Double cTr_grt_data[16] = {1.0, 0.0, 0.0, -0.6/128.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};

   Rox_MatSE3 cTr_grt = NULL;
   error = rox_matse3_new ( &cTr_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_set_data ( cTr_grt, cTr_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_MatSE3 cTr_est = NULL;
   error = rox_matse3_new ( &cTr_est );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_MatSE3 cTr = NULL;
   error = rox_matse3_new ( &cTr );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double tra[3] = {0.0, 0.0, 0.0};

   error = rox_matse3_set_translation ( cTr, tra );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_MatUT3 K = NULL;
   error = rox_matut3_new ( &K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_calibration_matrix ( K, FU_M2D, FV_M2D, CU_M2D, CV_M2D );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Image Ir_uchar = NULL;
   error = rox_image_new_read_pgm ( &Ir_uchar, IMG_REF_M2D_PATH );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Image Ic_uchar = NULL;
   error = rox_image_new_read_pgm ( &Ic_uchar, IMG_CUR_M2D_PATH );
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

   error = rox_array2d_float_fillval ( depth, 1.0f );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Zir = NULL;
   error = rox_array2d_float_new ( &Zir, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_fillval ( Zir, 1.0f );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Ziur = NULL;
   error = rox_array2d_float_new ( &Ziur, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_fillval ( Ziur, 0.0f );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Zivr = NULL;
   error = rox_array2d_float_new ( &Zivr, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_fillval ( Zivr, 0.0f );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

#ifdef IMAGE_FLOAT
   Rox_Array2D_Float Ir_float = NULL;
   error = rox_array2d_float_new ( &Ir_float, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_from_uchar ( Ir_float, Ir_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Ic_float = NULL;
   error = rox_array2d_float_new ( &Ic_float, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_from_uchar ( Ic_float, Ic_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("Using a float image \n");
   Rox_Array2D_Float Ic = Ic_float;
   Rox_Array2D_Float Ir = Ir_float;
#else
   ROX_TEST_MESSAGE("Using a uchar image \n");
   Rox_Image Ic = Ic_uchar;
   Rox_Image Ir = Ir_uchar;
#endif

   Rox_Odometry_Dense_DepthMap odometry_dense_depthmap = NULL;

   error = rox_odometry_dense_depthmap_new ( &odometry_dense_depthmap, width, height );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_dense_depthmap_set_calibration ( odometry_dense_depthmap, K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_dense_depthmap_set_pose ( odometry_dense_depthmap, cTr );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

#ifdef DEPTH
   error = rox_odometry_dense_depthmap_set_reference_depth ( odometry_dense_depthmap, Ir, depth, Im );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
#else
   error = rox_odometry_dense_depthmap_set_reference ( odometry_dense_depthmap, Ir, Zir, Ziur, Zivr, Im );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
#endif

   //=======================================================================================

   error = rox_odometry_dense_depthmap_make ( odometry_dense_depthmap, Ic );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Sint success = 0;
   Rox_Double score = 0.0;
   error = rox_odometry_dense_depthmap_get_results ( &success, &score, cTr_est, odometry_dense_depthmap );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("score = " + std::to_string(score));
   rox_matse3_print(cTr_est);

   //=======================================================================================

   error = rox_matse3_distance ( &err_tra, &err_rot, cTr_grt, cTr_est );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("err_tra = " + std::to_string(err_tra));
   ROX_TEST_MESSAGE("err_rot = " + std::to_string(err_rot));

   ROX_TEST_CHECK_CLOSE (err_tra, 0.0, 1e-4); // Test if error is less than a micro meter
   ROX_TEST_CHECK_CLOSE (err_rot, 0.0, 1e-4); // Test if error is less than a micro radians

   //=======================================================================================

   error = rox_matse3_del ( &cTr_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_dense_depthmap_del ( &odometry_dense_depthmap );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del ( &Ir_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del ( &Ic_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

#ifdef IMAGE_FLOAT
   error = rox_array2d_float_del ( &Ir_float );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Ic_float );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
#endif

}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_dense_depthmap_track_model3d )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_MatSE3 cTr_est = NULL;
   error = rox_matse3_new ( &cTr_est );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_MatSE3 cTr = NULL;
   error = rox_matse3_new ( &cTr );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Rox_Double tra[3] = {0.05, 0.0, 0.0};
   Rox_Double tra[3] = {0.00, 0.0, 0.0};

   error = rox_matse3_set_translation ( cTr, tra );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_MatUT3 K = NULL;
   error = rox_matut3_new ( &K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_calibration_matrix ( K, FU_M3D, FV_M3D, CU_M3D, CV_M3D );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Imask imask = NULL;
   error = rox_imask_new_read_pgm ( &imask, Zm_PATH );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, IMG_M3D_PATH, 0);
   ROX_TEST_MESSAGE("Reading image : " + std::string(filename));

   Rox_Image Ir_uchar = NULL;
   error = rox_image_new_read_pgm ( &Ir_uchar, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Sint width = 0, height = 0;
   error = rox_image_get_size ( &height, &width, Ir_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Image Ic_uchar = NULL;
   error = rox_image_new ( &Ic_uchar, width, height );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

#ifdef IMAGE_FLOAT
   Rox_Array2D_Float Ir_float = NULL;
   error = rox_array2d_float_new ( &Ir_float, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_from_uchar ( Ir_float, Ir_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Ic_float = NULL;
   error = rox_array2d_float_new ( &Ic_float, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Ir = Ir_float;
   Rox_Array2D_Float Ic = Ic_float;
#else
   Rox_Image Ir = Ir_uchar;
   Rox_Image Ic = Ic_uchar;
#endif

   Rox_Array2D_Float depth = NULL;
   error = rox_array2d_float_new ( &depth, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_read ( depth, Z_PATH );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Zir = NULL;
   error = rox_array2d_float_new ( &Zir, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_read ( Zir, Zi_PATH );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Ziur = NULL;
   error = rox_array2d_float_new ( &Ziur, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_fillval ( Ziur, 0.0f );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Zivr = NULL;
   error = rox_array2d_float_new ( &Zivr, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_fillval ( Zivr, 0.0f );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Odometry_Dense_DepthMap odometry_dense_depthmap = NULL;

   error = rox_odometry_dense_depthmap_new ( &odometry_dense_depthmap, width, height );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_dense_depthmap_set_calibration ( odometry_dense_depthmap, K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_dense_depthmap_set_pose ( odometry_dense_depthmap, cTr );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for ( Rox_Sint i = 0; i < 10; i++ )
   {
      // Set the reference
#ifdef DEPTH
      error = rox_odometry_dense_depthmap_set_reference_depth ( odometry_dense_depthmap, Ir, depth, imask );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
#else
      error = rox_odometry_dense_depthmap_set_reference ( odometry_dense_depthmap, Ir, Zir, Ziur, Zivr, imask );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
#endif

      // Read another current image
      sprintf(filename, IMG_M3D_PATH, i);
      ROX_TEST_MESSAGE("Reading image : " + std::string(filename));

      error = rox_image_read_pgm ( Ic_uchar, filename );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

#ifdef IMAGE_FLOAT
      error = rox_array2d_float_from_uchar ( Ic_float, Ic_uchar );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
#endif

      // Odometry
      error = rox_odometry_dense_depthmap_make ( odometry_dense_depthmap, Ic );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Get the estimated pose
      Rox_Sint success = 0;
      Rox_Double score = 0.0;
      error = rox_odometry_dense_depthmap_get_results ( &success, &score, cTr_est, odometry_dense_depthmap );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      ROX_TEST_MESSAGE("score = " + std::to_string(score));
      rox_matse3_print(cTr_est); 
   }

   //=======================================================================================

   error = rox_odometry_dense_depthmap_del ( &odometry_dense_depthmap );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del ( &Ir_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del ( &Ic_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

#ifdef IMAGE_FLOAT
   error = rox_array2d_float_del ( &Ir_float );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Ic_float );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
#endif
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_dense_depthmap_track_both)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE("This test has not been implemented yet !!! \n");

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
