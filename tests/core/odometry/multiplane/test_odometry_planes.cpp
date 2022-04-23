//==============================================================================
//
//    OPENROX   : File test_odometry_planes.cpp
//
//    Contents  : Tests for odometry_planes.c
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
	#include <core/odometry/multiplane/odometry_planes.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN(odometry_planes)

// A (0.2 x 0.2) m square seen at T = [ 1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 5; 0, 0, 0, 1 ]
// m = rox_geometry_rectangle_model(0.2, 0.2);
// K = [2560, 0, 255.5; 0, 2560, 255.5; 0, 0, 1];
// T = eye(4);T(3,4)=1;
// [p, z] = rox_camera_perspective_projection(K, T, m);

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


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_planes_new_del)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_planes_make)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_planes_set_pose)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_planes_get_pose)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_planes_set_camera_calibration)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_planes_nreg)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Planes odometry_planes = NULL; 
   Rox_Model_Multi_Plane model = NULL;

   error = rox_model_multi_plane_new ( &model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_planes_new ( &odometry_planes, model );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   error = rox_odometry_planes_del ( &odometry_planes);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_model_multi_plane_del ( &model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_planes_compare_plane)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // We compare the odometry considering 4 square of size 0.1 x 0.1 and one single plane of size 0.2 x 0.2
    
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}


ROX_TEST_SUITE_END()
