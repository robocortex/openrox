//==============================================================================
//
//    OPENROX   : File test_odometry_se2.cpp
//
//    Contents  : Tests for odometry_se2.c
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
	#include <core/odometry/affine/odometry_se2.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN(odometry_se2)

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_se2_new)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_se2_del)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_se2_make)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_se2_get_pose)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_se2_set_calib)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
