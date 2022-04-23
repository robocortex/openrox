//==============================================================================
//
//    OPENROX   : File test_odometry_ellipses.cpp
//
//    Contents  : Tests for odometry_ellipses.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== INCLUDED HEADERS   =====================================================

#include <stdio.h>

#include <openrox_tests.hpp>

extern "C"
{
	#include <user/odometry/edge/odometry_ellipses.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN(odometry_ellipses)

#define MODEL_PATH ROX_DATA_HOME"/devapps/model_based/test_cadmodel/test_segments_ellipses.cao"
#define IMAGE_PATH ROX_DATA_HOME"/devapps/model_based/test_cadmodel/test_segments_ellipses.ppm"

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_ellipses_new_del)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Ellipses odometry_ellipses = NULL;
   const Rox_Double fu = 1000.0;
   const Rox_Double fv = 1000.0;
   const Rox_Double cu =  960.0;
   const Rox_Double cv =  540.0;

   const Rox_Sint search_range = 20;
   const Rox_Double contrast_threshold = 51;

	error = rox_odometry_ellipses_new(&odometry_ellipses, fu, fv, cu, cv, search_range, contrast_threshold);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_ellipses_del(&odometry_ellipses);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_ellipses_add_ellipse)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   const Rox_Double fu = 1000.0;
   const Rox_Double fv = 1000.0;
   const Rox_Double cu =  960.0;
   const Rox_Double cv =  540.0;

   const Rox_Sint search_range = 20;
   const Rox_Double contrast_threshold = 51;
   const Rox_Double sampling_step = 5;
   Rox_Ellipse3D ellipse3d = NULL;
   Rox_Odometry_Ellipses odometry_ellipses = NULL;
   
   error = rox_ellipse3d_new(&ellipse3d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_ellipses_new(&odometry_ellipses, fu, fv, cu, cv, search_range, contrast_threshold);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_ellipses_add_ellipse(odometry_ellipses, ellipse3d, sampling_step);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_ellipses_del(&odometry_ellipses);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ellipse3d_del(&ellipse3d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_ellipses_make)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   const Rox_Double fu = 1000.0;
   const Rox_Double fv = 1000.0;
   const Rox_Double cu =  960.0;
   const Rox_Double cv =  540.0;

   const Rox_Sint search_range = 20;
   const Rox_Double contrast_threshold = 51;
   const Rox_Double sampling_step = 5;
   const Rox_Uint max_iters = 10;
   Rox_Image image = NULL;
   Rox_Ellipse3D ellipse3d = NULL;
   Rox_Odometry_Ellipses odometry_ellipses = NULL;
   
   error = rox_ellipse3d_new(&ellipse3d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_ellipses_new(&odometry_ellipses, fu, fv, cu, cv, search_range, contrast_threshold);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_ellipses_add_ellipse(odometry_ellipses, ellipse3d, sampling_step);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_ellipses_make(odometry_ellipses, image, max_iters);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_ellipses_del(&odometry_ellipses);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_ellipse3d_del(&ellipse3d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_ellipses_set_pose)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_ellipses_get_pose)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
