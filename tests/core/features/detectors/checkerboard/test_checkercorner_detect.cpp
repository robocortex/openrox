//==============================================================================
//
//    OPENROX   : File test_checkercorner_detect.cpp
//
//    Contents  : Tests for checkercorner_detect.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== INCLUDED HEADERS   =======================================================

#include <openrox_tests.hpp>

extern "C"
{
	#include <core/features/detectors/checkerboard/checkercorner_detect.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(checkercorner_detect)

#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/calibration/checkerboard/calibration_grid.pgm"
//#define IMAGE_PATH "/home/emalis/calibration_grid_553x768.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_checkercorner_detector_new_del)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_CheckerCorner_Detector checkercorner_detector = NULL;
   Rox_Sint width  = 0;
   Rox_Sint height = 0;
   Rox_Uint kernel_blur_levels = 0;
   Rox_Uint score_blur_levels = 0;

   error = rox_checkercorner_detector_new ( NULL, width, height, kernel_blur_levels, score_blur_levels );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_checkercorner_detector_new ( &checkercorner_detector, width, height, kernel_blur_levels, score_blur_levels );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_checkercorner_detector_process)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_CheckerCorner_Detector checkercorner_detector = NULL;
   Rox_Sint cols  = 0;
   Rox_Sint rows = 0;
   Rox_Uint kernel_blur_levels = 1;
   Rox_Uint score_blur_levels = 3;
   Rox_Image image = NULL;

   error = rox_image_new_read_pgm( &image, IMAGE_PATH );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_size ( &rows, &cols, image );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_checkercorner_detector_new ( &checkercorner_detector, cols, rows, kernel_blur_levels, score_blur_levels );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_checkercorner_detector_process ( checkercorner_detector, image );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_checkercorner_detector_del( &checkercorner_detector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del ( &image );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_checkercorner_detector_build_kernels)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
