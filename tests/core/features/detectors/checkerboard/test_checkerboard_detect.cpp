//==============================================================================
//
//    OPENROX   : File test_checkerboard_detect.cpp
//
//    Contents  : Tests for checkerboard_detect.c
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
	#include <core/features/detectors/checkerboard/checkerboard_detect.h>
   #include <core/features/detectors/checkerboard/checkercorner_detect.h>   
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(checkerboard_detect)

#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/calibration/checkerboard/calibration_grid.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_unit_checkerboard_detector)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_CheckerBoard_Detector checkerboard_detector = NULL;
   Rox_CheckerCorner_Detector checkercorner_detector = NULL;
   
   // Test NULL pointer
   error = rox_checkerboard_detector_new(NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Test NULL pointer
   error = rox_checkerboard_detector_process ( NULL, checkercorner_detector);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
   
   // Test NULL pointer
   error = rox_checkerboard_detector_process ( checkerboard_detector, checkercorner_detector);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Test NULL pointer
   error = rox_checkerboard_detector_del(NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Test GOOD pointer
   error = rox_checkerboard_detector_new(&checkerboard_detector);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test GOOD pointer
   error = rox_checkerboard_detector_del(&checkerboard_detector);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_checkerboard_detector)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint cols  = 0;
   Rox_Sint rows = 0;
   Rox_Uint kernel_blur_levels = 1;
   Rox_Uint score_blur_levels = 3;

   Rox_CheckerBoard_Detector checkerboard_detector = NULL;
   Rox_CheckerCorner_Detector checkercorner_detector = NULL;

   Rox_Image image = NULL;

   error = rox_image_new_read_pgm(&image, IMAGE_PATH);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_size ( &rows, &cols, image );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_checkerboard_detector_new(&checkerboard_detector);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_checkercorner_detector_new ( &checkercorner_detector, cols, rows, kernel_blur_levels, score_blur_levels );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Detection
   error = rox_checkercorner_detector_process ( checkercorner_detector, image );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test GOOD pointer
   error = rox_checkerboard_detector_process ( checkerboard_detector, checkercorner_detector );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   // Test GOOD pointer
   error = rox_checkerboard_detector_del(&checkerboard_detector);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del ( &image );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
