//==============================================================================
//
//    OPENROX   : File test_medianfilter.cpp
//
//    Contents  : Tests for medianfilter.c
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
	#include <baseproc/image/filter/median/medianfilter.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(medianfilter)

// Image  512  x   512 =    262 144 pixels
//#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D000.pgm"
// Image  640  x   480 =    307 200 pixels
//#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_640x480.pgm"
// Image 1280  x   720 =    921 600 pixels
//#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_1280x720.pgm"
// Image 1920  x  1080 =  2 073 600 pixels
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_1920x1080.pgm"
// Image 3840  x  2160 =  8 294 400 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_3840x2160.pgm"
// Image 4096  x  2160 =  8 847 360 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_4096x2160.pgm"
// Image 5120  x  3200 = 16 384 000 pixels
//#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_5120x3200.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_uchar_filter_median)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Sint radius = 5;
   Rox_Sint rows = 0;
   Rox_Sint cols = 0;
   
   Rox_Image image_uchar_filtered = NULL;
   Rox_Image image_uchar = NULL; 

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("Read file %s\n", filename);
   
   // Create and read the uchar image
   error = rox_image_new_read_pgm ( &image_uchar, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_new ( &image_uchar_filtered, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_filter_median ( image_uchar_filtered, image_uchar, radius );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_image_del ( &image_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del ( &image_uchar_filtered );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
