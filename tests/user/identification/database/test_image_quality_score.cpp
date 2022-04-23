//==============================================================================
//
//    OPENROX   : File test_image.cpp
//
//    Contents  : Tests for image.c
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
   #include <system/time/timer.h>
   #include <inout/numeric/array2d_print.h>
   #include <inout/system/print.h>
   #include <user/identification/database/image_score.h>
   #include <baseproc/image/image.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(image)

// Image  512 x  512 =  262144 pixels
// #define IMAGE_PATH ROX_DATA_HOME"plane/image_plane3D000.pgm"
// Image 1280 x  720 =  921600 pixels
//#define IMAGE_PATH ROX_DATA_HOME"model_based/switch/shield/sequence_1/ppm/switch_shield_seq1_img0001.ppm"
//#define IMAGE_PATH ROX_DATA_HOME"model_based/switch/shield/sequence_1/pgm/switch_shield_seq1_img0001.pgm"
// Image 1920 x 1080 = 2073600 pixels
// #define IMAGE_PATH ROX_DATA_HOME"model_based/switch/lumia/sequence_1/ppm/switch_lumia_seq1_img0001.ppm"
// Image 4096 x 2160 = 8847360 pixels
// #define IMAGE_PATH ROX_DATA_HOME"projects/apra/deformation/img005.ppm"
// Image  517 x  719 =  371723 pixels
//#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/gormiti_517x719.pgm"

#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/toy_512x512.pgm"
// #define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/debug/areva_quality_score_1_900x506.pgm"

//#define IMAGE_PATH "/home/emalis/Data/database/test_database_black_1920x1080.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

// const type * pointer : is a pointer to a constant (i.e. we cannot do *pointer = NULL)

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_image_quality_score)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Image image = NULL;
   Rox_Char filename[FILENAME_MAX];

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);

   error = rox_image_new_read_pgm(&image, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double score = 0;

   error = rox_image_get_quality_score(&score, image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("quality score = %f \n", score);

   error = rox_image_del(&image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_image_quality_score_precise)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Image image = NULL;
   Rox_Char filename[FILENAME_MAX];

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);

   error = rox_image_new_read_pgm(&image, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double score = 0;

   error = rox_image_get_quality_score_precise(&score, image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("quality score precise = %f \n", score);

   error = rox_image_del(&image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
