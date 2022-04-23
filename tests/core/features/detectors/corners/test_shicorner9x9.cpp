//==============================================================================
//
//    OPENROX   : File test_shicorner9x9.cpp
//
//    Contents  : Tests for shicorner9x9.c
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
   #include <baseproc/image/image.h>
	#include <core/features/detectors/corners/shicorner9x9.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(shicorner9x9)

#define IMG_REF_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D000.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_shicorner9x9_test)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Float response = 0.0;
   Rox_Uchar ** image_data = NULL;

   Rox_Uint v = 287; 
   Rox_Uint u = 364;

   Rox_Image image = NULL;

   error = rox_image_new_read_pgm ( &image, IMG_REF_PATH );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_image_get_data_pointer_to_pointer ( &image_data, image );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_shicorner9x9_test ( &response, image_data, v, u );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("shicorner9x9_test response (%d, %d) = %16.16f\n", u, v, response);

   ROX_TEST_CHECK_CLOSE(response, 7.9604167938232422, 1e-16);

   v = 128;
   u = 534;

   error = rox_shicorner9x9_test ( &response, image_data, v, u );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("shicorner9x9_test response (%d, %d) = %16.16f\n", u, v, response);
   ROX_TEST_CHECK_CLOSE(response, 10.4657497406005859, 1e-16);

   v = 439;
   u = 245;

   error = rox_shicorner9x9_test ( &response, image_data, v, u );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("shicorner9x9_test response (%d, %d) = %16.16f\n", u, v, response);
   ROX_TEST_CHECK_CLOSE(response, 16.7567749023437500, 1e-16);

function_terminate:
   error = rox_image_del(&image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
