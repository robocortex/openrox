//==============================================================================
//
//    OPENROX   : File test_remap_bilinear_nomask_float_to_float_doubled.cpp
//
//    Contents  : Tests for remapdouble.c
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
	#include <baseproc/image/remap/remap_bilinear_nomask_float_to_float/remap_bilinear_nomask_float_to_float_doubled.h>
   #include <baseproc/array/conversion/array2d_float_from_uchar.h>
   #include <inout/numeric/array2d_save.h>
   #include <inout/image/pgm/pgmfile.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN ( remap_bilinear_nomask_float_to_float_doubled )

#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D000.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_remap_bilinear_nomask_float_to_float_doubled )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Sint cols = 0, rows = 0;

   Rox_Array2D_Float dest = NULL;
   Rox_Array2D_Float source = NULL;

   // Declare the source image
   Rox_Array2D_Uchar image_uchar = NULL;

   sprintf(filename, "%s", IMAGE_PATH);
   ROX_TEST_MESSAGE("read file %s\n", filename);

   // Read source image
   error = rox_array2d_uchar_new_pgm ( &image_uchar, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size ( &rows, &cols, image_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // New float image
   error = rox_array2d_float_new ( &dest, 2*rows, 2*cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New float image
   error = rox_array2d_float_new ( &source, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_from_uchar_normalize(source, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );


   error = remap_bilinear_nomask_float_to_float_doubled ( dest, source );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_save("test_remap_bilinear_nomask_float_to_float_doubled.txt", dest);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE ( "This test dows not check the output yet !!! \n" );
}

ROX_TEST_SUITE_END()
