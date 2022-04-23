//==============================================================================
//
//    OPENROX   : File test_array2d.cpp
//
//  	Contents  : Tests for array2d.c
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
   #include <system/memory/array2d.h>
   #include <generated/array2d_point2d_double.h>
   #include <generated/array2d_point2d_float.h>
   #include <generated/array2d_float.h>
   #include <generated/array2d_uchar.h>
   #include <baseproc/array/conversion/array2d_float_from_uchar.h>

   #include <inout/image/pgm/pgmfile.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(array2d)

//#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/debug/america_1920x1080.pgm"
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D000.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_new)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint size_normal = 10, size_badsize = 0;
   Rox_Array2D array_badtype = NULL, array_badsize = NULL, array_normal = NULL;
   Rox_Datatype_Description dataType_normal = ROX_TYPE_UCHAR, dataType_badtype = (Rox_Datatype_Description_Enum)0;

   // NULL pointer
   error = rox_array2d_new(NULL, dataType_normal, size_normal, size_normal);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   //bad or unknown data type
   error = rox_array2d_new(&array_badtype, dataType_badtype, size_normal, size_normal);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_TYPE);

   //bad size
   error = rox_array2d_new(&array_badsize, dataType_normal, size_badsize, size_normal);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   error = rox_array2d_new(&array_badsize, dataType_normal, size_normal, size_badsize);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   error = rox_array2d_new(&array_badsize, dataType_normal, size_badsize, size_badsize);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   //normal case
   error = rox_array2d_new(&array_normal, dataType_normal, size_normal, size_normal);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   if (array_normal)  rox_array2d_del(&array_normal);
   if (array_badsize) rox_array2d_del(&array_badsize);
   if (array_badtype) rox_array2d_del(&array_badtype);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_del)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D array_null = NULL, array_normal = NULL;
   Rox_Datatype_Description dataType_normal = ROX_TYPE_UCHAR;
   Rox_Uint size_normal = 10;

   //normal case
   error = rox_array2d_new(&array_normal, dataType_normal, size_normal, size_normal);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_del(&array_normal);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //NULL pointer
   error = rox_array2d_del(&array_null);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   if (array_normal) rox_array2d_del(&array_normal);
   if (array_null) rox_array2d_del(&array_null);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_new_subarray2d)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

	   
ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_subarray2d_shift)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

	   
ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_create_rowsptr)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

	   
ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_delete_rowsptr)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

	   
ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_create_blocksptr)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );
   
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_delete_blocksptr)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );
	   
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_get_blocks)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_get_cols)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_get_rows)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_get_stride)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_get_type)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );
 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_match_size)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_match)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_copy)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float dest_float = NULL;
   Rox_Array2D_Float source_float = NULL;
   Rox_Array2D_Uchar source_uchar = NULL;
   Rox_Char filename[FILENAME_MAX];
   Rox_Sint rows = 0, cols = 0;
   Rox_Double time_ms = 0.0, total_time = 0.0;

#ifdef DEBUG
   // Small number of tests for slow debug (e.g. with valgrind)
   Rox_Sint nb_tests = 1;
#else 
   // High number of tests for measuring average performance in release
   Rox_Sint nb_tests = 1000;
#endif

   // Define timer to measure performances
   Rox_Timer timer = NULL;

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);

   // Read source image
   error = rox_array2d_uchar_new_pgm ( &source_uchar, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new_from_uchar ( &source_float, source_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_uchar_get_size ( &rows, &cols, source_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New float image
   error = rox_array2d_float_new ( &dest_float, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Sint i = 0; i < nb_tests; ++i)
   {  
      rox_timer_start(timer);

      error = rox_array2d_float_copy ( dest_float, source_float );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time_ms, timer);
      total_time += time_ms;
   }

   rox_log("mean time to copy a (%d x %d) float image = %f (ms)\n", cols, rows, total_time/nb_tests);
   
   error = rox_array2d_uchar_del ( &source_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_float_del ( &source_float );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &dest_float );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_timer_del ( &timer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_point2d_double_new_del)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Point2D_Double points = NULL;

   error = rox_array2d_point2d_double_new(&points, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_point2d_double_del(&points);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_point2d_float_new_del)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Point2D_Float points = NULL;

   error = rox_array2d_point2d_float_new(&points, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_point2d_float_del(&points);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
