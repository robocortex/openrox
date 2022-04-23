//==============================================================================
//
//    OPENROX   : File test_znccrosscor.cpp
//
//    Contents  : Tests for znccrosscor.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//====== INCLUDED HEADERS   ================================================

#include <openrox_tests.hpp>

extern "C"
{
   #include <float.h>

   #include <system/time/timer.h>

   #include <baseproc/array/conversion/array2d_float_from_uchar.h>
	#include <baseproc/array/crosscor/zncrosscor.h>

   #include <inout/image/pgm/pgmfile.h>
   #include <inout/system/print.h>
}

//====== INTERNAL MACROS    ================================================

ROX_TEST_SUITE_BEGIN ( zncc_crosscor )

#ifdef ANDROID
   #define RESULT_PATH "/storage/emulated/0/Documents/Robocortex/Tests/Results/"
#else
   #define RESULT_PATH "./"
#endif

#define SIZE 8

#define TEST_IMG ROX_DATA_HOME"/regression_tests/openrox/plane/model_corkes_032x032.pgm"

//====== INTERNAL TYPESDEFS ================================================

//====== INTERNAL DATATYPES ================================================

//====== INTERNAL VARIABLES ================================================

//====== INTERNAL FUNCTDEFS ================================================

//====== INTERNAL FUNCTIONS ================================================

//====== EXPORTED FUNCTIONS ================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_uchar_zncc_nomask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double zncc = 0.0;

   Rox_Uchar v1_data[SIZE] =
   {
      0, 5, 255, 255 ,255, 169, 10, 0
   };

   Rox_Uchar v2_data[SIZE] =
   {
      0, 5, 255, 255 ,255, 169, 10, 0
   };

   Rox_Array2D_Uchar v1 = NULL;
   Rox_Array2D_Uchar v2 = NULL;
   Rox_Array2D_Uchar vb = NULL;

   error = rox_array2d_uchar_new(&v1, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new(&v2, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new(&vb, 1, SIZE - 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_set_buffer_no_stride(v1, v1_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_set_buffer_no_stride(v2, v2_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test NULL pointers 
   error = rox_array2d_uchar_zncc_nomask_normalizedscore(&zncc, v1, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_zncc_nomask_normalizedscore(&zncc, NULL, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_zncc_nomask_normalizedscore(NULL, v1, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_zncc_nomask_normalizedscore(&zncc, NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_zncc_nomask_normalizedscore(NULL, v1, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_zncc_nomask_normalizedscore(NULL, NULL, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_zncc_nomask_normalizedscore(NULL, NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Test bad sizes 
   error = rox_array2d_uchar_zncc_nomask_normalizedscore(&zncc, vb, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   error = rox_array2d_uchar_zncc_nomask_normalizedscore(&zncc, v1, vb);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   // Test correct values 
   error = rox_array2d_uchar_zncc_nomask_normalizedscore(&zncc, v1, v2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_SMALL(1.0 - zncc, 1e-15);

   error = rox_array2d_uchar_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&v2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&vb);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_float_zncc_nomask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double zncc = 0.0;

   Rox_Float v1_data[SIZE] =
   {
      0.0, 14415.0, 114145.0, 254455.0, 88421.0, 169.0, 10.0, 0.0
   };

   Rox_Float v2_data[SIZE] =
   {
      0.0, 14415.0, 114145.0, 254455.0, 88421.0, 169.0, 10.0, 0.0
   };

   Rox_Array2D_Float v1 = NULL;
   Rox_Array2D_Float v2 = NULL;
   Rox_Array2D_Float vb = NULL;

   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);

   error = rox_array2d_float_new(&v1, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&v2, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&vb, 1, SIZE - 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride(v1, v1_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride(v2, v2_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test NULL pointers 
   error = rox_array2d_float_zncc_nomask_normalizedscore(&zncc, v1, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_zncc_nomask_normalizedscore(&zncc, NULL, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_zncc_nomask_normalizedscore(NULL, v1, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_zncc_nomask_normalizedscore(&zncc, NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_zncc_nomask_normalizedscore(NULL, v1, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_zncc_nomask_normalizedscore(NULL, NULL, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_zncc_nomask_normalizedscore(NULL, NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Test bad sizes 
   error = rox_array2d_float_zncc_nomask_normalizedscore(&zncc, vb, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   error = rox_array2d_float_zncc_nomask_normalizedscore(&zncc, v1, vb);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   // Test correct values 
   error = rox_array2d_float_zncc_nomask_normalizedscore(&zncc, v1, v2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_SMALL(1.0 - zncc, 1e-15);

   error = rox_array2d_float_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_float_del(&v2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_float_del(&vb);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_float_zncc)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double zncc = 0.0;

   Rox_Float v1_data[SIZE] =
   {
      FLT_EPSILON-1E-6, 14415.0, 114145.0, 254455.0, 88421.0, 169.0, 10.0, FLT_EPSILON-1E-6
   };

   Rox_Float v2_data[SIZE] =
   {
      FLT_EPSILON-1E-6, 14415.0, 114145.0, 254455.0, 88421.0, 169.0, 10.0, FLT_EPSILON-1E-6
   };

   Rox_Uint mask_data[SIZE] =
   {
      1U, 0U, 0U, 0U, 0U, 0U, 0U, 1U
   };

   Rox_Array2D_Float v1 = NULL;
   Rox_Array2D_Float v2 = NULL;
   Rox_Array2D_Float vb = NULL;
   Rox_Array2D_Uint mask = NULL;

   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);

   error = rox_array2d_float_new(&v1, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&v2, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_new(&mask, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&vb, 1, SIZE - 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride(v1, v1_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride(v2, v2_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_set_buffer_no_stride(mask, mask_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test NULL pointers 
   error = rox_array2d_float_zncc_normalizedscore(&zncc, v1, v2, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_zncc_normalizedscore(&zncc, v1, NULL, mask);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_zncc_normalizedscore(&zncc, NULL, v2, mask);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_zncc_normalizedscore(NULL, v1, v2, mask);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_zncc_normalizedscore(NULL, v1, v2, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_zncc_normalizedscore(&zncc, NULL, NULL, mask);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_zncc_normalizedscore(&zncc, NULL, NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_zncc_normalizedscore(NULL, v1, NULL, mask);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_zncc_normalizedscore(NULL, v1, NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_zncc_normalizedscore(NULL, NULL, v2, mask);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_zncc_normalizedscore(NULL, NULL, v2, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_zncc_normalizedscore(NULL, NULL, NULL, mask);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_zncc_normalizedscore(NULL, NULL, NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Test bad sizes 
   error = rox_array2d_float_zncc_normalizedscore(&zncc, vb, v2, mask);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   error = rox_array2d_float_zncc_normalizedscore(&zncc, v1, vb, mask);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   error = rox_array2d_float_zncc_normalizedscore(&zncc, v1, v2, mask);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&v2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&vb);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&mask);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_uchar_zncc)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double zncc = 0.0;

   Rox_Uchar v1_data[SIZE] =
   {
      0, 5, 255, 255 ,255, 169, 10, 0
   };

   Rox_Uchar v2_data[SIZE] =
   {
      0, 5, 255, 255 ,255, 169, 10, 0
   };

   Rox_Uint mask_data[SIZE] =
   {
      1U, 0U, 0U, 0U, 0U, 0U, 0U, 1U
   };

   Rox_Array2D_Uchar v1 = NULL;
   Rox_Array2D_Uchar v2 = NULL;
   Rox_Array2D_Uchar vb = NULL;
   Rox_Array2D_Uint mask = NULL;

   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);

   error = rox_array2d_uchar_new(&v1, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_uchar_new(&v2, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_uint_new(&mask, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_uchar_new(&vb, 1, SIZE - 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_set_buffer_no_stride(v1, v1_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_uchar_set_buffer_no_stride(v2, v2_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_uint_set_buffer_no_stride(mask, mask_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test NULL pointers 
   error = rox_array2d_uchar_zncc_normalizedscore(&zncc, v1, v2, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_zncc_normalizedscore(&zncc, v1, NULL, mask);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_zncc_normalizedscore(&zncc, NULL, v2, mask);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_zncc_normalizedscore(NULL, v1, v2, mask);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_zncc_normalizedscore(NULL, v1, v2, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_zncc_normalizedscore(&zncc, NULL, NULL, mask);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_zncc_normalizedscore(&zncc, NULL, NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_zncc_normalizedscore(NULL, v1, NULL, mask);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_zncc_normalizedscore(NULL, v1, NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_zncc_normalizedscore(NULL, NULL, v2, mask);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_zncc_normalizedscore(NULL, NULL, v2, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_zncc_normalizedscore(NULL, NULL, NULL, mask);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_zncc_normalizedscore(NULL, NULL, NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Test bad sizes 
   error = rox_array2d_uchar_zncc_normalizedscore ( &zncc, vb, v2, mask );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   ROX_TEST_MESSAGE("error = %d \n", error);

   error = rox_array2d_uchar_zncc_normalizedscore ( &zncc, v1, vb, mask );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   ROX_TEST_MESSAGE("error = %d \n", error);
   
   //Will return a 0 score, not an error as previously
   error = rox_array2d_uchar_zncc_normalizedscore ( &zncc, v1, v2, mask );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   ROX_TEST_MESSAGE("error = %d \n", error);

   ROX_TEST_MESSAGE ( "zncc = %f \n", zncc );

   error = rox_array2d_uchar_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&v2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&vb);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&mask);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_uchar_zncc_performance)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Double zncc = 0.0;
   Rox_Sint nbzncc = 1000;

   Rox_Array2D_Uchar Iu = NULL;
   Rox_Array2D_Uchar Tu = NULL;

   Rox_Timer timer = NULL ;
   Rox_Double time = 0.0, total_time = 0.0;
   
   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);

   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", TEST_IMG);
   ROX_TEST_MESSAGE("read file %s\n", filename);

   error = rox_array2d_uchar_new_pgm(&Iu, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new_pgm(&Tu, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Sint rows = 0, cols = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, Iu);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test the image gradient without mask
   for (Rox_Sint i = 0; i < nbzncc; ++i)
   {
      rox_timer_start(timer);

      error = rox_array2d_uchar_zncc_nomask(&zncc, Iu, Tu);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   ROX_TEST_MESSAGE("mean time to compute the zncc of a (%d x %d) uchar image = %f (ms)\n", cols, rows, total_time/nbzncc);

   error = rox_array2d_uchar_del ( &Iu );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del ( &Tu );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_del ( &timer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_float_zncc_performance)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Double zncc = 0.0;
   Rox_Sint nbzncc = 1000;

   Rox_Array2D_Uchar Iu = NULL;
   Rox_Array2D_Uchar Tu = NULL;

   Rox_Array2D_Float If = NULL;
   Rox_Array2D_Float Tf = NULL;

   Rox_Timer timer = NULL ;
   Rox_Double time = 0.0, total_time = 0.0;

   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);
   
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", TEST_IMG);
   ROX_TEST_MESSAGE("read file %s\n", filename);

   error = rox_array2d_uchar_new_pgm(&Iu, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new_pgm(&Tu, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Sint rows = 0, cols = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, Iu);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&If, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&Tf, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_from_uchar(If, Iu);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_from_uchar(Tf, Tu);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test the image gradient without mask
   for (Rox_Sint i = 0; i < nbzncc; ++i)
   {
      rox_timer_start(timer);

      error = rox_array2d_float_zncc_nomask(&zncc, If, Tf);
      //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   ROX_TEST_MESSAGE("mean time to compute the zncc of a (%d x %d) float image = %f (ms)\n", cols, rows, total_time/nbzncc);

   error = rox_array2d_uchar_del(&Iu);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&Tu);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &If );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Tf );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_del ( &timer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
