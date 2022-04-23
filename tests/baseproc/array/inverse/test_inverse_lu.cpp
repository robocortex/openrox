//==============================================================================
//
//    OPENROX   : File test_inverse_lu.cpp
//
//    Contents  : Tests for inverse_lu.c
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
	#include <baseproc/array/inverse/inverse_lu.h>
   #include <baseproc/maths/random/random.h>
   #include <inout/numeric/array2d_print.h>
   #include <system/time/timer.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(inverse_lu)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_array2d_double_inverse_lu)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double M = NULL;
   Rox_Array2D_Double M_inv = NULL;
   // Rox_Double data[9] = { 2.0, 2.0, 2.0, 
   //                        2.0, 2.0, 2.0,
   //                        2.0, 2.0, 2.0 };
   
   Rox_Double data[9] = { 2.0, 2.0, -2.0, 
                          3.0, 2.0,  5.0,
                          2.0, 4.0,  2.0 };
  
   // Inverse of the matrix
   //  0.400000000000000   0.300000000000000  -0.350000000000000
   // -0.100000000000000  -0.200000000000000   0.400000000000000
   // -0.200000000000000   0.100000000000000   0.050000000000000

   error = rox_array2d_double_new(&M, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride(M, data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&M_inv, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_inverse_lu(NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_inverse_lu(NULL, M);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_inverse_lu(M_inv, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_inverse_lu(M_inv, M);
   // ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 

   error = rox_array2d_double_print(M_inv);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 

   error = rox_array2d_double_del(&M);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&M_inv);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_perf_array2d_double_inverse_lu)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   const Rox_Sint rows = 268;
   const Rox_Sint cols = 268;
   Rox_Double time = 0.0, total_time = 0.0;

#ifdef DEBUG
   // Small number of tests for slow debug (e.g. with valgrind)
   Rox_Sint nb_tests = 1;
#else 
   // High number of tests for measuring average performance in release
   Rox_Sint nb_tests = 100;
#endif

   Rox_Double data[rows*cols];
   Rox_Array2D_Double M = NULL;
   Rox_Array2D_Double M_inv = NULL;

   Rox_Random random_generator = NULL;
   Rox_Timer timer = NULL ;
  
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_random_new(&random_generator);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&M, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // fill data with random values
   for (Rox_Sint k=0; k<rows*cols; k++)
   {
      Rox_Sint draw = 0;
      error = rox_random_get_lcg_draw(&draw, random_generator);      
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      data[k] = 1.0/ ((double) draw);
   }

   error = rox_array2d_double_set_buffer_no_stride(M, data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&M_inv, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Sint k=0; k<nb_tests; k++)
   {
      rox_timer_start(timer);
      
      error = rox_array2d_double_inverse_lu(M_inv, M);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 

      // // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to invert a (%d x %d) matrix = %f (ms)\n", rows, cols, total_time/nb_tests);

   //error = rox_array2d_double_print(M_inv);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 

   error = rox_array2d_double_del(&M);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&M_inv);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_random_del(&random_generator);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
