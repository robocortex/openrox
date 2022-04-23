//==============================================================================
//
//    OPENROX   : File test_l2_error.cpp
//
//    Contents  : Tests for l2_error.c
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
	#include <baseproc/array/error/l2_error.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(l2_error)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_double_l2_error)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double l2_error = 0.0;
   
   Rox_Double v1_data[10] = { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0 };
   Rox_Double v2_data[10] = { 0.0, 1.0, 0.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0 };

   Rox_Array2D_Double v1 = NULL;
   Rox_Array2D_Double v2 = NULL;
   
   error = rox_array2d_double_new(&v1, 1, 10);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // Set the grt vector
   error = rox_array2d_double_set_buffer_no_stride ( v1, v1_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&v2, 1, 10);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // Set the grt vector
   error = rox_array2d_double_set_buffer_no_stride ( v2, v2_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test NULL pointers 
   error = rox_array2d_double_difference_l2_norm(NULL, v1, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_difference_l2_norm(&l2_error, NULL, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_difference_l2_norm(&l2_error, v1, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error =  rox_array2d_double_difference_l2_norm( &l2_error, v1, v2 );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   ROX_TEST_MESSAGE("l2_error (v1-v2) = %f \n", l2_error);
   ROX_TEST_CHECK_CLOSE (l2_error, 2.0, 1e-12);

   error = rox_array2d_double_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_del(&v2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_double_l2_error_mask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double l2_error = 0.0;
   
   Rox_Double v1_data[10] = { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0 };
   Rox_Double v2_data[10] = { 0.0, 1.0, 0.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0 };

   Rox_Array2D_Double v1 = NULL;
   Rox_Array2D_Double v2 = NULL;
   Rox_Array2D_Uint mask = NULL;
   
   error = rox_array2d_double_new(&v1, 1, 10);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // Set the grt vector
   error = rox_array2d_double_set_buffer_no_stride ( v1, v1_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&v2, 1, 10);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // Set the grt vector
   error = rox_array2d_double_set_buffer_no_stride ( v2, v2_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_new(&mask, 1, 10);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test NULL pointers 
   error = rox_array2d_double_difference_l2_norm_mask(NULL, v1, v2, mask);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_difference_l2_norm_mask(&l2_error, NULL, v2, mask);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_difference_l2_norm_mask(&l2_error, v1, NULL, mask);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_difference_l2_norm_mask(&l2_error, v1, v2, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error =  rox_array2d_double_difference_l2_norm_mask ( &l2_error, v1, v2, mask );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   ROX_TEST_MESSAGE("l2_error (v1-v2) = %f \n", l2_error);
   ROX_TEST_CHECK_CLOSE (l2_error, 2.0, 1e-12);

   error = rox_array2d_double_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_del(&v2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&mask);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
