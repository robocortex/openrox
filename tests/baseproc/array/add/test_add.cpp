//==============================================================================
//
//    OPENROX   : File test_add.cpp
//
//    Contents  : Tests for add.c
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
	#include <baseproc/array/add/add.h>
   #include <inout/system/errors_print.h>   
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(add)

#define SIZE 7

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_float_add)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	// Init ground truth data
   Rox_Float v1_data[SIZE] =
   {
      -0.8637f,    0.0774f,   -1.2141f,   -1.1135f,   -0.0068f,    1.5326f,   -0.7697f
   };

   Rox_Float v2_data[SIZE] =
   {
       0.3714f,   -0.2256f,    1.1174f,   -1.0891f,    0.0326f,    0.5525f,    1.1006f
   };

   Rox_Float v_data_res[SIZE] =
   {
     -0.4923f,  -0.1482f,  -0.0967f,  -2.2026f,   0.0258f,   2.0851f,   0.3309f
   };

	// Init variables
	Rox_Array2D_Float v1 = NULL;
   Rox_Array2D_Float v2 = NULL;
   Rox_Array2D_Float vr = NULL;
   Rox_Array2D_Float vb = NULL;

   error = rox_array2d_float_new(&v1, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&v2, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&vr, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&vb, 1, SIZE-1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride(v1, v1_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride(v2, v2_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test NULL pointers 
   error = rox_array2d_float_add(vr, v1, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_add(vr, NULL, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_add(NULL, v1, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_add(vr, NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_add(NULL, v1, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_add(NULL, NULL, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_add(NULL, NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
   
   error = rox_array2d_float_add(vb, v1, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Test bad sizes
   error = rox_array2d_float_add(vr, vb, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

   error = rox_array2d_float_add(vr, v1, vb);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

   // Test correct values 
   error = rox_array2d_float_add(vr, v1, v2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for ( Rox_Sint k=0; k<SIZE; k++)
   {
		Rox_Float value = 0.0;
		error = rox_array2d_float_get_value(&value, vr, 0, k);
		ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
		ROX_TEST_CHECK_SMALL(value - v_data_res[k], 1e-6f);
   }

   error = rox_array2d_float_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_float_del(&v2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&vr);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&vb);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_double_add)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double v1_data[SIZE] =
   {
      -0.8637,    0.0774,   -1.2141,   -1.1135,   -0.0068,    1.5326,   -0.7697
   };

   Rox_Double v2_data[SIZE] =
   {
       0.3714,   -0.2256,    1.1174,   -1.0891,    0.0326,    0.5525,    1.1006
   };

   Rox_Double v_data_res[SIZE] =
   {
     -0.492300000000000,  -0.148200000000000,  -0.096700000000000,  -2.202600000000000,   0.025800000000000,   2.085100000000000,   0.330900000000000
   };

   Rox_Array2D_Double v1 = NULL;
   Rox_Array2D_Double v2 = NULL;
   Rox_Array2D_Double v3 = NULL;
   Rox_Array2D_Double vr = NULL;

   error = rox_array2d_double_new(&v1, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&v2, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&v3, 1, SIZE-1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&vr, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride(v1, v1_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride(v2, v2_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test NULL pointers 
   error = rox_array2d_double_add(vr, v1, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_add(vr, NULL, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_add(NULL, v1, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_add(vr, NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_add(NULL, v1, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_add(NULL, NULL, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_add(NULL, NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Test bad size 
   error = rox_array2d_double_add(v3, v1, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

   error = rox_array2d_double_add(vr, v3, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

   error = rox_array2d_double_add(vr, v1, v3);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

   // Test correct values 
   error = rox_array2d_double_add(vr, v1, v2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for ( Rox_Sint k=0;k<SIZE;k++)
   {
		Rox_Double value = 0.0;
      error = rox_array2d_double_get_value(&value, vr, 0, k);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      ROX_TEST_CHECK_SMALL(value - v_data_res[k], 1e-12);
   }

   error = rox_array2d_double_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&v2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&v3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&vr);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()