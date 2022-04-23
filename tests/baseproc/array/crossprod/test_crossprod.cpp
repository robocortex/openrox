//==============================================================================
//
//    OPENROX   : File test_crossprod.cpp
//
//    Contents  : Tests for crossprod.c
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
	#include <baseproc/array/crossprod/crossprod.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(crossprod)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_crossprod)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double v1_data[3] =
   {
      2.0, 3.0, 4.0
   };

   Rox_Double v2_data[3] =
   {
      5.0, 6.0, 7.0
   };

   Rox_Double v_data_res[3] =
   {
      -3, 6, -3
   };

   Rox_Array2D_Double v1 = NULL;
   Rox_Array2D_Double v2 = NULL;
   Rox_Array2D_Double vb = NULL;
   Rox_Array2D_Double vr = NULL;

   error = rox_array2d_double_new(&v1, 1, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_new(&v2, 1, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_new(&vb, 1, 2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_new(&vr, 1, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride(v1, v1_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_buffer_no_stride(v2, v2_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test NULL pointers
   error = rox_array2d_double_crossprod(vr, v1, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_crossprod(vr, NULL, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_crossprod(NULL, v1, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_crossprod(vr, NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_crossprod(NULL, v1, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_crossprod(NULL, NULL, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_crossprod(NULL, NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Test bad size 
   error = rox_array2d_double_crossprod(vb, v1, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   error = rox_array2d_double_crossprod(vr, vb, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   error = rox_array2d_double_crossprod(vr, v1, vb);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   // Test correct values 
   error = rox_array2d_double_crossprod(vr, v1, v2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Uint k = 0; k < 3; k++)
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
   error = rox_array2d_double_del(&vb);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_del(&vr);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
