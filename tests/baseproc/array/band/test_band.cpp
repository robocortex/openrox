//==============================================================================
//
//    OPENROX   : File test_band.cpp
//
//    Contents  : Tests for band.c
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
   #include <baseproc/array/band/band.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(band)

#define SIZE 9

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_uint_band)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Uint v1_data[SIZE] =
   {
       0U, 0U, 1U, 1U, 10U, 5U, 2048U, ~0U, 3451627U
   };

   Rox_Uint v2_data[SIZE] =
   {
       0U, 1U, 0U, 1U, 10U, 3U,   64U, ~0U, 1262373U
   };

   Rox_Uint v_data_res[SIZE] =
   {
       0U, 0U, 0U, 1U, 10U, 1U,    0U, ~0U, 1049121U
   };

   Rox_Array2D_Uint v1 = NULL;
   Rox_Array2D_Uint v2 = NULL;
   Rox_Array2D_Uint vr = NULL;
   Rox_Array2D_Uint vb = NULL;

   error = rox_array2d_uint_new(&v1, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uint_new(&v2, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uint_new(&vr, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uint_new(&vb, 1, SIZE-1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_set_buffer_no_stride(v1, v1_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uint_set_buffer_no_stride(v2, v2_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test NULL pointers 
   error = rox_array2d_uint_band(vr, v1, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uint_band(vr, NULL, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uint_band(NULL, v1, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uint_band(vr, NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uint_band(NULL, v1, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uint_band(NULL, NULL, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uint_band(NULL, NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Test bad sizes 
   error = rox_array2d_uint_band(vb, v1, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

   error = rox_array2d_uint_band(vr, vb, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

   error = rox_array2d_uint_band(vr, v1, vb);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

   // Test correct values 
   error = rox_array2d_uint_band(vr, v1, v2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Uint k = 0; k < SIZE; k++)
   {
		Rox_Uint value = 0;
      error = rox_array2d_uint_get_value(&value, vr, 0, k);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      ROX_TEST_CHECK_EQUAL(value, v_data_res[k]);
   }

   error = rox_array2d_uint_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uint_del(&v2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uint_del(&vr);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uint_del(&vb);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
