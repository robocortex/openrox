//==============================================================================
//
//    OPENROX   : File test_bnot.cpp
//
//    Contents  : Tests for bnot.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== INCLUDED HEADERS   =====================================================

#include <openrox_tests.hpp>

extern "C"
{
	#include <baseproc/array/bnot/bnot.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN(band)

#define SIZE 5

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_uint_bnot)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint k;
   Rox_Uint value = 0;

   Rox_Uint v1_data[SIZE] =
   {
       ~0U, 0U, 0x0000FFFFU, 0xFFFF0000U, 0x1U
   };

   Rox_Uint v_data_res[SIZE] =
   {
       0U, ~0U, 0xFFFF0000U, 0x0000FFFFU, 0xFFFFFFFEU
   };

   Rox_Array2D_Uint v1 = NULL;
   Rox_Array2D_Uint vr = NULL;
   Rox_Array2D_Uint vb = NULL;

   error = rox_array2d_uint_new(&v1, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uint_new(&vr, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uint_new(&vb, 1, SIZE-1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_set_buffer_no_stride(v1, v1_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   /* Test NULL pointers */
   error = rox_array2d_uint_bnot(NULL, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uint_bnot(vr, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uint_bnot(NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   /* Test bad sizes */
   error = rox_array2d_uint_bnot(vb, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

   error = rox_array2d_uint_bnot(vr, vb);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

   /* Test correct values */
   error = rox_array2d_uint_bnot(vr, v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (k = 0; k < SIZE; k++)
   {
      error = rox_array2d_uint_get_value(&value, vr, 0, k);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      ROX_TEST_CHECK_EQUAL(value, v_data_res[k]);
   }

   error = rox_array2d_uint_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uint_del(&vr);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uint_del(&vb);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
