//==============================================================================
//
//    OPENROX   : File test_avx.cpp
//
//    Contents  : Tests for avx.c
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
   #include <generated/array2d_float.h>
   #include <system/vectorisation/avx.h>
   #include <system/errors/errors.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(avx)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_avx_uint)
{
   // Set var to {7, 6, 5, 4, 3, 2, 1, 0}
   __m256i var = _mm256_set_epi32(7, 6, 5, 4, 3, 2, 1, 0);
   rox_mm256i_printf_uint16 ( var );

   var = _mm256_set_epi32(7, 6, 5, 4, 3, 2, 1, ~0);
   rox_mm256i_printf_uint16 ( var );

}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_avx_float)
{
   Rox_ErrorCode  error = ROX_ERROR_NONE;
   Rox_Float sum = 0.0f;

   // Set var to {7, 6, 5, 4, 3, 2, 1, 0}
   __m256 var = _mm256_set_ps(7, 6, 5, 4, 3, 2, 1, 0);

   rox_mm256_printf_float ( var );

   sum = rox_mm256_hsum_ps ( var );
   ROX_TEST_CHECK_CLOSE ( sum, 28.0f, 1e-12 );

   rox_log("sum = %f\n", sum);

   Rox_Float data[4][4] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};

   __m256 avx = _mm256_loadu_ps(data[0]);
   rox_mm256_printf_float ( avx );
   avx = _mm256_loadu_ps(data[1]);
   rox_mm256_printf_float ( avx );


   rox_log("----------------------------------\n");
   // Test 
   Rox_Float buffer0[8] = {0};
   Rox_Float buffer1[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
   Rox_Float buffer2[8] = { 9, 10, 11, 12, 13, 14, 15, 16 };

   __m256 sse_4 = _mm256_set1_ps(4);

   __m256 b1 = _mm256_loadu_ps(buffer1);
   __m256 b2 = _mm256_loadu_ps(buffer2);

   rox_mm256_printf_float ( b1 );
   rox_mm256_printf_float ( b2 );

   __m256 add_rows = _mm256_add_ps( b1, b2 );

   rox_mm256_printf_float ( add_rows );

   __m256 add_cols =_mm256_hadd_ps ( add_rows, add_rows );

   rox_mm256_printf_float ( add_cols );

   __m256 mean = _mm256_div_ps ( add_cols, sse_4 );

   rox_mm256_printf_float ( mean );

   _mm256_storeu_ps ( buffer0, mean );

   rox_log("sum4 = %f\n", buffer0[0]);
   rox_log("sum3 = %f\n", buffer0[1]);
   rox_log("sum2 = %f\n", buffer0[4]);
   rox_log("sum1 = %f\n", buffer0[5]);

   Rox_Array2D_Float vector = NULL;
   error = rox_array2d_float_new(&vector, 1, 8);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Float ** vector_data = NULL;

   error = rox_array2d_float_get_data_pointer_to_pointer ( &vector_data, vector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Sint stride = 0;
   rox_array2d_float_get_stride(&stride, vector);

   rox_log("ROX_DEFAULT_ALIGNMENT = %d\n", ROX_DEFAULT_ALIGNMENT);

   rox_log("stride = %d\n", stride);
   rox_log("sizeof = %lu\n", sizeof(vector_data[0]));

   rox_log("%p\n", (void *) &vector_data[0]);

   // Test if memory is correctly aligned
   if (((intptr_t) vector_data[0] & (ROX_DEFAULT_ALIGNMENT-1) ) == 0) {
      rox_log("The address is 32-byte aligned here");
   }
   else
   {
      rox_log("The address is NOT 32-byte aligned here");
   }

}

ROX_TEST_SUITE_END()
