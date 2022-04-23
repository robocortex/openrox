//==============================================================================
//
//    OPENROX   : File test_sse.cpp
//
//    Contents  : Tests for sse.c
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
   #include <system/vectorisation/sse.h>
   #include <system/errors/errors.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(sse)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_sse)
{
   Rox_ErrorCode  error = ROX_ERROR_NONE;
   Rox_Float sum = 0.0f;

   // Set var to {3,2,1,0}
   __m128 var = _mm_set_ps ( 3, 2, 1, 0 );

   rox_mm128_printf_float ( var );

   sum = rox_mm128_hsum_ps ( var );
   ROX_TEST_CHECK_CLOSE ( sum, 6.0f, 1e-12 );

   rox_log("sum = %f\n", sum);

   __m128 hadd;

   hadd = _mm_hadd_ps(var, var);
   rox_mm128_printf_float ( hadd );

   __m128 var2 = _mm_set_ps ( 4, 3, 2, 1 );

   // 
   rox_log("--------------------------\n"); 
   rox_mm128_printf_float ( var );
   rox_mm128_printf_float ( var2 );

   __m128 var3 = _mm_add_ps(var, var2);

   rox_mm128_printf_float ( var3 );

   hadd = _mm_hadd_ps(var3, var3);
   rox_mm128_printf_float ( hadd );

   // The sum of the first 4 float is in var3[0], the 
   Rox_Float buffer_4_bytes[4];
   _mm_store_ps(buffer_4_bytes, hadd);

   rox_log("sum1 = %f\n", buffer_4_bytes[0]);
   rox_log("sum2 = %f\n", buffer_4_bytes[1]);

   __m128i a = _mm_set_epi16 ( 1, 2, 3, 4, 5, 6, 7, 8 );

  rox_mm128i_printf_uint16 ( a );


   Rox_Ushort buffer1[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
   Rox_Ushort buffer2[8] = { 9, 10, 11, 12, 13, 14, 15, 16 };

   __m128i b1 = _mm_loadu_si128(( __m128i* )buffer1);
   __m128i b2 = _mm_loadu_si128(( __m128i* )buffer2);

   rox_mm128i_printf_uint16 ( b1 );
   rox_mm128i_printf_uint16 ( b2 );

   __m128i add_rows = _mm_add_epi16( b1, b2 );

   rox_mm128i_printf_uint16 ( add_rows );

   __m128i add_cols =_mm_hadd_epi16 ( add_rows, add_rows );

   rox_mm128i_printf_uint16 ( add_cols );

   __m128i shift = _mm_srai_epi16 ( add_cols, 2 );

   rox_mm128i_printf_uint16 ( shift );

   _mm_storeu_si128 ( ( __m128i* ) buffer1, shift );
//
   rox_log("sum4 = %d\n", buffer1[3]);
   rox_log("sum3 = %d\n", buffer1[2]);
   rox_log("sum2 = %d\n", buffer1[1]);
   rox_log("sum1 = %d\n", buffer1[0]);

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
      rox_log("The address is 16-byte aligned here\n");
   }
   else
   {
      rox_log("The address is NOT 16-byte aligned here\n");
   }

//-----------------
   rox_log("load 2 vectors of 8 unsigned short\n");
   Rox_Uchar vector_16_uchar[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
   __m128i vector_8_ushort_1;
   __m128i vector_8_ushort_2;
   rox_mm128i_loadu_16_uchar_to_vectors_8_short ( &vector_8_ushort_1, &vector_8_ushort_2, vector_16_uchar);

   rox_mm128i_printf_uint16 ( vector_8_ushort_1 );
   rox_mm128i_printf_uint16 ( vector_8_ushort_2 );

}

ROX_TEST_SUITE_END()
