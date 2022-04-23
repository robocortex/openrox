//==============================================================================
//
//    OPENROX   : File test_conversion_array2d_float_from_uchar.cpp
//
//    Contents  : Tests for array2d_float_to_uchar.c and array2d_uchar_to_float.h
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
   #include <float.h>
   #include <baseproc/maths/maths_macros.h>

	#include <baseproc/array/conversion/array2d_float_from_uchar.h>
	#include <baseproc/array/conversion/array2d_uchar_from_float.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN ( conversion_array2d_float_from_uchar )

#define SIZE 9

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, array2d_float_to_uchar)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uchar value = 0;

   Rox_Float v1_data[SIZE] =
   {
      FLT_MAX, 0.0f,   FLT_MIN,   -1.1135f,   -0.0068f,    1.5326f, 2.0851f, 139.272f, 254.99999f
   };

#ifdef ANDROID
   Rox_Uchar v_data_res[SIZE] =
   {
      255, 0, 0, 255, 0, 1, 2, 139, 254
   };
#else   
   Rox_Uchar v_data_res[SIZE] =
   {
        0, 0, 0, 255, 0, 1, 2, 139, 254
   };
#endif

   Rox_Array2D_Float v1 = NULL;
   Rox_Array2D_Float v1b = NULL;
   Rox_Array2D_Uchar vr = NULL;
   Rox_Array2D_Uchar vrb = NULL;

   error = rox_array2d_float_new ( &v1, 1, SIZE );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new ( &vr, 1, SIZE );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&v1b, 1, SIZE-1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new(&vrb, 1, SIZE-1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride(v1, v1_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test NULL pointers
   error = rox_array2d_uchar_from_float(vr, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_from_float(NULL, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_from_float(NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Test bad sizes 
   error = rox_array2d_uchar_from_float(vr, v1b);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

   error = rox_array2d_uchar_from_float ( vrb, v1 );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

   // Test correct values 
   error = rox_array2d_uchar_from_float ( vr, v1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Uint k = 0; k < SIZE; k++)
   {
      error = rox_array2d_uchar_get_value ( &value, vr, 0, k );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      ROX_TEST_MESSAGE("value = %d \n", value);

      ROX_TEST_CHECK_EQUAL ( value, v_data_res[k] );
   }

   error = rox_array2d_float_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&vr);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&v1b);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&vrb);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_float_normalize_to_uchar )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uchar value = 0;

   Rox_Float v1_data[SIZE] =
   {
      FLT_MAX, 0.0f,   FLT_MIN,   -1.1135f,   -0.0068f,    1.5326f, 2.0851f, 139.272f, 254.99999f
   };

#ifdef ANDROID
   Rox_Uchar v_data_res[SIZE] =
   {
      255, 0, 0, 229, 255, 134, 19, 186, 0
   };
#else   
   Rox_Uchar v_data_res[SIZE] =
   {
      0, 0, 0, 229, 255, 134, 19, 186, 0
   };
#endif

   Rox_Array2D_Float v1 = NULL;
   Rox_Array2D_Float v1b = NULL;
   Rox_Array2D_Uchar vr = NULL;
   Rox_Array2D_Uchar vrb = NULL;

   error = rox_array2d_float_new(&v1, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new(&vr, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&v1b, 1, SIZE-1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new(&vrb, 1, SIZE-1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride(v1, v1_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test NULL pointers 
   error = rox_array2d_uchar_from_float_normalize(vr, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_from_float_normalize(NULL, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_from_float_normalize(NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Test bad sizes 
   error = rox_array2d_uchar_from_float_normalize(vr, v1b);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

   error = rox_array2d_uchar_from_float_normalize(vrb, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

   // Test correct values
   error = rox_array2d_uchar_from_float_normalize(vr, v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Uint k = 0; k < SIZE; k++)
   {
      error = rox_array2d_uchar_get_value(&value, vr, 0, k);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      ROX_TEST_MESSAGE("value = %d \n", value);

      ROX_TEST_CHECK_EQUAL(value, v_data_res[k]);
   }

   error = rox_array2d_float_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&vr);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&v1b);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&vrb);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_array2d_float_from_uchar )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Float value = 0.0;

   Rox_Uchar v1_data[SIZE] =
   {
      0, 255, 1, 2, 10, 139, 254, 128, 127
   };

   Rox_Float v_data_res[SIZE] =
   {
      0.0f, 255.0f, 1.0f, 2.0f, 10.0f, 139.0f, 254.0f, 128.0f, 127.0f
   };

   Rox_Array2D_Uchar v1 = NULL;
   Rox_Array2D_Uchar v1b = NULL;
   Rox_Array2D_Float vr = NULL;
   Rox_Array2D_Float vrb = NULL;

   error = rox_array2d_uchar_new(&v1, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&vr, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new(&v1b, 1, SIZE-1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&vrb, 1, SIZE-1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_set_buffer_no_stride(v1, v1_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test NULL pointers 
   error = rox_array2d_float_from_uchar(vr, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_from_uchar(NULL, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_from_uchar(NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Test bad sizes
   error = rox_array2d_float_from_uchar(vr, v1b);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

   error = rox_array2d_float_from_uchar(vrb, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

   // Test correct values
   error = rox_array2d_float_from_uchar(vr, v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Uint k = 0; k < SIZE; k++)
   {
      error = rox_array2d_float_get_value(&value, vr, 0, k);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      ROX_TEST_MESSAGE("value = %d \n", value);

      ROX_TEST_CHECK_SMALL(value - v_data_res[k], 1e-6f);
   }

   error = rox_array2d_uchar_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&vr);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&v1b);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&vrb);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_array2d_float_from_uchar_normalize)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Float value = 0.0;
   const int size = 4;

   Rox_Uchar v1_data[4] =
   {
      0, 255, 127, 1
   };

   Rox_Float v_data_res[4] =
   {
      0.0f, 1.0f, 0.498039f, 0.003922f
   };

   Rox_Array2D_Uchar v1 = NULL;
   Rox_Array2D_Uchar v1b = NULL;
   Rox_Array2D_Float vr = NULL;
   Rox_Array2D_Float vrb = NULL;

   error = rox_array2d_uchar_new(&v1, 1, size);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&vr, 1, size);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new(&v1b, 1, size-1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&vrb, 1, size-1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_set_buffer_no_stride(v1, v1_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test NULL pointers
   error = rox_array2d_float_from_uchar_normalize(vr, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_from_uchar_normalize(NULL, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_from_uchar_normalize(NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Test bad sizes 
   error = rox_array2d_float_from_uchar_normalize(vr, v1b);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

   error = rox_array2d_float_from_uchar_normalize(vrb, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

   // Test correct values 
   error = rox_array2d_float_from_uchar_normalize(vr, v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Uint k = 0; k < size; k++)
   {
      error = rox_array2d_float_get_value ( &value, vr, 0, k );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      ROX_TEST_MESSAGE("value = %d \n", value);

      ROX_TEST_CHECK_SMALL(value - v_data_res[k], 1e-6f);
   }

   error = rox_array2d_uchar_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&vr);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&v1b);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_float_del(&vrb);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_perf_array2d_float_from_uchar_normalize)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
