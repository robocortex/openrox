//==============================================================================
//
//    OPENROX   : File test_conversion_array2d_uint_uchar.cpp
//
//    Contents  : Tests for array2d_uint_uchar.c and array2d_uint_to_uchar.c
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
	#include <baseproc/array/conversion/array2d_uchar_from_uint.h>
	#include <baseproc/array/conversion/array2d_uint_from_uchar.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(band)

#define SIZE 6

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_uchar_from_uint_mask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Uint v1_data[SIZE] =
   {
      0, 0xFF, 0x100, 0xFF00, 0xFF0000, 0xFF000000
   };

   Rox_Uchar v_data_res[SIZE] =
   {
      0, 255,      1,    255,    255,    255
   };

   Rox_Array2D_Uint v1 = NULL;
   Rox_Array2D_Uint v1b = NULL;
   Rox_Array2D_Uchar vr = NULL;
   Rox_Array2D_Uchar vrb = NULL;

   error = rox_array2d_uint_new(&v1, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new(&vr, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_new(&v1b, 1, SIZE-1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new(&vrb, 1, SIZE-1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_set_buffer_no_stride(v1, v1_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   /* Test NULL pointers */
   error = rox_array2d_uchar_from_uint_mask(vr, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
   error = rox_array2d_uchar_from_uint_mask(NULL, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
   error = rox_array2d_uchar_from_uint_mask(NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   /* Test bad sizes */
   error = rox_array2d_uchar_from_uint_mask(vr, v1b);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);
   error = rox_array2d_uchar_from_uint_mask(vrb, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

   /* Test correct values */
   error = rox_array2d_uchar_from_uint_mask(vr, v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Uint k = 0; k < SIZE; k++)
   {
		Rox_Uchar value = 0;
      error = rox_array2d_uchar_get_value(&value, vr, 0, k);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      ROX_TEST_CHECK_EQUAL(value, v_data_res[k]);
   }

   error = rox_array2d_uint_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uchar_del(&vr);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uint_del(&v1b);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uchar_del(&vrb);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_uint_from_uint_mask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Uchar v1_data[SIZE] =
   {
      0, 1, 64, 127, 254, 255
   };

   Rox_Uint v_data_res[SIZE] =
   {
      0U, 0x01010101, 0x40404040, 0x7F7F7F7F, 0xFEFEFEFE, 0xFFFFFFFF
   };

   Rox_Array2D_Uchar v1 = NULL;
   Rox_Array2D_Uchar v1b = NULL;
   Rox_Array2D_Uint vr = NULL;
   Rox_Array2D_Uint vrb = NULL;

   error = rox_array2d_uchar_new(&v1, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uint_new(&vr, 1, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uchar_new(&v1b, 1, SIZE-1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uint_new(&vrb, 1, SIZE-1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_set_buffer_no_stride(v1, v1_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   /* Test NULL pointers */
   error = rox_array2d_uint_from_uchar_mask(vr, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
   error = rox_array2d_uint_from_uchar_mask(NULL, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
   error = rox_array2d_uint_from_uchar_mask(NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   /* Test bad sizes */
   error = rox_array2d_uint_from_uchar_mask(vr, v1b);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);
   error = rox_array2d_uint_from_uchar_mask(vrb, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

   /* Test correct values */
   error = rox_array2d_uint_from_uchar_mask(vr, v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Uint k = 0; k < SIZE; k++)
   {
		Rox_Uint value = 0;
		error = rox_array2d_uint_get_value(&value, vr, 0, k);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      ROX_TEST_CHECK_EQUAL(value, v_data_res[k]);
   }

   error = rox_array2d_uchar_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uint_del(&vr);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uchar_del(&v1b);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uint_del(&vrb);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
