//==============================================================================
//
//    OPENROX   : File test_fill.cpp
//
//    Contents  : Tests for fill.c
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
	#include "baseproc/array/fill/fillunit.h"
	#include "baseproc/array/fill/fillval.h"
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(fill)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_double_fillunit)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double value = 0.0;
   Rox_Uint i, j;

   Rox_Array2D_Double v1 = NULL;

   Rox_Double v_data_res[9] =
   {
       1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0
   };

   error = rox_array2d_double_new(&v1, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   /* Test NULL pointer */
   error = rox_array2d_double_fillunit(NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   /* Test correct values */
   error = rox_array2d_double_fillunit(v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (i = 0; i < 3; i++)
   {
      for (j = 0; j < 3; j++)
      {
         error = rox_array2d_double_get_value(&value, v1, i, j);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
         ROX_TEST_CHECK_EQUAL(value, v_data_res[i*3+j]);
      }
   }

   error = rox_array2d_double_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_double_fillval)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double value = 0.0;

   Rox_Array2D_Double v1 = NULL;

   error = rox_array2d_double_new(&v1, 1, 6);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   /* Test NULL pointer */
   error = rox_array2d_double_fillval(NULL, 1.23456789);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   /* Test correct values */
   error = rox_array2d_double_fillval(v1, 1.23456789);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Uint k = 0; k < 6; k++)
   {
      error = rox_array2d_double_get_value(&value, v1, 0, k);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      ROX_TEST_CHECK_SMALL(value - 1.23456789, 1e-8);
   }

   error = rox_array2d_double_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_float_fillval)
{
	   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Float value = 0.0f;
   Rox_Uint k;

   Rox_Array2D_Float v1 = NULL;

   error = rox_array2d_float_new(&v1, 1, 6);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   /* Test NULL pointer */
   error = rox_array2d_float_fillval(NULL, 1.23456789f);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   /* Test correct values */
   error = rox_array2d_float_fillval(v1, 1.23456789f);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (k = 0; k < 6; k++)
   {
      error = rox_array2d_float_get_value(&value, v1, 0, k);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      ROX_TEST_CHECK_EQUAL(value, 1.23456789f);
   }

   error = rox_array2d_float_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_farray2d_uint_fillval)
{   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint value = 0;
   Rox_Uint k;

   Rox_Array2D_Uint v1 = NULL;

   error = rox_array2d_uint_new(&v1, 1, 6);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   /* Test NULL pointer */
   error = rox_array2d_uint_fillval(NULL, 123456789);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   /* Test correct values */
   error = rox_array2d_uint_fillval(v1, 123456789);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (k = 0; k < 6; k++)
   {
      error = rox_array2d_uint_get_value(&value, v1, 0, k);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      ROX_TEST_CHECK_EQUAL(value, 123456789u);
   }

   error = rox_array2d_uint_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_uchar_fillval)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uchar value = 0;
   Rox_Uchar k;

   Rox_Array2D_Uchar v1 = NULL;

   error = rox_array2d_uchar_new(&v1, 1, 6);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   /* Test NULL pointer */
   error = rox_array2d_uchar_fillval(NULL, 123);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   /* Test correct values */
   error = rox_array2d_uchar_fillval(v1, 123);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (k = 0; k < 6; k++)
   {
      error = rox_array2d_uchar_get_value(&value, v1, 0, k);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      ROX_TEST_CHECK_EQUAL(value, 123);
   }

   error = rox_array2d_uchar_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
