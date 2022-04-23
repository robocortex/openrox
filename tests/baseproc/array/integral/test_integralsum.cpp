//==============================================================================
//
//    OPENROX   : File test_integralsum.cpp
//
//    Contents  : Tests for integralsum.c
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
	#include <baseproc/array/integral/integralsum.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(integralsum)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_float_integralsum)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest,test_array2d_sint_integralsum)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}
	
ROX_TEST_CASE_DECLARE(rox::OpenROXTest,test_array2d_uchar_integralsum)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // Rox_Uchar value = 0U;
   Rox_Uint k;

   Rox_Array2D_Uchar v1 = NULL;
   Rox_Array2D_Uchar vbr = NULL;
   Rox_Array2D_Uchar vbc = NULL;
	
	Rox_Array2D_Uint vres = NULL;

   Rox_Uchar v_data_1[6] =
   {
       1, 2, 3, 4, 5, 6
   };

//   Rox_Uchar v_data_res[6] =
//   {
//
//   };

   error = rox_array2d_uchar_new(&v1, 6, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new(&vbr, 6, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new(&vbc, 7, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_new(&vres, 6, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
	
   error = rox_array2d_uchar_set_buffer_no_stride(v1, v_data_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test NULL pointer 
   error = rox_array2d_uchar_integralsum(vres, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_integralsum(NULL, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_integralsum(NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Test bad sizes 
	
   // error = rox_array2d_uchar_integralsum(vbr, v1);
   // ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   // error = rox_array2d_uchar_integralsum(vres, vbr);
   // ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   // error = rox_array2d_uchar_integralsum(vbc, v1);
   // ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   // error = rox_array2d_uchar_integralsum(vres, vbc);
   // ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);
	
	
   // Test correct values 
   error = rox_array2d_uchar_integralsum(vres, v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (k = 0; k < 6; k++)
   {
      // error = rox_array2d_uchar_get_value(&value, vres, k, 0);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      // ROX_TEST_CHECK_EQUAL(value, v_data_res[k]);
   }

   error = rox_array2d_uchar_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&vres);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&vbr);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&vbc);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
