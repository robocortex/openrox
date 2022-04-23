//==============================================================================
//
//    OPENROX   : File test_flip.cpp
//
//    Contents  : Tests for flip.c
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
	#include <baseproc/array/flip/fliplr.h>
	#include <baseproc/array/flip/flipud.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(flip)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_double_fliplr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double value = 0.0;

   Rox_Array2D_Double v1 = NULL;
   Rox_Array2D_Double vres = NULL;
   Rox_Array2D_Double vbr = NULL;
   Rox_Array2D_Double vbc = NULL;

   Rox_Double v_data_1[6] =
   {
       1.0, 2.0, 3.0, 4.0, 5.0, 6.0
   };

   Rox_Double v_data_res[6] =
   {
       6.0, 5.0, 4.0, 3.0, 2.0, 1.0
   };

   error = rox_array2d_double_new(&v1, 1, 6);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&vres, 1, 6);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&vbr, 2, 6);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&vbc, 1, 7);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride(v1, v_data_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test NULL pointer 
   error = rox_array2d_double_copy_flip_lr(vres, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_copy_flip_lr(NULL, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_copy_flip_lr(NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Test bad sizes 
   error = rox_array2d_double_copy_flip_lr(vbr, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   error = rox_array2d_double_copy_flip_lr(vres, vbr);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   error = rox_array2d_double_copy_flip_lr(vbc, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   error = rox_array2d_double_copy_flip_lr(vres, vbc);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   // Test correct values 
   error = rox_array2d_double_copy_flip_lr(vres, v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Uint k = 0; k < 6; k++)
   {
      error = rox_array2d_double_get_value(&value, vres, 0, k);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      ROX_TEST_CHECK_EQUAL(value, v_data_res[k]);
   }

   error = rox_array2d_double_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&vres);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&vbr);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&vbc);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_double_flipud)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double value = 0.0;

   Rox_Array2D_Double v1 = NULL;
   Rox_Array2D_Double vres = NULL;
   Rox_Array2D_Double vbr = NULL;
   Rox_Array2D_Double vbc = NULL;

   Rox_Double v_data_1[6] =
   {
       1.0, 2.0, 3.0, 4.0, 5.0, 6.0
   };

   Rox_Double v_data_res[6] =
   {
       6.0, 5.0, 4.0, 3.0, 2.0, 1.0
   };

   error = rox_array2d_double_new(&v1, 6, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&vres, 6, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&vbr, 6, 2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&vbc, 7, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride(v1, v_data_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test NULL pointer 
   error = rox_array2d_double_copy_flip_ud(vres, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_copy_flip_ud(NULL, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_copy_flip_ud(NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Test bad sizes
   error = rox_array2d_double_copy_flip_ud(vbr, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   error = rox_array2d_double_copy_flip_ud(vres, vbr);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   error = rox_array2d_double_copy_flip_ud(vbc, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   error = rox_array2d_double_copy_flip_ud(vres, vbc);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   // Test correct values
   error = rox_array2d_double_copy_flip_ud(vres, v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Uint k = 0; k < 6; k++)
   {
      error = rox_array2d_double_get_value(&value, vres, k, 0);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      ROX_TEST_CHECK_EQUAL(value, v_data_res[k]);
   }

   error = rox_array2d_double_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&vres);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&vbr);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&vbc);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_uchar_flipud)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uchar value = 0.0;

   Rox_Array2D_Uchar v1 = NULL;
   Rox_Array2D_Uchar vres = NULL;
   Rox_Array2D_Uchar vbr = NULL;
   Rox_Array2D_Uchar vbc = NULL;

   Rox_Uchar v_data_1[6] =
   {
       1, 2, 3, 4, 5, 6
   };

   Rox_Uchar v_data_res[6] =
   {
       6, 5, 4, 3, 2, 1
   };

   error = rox_array2d_uchar_new(&v1, 6, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new(&vres, 6, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new(&vbr, 6, 2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new(&vbc, 7, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_set_buffer_no_stride(v1, v_data_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test NULL pointer 
   error = rox_array2d_uchar_copy_flip_ud(vres, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_copy_flip_ud(NULL, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_uchar_copy_flip_ud(NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Test bad sizes
   error = rox_array2d_uchar_copy_flip_ud(vbr, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   error = rox_array2d_uchar_copy_flip_ud(vres, vbr);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   error = rox_array2d_uchar_copy_flip_ud(vbc, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   error = rox_array2d_uchar_copy_flip_ud(vres, vbc);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   // Test correct values
   error = rox_array2d_uchar_copy_flip_ud(vres, v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Uint k = 0; k < 6; k++)
   {
      error = rox_array2d_uchar_get_value(&value, vres, k, 0);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      ROX_TEST_CHECK_EQUAL(value, v_data_res[k]);
   }

   error = rox_array2d_uchar_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&vres);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&vbr);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&vbc);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
