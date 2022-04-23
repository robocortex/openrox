//==============================================================================
//
//    OPENROX   : File test_mulmattransmat.cpp
//
//    Contents  : Tests for mulmattransmat.c
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
   #include <baseproc/array/multiply/mulmattransmat.h>
   #include <baseproc/array/multiply/ansi_mulmattransmat.h>
   #include <inout/numeric/ansi_array_print.h>
   #include <inout/numeric/array2d_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(mulmattransmat)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_double_mulmattransmat)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
	Rox_Array2D_Double res = NULL;
	Rox_Array2D_Double one = NULL;
	Rox_Array2D_Double two = NULL;

	Rox_Double one_data[5]  = {1.0,2.0,3.0,4.0,5};
	Rox_Double two_data[30] = {1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,16.0,17.0,18.0,19.0,20.0,21.0,22.0,23.0,24.0,25.0,26.0,27.0,28.0,29.0,30.0};
	Rox_Double res_data[6]  = {255.0,270.0,285.0,300.0,315.0,330.0};

	error = rox_array2d_double_new (&res, 1, 6);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	error = rox_array2d_double_new (&one, 5, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	error = rox_array2d_double_new (&two, 5, 6);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride(one, one_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride(two, two_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	error = rox_array2d_double_mulmattransmat(res, one, two);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_print(res);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double ** dres = NULL;
   rox_array2d_double_get_data_pointer_to_pointer ( &dres, res );

   for(Rox_Sint i =0; i <6; i++)
   {
	   ROX_TEST_CHECK_SMALL(dres[0][i] - res_data[i], 1e-16);
   }

	error = rox_array2d_double_del(&res);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	error = rox_array2d_double_del(&one);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	error = rox_array2d_double_del(&two);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_ansi_array_double_mulmattransmat)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   double res_data[1*6] = {0.0};

   double one_data[5*1] = {1.0,2.0,3.0,4.0,5};
   double two_data[5*6] = {1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,16.0,17.0,18.0,19.0,20.0,21.0,22.0,23.0,24.0,25.0,26.0,27.0,28.0,29.0,30.0};
   double res_grt_data[1*6]  = {255.0,270.0,285.0,300.0,315.0,330.0};

   int res_rows = 1; int res_cols = 6;
   // int one_rows = 5; int one_cols = 1;
   int two_rows = 5; // int two_cols = 6;

   error = rox_ansi_array_double_mulmattransmat ( res_data, res_rows, res_cols, one_data, two_data, two_rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_ansi_array_double_print_as_array2d ( res_grt_data, res_rows, res_cols);
   rox_ansi_array_double_print_as_array2d ( res_data, res_rows, res_cols);

   for ( int i = 0; i < res_rows*res_cols; i++ )
   {
      ROX_TEST_CHECK_SMALL( res_grt_data[i] - res_data[i], 1e-8 );
   }

}

ROX_TEST_SUITE_END()
