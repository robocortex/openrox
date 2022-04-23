//==============================================================================
//
//    OPENROX   : File test_matso3.cpp
//
//    Contents  : Tests for matso3.c
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
   #include <baseproc/maths/linalg/matso3.h>
   #include <baseproc/maths/linalg/matrix.h>
   #include <inout/system/print.h>   
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(matso3)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_unit_matso3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSO3 matso3 = NULL;
   
   error = rox_matso3_new(NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_matso3_new(&matso3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matso3_del(NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_matso3_del(&matso3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_matso3_mulmatmat)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_MatSO3 result  = NULL;
   Rox_MatSO3 input_1 = NULL;
   Rox_MatSO3 input_2 = NULL;

   error = rox_matso3_new(&result);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matso3_new(&input_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matso3_new(&input_2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   //Rox_Double ** R = rox_array2d_double_get_data_pointer_to_pointer( result );
   //Rox_Double ** R1 = rox_array2d_double_get_data_pointer_to_pointer( input_1 );
   //Rox_Double ** R2 = rox_array2d_double_get_data_pointer_to_pointer( input_2 );

   // Change here input data

   error = rox_matso3_mulmatmat( result, input_1, input_2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matso3_print(result);

   error = rox_matso3_del(&result);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matso3_del(&input_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matso3_del(&input_2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_matso3_update_left)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSO3 matso3 = NULL;
   Rox_Array2D_Double vector = NULL;

   error = rox_matrix_new(&vector, 3, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Double ** data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &data, vector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   data[0][0] =  0.0;
   data[1][0] =  0.0;
   data[2][0] = -3.14159265359;

   error = rox_matso3_new(&matso3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // error = rox_matso3_print ( matso3 );
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matso3_update_left ( matso3, vector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_matso3_print ( matso3 );
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matso3_del ( &matso3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &vector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_matso3_log)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSO3 matso3 = NULL;
   Rox_Array2D_Double vector = NULL;

   error = rox_matrix_new(&vector, 3, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Double ** data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &data, vector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   data[0][0] =  0.0;
   data[1][0] =  0.0;
   data[2][0] = -3.14159265359;

   error = rox_matso3_new(&matso3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // error = rox_matso3_print ( matso3 );
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matso3_update_left ( matso3, vector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_matso3_print ( matso3 );
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double axis_x;
   Rox_Double axis_y; 
   Rox_Double axis_z; 
   Rox_Double angle;

   error = rox_array2d_double_logmat_so3 ( &axis_x, &axis_y, &axis_z, &angle, matso3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("u = [%f; %f; %f]; theta = %f \n", axis_x, axis_y, axis_z, angle );

   error = rox_matso3_del ( &matso3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &vector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_matso3_change_coordinate_system )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_MatSO3 matso3_2 = NULL;
   Rox_MatSO3 matso3_12 = NULL; 
   Rox_MatSO3 matso3_1 = NULL;

   error = rox_matso3_new ( &matso3_2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matso3_new ( &matso3_12 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matso3_new ( &matso3_1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matso3_change_coordinate_system ( matso3_2, matso3_12, matso3_1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matso3_del ( &matso3_2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matso3_del ( &matso3_12 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matso3_del ( &matso3_1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
