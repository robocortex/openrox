//==============================================================================
//
//    OPENROX   : File test_matse3.cpp
//
//    Contents  : Tests for matse3.c
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
   #include <baseproc/maths/maths_macros.h>
   #include <baseproc/maths/linalg/matse3.h>
   #include <baseproc/maths/linalg/ansi_matse3.h>
   #include <inout/numeric/ansi_array_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(matse3)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_unit_matse3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSE3 matse3 = NULL;
   
   error = rox_matse3_new(NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_matse3_new(&matse3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matse3_del(NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_matse3_del(&matse3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_matse3_mulmatmat )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_MatSE3 result  = NULL;
   Rox_MatSE3 input_1 = NULL;
   Rox_MatSE3 input_2 = NULL;

   error = rox_matse3_new(&result);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matse3_new(&input_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matse3_new(&input_2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // Change here input data

   error = rox_matse3_mulmatmat( result, input_1, input_2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // rox_matse3_print(result);

   error = rox_matse3_del(&result);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del(&input_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del(&input_2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_matse3_set_tra )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_matse3_set_rot_tra )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_matse3_change_coordinate_system )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_MatSE3 matse3_2 = NULL;
   Rox_MatSE3 matse3_12 = NULL; 
   Rox_MatSE3 matse3_1 = NULL;

   error = rox_matse3_new ( &matse3_2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_new ( &matse3_12 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_new ( &matse3_1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_change_coordinate_system ( matse3_2, matse3_12, matse3_1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del ( &matse3_2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del ( &matse3_12 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del ( &matse3_1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_matse3_update_right )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSE3 matse3 = NULL;
   Rox_Array2D_Double vector = NULL;

   error = rox_matrix_new(&vector, 6, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Double ** data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &data, vector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   data[0][0] = -3.14159265359/2.0;
   data[1][0] = 0.0;
   data[2][0] = 0.0;
   data[3][0] = 0.0;
   data[4][0] = 0.0;
   data[5][0] = -3.14159265359;

   error = rox_matse3_new ( &matse3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   rox_matse3_print ( matse3 );

   error = rox_matse3_update_right ( matse3, vector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matse3_print ( matse3 );

   error = rox_matse3_del ( &matse3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &vector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_matse3_update_left )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSE3 matse3 = NULL;
   Rox_Array2D_Double vector = NULL;

   error = rox_matrix_new(&vector, 6, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Double ** data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &data, vector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   data[0][0] = -3.14159265359/2.0;
   data[1][0] = 0.0;
   data[2][0] = 0.0;
   data[3][0] = 0.0;
   data[4][0] = 0.0;
   data[5][0] = -3.14159265359;

   error = rox_matse3_new ( &matse3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   rox_matse3_print ( matse3 );

   error = rox_matse3_update_left ( matse3, vector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matse3_print ( matse3 );

   error = rox_matse3_del ( &matse3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &vector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );  
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_matse3_set_rot )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_MatSE3 Rx_Pi = NULL;
   Rox_Double r[3] = { ROX_PI, 0.0, 0.0 };

   error = rox_matse3_new ( &Rx_Pi );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_set_rotation(Rx_Pi, r);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matse3_print(Rx_Pi);

   error = rox_matse3_del ( &Rx_Pi );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_ansi_array_float_matse3_update_left )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   float matse3_data[16] = {  1.0, 0.0, 0.0, 0.0, 
                              0.0, 1.0, 0.0, 0.0, 
                              0.0, 0.0, 1.0, 0.0, 
                              0.0, 0.0, 0.0, 1.0 };

   //float vector_data[6] = { -3.14159265359/2.0, 0.0, 0.0, 0.0, 0.0, -3.14159265359 };
   float vector_data[6] = { 0.1, 0.0, 0.0, 0.0, 0.0, -3.14159265359 };

   error = rox_ansi_array_float_matse3_update_left ( matse3_data, vector_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_ansi_array_float_print_as_array2d ( matse3_data, 4, 4 ); 
}



ROX_TEST_SUITE_END()
