//==============================================================================
//
//    OPENROX   : File test_expmat_se3.cpp
//
//    Contents  : Tests for expmat_se3
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
   #include <baseproc/maths/linalg/generators/algse3.h>
   #include <baseproc/maths/linalg/generators/ansi_algse3.h>
   #include <baseproc/maths/linalg/matse3.h>
   #include <baseproc/maths/linalg/ansi_matse3.h>
   #include <inout/numeric/ansi_array_print.h>
   #include <inout/system/print.h>   
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(expmat_se3)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_expmat_se3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //Rox_Double data[6] = {0.1, 0.2, 0.3, 0.03, 0.02, 0.01};
   // Rox_Double data[6] = {0.00142491790959718, -0.002071262739918059, 0.001239055088681452, -0.0002015452134074471, -0.0001542767239594148, -5.332182581774111e-05};
   Rox_Double data[6] = {0.3211767852132642, 0.3412693199973533, 0.1598871659721275, 0.04550322840265807, -0.0469693373136278, -0.02541554264104087};

   Rox_Double determinant = 0.0;
   Rox_Array2D_Double algse3 = NULL;
   Rox_MatSE3 matse3 = NULL;
   Rox_Array2D_Double vector = NULL;

   error = rox_array2d_double_new(&vector, 6, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride(vector, data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_new(&matse3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&algse3, 4, 4);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_algse3_set_velocity(algse3, vector);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_exponential_algse3(NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_matse3_exponential_algse3(NULL, algse3);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_matse3_exponential_algse3(matse3, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_matse3_exponential_algse3(matse3, algse3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_determinant(&determinant, matse3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("%32.32f\n", determinant);
   ROX_TEST_CHECK_SMALL(determinant-1.0, 1e-15);
   
   error = rox_matse3_print(matse3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&vector);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del(&matse3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&algse3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_mul_expmat_se3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Double data[6] = {0.00142491790959718, -0.002071262739918059, 0.001239055088681452, -0.0002015452134074471, -0.0001542767239594148, -5.332182581774111e-05};
   Rox_Double determinant = 0.0;

   Rox_Array2D_Double algse3 = NULL;
   Rox_MatSE3 matse3_1 = NULL;
   Rox_MatSE3 matse3_2 = NULL;
   Rox_MatSE3 matse3_3 = NULL;
   Rox_Array2D_Double vector = NULL;
   // Rox_Double data[6] = {0.1, 0.2, 0.3, 0.03, 0.02, 0.01};

   error = rox_array2d_double_new(&vector, 6, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&algse3, 4, 4);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_new(&matse3_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_new(&matse3_2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_new(&matse3_3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   srand(1); // initialisation de rand
   for (Rox_Uint k = 0; k < 10000; k++)
   {
      for ( Rox_Sint i =0; i<6; i++)
      {  
         data[i] = rand()/((double)RAND_MAX)/10.0;
      }

      error = rox_array2d_double_set_buffer_no_stride(vector, data);
	   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_algse3_set_velocity(algse3, vector);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_matse3_exponential_algse3(matse3_1, algse3);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_matse3_mulmatmat(matse3_3, matse3_2, matse3_1);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_matse3_copy(matse3_2, matse3_3);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

#if 0
      error = rox_matse3_print(matse3_1);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_matse3_print(matse3_2);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_matse3_print(matse3_3);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
#endif 
      
      error = rox_matse3_determinant(&determinant, matse3_3);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      ROX_TEST_CHECK_SMALL(determinant-1.0, 1e-9);
   }

   error = rox_matse3_print(matse3_3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&vector);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&algse3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del(&matse3_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del(&matse3_2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del(&matse3_3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array_double_expmat_se3 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //Rox_Double data[6] = {0.1, 0.2, 0.3, 0.03, 0.02, 0.01};
   // Rox_Double data[6] = {0.00142491790959718, -0.002071262739918059, 0.001239055088681452, -0.0002015452134074471, -0.0001542767239594148, -5.332182581774111e-05};
   double v_data[6] = {0.3211767852132642, 0.3412693199973533, 0.1598871659721275, 0.04550322840265807, -0.0469693373136278, -0.02541554264104087};

   double algse3_data[4*4] = {0.0};
   double matse3_data[4*4] = {0.0};

   error = rox_ansi_array_double_algse3_set_velocity ( algse3_data, v_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ansi_array_double_matse3_exponential_algse3 ( matse3_data, algse3_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_ansi_array_double_print_as_array2d ( matse3_data, 4, 4 );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array_float_expmat_se3 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //Rox_Double data[6] = {0.1, 0.2, 0.3, 0.03, 0.02, 0.01};
   // Rox_Double data[6] = {0.00142491790959718, -0.002071262739918059, 0.001239055088681452, -0.0002015452134074471, -0.0001542767239594148, -5.332182581774111e-05};
   float v_data[6] = {0.3211767852132642, 0.3412693199973533, 0.1598871659721275, 0.04550322840265807, -0.0469693373136278, -0.02541554264104087};

   float algse3_data[4*4] = {0.0};
   float matse3_data[4*4] = {0.0};

   error = rox_ansi_array_float_algse3_set_velocity ( algse3_data, v_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ansi_array_float_matse3_exponential_algse3 ( matse3_data, algse3_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_ansi_array_float_print_as_array2d ( matse3_data, 4, 4 );
}

ROX_TEST_SUITE_END()
