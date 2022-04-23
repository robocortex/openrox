//==============================================================================
//
//    OPENROX   : File test_complementary_filter.cpp
//
//    Contents  : Tests for complementary_filter.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//====== INCLUDED HEADERS   ====================================================

#include <openrox_tests.hpp>

extern "C"
{
   #include <math.h>
   #include <baseproc/geometry/transforms/transform_tools.h>
   #include <core/inertial/observer/complementary_filter.h>
   #include <inout/numeric/array2d_print.h>
}

//====== INTERNAL MACROS    ====================================================

ROX_TEST_SUITE_BEGIN(complementary_filter)

//====== INTERNAL TYPESDEFS ====================================================

//====== INTERNAL DATATYPES ====================================================

//====== INTERNAL VARIABLES ====================================================

//====== INTERNAL FUNCTDEFS ====================================================

//====== INTERNAL FUNCTIONS ====================================================

//====== EXPORTED FUNCTIONS ====================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_cayley_xy_from_vectors)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double rc = NULL;
   Rox_Array2D_Double v1 = NULL;
   Rox_Array2D_Double v2 = NULL;
   Rox_Double rc_true[3] = { 0.258198889747161,0.516397779494322,0.0};
   Rox_Double v1_data[3] = {-0.774596669241483,0.387298334620742,0.5};
   Rox_Double v2_data[3] = {0.0,0.0,1.0};

   error = rox_array2d_double_new(&rc,3,1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_new_frombuffer(&v1,3,1,v1_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_new_frombuffer(&v2,3,1,v2_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_cayley_xy_from_vectors(rc, v1, v2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Double * rc_data = NULL;
   rox_array2d_double_get_data_pointer (&rc_data, rc);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_CHECK_SMALL(rc_data[0]-rc_true[0],1e-15);
   ROX_TEST_CHECK_SMALL(rc_data[1]-rc_true[1],1e-15);
   ROX_TEST_CHECK_SMALL(rc_data[2]-rc_true[2],1e-15);

   error = rox_array2d_double_print(rc);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&rc);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_del(&v2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_complementary_filter_make_predictions)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSO3 p_R_i_pred = NULL;
   Rox_MatSO3 p_R_i_prev = NULL;
   Rox_Array2D_Double wi = NULL;
   Rox_Double dt = 0.010; // seconds

   error = rox_complementary_filter_make_predictions(p_R_i_pred, p_R_i_prev, wi, dt);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
   
   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_complementary_filter_make_corrections)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSO3 p_R_i_next = NULL;
   Rox_MatSO3 p_R_i_pred = NULL;
   Rox_MatSO3 p_R_i_meas = NULL;
   Rox_Double k = 1.0; 
   Rox_Bool update = 1;

   error = rox_complementary_filter_make_corrections(p_R_i_next, p_R_i_pred, p_R_i_meas, k, update);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
   
   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_matso3_from_accelerometer_heading)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSO3 p_R_i_meas = NULL;
   Rox_Array2D_Double fi = NULL;
   //%Rox_Double hi = 0.5;
   //Rox_Double fi_data[3] = {0.0,0.0,-1.0};
   Rox_Double hi = -5.014741524381210;
   Rox_Double fi_data[3] = {0.034392830000000, 0.004770358000000, -0.999397000000000};
   error = rox_matso3_new(&p_R_i_meas);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_new_frombuffer(&fi, 3, 1, fi_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matso3_from_accelerometer_heading(p_R_i_meas, fi, hi);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_print(p_R_i_meas);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matso3_del(&p_R_i_meas);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_del(&fi);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_matso3_from_accelerometer_magnetometer)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSO3 p_R_i_meas = NULL;
   Rox_Array2D_Double fi = NULL;
   Rox_Array2D_Double mi = NULL;
   Rox_Double fi_data[3] = {0.0,0.0,-1.0};
   Rox_Double mi_data[3] = {sin(0.5),cos(0.5),0.0};

   error = rox_matso3_new(&p_R_i_meas);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_new_frombuffer(&fi, 3, 1, fi_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_new_frombuffer(&mi, 3, 1, mi_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matso3_from_accelerometer_magnetometer(p_R_i_meas, fi, mi);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_print(p_R_i_meas);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matso3_del(&p_R_i_meas);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_del(&fi);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_del(&mi);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
