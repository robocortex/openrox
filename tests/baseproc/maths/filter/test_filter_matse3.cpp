//=============================================================================
//
//    OPENROX   : File test_filter_matse3.cpp
//
//    Contents  : Test filter matse3 module 
// 
//    Author(s) : R&D department directed by Ezio MALIS
// 
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//=============================================================================

//====== INCLUDED HEADERS   ===================================================

#include <openrox_tests.hpp>

extern "C"
{
   #include <generated/array2d_double.h>
   #include <baseproc/maths/linalg/matrix.h>
   #include <baseproc/maths/filter/filter_matse3.h>
   #include <baseproc/array/scale/scale.h>
   #include <baseproc/array/error/l2_error.h>
   #include <inout/system/print.h>   
}

//====== INTERNAL MACROS    ===================================================

ROX_TEST_SUITE_BEGIN ( filter_matse3 )

#define TEST_PRECISION 1e-12
#define TEST_PRECISION_LOW 1e-8

//====== INTERNAL TYPESDEFS ===================================================

//====== INTERNAL DATATYPES ===================================================

//====== INTERNAL VARIABLES ===================================================

//====== INTERNAL FUNCTDEFS ===================================================

//====== INTERNAL FUNCTIONS ===================================================

//====== EXPORTED FUNCTIONS ===================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_filter_matse3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // Rox_Char filename[FILENAME_MAX];
   Rox_Double l2_error = 0.0;

   Rox_Double dt = 10.0/1000.0; // 100 Hz = 10 ms 

   Rox_Double Th_grt_data[16] = { 1.0, 0.0, 0.0, 1.000, 
                                  0.0, 1.0, 0.0, 0.000,
                                  0.0, 0.0, 1.0, 0.000,
                                  0.0, 0.0, 0.0, 1.000 };

   Rox_Double Tp_grt_data[16] = { 1.0, 0.0, 0.0, 1.001, 
                                  0.0, 1.0, 0.0, 0.000,
                                  0.0, 0.0, 1.0, 0.000,
                                  0.0, 0.0, 0.0, 1.000 };                       

   //    Rox_Double A_grt_data[16] = { 0.0, -0.3, +0.1,  0.02, 
   // 			  +0.3,  0.0, -0.2, -0.01, 
   // 			  -0.1, +0.2,  0.0,  0.03, 
   // 			   0.0,  0.0,  0.0,  0.00}; 

   //    Rox_Double A_grt_data[16] = { 0.0, -0.3, +0.1,  0.00, 
   // 			  +0.3,  0.0, -0.2, -0.00,  
   // 			  -0.1, +0.2,  0.0,  0.00, 
   // 			   0.0,  0.0,  0.0,  0.00}; 

   //    Rox_Double A_grt_data[16] = { 0.0, -0.0, +0.0,  2.0, 
   // 			  +0.0,  0.0, -0.0, -1.0, 
   // 			  -0.0, +0.0,  0.0,  3.0, 
   // 			   0.0,  0.0,  0.0,  0.00}; 

   // Rox_Double A_grt_data[16] = {  0.0, -0.3, +0.1,  2.0,
			//                        +0.3,  0.0, -0.2, -1.0,
			//                        -0.1, +0.2,  0.0,  3.0,
			//                         0.0,  0.0,  0.0,  0.0 };

   Rox_Double A_grt_data[16] = {  0.0,  0.0,  0.0,  0.1,
                              0.0,  0.0,  0.0,  0.0,
                              0.0,  0.0,  0.0,  0.0,
                              0.0,  0.0,  0.0,  0.0 };

   Rox_Matrix A_grt = NULL; 
   error = rox_matrix_new ( &A_grt, 4, 4 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Set the grt velocity
   error = rox_array2d_double_set_buffer_no_stride ( A_grt, A_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Matrix A_pre = NULL; error = rox_matrix_new ( &A_pre, 4, 4);
   Rox_Matrix A_est = NULL; error = rox_matrix_new ( &A_est, 4, 4);
   Rox_Matrix Adt   = NULL; error = rox_matrix_new ( &Adt, 4, 4);

   Rox_MatSE3 T_old = NULL; error = rox_matse3_new ( &T_old );
   Rox_MatSE3 T_inc = NULL; error = rox_matse3_new ( &T_inc );
   Rox_MatSE3 T_new = NULL; error = rox_matse3_new ( &T_new );
   Rox_MatSE3 T_pre = NULL; error = rox_matse3_new ( &T_pre );
   Rox_MatSE3 T_est = NULL; error = rox_matse3_new ( &T_est );

   Rox_MatSE3 Th_grt = NULL; error = rox_matse3_new ( &Th_grt );
   Rox_MatSE3 Tp_grt = NULL; error = rox_matse3_new ( &Tp_grt );

   error = rox_array2d_double_set_buffer_no_stride ( Th_grt, Th_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( Tp_grt, Tp_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Rox_Double  *t = rox_vector_get_data(rox_matse3_get_translation(T_new));
   // Rox_Double **R = rox_matso3_get_data(rox_matse3_get_matso3(T_new));

   // Pose file
   // Rox_File pose_file;
   
   // Measure file
   // Rox_File meas_file;

   // Calibration file
   // Rox_File cali_file;

   Rox_Sint save = 0, display = 1 ;
   Rox_Filter_MatSE3 filter_matse3 = NULL;
   error = rox_filter_matse3_new ( &filter_matse3, dt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   if (save)
   {
      //  sprintf(filename, "esm_pose_simu.txt");
      //  pose_file = rox_file_open(filename, "w");
    
      //  sprintf(filename, "Measure_simu.txt");
      //  meas_file = rox_file_open(filename, "w");
    
      //  sprintf(filename, "calibration_simu.txt");
      //  cali_file = rox_file_open(filename, "w");
    
      //  fprintf(cali_file, "%6.16f\t %6.16f\t %6.16f\t %6.16f\t %6.16f\t %6.16f\t %6.16f\t %6.16f\t %6.16f\t %6.16f\t %6.16f\t %6.16f\t \r\n", 
	   // R[0][0], R[1][0], R[2][0], 
	   // R[0][1], R[1][1], R[2][1], 
	   // R[0][2], R[1][2], R[2][2], 
	   // t[0], t[1], t[2]); 
   }

   Rox_Double trace = 0.0;
   error = rox_matrix_trace ( &trace, A_grt );
   ROX_TEST_CHECK_CLOSE ( trace, 0.0, TEST_PRECISION );

   // rox_matse3_display(T_new); 
  
   // Create first measure
   error = rox_array2d_double_scale ( Adt, A_grt, dt ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_exponential_algse3 ( T_inc, Adt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_mulmatmat ( T_new, T_old, T_inc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_filter_matse3_init_pose ( filter_matse3, T_new );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // Update old pose
   error = rox_matse3_copy ( T_old, T_new );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );


   if (display)
   {
      rox_log("measure : \n");
      rox_matse3_print ( T_new );

      error = rox_filter_matse3_get_predicted_pose ( T_pre, filter_matse3 );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_filter_matse3_get_estimated_pose ( T_est, filter_matse3 );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_filter_matse3_get_predicted_velocity ( A_pre, filter_matse3 );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_filter_matse3_get_estimated_velocity ( A_est, filter_matse3 );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      rox_log("pose estimation : \n");
      rox_matse3_print ( T_est );

      rox_log("velocity estimation : \n");
      rox_matrix_print ( A_est );

      rox_log("pose prediciton : \n");
      rox_matse3_print ( T_pre );
         
      rox_log("velocity prediciton : \n");
      rox_matrix_print ( A_pre );

      rox_log("=======================================================================================\n");
   }

   for ( Rox_Sint i = 1; i < 1000; i++ )
   {
      // Get new measure
      error = rox_array2d_double_scale ( Adt, A_grt, dt ); 
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_matse3_exponential_algse3 ( T_inc, Adt );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_matse3_mulmatmat ( T_new, T_old, T_inc );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      if ( save )
      {
      	// fprintf(pose_file, "%6.16f\t %6.16f\t %6.16f\t %6.16f\t %6.16f\t %6.16f\t %6.16f\t %6.16f\t %6.16f\t %6.16f\t %6.16f\t %6.16f\t \r\n", 
      	// R[0][0], R[1][0], R[2][0], 
      	// R[0][1], R[1][1], R[2][1], 
         // R[0][2], R[1][2], R[2][2], 
      	// t[0], t[1], t[2]); 
      }

      // Filter update with  new measure
      error = rox_filter_matse3_update ( filter_matse3, T_new );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      if (display)
      {
         rox_log("measure : \n");
         rox_matse3_print ( T_new );

         error = rox_filter_matse3_get_predicted_pose ( T_pre, filter_matse3 );
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

         error = rox_filter_matse3_get_estimated_pose ( T_est, filter_matse3 );
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

         error = rox_filter_matse3_get_predicted_velocity ( A_pre, filter_matse3 );
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

         error = rox_filter_matse3_get_estimated_velocity ( A_est, filter_matse3 );
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

         rox_log("pose estimation : \n");
         rox_matse3_print ( T_est );

         rox_log("velocity estimation : \n");
         rox_matrix_print ( A_est );
         
         rox_log("pose prediciton : \n");
         rox_matse3_print ( T_pre );

         rox_log("velocity prediciton : \n");
         rox_matrix_print ( A_pre );

         rox_log("=======================================================================================\n");
      }

      // Update old pose
      error = rox_matse3_copy ( T_old, T_new );
   }

   error = rox_array2d_double_difference_l2_norm ( &l2_error, A_grt, A_est );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error Ah-A_grt = %f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);


   error = rox_array2d_double_difference_l2_norm ( &l2_error, Th_grt, T_est );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error Th-Th_grt = %f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);
   
   error = rox_array2d_double_difference_l2_norm ( &l2_error, Tp_grt, T_pre );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error Tp-Tp_grt = %f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);

   error = rox_matse3_del ( &T_inc ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del ( &T_old ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del ( &T_new );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_filter_matse3_del ( &filter_matse3 ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   if (save)
   {
      // Close file 
      //rox_file_close(pose_file);
      //rox_file_close(meas_file);
      //rox_file_close(cali_file);
   }
}

ROX_TEST_SUITE_END()
