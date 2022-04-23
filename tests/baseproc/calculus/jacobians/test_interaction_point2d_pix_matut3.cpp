//==============================================================================
//
//    OPENROX   : File test_interaction_matut3_point2d_pix.cpp
//
//    Contents  : Tests for interaction_matut3_point2d_pix.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== INCLUDED HEADERS   =====================================================

#include <openrox_tests.hpp>

extern "C"
{
	#include <baseproc/calculus/jacobians/interaction_point2d_pix_matut3.h>
   #include <baseproc/geometry/point/point2d_struct.h>
   #include <baseproc/maths/linalg/matrix.h>
   #include <baseproc/array/error/l2_error.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN ( interaction_point2d_pix_matut3 )

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================


ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_interaction_matut3_point2d_pix )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double l2_error = 0.0;

   Rox_Double L_grt_data[2*3*5] = { 120.0, 140.0, 1.0,   0.0, 0.0,                                                     
                                      0.0,   0.0, 0.0, 140.0, 1.0,                                                       
                                     20.0,  40.0, 1.0,   0.0, 0.0,                                                       
                                      0.0,   0.0, 0.0,  40.0, 1.0,                                                        
                                    220.0, 140.0, 1.0,   0.0, 0.0,                                                     
                                      0.0,   0.0, 0.0, 140.0, 1.0 };

   Rox_Matrix L = NULL;
   Rox_Matrix L_grt = NULL;
   Rox_Point2D_Double_Struct p[3]; 
   Rox_Double cu = 320; 
   Rox_Double cv = 240;
   Rox_Uint nbp = 3; 
   Rox_Uint ddl = 5;
   
   p[0].u = 120.0; p[0].v = 140.0;
   p[1].u =  20.0; p[1].v =  40.0;
   p[2].u = 220.0; p[2].v = 140.0;

   error = rox_matrix_new ( &L, 2*nbp, ddl );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matrix_new ( &L_grt, 2*nbp, ddl );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( L_grt, L_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // test_interaction_matut3_point2d_pix
   error =  rox_jacobian_points_2d_campar ( L, p, cu, cv, nbp, ddl );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matrix_print(L);
   
   error = rox_array2d_double_difference_l2_norm ( &l2_error, L_grt, L );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error = %f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);

   error = rox_matrix_del ( &L );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &L_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_interaction_fu_point2d_pix )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double l2_error = 0.0;

   Rox_Double L_grt_data[2*3*1] = {-200.0, -100.0, -300.0, -200.0, -100.0, -100.0};
   Rox_Matrix L = NULL;
   Rox_Matrix L_grt = NULL;
   Rox_Point2D_Double_Struct p[3]; 
   Rox_Double cu = 320; 
   Rox_Double cv = 240;
   Rox_Sint nbp = 3;
   Rox_Sint ddl = 1;

   p[0].u = 120.0; p[0].v = 140.0;
   p[1].u =  20.0; p[1].v =  40.0;
   p[2].u = 220.0; p[2].v = 140.0;

   error = rox_matrix_new ( &L, 2*nbp, ddl );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matrix_new ( &L_grt, 2*nbp, ddl );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( L_grt, L_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // test_interaction_fu_point2d_pix
   error = rox_jacobian_points_2d_campar_fu ( L, p, cu, cv, nbp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matrix_print(L);
   
   error = rox_array2d_double_difference_l2_norm ( &l2_error, L_grt, L );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error = %f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);

   error = rox_matrix_del ( &L );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &L_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_interaction_fu_fv_point2d_pix )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double l2_error = 0.0;
   Rox_Double L_grt_data[2*3*2] = {-200.0,    0.0,                                                                                                            
                                      0.0, -100.0,                                                                                                            
                                   -300.0,    0.0,                                                                                                            
                                      0.0, -200.0,                                                                                                            
                                   -100.0,    0.0,                                                                                                            
                                      0.0, -100.0 };

   Rox_Matrix L = NULL;
   Rox_Matrix L_grt = NULL;
   Rox_Point2D_Double_Struct p[3]; 
   Rox_Double cu = 320; 
   Rox_Double cv = 240;
   Rox_Uint nbp = 3;
   Rox_Sint ddl = 2;

   p[0].u = 120.0; p[0].v = 140.0;
   p[1].u =  20.0; p[1].v =  40.0;
   p[2].u = 220.0; p[2].v = 140.0;

   error = rox_matrix_new ( &L, 2*nbp, ddl );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matrix_new ( &L_grt, 2*nbp, ddl );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( L_grt, L_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // test_interaction_fu_fv_point2d_pix
   error = rox_jacobian_points_2d_campar_fu_fv ( L, p, cu, cv, nbp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matrix_print(L);
   
   error = rox_array2d_double_difference_l2_norm ( &l2_error, L_grt, L );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error = %f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);

   error = rox_matrix_del ( &L );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &L_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_interaction_fu_cu_cv_point2d_pix )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double l2_error = 0.0;
   Rox_Double L_grt_data[2*3*3] = { 120.0, 1.0, 0.0,                      
                                    140.0, 0.0, 1.0,                                                                                           
                                     20.0, 1.0, 0.0,                                                                                            
                                     40.0, 0.0, 1.0,                                                                                            
                                    220.0, 1.0, 0.0,                                                                                           
                                    140.0, 0.0, 1.0 };

   Rox_Matrix L = NULL;
   Rox_Matrix L_grt = NULL;
   Rox_Point2D_Double_Struct p[3]; 
   Rox_Uint nbp = 3;
   Rox_Sint ddl = 3;

   p[0].u = 120.0; p[0].v = 140.0;
   p[1].u =  20.0; p[1].v =  40.0;
   p[2].u = 220.0; p[2].v = 140.0;

   error = rox_matrix_new ( &L, 2*nbp, ddl );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_new ( &L_grt, 2*nbp, ddl );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( L_grt, L_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // test_interaction_fu_cu_cv_point2d_pix
   error = rox_jacobian_points_2d_campar_fu_cu_cv ( L, p, nbp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matrix_print(L);
   
   error = rox_array2d_double_difference_l2_norm ( &l2_error, L_grt, L );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error = %f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);

   error = rox_matrix_del ( &L );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &L_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_interaction_fu_fv_cu_cv_point2d_pix )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double l2_error = 0.0;
   Rox_Double L_grt_data[2*3*4] = {120.0, 1.0, 0.0, 0.0,                                                                         
                                    0.0, 0.0, 140.0, 1.0,                                                                         
                                    20.0, 1.0, 0.0, 0.0,                                                                          
                                    0.0, 0.0, 40.0, 1.0,                                                                          
                                    220.0, 1.0, 0.0, 0.0,                                                                         
                                    0.0, 0.0, 140.0, 1.0};

   Rox_Matrix L = NULL;
   Rox_Matrix L_grt = NULL;
   Rox_Point2D_Double_Struct p[3]; 
   Rox_Uint nbp = 3;
   Rox_Sint ddl = 4;

   p[0].u = 120.0; p[0].v = 140.0;
   p[1].u =  20.0; p[1].v =  40.0;
   p[2].u = 220.0; p[2].v = 140.0;

   error = rox_matrix_new ( &L, 2*nbp, ddl );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matrix_new ( &L_grt, 2*nbp, ddl );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( L_grt, L_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // test_interaction_fu_fv_cu_cv_point2d_pix
   error = rox_jacobian_points_2d_campar_fu_fv_cu_cv ( L, p, nbp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matrix_print(L);
   
   error = rox_array2d_double_difference_l2_norm ( &l2_error, L_grt, L );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error = %f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);

   error = rox_matrix_del ( &L );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &L_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_interaction_fu_fv_cu_cv_s_point2d_pix )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double l2_error = 0.0;
   Rox_Double L_grt_data[2*3*5] = { 120.0, 140.0, 1.0,   0.0, 0.0,                                                     
                                      0.0,   0.0, 0.0, 140.0, 1.0,                                                       
                                     20.0,  40.0, 1.0,   0.0, 0.0,                                              
                                      0.0,   0.0, 0.0,  40.0, 1.0,               
                                    220.0, 140.0, 1.0,   0.0, 0.0,                                                     
                                      0.0,   0.0, 0.0, 140.0, 1.0};

   Rox_Matrix L = NULL; 
   Rox_Matrix L_grt = NULL;
   Rox_Point2D_Double_Struct p[3]; 
   Rox_Sint nbp = 3;
   Rox_Sint ddl = 5;

   p[0].u = 120.0; p[0].v = 140.0;
   p[1].u =  20.0; p[1].v =  40.0;
   p[2].u = 220.0; p[2].v = 140.0;

   error = rox_matrix_new ( &L, 2*nbp, ddl );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matrix_new ( &L_grt, 2*nbp, ddl );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( L_grt, L_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // test_interaction_fu_fv_cu_cv_s_point2d_pix
   error = rox_jacobian_points_2d_campar_fu_fv_cu_cv_s ( L, p, nbp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );


   rox_matrix_print(L);
   
   error = rox_array2d_double_difference_l2_norm ( &l2_error, L_grt, L );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error = %f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);

   error = rox_matrix_del ( &L );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matrix_del ( &L_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_campar_ddl )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
