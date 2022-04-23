//==============================================================================
//
//    OPENROX   : File test_interaction_matse3_point2d_pix.cpp
//
//    Contents  : Tests for interaction_matse3_point2d_pix.c
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
	#include <baseproc/calculus/jacobians/interaction_point2d_pix_matse3.h>
   #include <baseproc/geometry/point/point2d_struct.h>
   #include <baseproc/maths/linalg/matrix.h>
   #include <baseproc/maths/linalg/matut3.h>
   #include <baseproc/array/error/l2_error.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN ( interaction_point2d_pix_matse3 )

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================


ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_interaction_matse3_point2d_pix )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double l2_error = 0.0;
   Rox_Double L_grt_data[2*3*6] = {-1000.0,    -0.0, -200.0,   20.0, -1040.0, -100.0,                                                                  
                                  0.0, -1000.0, -100.0, 1010.0,   -20.0,  200.0,                                                                 
                              -1000.0,    -0.0, -300.0,   60.0, -1090.0, -200.0,                                                                  
                                  0.0, -1000.0, -200.0, 1040.0,   -60.0,  300.0, 
                              -1000.0,    -0.0, -100.0,   10.0, -1010.0, -100.0, 
                                  0.0, -1000.0, -100.0, 1010.0,   -10.0,  100.0};

   Rox_Matrix L = NULL;
   Rox_Matrix L_grt = NULL;
   Rox_Sint nbp = 3;
   Rox_Point2D_Double_Struct p[3]; 
   Rox_Double z[3] = {1.0, 1.0, 1.0};
   Rox_MatUT3 K = NULL; 

   p[0].u = 120.0; p[0].v = 140.0;
   p[1].u =  20.0; p[1].v =  40.0;
   p[2].u = 220.0; p[2].v = 140.0;

   error = rox_matrix_new ( &L, 2*nbp, 6 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_new ( &L_grt, 2*nbp, 6 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( L_grt, L_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_new ( &K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_build_calibration_matrix ( K, 1000.0, 1000.0, 320.0, 240.0 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_interaction_matse3_point2d_pix ( L, p, z, K, nbp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_difference_l2_norm ( &l2_error, L_grt, L );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error = %f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);

   error = rox_matrix_del ( &L );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matrix_del ( &L_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
