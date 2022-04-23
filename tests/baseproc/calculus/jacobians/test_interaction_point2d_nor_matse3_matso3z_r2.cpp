//==============================================================================
//
//    OPENROX   : File test_interaction_matse3_matso3z_r2_point2d_nor.cpp
//
//    Contents  : Tests for interaction_matse3_matso3z_r2_point2d_nor.c
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
   #include <iostream>

	#include <baseproc/maths/linalg/matrix.h>
	#include <baseproc/maths/linalg/matse3.h>
   #include <baseproc/geometry/point/point3d_tools.h>
   #include <baseproc/geometry/point/dynvec_point3d_tools.h>

	#include <baseproc/calculus/jacobians/interaction_point2d_nor_matse3_matso3z_r2.h>
	
	#include <inout/system/errors_print.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN(interaction_point2d_nor_matse3_matso3z_r2)

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_interaction_matse3_matso3z_r2_point2d_nor)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	Rox_Uint nb_points = 4;

   Rox_Double model_size_x = 0.025333; //1.0; 
   Rox_Double model_size_y = 0.025333; //1.0;

   Rox_Double cTo_data[16] = { 0.999999990082018,  0.000018737298023, -0.000139588244254, -0.156384858100393, 
                              -0.000018727314191,  0.999999997266774,  0.000071524405530, -0.078990632966740, 
                               0.000139589584047, -0.000071521790707,  0.999999987699691,  1.007845834185510, 
                               0.0              ,  0.0              ,  0.0              ,  1.0              };

   Rox_MatSE3 cTo = NULL; 

   Rox_Matrix Lo = NULL;
   Rox_Matrix Lb = NULL;

   Rox_DynVec_Point3D_Double  mo = NULL;
	
   error = rox_dynvec_point3d_double_new ( &mo, 4);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_append_rectangle( mo, model_size_x, model_size_y );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	error = rox_matse3_new ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	error = rox_matse3_set_data ( cTo, cTo_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_new ( &Lo, 2*nb_points, 6);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matrix_new ( &Lb, 2*nb_points, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	error = rox_interaction_matse3_matso3z_r2_point2d_nor ( Lo, Lb, cTo, mo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_del ( &mo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &Lo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matrix_del( &Lb );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()

