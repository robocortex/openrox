//==============================================================================
//
//    OPENROX   : File test_interaction_row_point_to_line_matse3.cpp
//
//    Contents  : Tests for interaction_row_point_to_line_matse3.c
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
	#include <baseproc/calculus/jacobians/interaction_row_point_to_line_matse3.h>
   #include <baseproc/geometry/line/line3d_struct.h>
   #include <baseproc/geometry/line/line2d_struct.h>
   #include <baseproc/geometry/line/line_from_points.h>
   #include <baseproc/geometry/line/line_project.h>
   #include <inout/geometry/line/line2d_print.h>
   #include <inout/geometry/line/line3d_print.h>
   #include <baseproc/array/error/l2_error.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN ( interaction_row_point_to_line_matse3 )

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================


ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_interaction_row_point_to_line_matse3 )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double l2_error = 0.0;
   Rox_Double L_row_grt_data[6] = { 0.0, -1.0, 0.0, 1.0, 0.0, 0.0 };

   Rox_Matrix L_row = NULL;
   Rox_Matrix L_row_grt = NULL;

   error = rox_matrix_new ( &L_row, 1, 6 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_new ( &L_row_grt, 1, 6 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( L_row_grt, L_row_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Line3D_Planes_Struct line3D; 
   Rox_Point3D_Double_Struct pt1;
   Rox_Point3D_Double_Struct pt2;

   pt1.X = 0.0;
   pt1.Y = 0.0;
   pt1.Z = 1.0;

   pt2.X = 1.0;
   pt2.Y = 0.0;
   pt2.Z = 1.0;

   error =  rox_line3d_planes_from_2_point3d ( &line3D, &pt1, &pt2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_line3d_planes_print(&line3D);

   Rox_Line2D_Normal_Struct line2D; 

   error = rox_line3d_planes_project_meters ( &line2D, &line3D );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   rox_line2d_normal_print(&line2D);

   Rox_Double x = 0.0; 
   Rox_Double y = 1.0;

   error = rox_interaction_row_point_to_line_matse3 ( L_row, &line3D, &line2D, x, y );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matrix_print ( L_row );

   error = rox_array2d_double_difference_l2_norm ( &l2_error, L_row_grt, L_row );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error = %f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);

   error = rox_matrix_del ( &L_row );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matrix_del ( &L_row_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
