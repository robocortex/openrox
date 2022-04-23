//==============================================================================
//
//    OPENROX   : File test_matse3_from_n_points.cpp
//
//    Contents  : Tests for matsl3_from_n_points.c
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
   #include <baseproc/geometry/point/point2d_struct.h>
   #include <baseproc/geometry/point/point3d_struct.h>
   #include <baseproc/geometry/transforms/matse3/matse3_from_n_points3d_to_points2d.h>
   #include <baseproc/maths/maths_macros.h>
   #include <baseproc/maths/linalg/matse3.h>
   #include <inout/numeric/objset_array2d_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(matse3_from_n_points)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

// Unitary test
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_unit_matse3_from_n_points_double)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_matse3_from_n_points3d_to_points2d_double(NULL, NULL, NULL, NULL, 0);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NULL_POINTER );

   ROX_TEST_MESSAGE("This test is not completed yet !!! \n");
}

// Non-regression test
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_matse3_from_n_points_double)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint nb_used = 0;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   // The projected 2D points  (on the image plane)
   Rox_Point2D_Double_Struct q[4];
   {
      q[0].u =  0.045753175473055; 
      q[0].v = -0.170753175473055;
      q[1].u =  0.170753175473055;
      q[1].v =  0.045753175473055;
      q[2].u = -0.045753175473055;
      q[2].v =  0.170753175473055;
      q[3].u = -0.170753175473055;
      q[3].v = -0.045753175473055;
   };

   // 3D corresponding points with 
   Rox_Point3D_Double_Struct m[4];
   { // Output points from Matlab
      m[0].X = -0.5;
      m[0].Y = -0.5;
      m[0].Z =  0.0;

      m[1].X =  0.5;
      m[1].Y = -0.5;
      m[1].Z =  0.0;  

      m[2].X =  0.5;
      m[2].Y =  0.5;
      m[2].Z =  0.0;  

      m[3].X = -0.5;
      m[3].Y =  0.5;
      m[3].Z =  0.0;  
   }

   // Rox_Double T_grt[16] = {0.5, -0.866025403784439, 0, 0, 0.866025403784439, 0.5, 0, 0, 0, 0, 1, 4, 0, 0, 0, 1};
   
   Rox_ObjSet_Array2D_Double T = NULL;
   Rox_ObjSet_Array2D_Double z = NULL;

   error = rox_objset_array2d_double_new(&T, 1);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   error = rox_objset_array2d_double_new(&z, 1);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   error = rox_matse3_from_n_points3d_to_points2d_double(T, z, q, m, 4);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   error = rox_objset_array2d_double_get_used(&nb_used, T);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("Found %d poses \n", nb_used);

   error = rox_objset_array2d_double_print(T);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   error = rox_objset_array2d_double_print(z);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   error = rox_objset_array2d_double_del(&T);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   error = rox_objset_array2d_double_del(&z);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );
}

// Performance test
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_perf_matsl3_from_n_points_double)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_matsl3_from_n_points_float)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
