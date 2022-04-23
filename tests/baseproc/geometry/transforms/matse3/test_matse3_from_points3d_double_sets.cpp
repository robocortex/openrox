//==============================================================================
//
//    OPENROX   : File test_matse3_from_points3d_double_sets.cpp
//
//    Contents  : Tests for matse3_from_points3d_double_sets.c
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

ROX_TEST_SUITE_BEGIN ( matse3_from_points3d_double_sets )

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

// Unitary test
ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_unit_matse3_from_points3d_double_sets )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_matse3_from_n_points3d_to_points2d_double ( NULL, NULL, NULL, NULL, 0 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NULL_POINTER );

   ROX_TEST_MESSAGE("This test is not completed yet !!! \n");
}

// Non-regression test
ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_matse3_from_points3d_double_sets )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint nb_used = 0;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   // The projected 2D points  (on the image plane)
   // Rox_Point3D_Double_Struct m1[4];
   // {  // Output points from Matlab
   //    m1[0].X = -0.05;
   //    m1[0].Y = -0.05;
   //    m1[0].Z =  0.00;

   //    m1[1].X =  0.05;
   //    m1[1].Y = -0.05;
   //    m1[1].Z =  0.00;  

   //    m1[2].X =  0.05;
   //    m1[2].Y =  0.05;
   //    m1[2].Z =  0.00;  

   //    m1[3].X = -0.05;
   //    m1[3].Y =  0.05;
   //    m1[3].Z =  0.00;  
   // }

   // 3D corresponding points with 
   // Rox_Point3D_Double_Struct m2[4];
   // {  // Output points from Matlab
   //    m2[0].X = -0.05;
   //    m2[0].Y = -0.05;
   //    m2[0].Z =  0.00;

   //    m2[1].X =  0.05;
   //    m2[1].Y = -0.05;
   //    m2[1].Z =  0.00;  

   //    m2[2].X =  0.05;
   //    m2[2].Y =  0.05;
   //    m2[2].Z =  0.00;  

   //    m2[3].X = -0.05;
   //    m2[3].Y =  0.05;
   //    m2[3].Z =  0.00;  
   // }

   // Rox_Double T_grt[16] = {0.5, -0.866025403784439, 0, 0, 0.866025403784439, 0.5, 0, 0, 0, 0, 1, 4, 0, 0, 0, 1};
   
   Rox_ObjSet_Array2D_Double T = NULL;
   Rox_ObjSet_Array2D_Double z = NULL;

   error = rox_objset_array2d_double_new(&T, 1);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   error = rox_objset_array2d_double_new(&z, 1);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   //error = rox_matse3_from_n_points3d_to_points2d_double ( T, z, q, m, 4 );
   //ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

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
ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_perf_matse3_from_points3d_double_sets )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
