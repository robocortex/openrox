//==============================================================================
//
//    OPENROX   : File test_line_from_points.cpp
//
//    Contents  : Tests for line_from_points.c
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
   #include <baseproc/geometry/line/line3d_struct.h>
   #include <baseproc/geometry/line/line_from_points.h>
   #include <baseproc/geometry/point/dynvec_point2d_tools.h>
   #include <inout/geometry/point/dynvec_point2d_print.h>   
   #include <baseproc/maths/maths_macros.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(line_from_points)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, rox_line2d_homogeneous_from_rho_theta)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;   
   Rox_Line2D_Homogeneous_Struct line2d_homogeneous;
   Rox_Double rho = 45.0 * ROX_PI / 180.0 ;
   Rox_Double theta = 45.0 * ROX_PI / 180.0 ;

   error = rox_line2d_homogeneous_from_rho_theta ( &line2d_homogeneous, rho, theta );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   ROX_TEST_CHECK_SMALL(line2d_homogeneous.a-0.7071067811865 , 1e-12);
   ROX_TEST_CHECK_SMALL(line2d_homogeneous.b-0.7071067811865 , 1e-12);
   ROX_TEST_CHECK_SMALL(line2d_homogeneous.c+45.0 * ROX_PI / 180.0, 1e-12);
   rox_line2d_parmetric_print(&line2d_homogeneous);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_line3d_planes_from_2_points)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;   
   Rox_Point2D_Double_Struct point2d_1, point2d_2;
   Rox_Line2D_Homogeneous_Struct line2d_homogeneous;
   
   point2d_1.u = 100.0;
   point2d_1.v = 100.0;
   
   point2d_2.u = 200.0;
   point2d_2.v = 200.0;
   
   error = rox_line2d_homogeneous_from_2_point2d(&line2d_homogeneous, &point2d_1, &point2d_2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   ROX_TEST_CHECK_SMALL(line2d_homogeneous.a+0.7071067811865 , 1e-12);
   ROX_TEST_CHECK_SMALL(line2d_homogeneous.b-0.7071067811865 , 1e-12);
   ROX_TEST_CHECK_SMALL(line2d_homogeneous.c                 , 1e-12);
   rox_line2d_parmetric_print(&line2d_homogeneous);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_line2d_homogeneous_from_n_points)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_Point2D_Double points2D = NULL;
   Rox_Point2D_Double_Struct point2D;
   Rox_Line2D_Homogeneous_Struct line2d_homogeneous;
   
   error = rox_dynvec_point2d_double_new(&points2D, 10);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   point2D.u = 100.0;
   point2D.v = 100.0;

   rox_dynvec_point2d_double_append(points2D, &point2D);
   
   point2D.u = 200.0;
   point2D.v = 200.0;

   rox_dynvec_point2d_double_append(points2D, &point2D);
   
   point2D.u = 300.0;
   point2D.v = 300.0;

   rox_dynvec_point2d_double_append(points2D, &point2D);
   
   error = rox_line2d_homogeneous_from_n_point2d(&line2d_homogeneous, points2D);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   ROX_TEST_CHECK_SMALL(line2d_homogeneous.a -0.7071067811865, 1e-12);
   ROX_TEST_CHECK_SMALL(line2d_homogeneous.b +0.7071067811865, 1e-12);
   ROX_TEST_CHECK_SMALL(line2d_homogeneous.c                 , 1e-12);
   
   rox_line2d_parmetric_print(&line2d_homogeneous);

   error = rox_dynvec_point2d_double_del(&points2D);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_line2d_homogeneous_from_n_point2d_ransac)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Line2D_Homogeneous_Struct line2d_homogeneous;
   Rox_Float distance_threshold = 1.0;
   Rox_DynVec_Point2D_Float points2D = NULL;
   Rox_Uint numb_points2D = 11;
   Rox_Float data_points2D[2*11] = {210.0000, 320.0000, 202.4000, 290.1000, 175.1910, 240.5910, 187.2000, 230.3000, 179.6000, 200.4000, 172.0000,  170.5000, 
  162.4230, 138.6230, 156.8000, 110.7000, 137.1215, 68.7215, 141.6000, 50.9000, 134.0000, 21.0000};

   // Check null pointer since points are not defined
   error = rox_line2d_homogeneous_from_n_point2d_ransac(&line2d_homogeneous, points2D, distance_threshold);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Allocate a vector with 10 points but empty
   rox_dynvec_point2d_float_new(&points2D,10);

   // Check algorithm failure since we need at least 2 points
   error = rox_line2d_homogeneous_from_n_point2d_ransac(&line2d_homogeneous, points2D, distance_threshold);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ALGORITHM_FAILURE);

   error = rox_dynvec_point2d_float_set_data(points2D, data_points2D, numb_points2D);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Check algorithm failure since we need at least 2 points
   error = rox_line2d_homogeneous_from_n_point2d_ransac(&line2d_homogeneous, points2D, distance_threshold);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_line2d_parmetric_print(&line2d_homogeneous);

   ROX_TEST_CHECK_SMALL(line2d_homogeneous.a -   0.9692, 1e-4);
   ROX_TEST_CHECK_SMALL(line2d_homogeneous.b +   0.2463, 1e-4);
   ROX_TEST_CHECK_SMALL(line2d_homogeneous.c + 124.6971, 1e-4);

   rox_dynvec_point2d_float_del(&points2D);
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_line3d_parametric_from_2_point3d )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Point3D_Double_Struct pt1 = { 1.0, 0.0, 0.0 };
   Rox_Point3D_Double_Struct pt2 = { 2.0, 0.0, 0.0 };

   Rox_Line3D_Parametric_Struct line3d_parametric;

   error = rox_line3d_parametric_from_2_point3d ( &line3d_parametric, &pt1, &pt2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("line3d_parametric = %f %f %f %f %f %f \n", line3d_parametric.origin[0], line3d_parametric.origin[1], line3d_parametric.origin[2], line3d_parametric.direction[0], line3d_parametric.direction[1], line3d_parametric.direction[2] );

}

ROX_TEST_SUITE_END()
