//==============================================================================
//
//    OPENROX   : File test_ellipse_from_points.cpp
//
//    Contents  : Tests for ellipse_from_points.c
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
   #include <baseproc/geometry/ellipse/ellipse2d_struct.h>
   #include <baseproc/geometry/ellipse/ellipse_from_points.h>
   #include <baseproc/geometry/point/dynvec_point2d_tools.h>
   #include <inout/geometry/point/dynvec_point2d_print.h>   
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(ellipse_from_points)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_ellipse2d_parametric_from_5_points)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_Point2D_Double points2D = NULL;
   Rox_Ellipse2D_Parametric_Struct ellipse2d_parametric;
   
   const Rox_Sint  numb_points2D = 5;

   Rox_Double data_points2D[2*numb_points2D] = {1.242809041582063, 1.171404520791032, -0.081154495664743, 2.190669032077521, -0.878375474867699,  1.149879607009784, -0.047121599312792, -0.512628144100030, 1.263842528263171, -0.499325015778308};

   error = rox_dynvec_point2d_double_new(&points2D, numb_points2D);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point2d_double_set_data(points2D, data_points2D, numb_points2D);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ellipse2d_parametric_from_n_point2d(&ellipse2d_parametric, points2D);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   ROX_TEST_CHECK_SMALL(ellipse2d_parametric.xc - 0.3, 1e-12);
   ROX_TEST_CHECK_SMALL(ellipse2d_parametric.yc - 0.7, 1e-12);
   ROX_TEST_CHECK_SMALL(ellipse2d_parametric.nxx - 0.8, 1e-12);
   ROX_TEST_CHECK_SMALL(ellipse2d_parametric.nyy - 0.5, 1e-12);
   ROX_TEST_CHECK_SMALL(ellipse2d_parametric.nxy - 0.2, 1e-12);

   //rox_ellipse2d_print(&ellipse2d_parametric);

   error = rox_dynvec_point2d_double_del(&points2D);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_ellipse2d_parametric_from_n_points_need_normalize)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_Point2D_Double points2D = NULL;
   Rox_Ellipse2D_Parametric_Struct ellipse2d_parametric;

   const Rox_Sint numb_points2D = 36;
   Rox_Double data_points2D[2 * numb_points2D] = { 1279.0000000000000 ,442.00000000000000
   , 1283.0000000000000 ,443.00000000000000
   , 1288.0000000000000 ,445.00000000000000
   , 1292.0000000000000 ,447.00000000000000
   , 1295.0000000000000 ,449.00000000000000
   , 1298.0000000000000 ,452.00000000000000
   , 1300.0000000000000 ,456.00000000000000
   , 1302.0000000000000 ,459.00000000000000
   , 1302.0000000000000 ,462.00000000000000
   , 1302.0000000000000 ,466.00000000000000
   , 1300.0000000000000 ,468.00000000000000
   , 1299.0000000000000 ,471.00000000000000
   , 1296.0000000000000 ,474.00000000000000
   , 1293.0000000000000 ,476.00000000000000
   , 1290.0000000000000 ,477.00000000000000
   , 1286.0000000000000 ,478.00000000000000
   , 1281.0000000000000 ,479.00000000000000
   , 1277.0000000000000 ,479.00000000000000
   , 1272.0000000000000 ,479.00000000000000
   , 1267.0000000000000 ,478.00000000000000
   , 1263.0000000000000 ,477.00000000000000
   , 1259.0000000000000 ,475.00000000000000
   , 1256.0000000000000 ,472.00000000000000
   , 1253.0000000000000 ,469.00000000000000
   , 1250.0000000000000 ,466.00000000000000
   , 1250.0000000000000 ,462.00000000000000
   , 1247.0000000000000 ,459.00000000000000
   , 1247.0000000000000 ,455.00000000000000
   , 1248.0000000000000 ,452.00000000000000
   , 1251.0000000000000 ,449.00000000000000
   , 1253.0000000000000 ,446.00000000000000
   , 1257.0000000000000 ,445.00000000000000
   , 1261.0000000000000 ,443.00000000000000
   , 1265.0000000000000 ,442.00000000000000
   , 1269.0000000000000 ,441.00000000000000
   , 1274.0000000000000 ,441.00000000000000 };

   ellipse2d_parametric.xc = 0.0;
   ellipse2d_parametric.yc = 0.0;
   ellipse2d_parametric.nxx = 1.0;
   ellipse2d_parametric.nyy = 1.0;
   ellipse2d_parametric.nxy = 0.0;

   error = rox_dynvec_point2d_double_new(&points2D, numb_points2D);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point2d_double_set_data(points2D, data_points2D, numb_points2D);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ellipse2d_parametric_from_n_point2d(&ellipse2d_parametric, points2D);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Compare results with Matlab  
   ROX_TEST_CHECK_SMALL(ellipse2d_parametric.xc - 1274.7, 1e-1);
   ROX_TEST_CHECK_SMALL(ellipse2d_parametric.yc - 460.21, 1e-1);
   ROX_TEST_CHECK_SMALL(ellipse2d_parametric.nxx - 0.0013634, 1e-4);
   ROX_TEST_CHECK_SMALL(ellipse2d_parametric.nyy - 0.0027881, 1e-4);
   ROX_TEST_CHECK_SMALL(ellipse2d_parametric.nxy + 2.2890e-004, 1e-4);

   //rox_ellipse2d_print(&ellipse2d_parametric);

   error = rox_dynvec_point2d_double_del(&points2D);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_ellipse2d_parametric_from_n_points)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_Point2D_Double points2D = NULL;
   Rox_Ellipse2D_Parametric_Struct ellipse2d_parametric;
   
   const Rox_Sint numb_points2D = 86;
   Rox_Double data_points2D[2*numb_points2D] = {787,   105,   800,   106,   813,   107,   826,   110,   839,   113,   851,   116,   863,   119,   875,   123,   886,   127,   897,   131,   907,   135,   916,   140,   925,   145,   932,   151,   939,   157,   945,   163,   950,   169,   954,   175,   958,   181,   960,   187,   962,   193,   962,   199,  961,   205,   960,   211,   958,   217,   957,   224,   959,   236,   954,   243,   945,   246,   937,   251,   924,   247,   915,   251,   906,   255,   896,   258,   886,   261,   875,   264,   863,   266,   851,   268,   839,   269,   826,   270,   813,   270,   800,   270,   787,   270,   773,   269,   760,   268,   747,   266,   734,   264,   721,   261,   709,   258,   697,   255,   685,   251,   674,   247,   664,   243,   654,   238,  645,   233,   636,   228,   628,   223,   622,   217,   615,   211,   610,   205,   606,   199,   603,   193,   600,   187,   600,   181,   598,   175,   599,   169,   601,   163,   605,   158,   605,   151,   611,   147,   616,   142,   622,   137,   629,   132,   637,   128,   645,   124,   655,   121,   664,   116,   674,   113,   686,   110,   697,   108,   709,   106,   722,   105,   734,   104,   747,   104,   760,   104,   774,   104};


   ellipse2d_parametric.xc = 0.0;
   ellipse2d_parametric.yc = 0.0;
   ellipse2d_parametric.nxx = 1.0;
   ellipse2d_parametric.nyy = 1.0;
   ellipse2d_parametric.nxy = 0.0;

   error = rox_dynvec_point2d_double_new(&points2D, numb_points2D);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point2d_double_set_data(points2D, data_points2D, numb_points2D);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ellipse2d_parametric_from_n_point2d(&ellipse2d_parametric, points2D);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Compare results with Matlab   
   ROX_TEST_CHECK_SMALL(ellipse2d_parametric.xc  - 7.819878901059075e+02, 1e-1);
   ROX_TEST_CHECK_SMALL(ellipse2d_parametric.yc  - 1.874959785739387e+02, 1e-1);
   ROX_TEST_CHECK_SMALL(ellipse2d_parametric.nxx - 3.046865109453060e-05, 1e-1);
   ROX_TEST_CHECK_SMALL(ellipse2d_parametric.nyy - 1.466349388485158e-04, 1e-1);
   ROX_TEST_CHECK_SMALL(ellipse2d_parametric.nxy + 1.042798153960074e-05, 1e-1);

   //rox_ellipse2d_print(&ellipse2d_parametric);

   error = rox_dynvec_point2d_double_del(&points2D);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

 //ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_line2d_homogeneous_from_n_point2d_ransac)
 //{
 //   Rox_ErrorCode error = ROX_ERROR_NONE;
 //   Rox_Line2D_Homogeneous_Struct line2d_homogeneous;
 //   Rox_Float distance_threshold = 1.0;
 //   Rox_DynVec_Point2D_Float points2D = NULL;
 //   Rox_Uint numb_points2D = 11;
 //   Rox_Float data_points2D[2*11] = {210.0000, 320.0000, 202.4000, 290.1000, 175.1910, 240.5910, 187.2000, 230.3000, 179.6000, 200.4000, 172.0000,  170.5000, 
 //  162.4230, 138.6230, 156.8000, 110.7000, 137.1215, 68.7215, 141.6000, 50.9000, 134.0000, 21.0000};

 //   // Check null pointer since points are not defined
 //   error = rox_line2d_homogeneous_from_n_point2d_ransac(&line2d_homogeneous, points2D, distance_threshold);
 //   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

 //   // Allocate a vector with 10 points but empty
 //   rox_dynvec_point2d_float_new(&points2D,10);

 //   // Check algorithm failure since we need at least 2 points
 //   error = rox_line2d_homogeneous_from_n_point2d_ransac(&line2d_homogeneous, points2D, distance_threshold);
 //   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ALGORITHM_FAILURE);

 //   error = rox_dynvec_point2d_float_set_data(points2D, data_points2D, numb_points2D);
 //   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

 //   // Check algorithm failure since we need at least 2 points
 //   error = rox_line2d_homogeneous_from_n_point2d_ransac(&line2d_homogeneous, points2D, distance_threshold);
 //   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

 //   rox_line2d_parmetric_print(line2d_homogeneous);

 //   ROX_TEST_CHECK_SMALL(line2d_homogeneous.a -   0.9692, 1e-4);
 //   ROX_TEST_CHECK_SMALL(line2d_homogeneous.b +   0.2463, 1e-4);
 //   ROX_TEST_CHECK_SMALL(line2d_homogeneous.c + 124.6971, 1e-4);
 //}

ROX_TEST_SUITE_END()
