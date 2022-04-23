//==============================================================================
//
//    OPENROX   : File test_matsl3_from_n_points.cpp
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
   #include <baseproc/geometry/transforms/matsl3/sl3fromNpoints.h>
   #include <baseproc/geometry/transforms/matsl3/sl3from4points.h>
   #include <baseproc/array/determinant/detgl3.h>
   #include <baseproc/maths/maths_macros.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(matsl3_from_n_points)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

// Unitary test
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_unit_matsl3_from_n_points_double)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );
}

// Non-regression test
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_matsl3_from_n_points_double)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   // 2D points (to be) projected (on a plane)
   Rox_Point2D_Double_Struct pattern_2D[9];
   {
      pattern_2D[0].u = 480;
      pattern_2D[0].v = 270;
      pattern_2D[1].u = 960;
      pattern_2D[1].v = 270;
      pattern_2D[2].u = 1440;
      pattern_2D[2].v = 270;
      pattern_2D[3].u = 480;
      pattern_2D[3].v = 540;
      pattern_2D[4].u = 960;
      pattern_2D[4].v = 540;
      pattern_2D[5].u = 1440;
      pattern_2D[5].v = 540;
      pattern_2D[6].u = 480;
      pattern_2D[6].v = 810;
      pattern_2D[7].u = 960;
      pattern_2D[7].v = 810;
      pattern_2D[8].u = 1440;
      pattern_2D[8].v = 810;
   };

   // 3D corresponding points with Z (==0) sliced
   Rox_Point2D_Double_Struct model_3D_sliced[2][9];
   { // Output points from Matlab
      model_3D_sliced[0][0].u = -0.1226055550293407;
      model_3D_sliced[0][0].v = -0.0558683443571411;
      model_3D_sliced[0][1].u = 0.3015161790693420;
      model_3D_sliced[0][1].v = 0.0625135394068960;
      model_3D_sliced[0][2].u = 0.841872812939966;
      model_3D_sliced[0][2].v = 0.213339189400854;
      model_3D_sliced[0][3].u = -0.1949580471196706;
      model_3D_sliced[0][3].v = 0.1766368608059075;
      model_3D_sliced[0][4].u = 0.2003256352474276;
      model_3D_sliced[0][4].v = 0.3159488506459699;
      model_3D_sliced[0][5].u = 0.6980146092359011;
      model_3D_sliced[0][5].v = 0.4913520974296383;
      model_3D_sliced[0][6].u = -0.2612851097084895;
      model_3D_sliced[0][6].v = 0.3897793089723444;
      model_3D_sliced[0][7].u = 0.1084838814486275;
      model_3D_sliced[0][7].v = 0.5459697852078469;
      model_3D_sliced[0][8].u = 0.5690794848711626;
      model_3D_sliced[0][8].v = 0.7405254358193833;

      model_3D_sliced[1][0].u = -0.1288305950305249;
      model_3D_sliced[1][0].v = -0.1086143557233240;
      model_3D_sliced[1][1].u = 0.3390683578856951;
      model_3D_sliced[1][1].v = 0.0269586350515828;
      model_3D_sliced[1][2].u = 0.914622777316211;
      model_3D_sliced[1][2].v = 0.193724629097058;
      model_3D_sliced[1][3].u = -0.2082463390843094;
      model_3D_sliced[1][3].v = 0.1512671364940747;
      model_3D_sliced[1][4].u = 0.2294000852126299;
      model_3D_sliced[1][4].v = 0.3058606022709359;
      model_3D_sliced[1][5].u = 0.7623832499371064;
      model_3D_sliced[1][5].v = 0.4941306533009239;
      model_3D_sliced[1][6].u = -0.2810127156993972;
      model_3D_sliced[1][6].v = 0.3893891205325921;
      model_3D_sliced[1][7].u = 0.1297747912083994;
      model_3D_sliced[1][7].v = 0.5592218507463163;
      model_3D_sliced[1][8].u = 0.6255269893327712;
      model_3D_sliced[1][8].v = 0.7641817093133279;
   }
   //{ // Output points from rox_example_camera_projector_calibration
      // // As reconstructed after camera calibration and observed in it
      //model_3D_sliced[0][0].u = -0.122605555029341;
      //model_3D_sliced[0][0].v = -0.055868344357141;
      //model_3D_sliced[0][1].u = 0.301516179069342 ;
      //model_3D_sliced[0][1].v = 0.062513539406896 ;
      //model_3D_sliced[0][2].u = 0.791250728327788 ;
      //model_3D_sliced[0][2].v = 0.199209430761657 ;
      //model_3D_sliced[0][3].u = -0.194958047119670;
      //model_3D_sliced[0][3].v = 0.176636860805907 ;
      //model_3D_sliced[0][4].u = 0.200325635247428 ;
      //model_3D_sliced[0][4].v = 0.315948850645970 ;
      //model_3D_sliced[0][5].u = 0.698014609235901 ;
      //model_3D_sliced[0][5].v = 0.491352097429638 ;
      //model_3D_sliced[0][6].u = -0.261285109708489;
      //model_3D_sliced[0][6].v = 0.389779308972343 ;
      //model_3D_sliced[0][7].u = 0.108483881448628 ;
      //model_3D_sliced[0][7].v = 0.545969785207846 ;
      //model_3D_sliced[0][8].u = 0.569079484871161 ;
      //model_3D_sliced[0][8].v = 0.740525435819381 ;

      //model_3D_sliced[1][0].u = -0.128830595030525;
      //model_3D_sliced[1][0].v = -0.108614355723324;
      //model_3D_sliced[1][1].u = 0.339068357885695 ;
      //model_3D_sliced[1][1].v = 0.026958635051583 ;
      //model_3D_sliced[1][2].u = 0.861650033348037 ;
      //model_3D_sliced[1][2].v = 0.178375859272807 ;
      //model_3D_sliced[1][3].u = -0.208246339084309;
      //model_3D_sliced[1][3].v = 0.151267136494074 ;
      //model_3D_sliced[1][4].u = 0.229400085212630 ;
      //model_3D_sliced[1][4].v = 0.305860602270936 ;
      //model_3D_sliced[1][5].u = 0.762383249937107 ;
      //model_3D_sliced[1][5].v = 0.494130653300925 ;
      //model_3D_sliced[1][6].u = -0.281012715699397;
      //model_3D_sliced[1][6].v = 0.389389120532592 ;
      //model_3D_sliced[1][7].u = 0.129774791208400 ;
      //model_3D_sliced[1][7].v = 0.559221850746316 ;
      //model_3D_sliced[1][8].u = 0.625526989332771 ;
      //model_3D_sliced[1][8].v = 0.764181709313328 ;
   //}

   Rox_Array2D_Double G_1_ref, G_2_ref, G_1_comp, G_2_comp;

   error = rox_array2d_double_new(&G_1_ref, 3, 3); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&G_2_ref, 3, 3); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&G_1_comp, 3, 3); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_new(&G_2_comp, 3, 3); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   // Reference homographies (computed with Matlab/Octave)
   Rox_Double **dG1r = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dG1r, G_1_ref);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   Rox_Double **dG2r = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dG2r, G_2_ref);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   {
      dG1r[0][0] =  1.20639281192216e+01; dG1r[0][1] =  2.93143653054230e+00; dG1r[0][2] =  6.04534793457032e+00;
      dG1r[1][0] = -1.85018597825220e+00; dG1r[1][1] =  9.14961025710094e+00; dG1r[1][2] =  2.76071921250416e+00;
      dG1r[2][0] =  2.83809239657193e-03; dG1r[2][1] = -8.30773015376068e-04; dG1r[2][2] =  9.47336203503737e-03;

      dG2r[0][0] =  1.14591814374489e+01; dG2r[0][1] =  2.70340066598252e+00; dG2r[0][2] =  6.51835491797310e+00;
      dG2r[1][0] = -1.95561144603113e+00; dG2r[1][1] =  8.78199323685547e+00; dG2r[1][2] =  3.37290186983664e+00;
      dG2r[2][0] =  2.44661058123053e-03; dG2r[2][1] = -9.15576847210228e-04; dG2r[2][2] =  1.01083235768341e-02;
   }

   // Computing homographies with openrox
   error = rox_matsl3_from_n_points_double( G_1_comp, model_3D_sliced[0], pattern_2D, 9 ); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );
   
   error = rox_matsl3_from_n_points_double( G_2_comp, model_3D_sliced[1], pattern_2D, 9 ); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   // Equaling determinant to 1
   Rox_Double det1, det2;
   error = rox_array2d_double_detgl3( &det1, G_1_comp ); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_detgl3( &det2, G_2_comp ); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   Rox_Double **dG1c = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dG1c, G_1_comp);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   Rox_Double **dG2c = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dG2c, G_2_comp);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   Rox_Double reg1, reg2;
   {
      reg1 = pow( det1, 1.0/3.0 );
      dG1c[0][0] /= reg1;
      dG1c[0][1] /= reg1;
      dG1c[0][2] /= reg1;
      dG1c[1][0] /= reg1;
      dG1c[1][1] /= reg1;
      dG1c[1][2] /= reg1;
      dG1c[2][0] /= reg1;
      dG1c[2][1] /= reg1;
      dG1c[2][2] /= reg1;
   }
   {
      reg2 = pow( det2, 1.0/3.0 );
      dG2c[0][0] /= reg2;
      dG2c[0][1] /= reg2;
      dG2c[0][2] /= reg2;
      dG2c[1][0] /= reg2;
      dG2c[1][1] /= reg2;
      dG2c[1][2] /= reg2;
      dG2c[2][0] /= reg2;
      dG2c[2][1] /= reg2;
      dG2c[2][2] /= reg2;
   }

   // Compute differences element-wise between homography matrices
   Rox_Double diff1=0.0, diff2=0.0;

   diff1 = fabs( dG1r[0][0] - dG1c[0][0] ) +
      fabs( dG1r[0][1] - dG1c[0][1] ) +
      fabs( dG1r[0][2] - dG1c[0][2] ) +
      fabs( dG1r[1][0] - dG1c[1][0] ) +
      fabs( dG1r[1][1] - dG1c[1][1] ) +
      fabs( dG1r[1][2] - dG1c[1][2] ) +
      fabs( dG1r[2][0] - dG1c[2][0] ) +
      fabs( dG1r[2][1] - dG1c[2][1] ) +
      fabs( dG1r[2][2] - dG1c[2][2] );

   diff2 = fabs( dG2r[0][0] - dG2c[0][0] ) +
      fabs( dG2r[0][1] - dG2c[0][1] ) +
      fabs( dG2r[0][2] - dG2c[0][2] ) +
      fabs( dG2r[1][0] - dG2c[1][0] ) +
      fabs( dG2r[1][1] - dG2c[1][1] ) +
      fabs( dG2r[1][2] - dG2c[1][2] ) +
      fabs( dG2r[2][0] - dG2c[2][0] ) +
      fabs( dG2r[2][1] - dG2c[2][1] ) +
      fabs( dG2r[2][2] - dG2c[2][2] );

   // Ouputs
   ROX_TEST_MESSAGE("===== G1 computed: \n %f %f %f \n %f %f %f \n %f %f %f \n", 
      dG1c[0][0], dG1c[0][1], dG1c[0][2], 
      dG1c[1][0], dG1c[1][1], dG1c[1][2], 
      dG1c[2][0], dG1c[2][1], dG1c[2][2] );

   ROX_TEST_MESSAGE("===== G1 reference: \n %f %f %f \n %f %f %f \n %f %f %f \n", 
      dG1r[0][0], dG1r[0][1], dG1r[0][2], 
      dG1r[1][0], dG1r[1][1], dG1r[1][2], 
      dG1r[2][0], dG1r[2][1], dG1r[2][2] );

   ROX_TEST_MESSAGE("===== G2 computed: \n %f %f %f \n %f %f %f \n %f %f %f \n", 
      dG2c[0][0], dG2c[0][1], dG2c[0][2], 
      dG2c[1][0], dG2c[1][1], dG2c[1][2], 
      dG2c[2][0], dG2c[2][1], dG2c[2][2] );

   ROX_TEST_MESSAGE("===== G2 reference: \n %f %f %f \n %f %f %f \n %f %f %f \n", 
      dG2r[0][0], dG2r[0][1], dG2r[0][2], 
      dG2r[1][0], dG2r[1][1], dG2r[1][2], 
      dG2r[2][0], dG2r[2][1], dG2r[2][2] );

   ROX_TEST_MESSAGE("differences: %f %f \n", diff1, diff2 );

   // TESTS
   ROX_TEST_CHECK_INFERIOR( diff1, 1e-02 );
   ROX_TEST_CHECK_INFERIOR( diff2, 1e-02 );


   error = rox_array2d_double_del(&G_1_ref); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&G_2_ref); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&G_1_comp); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&G_2_comp); 
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

// Non-regression test
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_matsl3_from_n_points3d_to_points2d_double)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   // 2D points (to be) projected (on a plane)
   Rox_Point2D_Double_Struct pattern_2D[9];
   {
      pattern_2D[0].u = 480;
      pattern_2D[0].v = 270;
      pattern_2D[1].u = 960;
      pattern_2D[1].v = 270;
      pattern_2D[2].u = 1440;
      pattern_2D[2].v = 270;
      pattern_2D[3].u = 480;
      pattern_2D[3].v = 540;
      pattern_2D[4].u = 960;
      pattern_2D[4].v = 540;
      pattern_2D[5].u = 1440;
      pattern_2D[5].v = 540;
      pattern_2D[6].u = 480;
      pattern_2D[6].v = 810;
      pattern_2D[7].u = 960;
      pattern_2D[7].v = 810;
      pattern_2D[8].u = 1440;
      pattern_2D[8].v = 810;
   };

   // 3D corresponding points with Z (==0) 
   Rox_Point3D_Double_Struct model_3D[2][9];
   { // Output points from Matlab
      model_3D[0][0].X = -0.1226055550293407; model_3D[0][0].Y = -0.0558683443571411; model_3D[0][0].Z = 0.0;
      model_3D[0][1].X =  0.3015161790693420; model_3D[0][1].Y =  0.0625135394068960; model_3D[0][1].Z = 0.0;
      model_3D[0][2].X =  0.8418728129399660; model_3D[0][2].Y =  0.2133391894008540; model_3D[0][2].Z = 0.0;
      model_3D[0][3].X = -0.1949580471196706; model_3D[0][3].Y =  0.1766368608059075; model_3D[0][3].Z = 0.0;
      model_3D[0][4].X =  0.2003256352474276; model_3D[0][4].Y =  0.3159488506459699; model_3D[0][4].Z = 0.0;
      model_3D[0][5].X =  0.6980146092359011; model_3D[0][5].Y =  0.4913520974296383; model_3D[0][5].Z = 0.0;
      model_3D[0][6].X = -0.2612851097084895; model_3D[0][6].Y =  0.3897793089723444; model_3D[0][6].Z = 0.0;
      model_3D[0][7].X =  0.1084838814486275; model_3D[0][7].Y =  0.5459697852078469; model_3D[0][7].Z = 0.0;
      model_3D[0][8].X =  0.5690794848711626; model_3D[0][8].Y =  0.7405254358193833; model_3D[0][8].Z = 0.0;

      model_3D[1][0].X = -0.1288305950305249; model_3D[1][0].Y = -0.1086143557233240; model_3D[1][0].Z = 0.0;
      model_3D[1][1].X =  0.3390683578856951; model_3D[1][1].Y =  0.0269586350515828; model_3D[1][1].Z = 0.0;
      model_3D[1][2].X =  0.9146227773162110; model_3D[1][2].Y =  0.1937246290970580; model_3D[1][2].Z = 0.0;
      model_3D[1][3].X = -0.2082463390843094; model_3D[1][3].Y =  0.1512671364940747; model_3D[1][3].Z = 0.0;
      model_3D[1][4].X =  0.2294000852126299; model_3D[1][4].Y =  0.3058606022709359; model_3D[1][4].Z = 0.0;
      model_3D[1][5].X =  0.7623832499371064; model_3D[1][5].Y =  0.4941306533009239; model_3D[1][5].Z = 0.0;
      model_3D[1][6].X = -0.2810127156993972; model_3D[1][6].Y =  0.3893891205325921; model_3D[1][6].Z = 0.0;
      model_3D[1][7].X =  0.1297747912083994; model_3D[1][7].Y =  0.5592218507463163; model_3D[1][7].Z = 0.0;
      model_3D[1][8].X =  0.6255269893327712; model_3D[1][8].Y =  0.7641817093133279; model_3D[1][8].Z = 0.0;
   }

   Rox_Array2D_Double G_1_ref, G_2_ref, G_1_comp, G_2_comp;

   error = rox_array2d_double_new(&G_1_ref, 3, 3); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );
   error = rox_array2d_double_new(&G_2_ref, 3, 3); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );
   error = rox_array2d_double_new(&G_1_comp, 3, 3); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );
   error = rox_array2d_double_new(&G_2_comp, 3, 3); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   // Reference homographies (computed with Matlab/Octave)
   Rox_Double **dG1r = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dG1r, G_1_ref);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   Rox_Double **dG2r = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dG2r, G_2_ref);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );


   {
      dG1r[0][0] =  1.20639281192216e+01; dG1r[0][1] =  2.93143653054230e+00; dG1r[0][2] =  6.04534793457032e+00;
      dG1r[1][0] = -1.85018597825220e+00; dG1r[1][1] =  9.14961025710094e+00; dG1r[1][2] =  2.76071921250416e+00;
      dG1r[2][0] =  2.83809239657193e-03; dG1r[2][1] = -8.30773015376068e-04; dG1r[2][2] =  9.47336203503737e-03;

      dG2r[0][0] =  1.14591814374489e+01; dG2r[0][1] =  2.70340066598252e+00; dG2r[0][2] =  6.51835491797310e+00;
      dG2r[1][0] = -1.95561144603113e+00; dG2r[1][1] =  8.78199323685547e+00; dG2r[1][2] =  3.37290186983664e+00;
      dG2r[2][0] =  2.44661058123053e-03; dG2r[2][1] = -9.15576847210228e-04; dG2r[2][2] =  1.01083235768341e-02;
   }

   // Computing homographies with openrox
   error = rox_matsl3_from_n_points3d_to_points2d_double( G_1_comp, model_3D[0], pattern_2D, 9 ); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );
   
   error = rox_matsl3_from_n_points3d_to_points2d_double( G_2_comp, model_3D[1], pattern_2D, 9 ); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   // Equaling determinant to 1
   Rox_Double det1, det2;
   error = rox_array2d_double_detgl3( &det1, G_1_comp ); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_detgl3( &det2, G_2_comp ); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   Rox_Double **dG1c = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dG1c, G_1_comp);
   
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );
   Rox_Double **dG2c = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dG2c, G_2_comp);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   Rox_Double reg1, reg2;
   {
      reg1 = pow( det1, 1.0/3.0 );
      dG1c[0][0] /= reg1;
      dG1c[0][1] /= reg1;
      dG1c[0][2] /= reg1;
      dG1c[1][0] /= reg1;
      dG1c[1][1] /= reg1;
      dG1c[1][2] /= reg1;
      dG1c[2][0] /= reg1;
      dG1c[2][1] /= reg1;
      dG1c[2][2] /= reg1;
   }
   {
      reg2 = pow( det2, 1.0/3.0 );
      dG2c[0][0] /= reg2;
      dG2c[0][1] /= reg2;
      dG2c[0][2] /= reg2;
      dG2c[1][0] /= reg2;
      dG2c[1][1] /= reg2;
      dG2c[1][2] /= reg2;
      dG2c[2][0] /= reg2;
      dG2c[2][1] /= reg2;
      dG2c[2][2] /= reg2;
   }

   // Compute differences element-wise between homography matrices
   Rox_Double diff1=0.0, diff2=0.0;

   diff1 = fabs( dG1r[0][0] - dG1c[0][0] ) +
      fabs( dG1r[0][1] - dG1c[0][1] ) +
      fabs( dG1r[0][2] - dG1c[0][2] ) +
      fabs( dG1r[1][0] - dG1c[1][0] ) +
      fabs( dG1r[1][1] - dG1c[1][1] ) +
      fabs( dG1r[1][2] - dG1c[1][2] ) +
      fabs( dG1r[2][0] - dG1c[2][0] ) +
      fabs( dG1r[2][1] - dG1c[2][1] ) +
      fabs( dG1r[2][2] - dG1c[2][2] );

   diff2 = fabs( dG2r[0][0] - dG2c[0][0] ) +
      fabs( dG2r[0][1] - dG2c[0][1] ) +
      fabs( dG2r[0][2] - dG2c[0][2] ) +
      fabs( dG2r[1][0] - dG2c[1][0] ) +
      fabs( dG2r[1][1] - dG2c[1][1] ) +
      fabs( dG2r[1][2] - dG2c[1][2] ) +
      fabs( dG2r[2][0] - dG2c[2][0] ) +
      fabs( dG2r[2][1] - dG2c[2][1] ) +
      fabs( dG2r[2][2] - dG2c[2][2] );

   // Ouputs
   ROX_TEST_MESSAGE("===== G1 computed: \n %f %f %f \n %f %f %f \n %f %f %f \n", 
      dG1c[0][0], dG1c[0][1], dG1c[0][2], 
      dG1c[1][0], dG1c[1][1], dG1c[1][2], 
      dG1c[2][0], dG1c[2][1], dG1c[2][2] );

   ROX_TEST_MESSAGE("===== G1 reference: \n %f %f %f \n %f %f %f \n %f %f %f \n", 
      dG1r[0][0], dG1r[0][1], dG1r[0][2], 
      dG1r[1][0], dG1r[1][1], dG1r[1][2], 
      dG1r[2][0], dG1r[2][1], dG1r[2][2] );

   ROX_TEST_MESSAGE("===== G2 computed: \n %f %f %f \n %f %f %f \n %f %f %f \n", 
      dG2c[0][0], dG2c[0][1], dG2c[0][2], 
      dG2c[1][0], dG2c[1][1], dG2c[1][2], 
      dG2c[2][0], dG2c[2][1], dG2c[2][2] );

   ROX_TEST_MESSAGE("===== G2 reference: \n %f %f %f \n %f %f %f \n %f %f %f \n", 
      dG2r[0][0], dG2r[0][1], dG2r[0][2], 
      dG2r[1][0], dG2r[1][1], dG2r[1][2], 
      dG2r[2][0], dG2r[2][1], dG2r[2][2] );

   ROX_TEST_MESSAGE("differences: %f %f \n", diff1, diff2 );

   // TESTS
   ROX_TEST_CHECK_INFERIOR( diff1, 1e-02 );
   ROX_TEST_CHECK_INFERIOR( diff2, 1e-02 );

   error = rox_array2d_double_del(&G_1_ref); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );
   error = rox_array2d_double_del(&G_2_ref); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );
   error = rox_array2d_double_del(&G_1_comp); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );
   error = rox_array2d_double_del(&G_2_comp); 
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
