//==============================================================================
//
//    OPENROX   : File test_ellipse2d.cpp
//
//    Contents  : Tests for ellipse2d.c
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
   #include <baseproc/geometry/ellipse/ellipse3d.h>
   #include <baseproc/geometry/ellipse/ellipse2d.h>
   #include <baseproc/geometry/ellipse/ellipse_project.h>
   #include <baseproc/geometry/ellipse/ellipse_transform.h>
   #include <baseproc/geometry/transforms/transform_tools.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(ellipse2d)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_ellipse2d_new_del)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Ellipse2D ellipse2d = NULL;
   
   error = rox_ellipse2d_new(&ellipse2d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_ellipse2d_del(&ellipse2d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_ellipse2d_transform)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Double fu = 1000.0;
   Rox_Double fv = 1000.0;
   Rox_Double cu =  320.0;
   Rox_Double cv =  240.0;

   Rox_Array2D_Double met_K_pix = NULL;
   error = rox_array2d_double_new(&met_K_pix, 3 ,3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Ellipse2D ellipse2d_met = NULL;
   error = rox_ellipse2d_new(&ellipse2d_met);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Ellipse2D ellipse2d_pix = NULL;
   error = rox_ellipse2d_new(&ellipse2d_pix);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double xc = 0.2; 
   Rox_Double yc = 0.3;
   Rox_Double nxx = 1.6; 
   Rox_Double nyy = 2.1;
   Rox_Double nxy = 0.4;

   error = rox_ellipse2d_set(ellipse2d_met, xc, yc, nxx, nyy, nxy);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_calibration_matrix(met_K_pix, 1.0/fu, 1.0/fv, -cu/fu, -cv/fv);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ellipse2d_transform_pixels_to_meters(ellipse2d_met, met_K_pix, ellipse2d_pix);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_ellipse2d_print(ellipse2d_pix);
   rox_ellipse2d_print(ellipse2d_met);

   error = rox_ellipse2d_del(&ellipse2d_met);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ellipse2d_del(&ellipse2d_pix);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&met_K_pix);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_ellipse2d_project_ellipse3d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Double fu = 561.0;
   Rox_Double fv = 561.0;
   Rox_Double cu = 480.0;
   Rox_Double cv = 270.0;

   Rox_Double T_data[16] = {0.5060459599761992, 0.3489417934037362, -0.7887693650288095, 0.2158861750645141, 0.2999310684825601, 0.7862401702134700, 0.5402478587663292, 0.1992330274436169, 0.8086772165398730, -0.5099666847538945, 0.2932151767737223, 0.2600450567885215, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 1.0000000000000000};

   Rox_Array2D_Double pix_K_met = NULL;
   error = rox_array2d_double_new(&pix_K_met, 3 ,3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_MatSE3 T = NULL;
   error = rox_array2d_double_new(&T, 4 ,4);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_set_data(T, T_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Ellipse3D ellipse3d_met = NULL;
   error = rox_ellipse3d_new(&ellipse3d_met);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Ellipse2D ellipse2d_pix = NULL;
   error = rox_ellipse2d_new(&ellipse2d_pix);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double a = 0.272; 
   Rox_Double b = 0.272;

   error = rox_ellipse3d_set(ellipse3d_met, a, b, T);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_calibration_matrix(pix_K_met, fu, fv, cu, cv);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ellipse2d_project_ellipse3d(ellipse2d_pix, pix_K_met, ellipse3d_met);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE);

   // rox_ellipse3d_print ( ellipse3d_met );
   // rox_ellipse2d_print ( ellipse2d_pix );

   error = rox_ellipse3d_del(&ellipse3d_met);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ellipse2d_del(&ellipse2d_pix);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&pix_K_met);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&T);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_ellipse2d_get_normal_angle)
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double normal_angle = 0.0;
   Rox_Ellipse2D ellipse2d = NULL; 

   Rox_Double xe = 0.0; 
   Rox_Double ye = 0.0;

   Rox_Double xc = 100; 
   Rox_Double yc = 200;
   Rox_Double nxx = 0.0001; 
   Rox_Double nyy = 0.0004;
   Rox_Double nxy = 0.0;

   error = rox_ellipse2d_new(&ellipse2d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_ellipse2d_set(ellipse2d, xc, yc, nxx, nyy, nxy);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   xe = 0.0; 
   ye = 200.0;
   
   error = rox_ellipse2d_get_normal_angle ( &normal_angle, ellipse2d, xe, ye );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("normal angle at point (%f, %f) = %f \n", xe, ye, normal_angle);
   ROX_TEST_MESSAGE("normal at point (%f, %f) = [%f, %f] \n", xe, ye, cos(normal_angle), sin(normal_angle));

   xe = 100.0; 
   ye = 150.0;

   error = rox_ellipse2d_get_normal_angle ( &normal_angle, ellipse2d, xe, ye );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("normal angle at point (%f, %f) = %f \n", xe, ye, normal_angle);
   ROX_TEST_MESSAGE("normal at point (%f, %f) = [%f, %f] \n", xe, ye, cos(normal_angle), sin(normal_angle));

   xe = 200.0; 
   ye = 200.0;
   
   error = rox_ellipse2d_get_normal_angle ( &normal_angle, ellipse2d, xe, ye );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("normal angle at point (%f, %f) = %f \n", xe, ye, normal_angle);
   ROX_TEST_MESSAGE("normal at point (%f, %f) = [%f, %f] \n", xe, ye, cos(normal_angle), sin(normal_angle));

   xe = 100.0; 
   ye = 250.0;

   error = rox_ellipse2d_get_normal_angle ( &normal_angle, ellipse2d, xe, ye );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("normal angle at point (%f, %f) = %f \n", xe, ye, normal_angle);
   ROX_TEST_MESSAGE("normal at point (%f, %f) = [%f, %f] \n", xe, ye, cos(normal_angle), sin(normal_angle));

   error = rox_ellipse2d_del(&ellipse2d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
