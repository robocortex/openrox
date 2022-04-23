//==============================================================================
//
//    OPENROX   : File test_point3d_tools.cpp
//
//    Contents  : Tests for points3d_tools.c
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
   #include <generated/dynvec_point3d_double.h>
   #include <baseproc/geometry/point/dynvec_point3d_tools.h>
	 #include <baseproc/geometry/point/point3d_tools.h>
   #include <inout/geometry/point/point3d_print.h>
   #include <inout/geometry/point/dynvec_point3d_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN ( point3_tools )

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_points3d_float_compute_bounding_box )
{
	 Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Float_Struct bounding_box[8];

   Rox_Float points_data[3*10] = {  0.4364, -0.5044,  0.1021, 
                                    1.1963,  0.1203, -1.0368, 
                                   -0.8571, -0.1699, -0.1917, 
                                   -0.8658,  0.1807,  1.2665,   
                                   -0.2512, -0.2046, -2.2015,
                                   -0.7745, -1.3933, -0.3862,
                                    0.5256,  1.5233,  1.7985,
                                   -0.1169, -0.3202,  0.8175,
                                    0.4902,  0.7653,  0.7783,
                                   -1.4803,  0.5404, -0.0915 };

   Rox_Sint nb_points = 10;
 
   error = rox_point3d_float_compute_bounding_box ( bounding_box, points_data, nb_points );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_point3d_float_vector_print ( bounding_box, 8 );

}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_vector_points3d_double_mean )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Double_Struct mean;
   Rox_Point3D_Double_Struct points[10];

   Rox_Double points_data[3*10] = { 0.4364, -0.5044,  0.1021, 
                                    1.1963,  0.1203, -1.0368, 
                                   -0.8571, -0.1699, -0.1917, 
                                   -0.8658,  0.1807,  1.2665,   
                                   -0.2512, -0.2046, -2.2015,
                                   -0.7745, -1.3933, -0.3862,
                                    0.5256,  1.5233,  1.7985,
                                   -0.1169, -0.3202,  0.8175,
                                    0.4902,  0.7653,  0.7783,
                                   -1.4803,  0.5404, -0.0915 };
   Rox_Sint nb_points = 10;

   error = rox_vector_point3d_double_set_data (points, points_data, nb_points);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_vector_point3d_double_mean ( &mean, points, nb_points );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_CHECK_CLOSE ( mean.X, -0.16973, 1e-16 );
   ROX_TEST_CHECK_CLOSE ( mean.Y,  0.05376, 1e-16 );
   ROX_TEST_CHECK_CLOSE ( mean.Z,  0.08552, 1e-16 );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_dynvec_points3d_double_mean )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Double_Struct mean;
   Rox_DynVec_Point3D_Double points;

   Rox_Double points_data[3*10] = { 0.4364, -0.5044,  0.1021, 
                                    1.1963,  0.1203, -1.0368, 
                                   -0.8571, -0.1699, -0.1917, 
                                   -0.8658,  0.1807,  1.2665,   
                                   -0.2512, -0.2046, -2.2015,
                                   -0.7745, -1.3933, -0.3862,
                                    0.5256,  1.5233,  1.7985,
                                   -0.1169, -0.3202,  0.8175,
                                    0.4902,  0.7653,  0.7783,
                                   -1.4803,  0.5404, -0.0915 };
   Rox_Sint nb_points = 10;

   error = rox_dynvec_point3d_double_new (&points, nb_points);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_set_data (points, points_data, nb_points);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_mean ( &mean, points );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_CHECK_CLOSE ( mean.X, -0.16973, 1e-16 );
   ROX_TEST_CHECK_CLOSE ( mean.Y,  0.05376, 1e-16 );
   ROX_TEST_CHECK_CLOSE ( mean.Z,  0.08552, 1e-16 );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_vector_points3d_double_shift )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Double_Struct shift = {1,2,3};
   Rox_Point3D_Double_Struct points[10];

   Rox_Double points_data[3*10] = { 0.4364, -0.5044,  0.1021, 
                                    1.1963,  0.1203, -1.0368, 
                                   -0.8571, -0.1699, -0.1917, 
                                   -0.8658,  0.1807,  1.2665,   
                                   -0.2512, -0.2046, -2.2015,
                                   -0.7745, -1.3933, -0.3862,
                                    0.5256,  1.5233,  1.7985,
                                   -0.1169, -0.3202,  0.8175,
                                    0.4902,  0.7653,  0.7783,
                                   -1.4803,  0.5404, -0.0915 };
   Rox_Sint nb_points = 10;

   error = rox_vector_point3d_double_set_data (points, points_data, nb_points);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_vector_point3d_double_shift ( points, nb_points, &shift );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_vector_point3d_double_print ( points, nb_points );
}


ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_dynvec_points3d_double_shift )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Double_Struct shift = {1,2,3};
   Rox_DynVec_Point3D_Double points;

   Rox_Double points_data[3*10] = { 0.4364, -0.5044,  0.1021, 
                                    1.1963,  0.1203, -1.0368, 
                                   -0.8571, -0.1699, -0.1917, 
                                   -0.8658,  0.1807,  1.2665,   
                                   -0.2512, -0.2046, -2.2015,
                                   -0.7745, -1.3933, -0.3862,
                                    0.5256,  1.5233,  1.7985,
                                   -0.1169, -0.3202,  0.8175,
                                    0.4902,  0.7653,  0.7783,
                                   -1.4803,  0.5404, -0.0915 };
   Rox_Sint nb_points = 10;

   error = rox_dynvec_point3d_double_new ( &points, nb_points );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_set_data ( points, points_data, nb_points );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_shift ( points, &shift );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_print ( points );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_vector_point3d_double_normalize_unit )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Double_Struct points[10];

   Rox_Double points_data[3*10] = { 0.4364, -0.5044,  0.1021, 
                                    1.1963,  0.1203, -1.0368, 
                                   -0.8571, -0.1699, -0.1917, 
                                   -0.8658,  0.1807,  1.2665,   
                                   -0.2512, -0.2046, -2.2015,
                                   -0.7745, -1.3933, -0.3862,
                                    0.5256,  1.5233,  1.7985,
                                   -0.1169, -0.3202,  0.8175,
                                    0.4902,  0.7653,  0.7783,
                                   -1.4803,  0.5404, -0.0915 };
   Rox_Sint nb_points = 10;

   error = rox_vector_point3d_double_set_data (points, points_data, nb_points);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_vector_point3d_double_normalize_unit ( points, nb_points );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_vector_point3d_double_print ( points, nb_points );
}


ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_dynvec_point3d_double_normalize_unit )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_Point3D_Double points;

   Rox_Double points_data[3*10] = { 0.4364, -0.5044,  0.1021, 
                                    1.1963,  0.1203, -1.0368, 
                                   -0.8571, -0.1699, -0.1917, 
                                   -0.8658,  0.1807,  1.2665,   
                                   -0.2512, -0.2046, -2.2015,
                                   -0.7745, -1.3933, -0.3862,
                                    0.5256,  1.5233,  1.7985,
                                   -0.1169, -0.3202,  0.8175,
                                    0.4902,  0.7653,  0.7783,
                                   -1.4803,  0.5404, -0.0915 };
   Rox_Sint nb_points = 10;

   error = rox_dynvec_point3d_double_new ( &points, nb_points );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_set_data ( points, points_data, nb_points );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_normalize_unit ( points );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_print ( points );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}


ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_vector_point3d_double_center_normalize )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Double_Struct points[10];
   Rox_Point3D_Double_Struct points_center_normal[10];

   Rox_Double points_data[3*10] = { 0.4364, -0.5044,  0.1021, 
                                    1.1963,  0.1203, -1.0368, 
                                   -0.8571, -0.1699, -0.1917, 
                                   -0.8658,  0.1807,  1.2665,   
                                   -0.2512, -0.2046, -2.2015,
                                   -0.7745, -1.3933, -0.3862,
                                    0.5256,  1.5233,  1.7985,
                                   -0.1169, -0.3202,  0.8175,
                                    0.4902,  0.7653,  0.7783,
                                   -1.4803,  0.5404, -0.0915 };
   Rox_Sint nb_points = 10;

   error = rox_vector_point3d_double_set_data (points, points_data, nb_points);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_vector_point3d_double_center_normalize ( points_center_normal, points, nb_points );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_vector_point3d_double_print ( points_center_normal, nb_points );
}


ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_dynvec_point3d_double_center_normalize )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_Point3D_Double points;
   Rox_DynVec_Point3D_Double points_center_normal;

   Rox_Double points_data[3*10] = { 0.4364, -0.5044,  0.1021, 
                                    1.1963,  0.1203, -1.0368, 
                                   -0.8571, -0.1699, -0.1917, 
                                   -0.8658,  0.1807,  1.2665,   
                                   -0.2512, -0.2046, -2.2015,
                                   -0.7745, -1.3933, -0.3862,
                                    0.5256,  1.5233,  1.7985,
                                   -0.1169, -0.3202,  0.8175,
                                    0.4902,  0.7653,  0.7783,
                                   -1.4803,  0.5404, -0.0915 };
   Rox_Sint nb_points = 10;

   error = rox_dynvec_point3d_double_new ( &points_center_normal, nb_points );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_new ( &points, nb_points );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_set_data ( points, points_data, nb_points );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_center_normalize ( points_center_normal, points );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_print ( points_center_normal );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}


ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_vector_point3d_double_check_visibility )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Double_Struct mo[4];
   Rox_Sint visibility = 0; 
   Rox_Sint image_rows = 1080;
   Rox_Sint image_cols = 1920;
   Rox_Double fu = 1200;
   Rox_Double fv = 1200;
   Rox_Double cu = (1920-1)/2;
   Rox_Double cv = (1080-1)/2;
   Rox_MatUT3 Kc = NULL; 
   Rox_MatSE3 cTo = NULL;

   Rox_Sint nbp = 4;

   Rox_Double tra[3] = {0.0, 0.0, 1.0};

   Rox_Double mo_data[3*4] = { -0.5, -0.5, 0.0, 
                                0.5, -0.5, 0.0, 
                                0.5,  0.5, 0.0, 
                               -0.5,  0.5, 0.0 };

   error = rox_vector_point3d_double_set_data ( mo, mo_data, nbp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_new ( &Kc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matut3_build_calibration_matrix ( Kc, fu, fv, cu, cv );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_new ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_set_translation ( cTo, tra );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_vector_point3d_double_check_visibility ( &visibility, image_rows, image_cols, Kc, cTo, mo, nbp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("Visibility = %d \n", visibility);

}


ROX_TEST_SUITE_END()
