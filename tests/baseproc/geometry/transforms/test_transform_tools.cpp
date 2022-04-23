//==============================================================================
//
//    OPENROX   : File test_transform_tools.cpp
//
//    Contents  : Tests for transform_tools.c
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
   #include <inout/numeric/array2d_print.h>
   #include <inout/geometry/point/point2d_print.h>
   #include <inout/geometry/point/point3d_print.h>
   #include <baseproc/geometry/point/point2d_struct.h>
	#include <baseproc/geometry/transforms/transform_tools.h>
   #include <baseproc/geometry/rectangle/rectangle.h>
   #include <baseproc/geometry/transforms/matsl3/sl3from4points.h>
   #include <inout/system/print.h>   
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(plane_transform)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_decompose_projection)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_basischange)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_basischangeinv)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_precalibright)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_pose_precalibrate)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_rotationmatrix_from_axisangle)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_rotationmatrix_from_euler_tait_bryan_x_y_z)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_rotationmatrix_from_quaternion)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_axisangle_from_rotationmatrix)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_skew_from_vector)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_rotationmatrix_from_cailey)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_logse3_from_pose)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_axisangle_translation_from_pose)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_quaternion_from_rotationmatrix)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_pose_from_axisangle_translation)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_pose_from_sphere)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_pose_inverse)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double pose = NULL;
   Rox_Point3D_Double_Struct vec;
   Rox_Double rotation = 10;

   vec.X = 0.1;
   vec.Y = 0.2;
   vec.Z = 0.3;

   error = rox_array2d_double_new(&pose, 4, 4);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_pose_from_sphere(pose, &vec, rotation);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // rox_array2d_double_print(pose);

   error = rox_array2d_double_del(&pose);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_pose_relative)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_build_calibration_matrix)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_build_calibration_matrix_for_template)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double calibmatrix = NULL;
   Rox_Double width_pixels = 128;
   Rox_Double height_pixels = 128;
   Rox_Double width_meters = 0.2;
   Rox_Double height_meters = 0.2;
   Rox_Double val = 0.0, val22 = 0.0;
   Rox_Point3D_Double_Struct m[4];
   Rox_Point2D_Double_Struct p[4];

   error = rox_array2d_double_new ( &calibmatrix, 3, 3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_calibration_matrix_for_template ( calibmatrix, width_pixels, height_pixels, width_meters, height_meters);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_array2d_double_print(calibmatrix);

   error = rox_rectangle3d_create_centered_plane_xright_ydown ( m, width_meters, height_meters );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_rectangle2d_create_image_coordinates ( p, width_pixels, height_pixels );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_vector_point3d_double_print ( m, 4 );
   rox_vector_point2d_double_print ( p, 4 );

   // Compute calibmatrix such that pp ~ calibmatrix * pm
   error = rox_transformtools_build_model_to_image_homography_from_4_points ( calibmatrix, m, p );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_array2d_double_print(calibmatrix);
   error = rox_array2d_double_get_value(&val22, calibmatrix, 2, 2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_get_value(&val, calibmatrix, 0, 0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(val/val22, width_pixels/width_meters, 1e-12);

   error = rox_array2d_double_get_value(&val, calibmatrix, 1, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(val/val22, height_pixels/height_meters, 1e-12);
   
   error = rox_array2d_double_get_value(&val, calibmatrix, 0, 2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(val/val22, (width_pixels-1.0)/2.0, 1e-12);
   
   error = rox_array2d_double_get_value(&val, calibmatrix, 1, 2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(val/val22, (height_pixels-1.0)/2.0, 1e-12);

   error = rox_array2d_double_del(&calibmatrix);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_pose_random)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_build_homography)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double pose_data[16] = { 0.935754803277919, -0.283164960565074,  0.210191705950743, 0.1,
                                0.302932713402637,  0.950580617906091, -0.068031316404940, 0.2,
                               -0.180540076694398,  0.127334574917630,  0.975290308953046, 1.3,
                                0.0,                0.0,                0.0,               1.0};

   // Rox_Double pose_data[16] = { 1.0, 0.0, 0.0, 0.0,
   //                              0.0, 1.0, 0.0, 0.0,
   //                              0.0, 0.0, 1.0, 1.0,
   //                              0.0, 0.0, 0.0, 1.0};

   Rox_MatSL3 homography = NULL;
   Rox_MatSE3 pose = NULL;
   Rox_MatSE3 pose_p1 = NULL;
   Rox_MatUT3 calib_output = NULL; 
   Rox_MatUT3 calib_input = NULL;
   Rox_Double a =   0.0; 
   Rox_Double b =   0.0;
   Rox_Double c =  -1.0;
   Rox_Double d =   1.0;

   Rox_Double fu = 128.0;
   Rox_Double fv = 128.0;
   Rox_Double cu = (1920.0-1.0)/2.0;
   Rox_Double cv = (1080.0-1.0)/2.0;

   Rox_Double sizeu = 128.0;
   Rox_Double sizev = 128.0;

   Rox_Double sizex =   0.2;
   Rox_Double sizey =   0.2;

   //Rox_Double tra[3] = {0.0,0.0,0.0};

   error = rox_matsl3_new ( &homography );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_new ( &pose_p1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_new ( &pose);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_set_data ( pose, pose_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_updateZref ( pose_p1, pose, -1.0 ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // rox_matse3_print(pose_p1);
   // rox_matse3_print(pose);

   error = rox_matut3_new ( &calib_output );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_build_calibration_matrix ( calib_output, fu, fv, cu, cv );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_new ( &calib_input );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_calibration_matrix_for_template ( calib_input, sizeu, sizev, sizex, sizey );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_homography ( homography, pose_p1, calib_output, calib_input, a, b, c, d );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // rox_matut3_print ( calib_input );

   rox_log("[a, b, c, d] = [%f, %f, %f, %f]\n",a, b, c, d);

   rox_matse3_print ( pose );

   rox_matut3_print ( calib_output );

   rox_matsl3_print ( homography );

   // A different way to compute the homography

   Rox_MatSE3 c_T_o = pose;
   Rox_MatUT3 Kc = calib_output;
   Rox_MatSL3 c_G_o = NULL;
   Rox_MatSL3 t_G_o = calib_input;
   Rox_MatSL3 c_G_t = homography;

   //error = rox_matse3_set_translation ( c_T_o, tra );
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_new ( &c_G_o );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_model_to_image_homography ( c_G_o, Kc, c_T_o );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      
   error = rox_matsl3_mulmatinv ( c_G_t, c_G_o, t_G_o );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // rox_matsl3_print ( homography );
   
   error = rox_matsl3_del ( &c_G_o );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del ( &homography );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del ( &pose_p1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del ( &pose);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_del ( &calib_output );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_del ( &calib_input );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_build_homography_intermodel)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, rox_transformtools_build_model_to_image_homography )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSL3 H = NULL;
   Rox_MatUT3 K = NULL;
   Rox_MatSE3 T = NULL;

   Rox_Double K_data[9]  = {1280, 10, 650, 0, 970, 360, 0, 0, 1};
   Rox_Double T_data[16] = {0.588338340765308, -0.626680773723587,  0.511008027951353, 0.224450673936319, 
                           0.756178383248855,  0.202530480258419, -0.622234407015107, 0.141996201434450, 
                           0.286447638308769,  0.752497582993793,  0.593038900997083, 1.076985448193790,
                                           0,                  0,                  0, 1.000000000000000};

   error = rox_matse3_new ( & T );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_set_data(T, T_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_new ( & K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matsl3_set_data(K, K_data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_new ( & H );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_transformtools_build_model_to_image_homography ( H, K, T );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // rox_matsl3_print( H );

   // Expected result

   // 939.2640410802947  -313.0279614202251   987.3374039644522
   // 836.6141815425463   467.3536957284322   525.4510767411806
   //   0.2864476383088     0.7524975829938     1.0769854481938

   error = rox_matse3_del ( & T );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_del ( & K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matsl3_del ( & H );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_build_pose_intermodel)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSE3 pose_grt = NULL; // ground truth pose
   Rox_MatSE3 pose_est = NULL; // estimated pose
   Rox_MatSL3 homography = NULL;
   Rox_MatUT3 calibration = NULL;

   Rox_Double T[16] = { 1.0,  0.0,  0.0, 0.0, 
                        0.0, -1.0,  0.0, 0.0, 
                        0.0,  0.0, -1.0, 1.0, 
                        0.0,  0.0,  0.0, 1.0};

   Rox_Double H[ 9] = { 128.0,    0.0, 958.5, 
                          0.0, -128.0, 539.5, 
                          0.0,    0.0,   1.0};
                          
   error = rox_matse3_new ( &pose_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_set_data( pose_grt, T );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_new ( &pose_est );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_new ( &calibration );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_new ( &homography );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_set_data( homography, H );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_calibration_matrix ( calibration, 128.0, 128.0, 958.5, 539.5 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_pose_intermodel ( pose_est, homography, calibration );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double err_tra;
   Rox_Double err_rot;

   error = rox_matse3_distance ( &err_tra, &err_rot, pose_grt, pose_est );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );


   rox_log("pose extracted from model to image homography \n");
   rox_matse3_print(pose_est);
   rox_log("translation error = %0.12f \n", err_tra);
   rox_log("rotation error = %0.12f \n", err_rot);

   ROX_TEST_CHECK_CLOSE (err_tra, 0.0, 1e-12);
   ROX_TEST_CHECK_CLOSE (err_rot, 0.0, 1e-12);

   error = rox_matse3_del ( &pose_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del ( &pose_est );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_del ( &calibration );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del ( &homography );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_build_homography_pointpatch_front)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_homography_optimalforward)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_build_essential)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_build_fundamental)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_matrix33_left_pyramidzoom)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_matrix33_left_pyramidzoom_exact)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_matrix33_right_pyramidzoom)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_matrix33_right_pyramidzoominv)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_affine_from_homography)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_homography_shift)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_homography_shiftleft)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_homography_get_areazoom)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_imagerotation)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_updateZref)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_transformtools_estimate_relativepose_from_general)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
