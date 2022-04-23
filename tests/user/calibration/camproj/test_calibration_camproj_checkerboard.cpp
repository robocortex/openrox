//==============================================================================
//
//    OPENROX   : File test_calibration_camproj_checkerboard.cpp
//
//    Contents  : Tests for calibration_camproj_checkerboard.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//====== INCLUDED HEADERS   ====================================================

#include <openrox_tests.hpp>

extern "C"
{
   #include <system/time/timer.h>

   #include <baseproc/image/image.h>
   #include <baseproc/geometry/transforms/transform_tools.h>
   #include <baseproc/array/error/l2_error.h>

   #include <inout/image/pgm/pgmfile.h>
   #include <inout/numeric/array2d_print.h>
   #include <inout/geometry/point/point2d_print.h>
   #include <inout/geometry/point/point3d_print.h>
   #include <inout/geometry/point/point2d_save.h>
   #include <inout/geometry/point/point3d_save.h>
   #include <user/calibration/camproj/calibration_camproj_checkerboard.h>
   #include <inout/system/print.h>
}

//====== INTERNAL MACROS    ====================================================

ROX_TEST_SUITE_BEGIN(calibration_camproj_checkerboard)

#define IMAGE_PATH_PRINT   ROX_DATA_HOME"/regression_tests/openrox/calibration/camproj/synthetic/images_pgm/print_%06d.pgm"
#define IMAGE_PATH_PROJ    ROX_DATA_HOME"/regression_tests/openrox/calibration/camproj/synthetic/images_pgm/proj_%06d.pgm"

#define POINTS_PATH_PRINT  ROX_DATA_HOME"/regression_tests/openrox/calibration/camproj/synthetic/grid_corners_grt/calib_pts_grid_print_%06d.txt"
#define POINTS_PATH_PROJ   ROX_DATA_HOME"/regression_tests/openrox/calibration/camproj/synthetic/grid_corners_grt/calib_pts_grid_proj_%06d.txt"

//#define POINTS_PATH_PRINT  ROX_DATA_HOME"/regression_tests/openrox/calibration/camproj/synthetic/grid_corners_mes/calib_pts_grid_print_%06d.txt"
//#define POINTS_PATH_PROJ   ROX_DATA_HOME"/regression_tests/openrox/calibration/camproj/synthetic/grid_corners_mes/calib_pts_grid_proj_%06d.txt"

#define NBI 9

#define PROJ_MOD_SPACE_WIDTH  80
#define PROJ_MOD_SPACE_HEIGHT 80

#define PROJ_MOD_COLS         11
#define PROJ_MOD_ROWS         6

#define PROJ_IMAGE_WIDTH      1920
#define PROJ_IMAGE_HEIGHT     1080

#define PRINT_MOD_SIZE_X      0.028
#define PRINT_MOD_SIZE_Y      0.028
#define PRINT_MOD_WIDTH        9
#define PRINT_MOD_HEIGHT      13

#define CANERA_IMAGE_WIDTH      2456
#define CAMERA_IMAGE_HEIGHT     2054

#define FU 3600.0
#define FV 3600.0
#define CU 1227.5
#define CV 1026.5

//====== INTERNAL TYPESDEFS ====================================================

//====== INTERNAL DATATYPES ====================================================

//====== INTERNAL VARIABLES ====================================================

Rox_Double Kc_grt_data[3*3] = { 3600.0, 0.0, 1227.5, 0.0, 3600.0, 1026.5, 0.0, 0.0, 1.0 };
Rox_Double Kp_grt_data[3*3] = { 2200.0, 0.0,  959.5, 0.0, 2200.0, 1079.0, 0.0, 0.0, 1.0 };
Rox_Double pTc_grt_data[4*4] = {  0.999873231034710, -0.002693840512419, -0.015692835419506,  0.0000,
                                 -0.002693840512419,  0.942755889111097, -0.333472752664501,  0.1325,
                                  0.015692835419506,  0.333472752664501,  0.942629120145806, -0.0129,
                                  0.0              ,  0.0              ,  0.0              ,  1.0000 };

// GRT data computed with Matlab
Rox_Double Kc2_grt_data[3*3] = {  300.0, 0.0, 320.0, 0.0,  300.0, 240.0, 0.0, 0.0, 1.0 };
Rox_Double Kp2_grt_data[3*3] = { 1000.0, 0.0, 959.5, 0.0, 1000.0, 539.5, 0.0, 0.0, 1.0 };
Rox_Double pTc2_grt_data[4*4] = { 0.87560,   0.42003,  -0.23855,  -0.03274,
                                 -0.38175,   0.90430,   0.19105,  -0.43811,
                                  0.29597,  -0.07621,   0.95215,  -0.13035,
                                  0.00000,   0.00000,   0.00000,   1.00000};

// GRT data computed with the devapp
Rox_Double Kc3_grt_data[3*3] = { 3603.2166348533455675,    0.0000000000000000, 1227.4998562956147907,                                                                                                                               
                                    0.0000000000000000, 3603.3154392689775705, 1026.5002425660020435,                                                                                                                               
                                    0.0000000000000000,    0.0000000000000000,    1.0000000000000000 };

Rox_Double Kp3_grt_data[3*3] = { 2201.7405566749516765,    0.0000000000000000,  959.6042640147879865,                                                                                                                                
                                    0.0000000000000000, 2201.4829255916642978, 1080.5523830430149701,                                                                                                                               
                                    0.0000000000000000,    0.0000000000000000,    1.0000000000000000 };
Rox_Double pTc3_grt_data[4*4] = { 0.9998702620897225, -0.0027237039296772, -0.0158757810999284,  0.0002164583722144,                                                                                                                
                                 -0.0027351251656433,  0.9425788135229142, -0.3339726027506808,  0.1326627188859966,                                                                                                                
                                  0.0158738174034368,  0.3339726960915206,  0.9424490756462141, -0.0128117875176568,                                                                                                               
                                  0.0000000000000000,  0.0000000000000000,  0.0000000000000000,  1.0000000000000000 };


//====== INTERNAL FUNCTDEFS ====================================================

//====== INTERNAL FUNCTIONS ====================================================

//====== EXPORTED FUNCTIONS ====================================================

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_calibration_camproj_checkerboard_images )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double l2_error = 0.0;

   Rox_Sint nbimg        = NBI;
   Rox_Sint valid_images = 0;

   // Prepare image reading
   char fimg_cam[FILENAME_MAX];
   char fimg_proj[FILENAME_MAX];
   // char filename[FILENAME_MAX];
   Rox_Image cur_cam  = NULL;
   Rox_Image cur_proj = NULL;

   // Define the ground truth matrices
   Rox_Matrix Kc_grt  = NULL;
   Rox_Matrix Kp_grt  = NULL;
   Rox_Matrix pTc_grt = NULL;

   // Define the result matrices
   Rox_MatUT3 Kc  = NULL;
   Rox_MatUT3 Kp  = NULL;
   Rox_MatSE3 pTc = NULL;
   Rox_MatUT3 Kin = NULL;

   Rox_Double error_cam             = -1.0;
   Rox_Double mean_error_proj_fwd   = -1.0;
   Rox_Double median_error_proj_fwd = -1.0;
   Rox_Double mean_error_proj_bwd   = -1.0;
   Rox_Double median_error_proj_bwd = -1.0;

   // Declare the 2D model
   Rox_Model_Projector_CheckerBoard proj_model  = NULL;
   Rox_Double proj_mod_space_width              = PROJ_MOD_SPACE_WIDTH;
   Rox_Double proj_mod_space_height             = PROJ_MOD_SPACE_HEIGHT;
   Rox_Uint proj_mod_cols                       = PROJ_MOD_COLS;
   Rox_Uint proj_mod_rows                       = PROJ_MOD_ROWS;
   Rox_Uint proj_image_width                    = PROJ_IMAGE_WIDTH;
   Rox_Uint proj_image_height                   = PROJ_IMAGE_HEIGHT;

   // Declare the 3D model
   Rox_Model_CheckerBoard print_model = NULL;
   Rox_Double print_mod_size_x                  = PRINT_MOD_SIZE_X;
   Rox_Double print_mod_size_y                  = PRINT_MOD_SIZE_Y;
   Rox_Uint print_mod_width                     = PRINT_MOD_WIDTH;
   Rox_Uint print_mod_height                    = PRINT_MOD_HEIGHT;

   // Declare the calibration object
   Rox_Calibration_CamProj_CheckerBoard calibration = 0;

   // Define the 2D model of the projected grid
   error = rox_model_projector_checkerboard_new ( &proj_model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error =  rox_model_projector_checkerboard_set_template ( proj_model, proj_mod_cols, proj_mod_rows, proj_mod_space_width, proj_mod_space_height, proj_image_width, proj_image_height );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Define the 3D model of the printed grid
   error = rox_model_checkerboard_new(&print_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error =  rox_model_checkerboard_set_template ( print_model, print_mod_width, print_mod_height, print_mod_size_x, print_mod_size_y );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Define the calibration object
   error = rox_calibration_camproj_checkerboard_new ( &calibration, print_model, proj_model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // If we give the camera intrinsics parameters its calibration is not necessary
   error = rox_matut3_new ( &Kin );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_transformtools_build_calibration_matrix ( Kin, FU, FV, CU, CV );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_calibration_camproj_checkerboard_set_camera_intrinsics ( calibration, Kin );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Create the ground truth intrinsics matrices
   error = rox_matut3_new ( &Kc_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;
   
   error = rox_array2d_double_set_buffer_no_stride ( Kc_grt, Kc_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_new ( &Kp_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_double_set_buffer_no_stride ( Kp_grt, Kp_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the ground truth extrinsic matrix
   error = rox_matse3_new ( &pTc_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_double_set_buffer_no_stride ( pTc_grt, pTc_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the result intrinsics matrices
   error = rox_matut3_new ( &Kc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_matut3_new ( &Kp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Create the extrinsic matrix
   error = rox_matse3_new ( &pTc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Initialize cam and proj images arrays
   sprintf( fimg_cam, IMAGE_PATH_PRINT, 1 );
   
   rox_log("---------------------\n");
   rox_log("read image : %s\n", fimg_cam);

   error = rox_image_new_read_pgm( &cur_cam, fimg_cam );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   sprintf( fimg_proj, IMAGE_PATH_PROJ, 1 );
   rox_log("---------------------\n");
   rox_log("read image : %s\n", fimg_proj);

   error = rox_image_new_read_pgm( &cur_proj, fimg_proj );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Add calibration images to a set
   for ( Rox_Sint i = 1; i <= nbimg; i++)
   {
      rox_log("---------------------\n");

      sprintf(fimg_cam, IMAGE_PATH_PRINT, i);
      sprintf(fimg_proj, IMAGE_PATH_PROJ, i);
      rox_log("read image : %s\n", fimg_cam);
      rox_log("read image : %s\n", fimg_proj);

      error = rox_image_read_pgm ( cur_cam, fimg_cam );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      error = rox_image_read_pgm ( cur_proj, fimg_proj );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      error = rox_calibration_camproj_checkerboard_add_images ( calibration, cur_cam, cur_proj );
      if (error)
      { 
         rox_log("The given %02d data cannot be used for camera calibration \n", i);
      }
      else
      {
         rox_log("Added image %02d to calibration set \n", i);
         valid_images++;
      }
   }

   error = rox_calibration_camproj_checkerboard_make ( calibration );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_calibration_camproj_checkerboard_get_raw_intrinsics( calibration, Kc, Kp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log("=====================\n");
   rox_log("The estimated camera intrinsics parameters are: \n");
   rox_matut3_print(Kc);

   rox_log("===\n");

   rox_log("The estimated raw projector intrinsics parameters are: \n");
   rox_matut3_print(Kp);

   rox_log("=====================\n");
   for ( Rox_Sint i = 0; i < valid_images; i++ )
   {
      error = rox_calibration_camproj_checkerboard_get_raw_pTc ( calibration, pTc, i );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      rox_log("The estimated raw pTc parameters for the pair %d are: \n", i+1);
      rox_matse3_print(pTc);

      error = rox_calibration_camproj_checkerboard_check_linear_results ( calibration, & error_cam, & mean_error_proj_fwd, i );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      rox_log("=\n");
      rox_log(" Errors cam : %f \n", error_cam  );
      rox_log(" Errors proj: %f \n", mean_error_proj_fwd );

      rox_log("===\n");
   }
   rox_log("=====================\n");

   rox_log("=====================\n");
   error = rox_calibration_camproj_checkerboard_get_fine_intrinsics ( calibration, Kc, Kp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log("The estimated fine projector intrinsics parameters are: \n");
   rox_matut3_print ( Kp );

   rox_log("===\n");

   error = rox_calibration_camproj_checkerboard_get_fine_pTc ( calibration, pTc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log("The estimated fine pTc parameters are:\n");
   rox_matse3_print ( pTc );

   rox_log("===\n");
   error = rox_calibration_camproj_checkerboard_check_fine_results ( calibration, &error_cam, &mean_error_proj_fwd, &median_error_proj_fwd, &mean_error_proj_bwd, &median_error_proj_bwd, -1.0 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log(" Fine errors cam     : %f \n", error_cam  );
   rox_log(" Fine errors proj fwd: ( %f, %f ) \n", mean_error_proj_fwd, median_error_proj_fwd );
   rox_log(" Fine errors proj bwd: ( %f, %f ) \n", mean_error_proj_bwd, median_error_proj_bwd );

   rox_log("=====================\n");

   error = rox_array2d_double_difference_l2_norm ( &l2_error, Kc_grt, Kc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error Kc = %f \n", l2_error);
   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);

   error = rox_array2d_double_difference_l2_norm ( &l2_error, Kp_grt, Kp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error Kp = %f \n", l2_error);
   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 0.5);

   error = rox_array2d_double_difference_l2_norm ( &l2_error, pTc_grt, pTc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error pTc = %f \n", l2_error);
   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 0.005);

function_terminate:
   // Delete objects and free memory
   error = rox_matut3_del ( &Kc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matut3_del ( &Kp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 
   
   rox_matut3_del ( &Kin );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );  
   
   rox_matse3_del ( &pTc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );  

   rox_image_del ( &cur_cam );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 
   
   rox_image_del ( &cur_proj );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 

   rox_calibration_camproj_checkerboard_del ( &calibration );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 
   
   rox_model_checkerboard_del ( &print_model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 

   rox_model_projector_checkerboard_del ( &proj_model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 

}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_calibration_camproj_checkerboard_corners )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double l2_error = 0.0;
   Rox_Sint nbimg = 2;

   // Define the ground truth matrices
   Rox_Matrix Kc_grt  = NULL;
   Rox_Matrix Kp_grt  = NULL;
   Rox_Matrix pTc_grt = NULL;

   // Define the result matrices
   Rox_Matrix Kc   = NULL;
   Rox_Matrix Kp   = NULL;
   Rox_Matrix pTc  = NULL;

   Rox_Matrix pTc0 = NULL;
   Rox_Matrix pTc1 = NULL;


   Rox_Double error_cam             = -1.0;
   Rox_Double mean_error_proj_fwd   = -1.0;
   Rox_Double median_error_proj_fwd = -1.0;
   Rox_Double mean_error_proj_bwd   = -1.0;
   Rox_Double median_error_proj_bwd = -1.0;

   // Declare the 2D model 
   // Using 9 points
   Rox_Model_Projector_CheckerBoard proj_model = NULL;
   Rox_Double proj_mod_space_width = 480;
   Rox_Double proj_mod_space_height = 270;
   Rox_Uint proj_mod_cols = 3;
   Rox_Uint proj_mod_rows = 3;
   Rox_Uint proj_image_width  = 1920;
   Rox_Uint proj_image_height = 1080;

   // Declare the 3D model 
   Rox_Model_CheckerBoard print_model = NULL;
   Rox_Double print_mod_size_x = 0.279;
   Rox_Double print_mod_size_y = 0.280;
   Rox_Uint print_mod_width  = 2;
   Rox_Uint print_mod_height = 2;

   // Declare projector and camera image sizes 
   Rox_Uint cam_image_width  = 640;
   Rox_Uint cam_image_height = 480;

   // Declare the calibration object 
   Rox_Calibration_CamProj_CheckerBoard calibration = NULL;

   // Define the 2D model 
   error = rox_model_projector_checkerboard_new(&proj_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error =  rox_model_projector_checkerboard_set_template ( proj_model, proj_mod_cols, proj_mod_rows,
      proj_mod_space_width,
      proj_mod_space_height, 
      proj_image_width, 
      proj_image_height );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Define the 3D model 
   error = rox_model_checkerboard_new(&print_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error =  rox_model_checkerboard_set_template(
      print_model,
      print_mod_width,
      print_mod_height,
      print_mod_size_x,
      print_mod_size_y );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Define the calibration object 
   error = rox_calibration_camproj_checkerboard_new(
      &calibration,
      print_model,
      proj_model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Create the ground truth intrinsics matrices
   error = rox_matut3_new ( &Kc_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;
   
   error = rox_array2d_double_set_buffer_no_stride ( Kc_grt, Kc2_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_new ( &Kp_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_double_set_buffer_no_stride ( Kp_grt, Kp2_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the ground truth extrinsic matrix
   error = rox_matse3_new ( &pTc_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_double_set_buffer_no_stride ( pTc_grt, pTc2_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the intrinsics matrix 
   error = rox_matut3_new ( &Kc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_matut3_new ( &Kp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Create the extrinsic matrices 
   error = rox_matse3_new ( &pTc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_matse3_new ( &pTc0 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_matse3_new ( &pTc1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Create printed pattern camera observations 
   Rox_Point2D_Double_Struct print_obs2D[2][4];
   {
      print_obs2D[0][0].u = 320.00000000000000;
      print_obs2D[0][0].v = 240.00000000000000;
      print_obs2D[0][1].u = 396.25257671474822;
      print_obs2D[0][1].v = 244.78929927536038;
      print_obs2D[0][2].u = 315.25040946549530;
      print_obs2D[0][2].v = 315.91977691531378;
      print_obs2D[0][3].u = 391.19506284562044;
      print_obs2D[0][3].v = 321.01155889120133;

      print_obs2D[1][0].u = 320.00000000000000;
      print_obs2D[1][0].v = 240.00000000000000;
      print_obs2D[1][1].u = 390.28084783068488;
      print_obs2D[1][1].v = 244.78089400747172;
      print_obs2D[1][2].u = 315.26942956402416;
      print_obs2D[1][2].v = 309.76407640049922;
      print_obs2D[1][3].u = 385.42190339903709;
      print_obs2D[1][3].v = 315.31046633152681;
   }

   // Create projected pattern camera observations 
   // Using 9 points
   Rox_Point2D_Double_Struct proj_obs2D[2][9];
   {
      proj_obs2D[0][0].u = 287.61943827116050;
      proj_obs2D[0][0].v = 222.71786277571030;
      proj_obs2D[0][1].u = 401.29650725339104;
      proj_obs2D[0][1].v = 262.25071958596442;
      proj_obs2D[0][2].u = 547.73137132713055;
      proj_obs2D[0][2].v = 313.17557686180669;
      proj_obs2D[0][3].u = 264.23120102367483;
      proj_obs2D[0][3].v = 284.50469862524767;
      proj_obs2D[0][4].u = 369.07941081616622;
      proj_obs2D[0][4].v = 329.30492222321863;
      proj_obs2D[0][5].u = 502.31858494840588;
      proj_obs2D[0][5].v = 386.23622133597462;
      proj_obs2D[0][6].u = 242.96287210239461;
      proj_obs2D[0][6].v = 340.69117963053907;
      proj_obs2D[0][7].u = 340.11311868395342;
      proj_obs2D[0][7].v = 389.59314919202393;
      proj_obs2D[0][8].u = 462.07319175630306;
      proj_obs2D[0][8].v = 450.98350064206966;

      proj_obs2D[1][0].u = 289.87879027037025;
      proj_obs2D[1][0].v = 210.86048910250082;
      proj_obs2D[1][1].u = 405.14507252824092;
      proj_obs2D[1][1].v = 252.63749263076846;
      proj_obs2D[1][2].u = 552.77320696959782;
      proj_obs2D[1][2].v = 306.14369559505303;
      proj_obs2D[1][3].u = 266.02946795571512;
      proj_obs2D[1][3].v = 273.90386980725316;
      proj_obs2D[1][4].u = 372.39121424146663;
      proj_obs2D[1][4].v = 320.80920837249306;
      proj_obs2D[1][5].u = 506.81873338654862;
      proj_obs2D[1][5].v = 380.09150222507174;
      proj_obs2D[1][6].u = 244.34694263019873;
      proj_obs2D[1][6].v = 331.21953189199917;
      proj_obs2D[1][7].u = 342.94251417623087;
      proj_obs2D[1][7].v = 382.10178624937754;
      proj_obs2D[1][8].u = 466.07858181233632;
      proj_obs2D[1][8].v = 445.64866381986235;
   }

   // Add calibration images to a set 
   for ( Rox_Sint i = 0; i < nbimg; i++)
   {
      error = rox_calibration_camproj_checkerboard_add_points ( calibration, print_obs2D[i], 4, proj_obs2D[i], 9, cam_image_width, cam_image_height );
      if ( error )
      {
        rox_log("The given %02d data cannot be used for camera calibration \n", i+1);
      }
      else
      {
        rox_log("Added image %02d to calibration set \n", i+1);
      }
   }

   error = rox_calibration_camproj_checkerboard_make(calibration);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_calibration_camproj_checkerboard_get_raw_intrinsics( calibration, Kc, Kp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log("=====================\n");

   rox_log("The estimated camera intrinsics parameters are: \n");
   rox_matut3_print(Kc);

   rox_log("\n");

   rox_log("The expected intrinsic parameters are: \n");
   rox_log("Matrix (3x3) \n");
   rox_log("300.0 0.000 320.0 \n");
   rox_log("  0.0 300.0 240.0 \n");
   rox_log("  0.0   0.0   1.0 \n");

   rox_log("===\n");

   rox_log("The estimated raw projector intrinsics parameters are: \n");
   rox_matut3_print(Kp);

   rox_log("\n");

   rox_log("The expected intrinsic parameters are: \n");
   rox_log("Matrix (3x3) \n");
   rox_log("1000.0    0.0 960.0 \n");
   rox_log("   0.0 1000.0 540.0 \n");
   rox_log("   0.0    0.0   1.0 \n");

   rox_log("=====================\n");
   error = rox_calibration_camproj_checkerboard_get_raw_pTc( calibration, pTc0, 0 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log("The estimated raw pTc parameters for the first pair are: \n");
   rox_matse3_print(pTc0);

   rox_log("===\n");

   error = rox_calibration_camproj_checkerboard_get_raw_pTc( calibration, pTc1, 1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log("The estimated raw pTc parameters for the second pair are: \n");
   rox_matse3_print(pTc1);

   rox_log("===\n");

   rox_log("The expected  pTc parameters are: \n");
   rox_log("Matrix (4x4) \n");
   rox_log("  0.87560   0.42003  -0.23855  -0.03274 \n");
   rox_log(" -0.38175   0.90430   0.19105  -0.43811 \n");
   rox_log("  0.29597  -0.07621   0.95215  -0.13035 \n");
   rox_log("  0.00000   0.00000   0.00000   1.00000 \n");

   error = rox_calibration_camproj_checkerboard_check_linear_results( calibration, & error_cam, & mean_error_proj_fwd, 0 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log(" Errors cam : %0.10f \n", error_cam  );
   rox_log(" Errors proj: %0.10f \n", mean_error_proj_fwd );

   error = rox_calibration_camproj_checkerboard_check_linear_results( calibration, & error_cam, & mean_error_proj_fwd, 1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log(" Errors cam : %0.10f \n", error_cam  );
   rox_log(" Errors proj: %0.10f \n", mean_error_proj_fwd );

   rox_log("=====================\n");

   rox_log("=====================\n");
   error = rox_calibration_camproj_checkerboard_get_fine_pTc( calibration, pTc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log("The estimated fine pTc parameters pair are: \n");
   rox_matse3_print(pTc);

   rox_log("===\n");

   error = rox_calibration_camproj_checkerboard_get_fine_intrinsics( calibration, Kc, Kp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log("The estimated fine projector intrinsics parameters are: \n");
   error = rox_matut3_print ( Kp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log("===\n");

   error = rox_calibration_camproj_checkerboard_check_fine_results ( calibration, &error_cam, &mean_error_proj_fwd, &median_error_proj_fwd, &mean_error_proj_bwd, &median_error_proj_bwd, -1.0 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log(" Fine errors cam     : %0.10f \n", error_cam  );
   rox_log(" Fine errors proj fwd: ( %f, %f ) \n", mean_error_proj_fwd, median_error_proj_fwd );
   rox_log(" Fine errors proj bwd: ( %f, %f ) \n", mean_error_proj_bwd, median_error_proj_bwd );

   rox_log("=====================\n");

   error = rox_array2d_double_difference_l2_norm ( &l2_error, Kc_grt, Kc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error Kc = %f \n", l2_error);
   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-5);

   error = rox_array2d_double_difference_l2_norm ( &l2_error, Kp_grt, Kp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error Kp = %f \n", l2_error);
   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-5);

   error = rox_array2d_double_difference_l2_norm ( &l2_error, pTc_grt, pTc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error pTc = %f \n", l2_error);
   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-5);

function_terminate:

   // Delete objects and free memory 
   error = rox_matrix_del(&Kc);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matrix_del(&Kp);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del(&pTc);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del(&pTc0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del(&pTc1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_calibration_camproj_checkerboard_del(&calibration);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_model_checkerboard_del(&print_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_model_projector_checkerboard_del(&proj_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_calibration_camproj_checkerboard_corners_from_file )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double l2_error = 0.0;
   Rox_Sint nbimg = NBI;

   char filename[FILENAME_MAX];

   // Define the ground truth matrices
   Rox_Matrix Kc_grt  = NULL;
   Rox_Matrix Kp_grt  = NULL;
   Rox_Matrix pTc_grt = NULL;

   // Define the result matrices
   Rox_Matrix Kc   = NULL;
   Rox_Matrix Kp   = NULL;
   Rox_Matrix pTc  = NULL;

   Rox_Matrix pTc0 = NULL;
   Rox_Matrix pTc1 = NULL;

   Rox_Double error_cam             = -1.0;
   Rox_Double mean_error_proj_fwd   = -1.0;
   Rox_Double median_error_proj_fwd = -1.0;
   Rox_Double mean_error_proj_bwd   = -1.0;
   Rox_Double median_error_proj_bwd = -1.0;

   // Declare the 2D model 
   Rox_Model_Projector_CheckerBoard proj_model = NULL;
   Rox_Uint proj_mod_cols = PROJ_MOD_COLS;
   Rox_Uint proj_mod_rows = PROJ_MOD_ROWS;
   Rox_Uint proj_image_width  = PROJ_IMAGE_WIDTH;
   Rox_Uint proj_image_height = PROJ_IMAGE_HEIGHT;
   Rox_Double proj_mod_space_width = PROJ_MOD_SPACE_WIDTH;
   Rox_Double proj_mod_space_height = PROJ_MOD_SPACE_HEIGHT;

   // Declare the 3D model 
   Rox_Model_CheckerBoard print_model = NULL;
   Rox_Double print_mod_size_x = PRINT_MOD_SIZE_X;
   Rox_Double print_mod_size_y = PRINT_MOD_SIZE_Y;
   Rox_Uint print_mod_width  = PRINT_MOD_WIDTH;
   Rox_Uint print_mod_height = PRINT_MOD_HEIGHT;

   // Declare projector and camera image sizes 
   Rox_Uint cam_image_width  = CANERA_IMAGE_WIDTH;
   Rox_Uint cam_image_height = CAMERA_IMAGE_HEIGHT;

   // Declare the calibration object 
   Rox_Calibration_CamProj_CheckerBoard calibration = NULL;

   // Define the 2D model 
   error = rox_model_projector_checkerboard_new(&proj_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error =  rox_model_projector_checkerboard_set_template ( proj_model, proj_mod_cols, proj_mod_rows,
      proj_mod_space_width,
      proj_mod_space_height, 
      proj_image_width, 
      proj_image_height );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Define the 3D model 
   error = rox_model_checkerboard_new(&print_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error =  rox_model_checkerboard_set_template(
      print_model,
      print_mod_width,
      print_mod_height,
      print_mod_size_x,
      print_mod_size_y );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Define the calibration object 
   error = rox_calibration_camproj_checkerboard_new (
      &calibration,
      print_model,
      proj_model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Create the ground truth intrinsics matrices
   error = rox_matut3_new ( &Kc_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;
   
   error = rox_array2d_double_set_buffer_no_stride ( Kc_grt, Kc_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_new ( &Kp_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_double_set_buffer_no_stride ( Kp_grt, Kp_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the ground truth extrinsic matrix
   error = rox_matse3_new ( &pTc_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_double_set_buffer_no_stride ( pTc_grt, pTc_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the intrinsics matrix 
   error = rox_matut3_new ( &Kc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_matut3_new ( &Kp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Create the extrinsic matrices 
   error = rox_matse3_new ( &pTc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_matse3_new ( &pTc0 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_matse3_new ( &pTc1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Create printed pattern camera observations 
   Rox_Point2D_Double_Struct print_obs2D[PRINT_MOD_HEIGHT*PRINT_MOD_WIDTH];
   
   // Create projected pattern camera observations 
   Rox_Point2D_Double_Struct proj_obs2D[PROJ_MOD_ROWS*PROJ_MOD_COLS];

   // Add calibration images to a set 
   for ( Rox_Sint i = 0; i < nbimg; i++)
   {
      sprintf ( filename, POINTS_PATH_PRINT, i+1 );
      rox_log("read file %s\n", filename);
      
      error = rox_vector_point2d_double_load ( print_obs2D, PRINT_MOD_HEIGHT*PRINT_MOD_WIDTH, filename );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      sprintf ( filename, POINTS_PATH_PROJ, i+1 );
      rox_log("read file %s\n", filename);

      error = rox_vector_point2d_double_load ( proj_obs2D, PROJ_MOD_ROWS*PROJ_MOD_COLS, filename );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      error = rox_calibration_camproj_checkerboard_add_points ( calibration, print_obs2D, PRINT_MOD_HEIGHT*PRINT_MOD_WIDTH, proj_obs2D, PROJ_MOD_ROWS*PROJ_MOD_COLS, cam_image_width, cam_image_height );
      if ( error )
      {
        rox_log("The given %02d data cannot be used for camera calibration \n", i+1);
      }
      else
      {
        rox_log("Added image %02d to calibration set \n", i+1);
      }
   }

   error = rox_calibration_camproj_checkerboard_set_camera_intrinsics ( calibration, Kc_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_calibration_camproj_checkerboard_make(calibration);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_calibration_camproj_checkerboard_get_raw_intrinsics( calibration, Kc, Kp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log("=====================\n");

   rox_log("The estimated camera intrinsics parameters are: \n");
   rox_matut3_print(Kc);

   rox_log("\n");

   rox_log("The expected intrinsic parameters are: \n");
   rox_log("Matrix (3x3) \n");
   rox_log("3600.0,    0.0, 1227.5; \n");
   rox_log("   0.0, 3600.0, 1026.5; \n");
   rox_log("   0.0,    0.0,    1.0 \n");

   rox_log("===\n");

   rox_log("The estimated raw projector intrinsics parameters are: \n");
   rox_matut3_print(Kp);

   rox_log("\n");

   rox_log("The expected intrinsic parameters are: \n");
   rox_log("Matrix (3x3) \n");
   rox_log("2200.0,    0.0,  959.5; \n");
   rox_log("   0.0, 2200.0, 1079.0; \n");
   rox_log("   0.0,    0.0,    1.0 \n");

   rox_log("=====================\n");
  
   rox_log("The expected  pTc parameters are: \n");
   rox_log("Matrix (4x4) \n");
   rox_log("  0.999873231034710,  -0.002693840512419,  -0.015692835419506,                   0; \n");
   rox_log(" -0.002693840512419,   0.942755889111097,  -0.333472752664501,   0.132500000000000; \n");
   rox_log("  0.015692835419506,   0.333472752664501,   0.942629120145806,  -0.012900000000000; \n");
   rox_log("  0                ,   0                ,   0                ,   1.000000000000000 \n");

   error = rox_calibration_camproj_checkerboard_check_linear_results( calibration, & error_cam, & mean_error_proj_fwd, 0 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log(" Errors cam : %0.10f \n", error_cam  );
   rox_log(" Errors proj: %0.10f \n", mean_error_proj_fwd );

   error = rox_calibration_camproj_checkerboard_check_linear_results( calibration, & error_cam, & mean_error_proj_fwd, 1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log(" Errors cam : %0.10f \n", error_cam  );
   rox_log(" Errors proj: %0.10f \n", mean_error_proj_fwd );

   rox_log("=====================\n");

   rox_log("=====================\n");
   error = rox_calibration_camproj_checkerboard_get_fine_pTc( calibration, pTc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log("The estimated fine pTc parameters pair are: \n");
   rox_matse3_print(pTc);

   rox_log("===\n");

   error = rox_calibration_camproj_checkerboard_get_fine_intrinsics( calibration, Kc, Kp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log("The estimated fine projector intrinsics parameters are: \n");
   error = rox_matut3_print ( Kp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log("===\n");

   error = rox_calibration_camproj_checkerboard_check_fine_results ( calibration, &error_cam, &mean_error_proj_fwd, &median_error_proj_fwd, &mean_error_proj_bwd, &median_error_proj_bwd, -1.0 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log(" Fine errors cam     : %0.10f \n", error_cam  );
   rox_log(" Fine errors proj fwd: ( %f, %f ) \n", mean_error_proj_fwd, median_error_proj_fwd );
   rox_log(" Fine errors proj bwd: ( %f, %f ) \n", mean_error_proj_bwd, median_error_proj_bwd );

   rox_log("=====================\n");

   error = rox_array2d_double_difference_l2_norm ( &l2_error, Kc_grt, Kc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error Kc = %f \n", l2_error);
   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-5);

   error = rox_array2d_double_difference_l2_norm ( &l2_error, Kp_grt, Kp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error Kp = %f \n", l2_error);
   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-5);

   error = rox_array2d_double_difference_l2_norm ( &l2_error, pTc_grt, pTc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error pTc = %f \n", l2_error);
   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-5);

function_terminate:

   // Delete objects and free memory 
   error = rox_matrix_del(&Kc);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matrix_del(&Kp);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del(&pTc);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del(&pTc0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del(&pTc1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_calibration_camproj_checkerboard_del(&calibration);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_model_checkerboard_del(&print_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_model_projector_checkerboard_del(&proj_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
