//==============================================================================
//
//    OPENROX   : File rox_example_camera_projector_calibration.c
//
//    Contents  : A simple example program for camera calibration.
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== HEADERS   ================================================================

#include <stdio.h>

#include <api/openrox.h>

#include <baseproc/geometry/point/points_struct.h>
#include <baseproc/array/fill/fillunit.h>

#include <core/model/model_projector_checkerboard.h>

#include <inout/system/errors_print.h>
#include <inout/image/pgm/pgmfile.h>
#include <inout/numeric/array2d_print.h>
#include <inout/geometry/point/point2d_save.h>

#include <user/calibration/camproj/calibration_camproj_checkerboard.h>


#ifdef WIN32
   #define PREFIX "F:"
#else
   #define PREFIX "/home/emalis/"
#endif

#ifdef WIN32
   #define SUFFIX_CAM_IMAGES  PREFIX"/Users/emalis/Documents/Robocortex/Calibration/Camera_Videoprojector/2018_05_23_18h31m20/print_%06d.png"
   #define SUFFIX_PROJ_IMAGES PREFIX"/Users/emalis/Documents/Robocortex/Calibration/Camera_Videoprojector/2018_05_23_18h31m20/proj_%06d.png"
#else

#endif

   #define NBI 8
   
   // Model we used with the acer x110
   #define PROJ_MOD_SPACE_WIDTH        50
   #define PROJ_MOD_SPACE_HEIGHT       50
   #define PROJ_MOD_COLS               15
   #define PROJ_MOD_ROWS               11
   #define PROJ_IMAGE_WIDTH            800
   #define PROJ_IMAGE_HEIGHT           600

#if 0
   // Model we used with the acer h7550st
   #define PROJ_MOD_SPACE_WIDTH        80
   #define PROJ_MOD_SPACE_HEIGHT       80
   #define PROJ_MOD_COLS               9
   #define PROJ_MOD_ROWS               7
   #define PROJ_IMAGE_WIDTH            1920
   #define PROJ_IMAGE_HEIGHT           1080
#endif

   #define PRINT_MOD_SIZE_X            0.027;
   #define PRINT_MOD_SIZE_Y            0.027;
   #define PRINT_MOD_WIDTH             9;
   #define PRINT_MOD_HEIGHT            13;


   #define SUFFIX_CAM_IMAGES  PREFIX"Data/rox_datasets/regression_tests/openrox/calibration/camproj/synthetic/images_pgm/print_%06d.pgm"
   #define SUFFIX_PROJ_IMAGES PREFIX"Data/rox_datasets/regression_tests/openrox/calibration/camproj/synthetic/images_pgm/proj_%06d.pgm"
   // #define NBI 9


#if 0
   #define PROJ_MOD_SPACE_WIDTH  80
   #define PROJ_MOD_SPACE_HEIGHT 80
   #define PROJ_MOD_COLS         11
   #define PROJ_MOD_ROWS         6
   #define PROJ_IMAGE_WIDTH      1920
   #define PROJ_IMAGE_HEIGHT     1080

   #define PRINT_MOD_SIZE_X            0.028;
   #define PRINT_MOD_SIZE_Y            0.028;
   #define PRINT_MOD_WIDTH             9;
   #define PRINT_MOD_HEIGHT            13;
#endif

   #define FU 3600.0
   #define FV 3600.0
   #define CU 1227.5
   #define CV 1026.5

//=== MAIN PROGRAM =============================================================

Rox_Sint main (Rox_Void)
{
   Rox_ErrorCode error;
   Rox_Uint nbimg        = NBI;
   Rox_Uint valid_images = 0;

   // Prepare image reading
   char* prefix = NULL;
   char fimg_cam[FILENAME_MAX];
   char fimg_proj[FILENAME_MAX];
   char path_cam_images[FILENAME_MAX];
   char path_proj_images[FILENAME_MAX];
   char filename[FILENAME_MAX];
   Rox_Image cur_cam  = NULL;
   Rox_Image cur_proj = NULL;

   prefix = getenv( "ROX_DATA_HOME" );
   sprintf(path_cam_images , "%s%s", prefix, SUFFIX_CAM_IMAGES );
   sprintf(path_proj_images, "%s%s", prefix, SUFFIX_PROJ_IMAGES);

   sprintf(path_cam_images , "%s", SUFFIX_CAM_IMAGES );
   sprintf(path_proj_images, "%s", SUFFIX_PROJ_IMAGES);

   // Define the result object
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
   if (error) goto function_terminate;

   error =  rox_model_projector_checkerboard_set_template ( proj_model, proj_mod_cols, proj_mod_rows, proj_mod_space_width, proj_mod_space_height, proj_image_width, proj_image_height );
   if (error) goto function_terminate;

   // Define the 3D model of the printed grid
   error = rox_model_checkerboard_new(&print_model);
   if (error) goto function_terminate;

   error =  rox_model_checkerboard_set_template ( print_model, print_mod_width, print_mod_height, print_mod_size_x, print_mod_size_y );
   if (error) goto function_terminate;

   // Define the calibration object
   error = rox_calibration_camproj_checkerboard_new ( &calibration, print_model, proj_model );
   if (error) goto function_terminate;

   // If we give the camera intrinsics parameters its calibration is not necessary
   error = rox_matut3_new ( &Kin );
   if (error) goto function_terminate;

   error = rox_transformtools_build_calibration_matrix ( Kin, FU, FV, CU, CV );
   if (error) goto function_terminate;

   error = rox_calibration_camproj_checkerboard_set_camera_intrinsics ( calibration, Kin );
   if (error) goto function_terminate;

   // Create the result intrinsics matrices
   error = rox_matut3_new ( &Kc );
   if (error) goto function_terminate;

   error = rox_matut3_new ( &Kp );
   if (error) goto function_terminate;

   // Create the extrinsic matrix
   error = rox_matse3_new ( &pTc );
   if (error) goto function_terminate;

   // Initialize cam and proj images arrays
   sprintf( fimg_cam, path_cam_images, 1 );
   
   printf("---------------------\n");
   printf("read image : %s\n", fimg_cam);

   error = rox_image_new_read_pgm( &cur_cam, fimg_cam );
   if (error) goto function_terminate;

   sprintf( fimg_proj, path_proj_images, 1 );
   error = rox_image_new_read_pgm( &cur_proj, fimg_proj );
   if (error) goto function_terminate;

   printf("print_model_corners:\n");
   rox_vector_point3d_double_print(calibration->cam_refs3D, calibration->print_npts );

   sprintf ( filename, "calib_model_grid_print.txt" );
   rox_vector_point3d_double_save ( filename, calibration->cam_refs3D, calibration->print_npts );

   printf("proj_model_corners:\n");
   rox_vector_point2d_double_print ( calibration->proj_refs2D, calibration->proj_npts );

   sprintf ( filename, "calib_model_grid_proj.txt" );
   rox_vector_point2d_double_save ( filename, calibration->proj_refs2D, calibration->proj_npts );

   // Add calibration images to a set
   for ( Rox_Sint i = 1; i <= nbimg; i++)
   {
      printf("---------------------\n");

      sprintf(fimg_cam, path_cam_images, i);
      sprintf(fimg_proj, path_proj_images, i);
      printf("read image : %s\n", fimg_cam);
      printf("read image : %s\n", fimg_proj);

      error = rox_image_read_pgm( cur_cam, fimg_cam );
      if (error) goto function_terminate;

      error = rox_image_read_pgm( cur_proj, fimg_proj );
      if (error) goto function_terminate;

      error = rox_calibration_camproj_checkerboard_add_images ( calibration, cur_cam, cur_proj );
      if (error)
      {
        printf("The given %02d data cannot be used for camera calibration \n", i);
      }
      else
      {
         printf("Added image %02d to calibration set \n", i);
         valid_images++;

         // Display detected points
         printf("print_detected_corners:\n");
         rox_vector_point2d_double_print(calibration->print_detected_corners, calibration->print_npts);

         sprintf ( filename, "calib_pts_grid_print_%06d.txt", valid_images );
         rox_vector_point2d_double_save ( filename, calibration->print_detected_corners, calibration->print_npts);

         printf("proj_detected_corners:\n");
         rox_vector_point2d_double_print(calibration->proj_detected_corners, calibration->proj_npts);

         sprintf ( filename, "calib_pts_grid_proj_%06d.txt", valid_images );
         rox_vector_point2d_double_save ( filename, calibration->proj_detected_corners, calibration->proj_npts);

         // getchar();
      }
   }

   error = rox_calibration_camproj_checkerboard_make ( calibration );
   if (error) goto function_terminate;

   error = rox_calibration_camproj_checkerboard_get_raw_intrinsics( calibration, Kc, Kp );
   if (error) goto function_terminate;

   printf("=====================\n");
   printf("The estimated camera intrinsics parameters are: \n");
   error = rox_matut3_print(Kc);
   if (error) goto function_terminate;

   printf("===\n");

   printf("The estimated raw projector intrinsics parameters are: \n");
   error = rox_matut3_print(Kp);
   if (error) goto function_terminate;

   printf("=====================\n");
   for ( Rox_Sint i = 0; i < valid_images; i++ )
   {
      error = rox_calibration_camproj_checkerboard_get_raw_pTc ( calibration, pTc, i );
      if (error) goto function_terminate;

      printf("The estimated raw pTc parameters for the pair %d are: \n", i+1);
      error = rox_matse3_print(pTc);
      if (error) goto function_terminate;

      error = rox_calibration_camproj_checkerboard_check_linear_results ( calibration, & error_cam, & mean_error_proj_fwd, i );
      if (error) goto function_terminate;

      printf("=\n");
      printf(" Errors cam : %f \n", error_cam  );
      printf(" Errors proj: %f \n", mean_error_proj_fwd );

      printf("===\n");
   }
   printf("=====================\n");

   printf("=====================\n");
   error = rox_calibration_camproj_checkerboard_get_fine_intrinsics ( calibration, Kc, Kp );
   if (error) goto function_terminate;

   printf("The estimated fine projector intrinsics parameters are: \n");
   error = rox_matut3_print ( Kp );
   if (error) goto function_terminate;

   printf("===\n");

   error = rox_calibration_camproj_checkerboard_get_fine_pTc ( calibration, pTc );
   if (error) goto function_terminate;

   printf("The estimated fine pTc parameters are:\n");
   error = rox_matse3_print ( pTc );
   if (error) goto function_terminate;

   printf("===\n");
   error = rox_calibration_camproj_checkerboard_check_fine_results(
         calibration,
         &error_cam,
         &mean_error_proj_fwd, &median_error_proj_fwd,
         &mean_error_proj_bwd, &median_error_proj_bwd,
         -1.0 );
   if (error) goto function_terminate;

   printf(" Fine errors cam     : %f \n", error_cam  );
   printf(" Fine errors proj fwd: ( %f, %f ) \n", mean_error_proj_fwd, median_error_proj_fwd );
   printf(" Fine errors proj bwd: ( %f, %f ) \n", mean_error_proj_bwd, median_error_proj_bwd );

   printf("=====================\n");

function_terminate:
   // Delete objects and free memory
   rox_matut3_del ( &Kc );
   rox_matut3_del ( &Kp );
   rox_matut3_del ( &Kin );
   rox_matse3_del ( &pTc );

   rox_image_del ( &cur_cam );
   rox_image_del ( &cur_proj );

   rox_calibration_camproj_checkerboard_del ( &calibration );
   rox_model_checkerboard_del ( &print_model );
   rox_model_projector_checkerboard_del ( &proj_model );

   // Display Error
   rox_error_print(error);

   return error;
}
