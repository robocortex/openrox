//==============================================================================
//
//    OPENROX   : File rox_example_camera_projector_calibration_mocked.c
//
//    Contents  : A simple example program for camera projector calibration.
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

#include <core/model/model_projector_checkerboard.h>

#include <inout/system/errors_print.h>

#include <user/calibration/camproj/calibration_camproj_checkerboard.h>

//=== MAIN PROGRAM =============================================================

Rox_Sint main (Rox_Void)
{
   Rox_ErrorCode error;
   Rox_Uint nbimg = 2;

   // Define the result object 
   Rox_Matrix Kc   = NULL;
   Rox_Matrix Kp   = NULL;
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
   Rox_Calibration_CamProj_CheckerBoard calibration = 0;

   // Define the 2D model 
   error = rox_model_projector_checkerboard_new(&proj_model);
   if(error) goto function_terminate;
   error =  rox_model_projector_checkerboard_set_template(
      proj_model,
      proj_mod_cols,
      proj_mod_rows,
      proj_mod_space_width,
      proj_mod_space_height, 
      proj_image_width, 
      proj_image_height );
   if(error) goto function_terminate;

   // Define the 3D model 
   error = rox_model_checkerboard_new(&print_model);
   if(error) goto function_terminate;
   error =  rox_model_checkerboard_set_template(
      print_model,
      print_mod_width,
      print_mod_height,
      print_mod_size_x,
      print_mod_size_y );
   if(error) goto function_terminate;

   // Define the calibration object 
   error = rox_calibration_camproj_checkerboard_new(
      &calibration,
      print_model,
      proj_model );
   if(error) goto function_terminate;


   // Create the intrinsics matrix 
   error = rox_matrix_new(&Kc, 3, 3);
   if(error) goto function_terminate;

   error = rox_matrix_new(&Kp, 3, 3);
   if(error) goto function_terminate;

   // Create the extrinsic matrices 
   error = rox_matrix_new(&pTc0, 4, 4);
   if(error) goto function_terminate;

   error = rox_matrix_new(&pTc1, 4, 4);
   if(error) goto function_terminate;

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
   // Using 4 points
   /*
   Rox_Point2D_Double proj_obs2D[2][4];
   {
      proj_obs2D[0][0].u =  215.67280308197337;
      proj_obs2D[0][0].v =  128.57861815810034;
      proj_obs2D[0][1].u =  820.38099115676073;
      proj_obs2D[0][1].v =  288.55978273163333;
      proj_obs2D[0][2].u =  149.17893235950376;
      proj_obs2D[0][2].v =  349.15833302275780;
      proj_obs2D[0][3].u =  569.23699479647030;
      proj_obs2D[0][3].v =  591.20297639689761;
                                                
      proj_obs2D[1][0].u =  216.65443656966715;
      proj_obs2D[1][0].v =  113.28822638985277;
      proj_obs2D[1][1].u =  825.82508276708654;
      proj_obs2D[1][1].v =  283.94388666264217;
      proj_obs2D[1][2].u =  148.74768526878691;
      proj_obs2D[1][2].v =  338.99199359476262;
      proj_obs2D[1][3].u =  573.37598122678196;
      proj_obs2D[1][3].v =  588.62363751293424;
   }
   */
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
   for( Rox_Sint i = 0; i < nbimg; i++)
   {
      error = rox_calibration_camproj_checkerboard_add_points(
            // Using 4 points
            //calibration, print_obs2D[i], 4, proj_obs2D[i], 4, cam_image_width, cam_image_height );
            // Using 9 points
             calibration, print_obs2D[i], 4, proj_obs2D[i], 9, cam_image_width, cam_image_height );
      if(error)
      {
        printf("The given %02d data cannot be used for camera calibration \n", i+1);
      }
      else
      {
        printf("Added image %02d to calibration set \n", i+1);
      }
   }

   error = rox_calibration_camproj_checkerboard_make(calibration);
   if(error) goto function_terminate;

   error = rox_calibration_camproj_checkerboard_get_raw_intrinsics( calibration, Kc, Kp );
   if(error) goto function_terminate;

   printf("=====================\n");

   printf("The estimated camera intrinsics parameters are: \n");
   error = rox_matrix_print(Kc);
   if(error) goto function_terminate;

   printf("\n");

   printf("The expected intrinsic parameters are: \n");
   printf("Matrix (3x3) \n");
   printf("300.0 0.000 320.0 \n");
   printf("  0.0 300.0 240.0 \n");
   printf("  0.0   0.0   1.0 \n");

   printf("===\n");

   printf("The estimated raw projector intrinsics parameters are: \n");
   rox_matrix_print(Kp);
   
   printf("\n");

   printf("The expected intrinsic parameters are: \n");
   printf("Matrix (3x3) \n");
   printf("1000.0    0.0 960.0 \n");
   printf("   0.0 1000.0 540.0 \n");
   printf("   0.0    0.0   1.0 \n");

   printf("=====================\n");
   error = rox_calibration_camproj_checkerboard_get_raw_pTc( calibration, pTc0, 0 );
   if(error) goto function_terminate;

   printf("The estimated raw pTc parameters for the first pair are: \n");
   error = rox_matrix_print(pTc0);
   if(error) goto function_terminate;

   printf("===\n");

   error = rox_calibration_camproj_checkerboard_get_raw_pTc( calibration, pTc1, 1 );
   if(error) goto function_terminate;

   printf("The estimated raw pTc parameters for the second pair are: \n");
   error = rox_matrix_print(pTc1);
   if(error) goto function_terminate;

   printf("===\n");

   printf("The expected  pTc parameters are: \n");
   printf("Matrix (4x4) \n");
   printf("  0.87560   0.42003  -0.23855  -0.03274 \n");
   printf(" -0.38175   0.90430   0.19105  -0.43811 \n");
   printf("  0.29597  -0.07621   0.95215  -0.13035 \n");
   printf("  0.00000   0.00000   0.00000   1.00000 \n");

   error = rox_calibration_camproj_checkerboard_check_linear_results( calibration, & error_cam, & mean_error_proj_fwd, 0 );
   if (error) goto function_terminate;
   printf(" Errors cam : %0.10f \n", error_cam  );
   printf(" Errors proj: %0.10f \n", mean_error_proj_fwd );
   error = rox_calibration_camproj_checkerboard_check_linear_results( calibration, & error_cam, & mean_error_proj_fwd, 1 );
   if (error) goto function_terminate;
   printf(" Errors cam : %0.10f \n", error_cam  );
   printf(" Errors proj: %0.10f \n", mean_error_proj_fwd );

   printf("=====================\n");


   printf("=====================\n");
   error = rox_calibration_camproj_checkerboard_get_fine_pTc( calibration, pTc0 );
   if(error) goto function_terminate;

   printf("The estimated fine pTc parameters for the first pair are: \n");
   error = rox_matrix_print(pTc0);
   if(error) goto function_terminate;

   printf("===\n");

   error = rox_calibration_camproj_checkerboard_get_fine_intrinsics( calibration, Kc, Kp );
   if(error) goto function_terminate;

   printf("The estimated fine projector intrinsics parameters are: \n");
   error = rox_matrix_print(Kp);
   if(error) goto function_terminate;

   printf("===\n");
   error = rox_calibration_camproj_checkerboard_check_fine_results(
         calibration,
         &error_cam,
         &mean_error_proj_fwd, &median_error_proj_fwd,
         &mean_error_proj_bwd, &median_error_proj_bwd,
         -1.0 );

   if (error) goto function_terminate;
   printf(" Fine errors cam     : %0.10f \n", error_cam  );
   printf(" Fine errors proj fwd: ( %f, %f ) \n", mean_error_proj_fwd, median_error_proj_fwd );
   printf(" Fine errors proj bwd: ( %f, %f ) \n", mean_error_proj_bwd, median_error_proj_bwd );

   printf("=====================\n");

function_terminate:

   // Delete objects and free memory 
   rox_matrix_del(&Kc);
   rox_matrix_del(&Kp);
   rox_matrix_del(&pTc0);
   rox_matrix_del(&pTc1);

   rox_calibration_camproj_checkerboard_del(&calibration);
   rox_model_checkerboard_del(&print_model);
   rox_model_projector_checkerboard_del(&proj_model);

   // Display Error 
   rox_error_print(error);

   return error;
}
