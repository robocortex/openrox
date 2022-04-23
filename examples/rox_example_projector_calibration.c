//==============================================================================
//
//    OPENROX   : File rox_example_projector_calibration.c
//
//    Contents  : A simple example program for projector calibration.
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== HEADERS   ================================================================

#include <api/openrox.h>

Rox_Uint i = 0;

//=== MAIN PROGRAM =============================================================

Rox_Sint main (Rox_Void)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint nbimg = 2;
   Rox_Uint method = 5;

   // Define the result object
   Rox_Matrix Ke = 0;
   Rox_Matrix T0 = 0;
   Rox_Matrix T1 = 0;

   // Define the 2D model
   Rox_Model_Projector_CheckerBoard model = NULL;
   Rox_Double space_width = 1920;
   Rox_Double space_height = 1080;
   Rox_Sint cols  = 2;
   Rox_Sint rows = 2;
   Rox_Uint image_width = 1920;
   Rox_Uint image_height = 1080;

   // Define the calibration object
   Rox_Calibration_Projector_CheckerBoard calibration = NULL;

   // Define the 2D model
   error = rox_model_projector_checkerboard_new(&model);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error =  rox_model_projector_checkerboard_set_template(model, cols, rows, space_width, space_height, image_width, image_height );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create the intrinsic matrix
   error = rox_matrix_new(&Ke, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Define the calibration object
   error = rox_calibration_projector_checkerboard_new(&calibration, model);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create the extrinsic matrices
   error = rox_matrix_new(&T0, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new(&T1, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create 3D z-sliced points, in world frame
   Rox_Point2D_Double_Struct obs2D[2][4];
   {
      obs2D[0][0].u =
         //-1.98477;
      -1.984772084919771;
      obs2D[0][0].v =
         //-0.51637;
      -0.516366991049261;
      obs2D[0][1].u =
         //1.64672;
      1.646715164139644;
      obs2D[0][1].v =
         //-0.71019;
      -0.710191425555277;
      obs2D[0][2].u =
         //-1.88203;
      -1.882026324859579;
      obs2D[0][2].v =
         //1.57674;
      1.576740948008220;
      obs2D[0][3].u =
         //1.79949;
      1.799488674326019;
      obs2D[0][3].v =
         //1.31023;
      1.310234435560389;

      obs2D[1][0].u =
         //-2.16223;
      -2.162226600736120;
      obs2D[1][0].v =
         //-0.58824;
      -0.588238416002130;
      obs2D[1][1].u =
         //1.69506;
      1.695055580233518;
      obs2D[1][1].v =
         //-0.75172;
      -0.751717127960673;
      obs2D[1][2].u =
         //-2.01176;
      -2.011758422377116;
      obs2D[1][2].v =
         //1.67551;
      1.675505648222384;
      obs2D[1][3].u =
         //1.83802;
      1.838021571832821;
      obs2D[1][3].v =
         //1.31499;
      1.314990707574192;
   }

   // Add calibration images to a set
   for(i = 0; i < nbimg; i++)
   {
      error = rox_calibration_projector_checkerboard_add_image( calibration, obs2D[i], 4);
      if(error)
      {
        printf("The given %02d data cannot be used for camera calibration \n", i+1);
      }
      else
      {
        printf("Added image %02d to calibration set \n", i+1);
      }
   }

   error = rox_calibration_projector_checkerboard_make(calibration, method);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_projector_checkerboard_get_intrinsics(Ke, calibration);
   ROX_ERROR_CHECK_TERMINATE ( error );

   printf("\n");

   printf("The estimated intrinsic parameters are: \n");
   error = rox_matrix_print(Ke);
   ROX_ERROR_CHECK_TERMINATE ( error );

   printf("The expected  intrinsic parameters are: \n");
   printf("Matrix (3x3) \n");
   printf("1000.0 0.0000 960.0 \n");
   printf("0.0000 1000.0 540.0 \n");
   printf("0.0000 0.0000 1.000 \n");

   printf("\n");

   error = rox_calibration_projector_checkerboard_get_pose(T0, calibration, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_projector_checkerboard_get_pose(T1, calibration, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   printf("The estimated pose parameters are: \n");
   error = rox_matrix_print(T0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   printf("and: \n");
   error = rox_matrix_print(T1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   printf("The expected  pose parameters are: \n");
   printf("Matrix (4x4) \n");
   printf("  0.99790  -0.06243   0.01753   0.10000 \n");
   printf("  0.06268   0.99794  -0.01375  -0.40000 \n");
   printf(" -0.01664   0.01482   0.99975   1.90000 \n");
   printf("  0.00000   0.00000   0.00000   1.00000 \n");

   printf("and: \n");
   printf("Matrix (4x4) \n");
   printf("  0.99659  -0.06765   0.04725   0.10000 \n");
   printf("  0.06779   0.99770  -0.00140  -0.40000 \n");
   printf(" -0.04704   0.00460   0.99888   2.00000 \n");
   printf("  0.00000   0.00000   0.00000   1.00000 \n");

function_terminate:

   // Delete objects and free memory
   ROX_ERROR_CHECK(rox_calibration_projector_checkerboard_del(&calibration));
   rox_matrix_del(&T0);
   rox_matrix_del(&T1);

   rox_matrix_del(&Ke);
   rox_model_projector_checkerboard_del(&model);

   // Display Error
   rox_error_print(error);
   return error;
}
