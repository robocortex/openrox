//==============================================================================
//
//    OPENROX   : File test_odometry_single_plane_photoframe.cpp
//
//    Contents  : Tests for odometry_single_plane_photoframe.c
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
   #include <api/openrox.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(odometry_single_plane_photoframe)

#define TEST_STANDARD

#ifdef TEST_STANDARD
   // Model and database paths
   #define mod_file ROX_DATA_HOME"/regression_tests/openrox/identification/photoframe/models/photoframe_%d.pgm"

   // File paths
   #define seq_file ROX_DATA_HOME"/regression_tests/openrox/identification/photoframe/images/image_%03d.pgm"

   // Define the number of model to be detected
   #define nb_models 4
   #define nb_images 401

   // Define the camera intrinsic parameters
   #define FU 617.55
   #define FV 616.98
   #define CU 314.23
   #define CV 246.51

   #define PHOTOFRAME_TYPE 1
   #define PHOTOFRAME_BORD 20

   #define MODEL_SIZX 0.2
   #define MODEL_SIZY 0.2

#endif

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_single_plane_photoframe)
{
   // ---------- ALLOCATE GLOBAL DATA -------------------------------------
   Rox_ErrorCode error;
   Rox_Char filename[FILENAME_MAX];

   Rox_Sint n_model_tracked[nb_models];

   Rox_Double score = 0.0;
   Rox_Double score_threshold = 0.89;

   // Define timer to measure performances
   Rox_Timer timer = NULL;
   Rox_Double time = 0.0;

   // ----------- ALLOCATE VISUAL DATA ------------------------------------
   // Define the image for processing data
   Rox_Camera camera = NULL;

   // Define pose matrix
   Rox_MatSE3 pose[nb_models];

   // ----------- ALLOCATE MODEL DATA -------------------------------------
   Rox_Image model_image[nb_models] = {NULL};

   // Define the 2D model with texture and size in meters
   Rox_Model_Single_Plane model[nb_models] = {NULL};

   // Define odometry object
   Rox_Odometry_Single_Plane odometry[nb_models] = {NULL};
   Rox_Odometry_Single_Plane_Params params[nb_models] = {NULL};
   Rox_Sint tracked[nb_models];

   // Define rox photoframe identification object
   Rox_Ident_PhotoFrame_SE3 ident = NULL;

   Rox_Matrix K = NULL;
   Rox_Sint rows = 0, cols = 0;

   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);

   // Define the camera containing the first image
   sprintf(filename, seq_file, 0);

   error = rox_camera_new_read_pgm(&camera, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_camera_set_pinhole_params(camera, FU, FV, CU, CV);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_matrix_new(&K, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_camera_get_intrinsic_parameters(K, camera);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_camera_get_size(&rows, &cols, camera); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define database objects
   error = rox_ident_photoframe_se3_new(&ident, cols, rows);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_ident_photoframe_se3_set_type(ident, PHOTOFRAME_TYPE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_ident_photoframe_se3_set_score_threshold(ident, score_threshold);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define timer object
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Initialize the odometry and ident for each photoframe
   for ( Rox_Sint k = 0; k < nb_models; k++)
   {
      tracked[k] = 0;
      n_model_tracked[k] = 0;

      // Create a pose per model
      error = rox_matse3_new(&pose[k]);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      // Define filename of the model
      sprintf(filename, mod_file, k+1);

      // Load model
      error = rox_image_new_read_pgm(&model_image[k], filename);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      // Define the 2D model with texture and size in meters
      error = rox_model_single_plane_new(&model[k]); 
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      error = rox_model_single_plane_set_template_xright_ydown ( model[k], model_image[k], MODEL_SIZX, MODEL_SIZY );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      // Add photoframe
      // error = rox_ident_photoframe_se3_addframe ( ident, model_image[k], MODEL_SIZX, MODEL_SIZY, PHOTOFRAME_BORD );
      // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      // Add photoframe with model 
      error = rox_ident_photoframe_se3_addframe_model ( ident, model[k], PHOTOFRAME_BORD );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      error = rox_odometry_single_plane_params_new ( &params[k] );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      // Create odometry
      error = rox_odometry_single_plane_new ( &odometry[k], params[k], model[k] );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;
   }

   for (Rox_Sint i = 0; i < nb_images; i++)
   {
      ROX_TEST_MESSAGE("image %d \n", i);
      sprintf(filename, seq_file, i);
      error = rox_camera_read_pgm(camera, filename);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      // Make the detection using the current image
      error = rox_ident_photoframe_se3_make(ident, camera);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      for ( Rox_Sint k = 0; k < nb_models; k++)
      {
         // Check odometry status
         if (tracked[k] < 1)
         {
            Rox_Sint identified = 0;

            error = rox_ident_photoframe_se3_get_result ( &identified, &score, pose[k], ident, k );
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

            if (identified)
            {
               ROX_TEST_MESSAGE ("photoframe %d identified with the following pose:\n", k); 
               ROX_TEST_MESSAGE("identification score = %f\n", score);
               rox_matse3_print(pose[k]);

               error = rox_odometry_single_plane_set_pose(odometry[k], pose[k]);
               ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

               error = rox_odometry_single_plane_make ( odometry[k], camera );
               ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
               if (error)
               {
                  tracked[k] = 0;
               }
               else
               {
                  n_model_tracked[k]++;
                  tracked[k] = 1;
               }
            }
         }
         else
         {
            error = rox_timer_start(timer);
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

            // Make tracking
            error = rox_odometry_single_plane_make(odometry[k], camera);
            // If the score is below the threshold the odometry return an error
            if(error)
            {
                tracked[k] = 0;
            }
            else
            {
                tracked[k] = 1;
            }

            error = rox_timer_stop(timer);
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

            error = rox_timer_get_elapsed_ms(&time, timer);
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

            ROX_TEST_MESSAGE("time for odometry of template [%d] = %f\n", k, time);
         }

         if(tracked[k] == 1)
         {
             error = rox_odometry_single_plane_get_score(&score, odometry[k]);
             ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

             ROX_TEST_CHECK_SUPERIOR_OR_EQUAL(score, 0.80);

             error = rox_odometry_single_plane_get_pose(pose[k], odometry[k]);
             ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

             n_model_tracked[k]++;

             ROX_TEST_MESSAGE("odometry score[%d] = %f\n", k, score);
             rox_matse3_print(pose[k]);
         }
      }
   }

   for (Rox_Sint k = 0; k<nb_models; k++)
   {
      ROX_TEST_MESSAGE("n_model_tracked[%d] = %d\n", k, n_model_tracked[k]);
   }

#if 0
   // OPTIMIZED
   ROX_TEST_CHECK_SMALL(n_model_tracked[0], 179);
   ROX_TEST_CHECK_SMALL(n_model_tracked[1], 136);
   ROX_TEST_CHECK_SMALL(n_model_tracked[2], 145);
   ROX_TEST_CHECK_SMALL(n_model_tracked[3], 196);
   // ANSI
   ROX_TEST_CHECK_SMALL(n_model_tracked[0], 179);
   ROX_TEST_CHECK_SMALL(n_model_tracked[1], 136);
   ROX_TEST_CHECK_SMALL(n_model_tracked[2], 146);
   ROX_TEST_CHECK_SMALL(n_model_tracked[3], 197);
#endif
   // workaround to test both ANSI and OPTIMIZED case
   // TODO : write separate tests for each case ?

#ifdef TEST_STANDARD
   ROX_TEST_CHECK_SMALL( (double) (n_model_tracked[0] - 181), 2);
   ROX_TEST_CHECK_SMALL( (double) (n_model_tracked[1] - 138), 2);
   ROX_TEST_CHECK_SMALL( (double) (n_model_tracked[2] - 148), 2);
   ROX_TEST_CHECK_SMALL( (double) (n_model_tracked[3] - 205), 2);
#endif

function_terminate:

   // Delete objects and free memory
   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_camera_del(&camera);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del(&K);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ident_photoframe_se3_del(&ident);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for ( Rox_Sint k = 0; k < nb_models; k++)
   {
       error = rox_matse3_del(&pose[k]);
       ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

       error = rox_image_del(&model_image[k]);
       ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

       error = rox_model_single_plane_del(&model[k]);
       ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

       error = rox_odometry_single_plane_params_del(&params[k]);
       ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

       error = rox_odometry_single_plane_del(&odometry[k]);
       ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }
}

ROX_TEST_SUITE_END()
