//==============================================================================
//
//    OPENROX   : File test_odometry_multi_plane_photoframe.cpp
//
//    Contents  : Tests for odometry_multi_plane_photoframe.c
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
   #include <user/identification/photoframe/ident_multi_photoframe_se3.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(odometry_multi_plane_photoframe)

#define TEST_STANDARD

#ifdef TEST_STANDARD
   // Model and database paths
   #define MODEL_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/photoframe/models/photoframe_%d.pgm"

   // File paths
   #define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/photoframe/images/image_%03d.pgm"


   // Define the camera intrinsic parameters
   #define FU 617.55
   #define FV 616.98
   #define CU 314.23
   #define CV 246.51   

   // Define the number of model to be detected
   #define NBP 4

   // Define the number of images
   #define NBI 401

   // Define the photoframe parameters
   #define PHOTOFRAME_TYPE 1
   #define PHOTOFRAME_BORD 20

   #define MODEL_SIZX 0.2
   #define MODEL_SIZY 0.2
#endif

// #define mod_file2  "/home/emalis/Tmp/debug/photoframe/squares/photoframe_texture_128_00.pgm"
// #define seq_file2 "/home/emalis/Tmp/debug/photoframe/squares/capturephoto_0.pgm"
// #define FU 1388.518897760294
// #define FV 1380.624743164325
// #define CU 975.5068896278956
// #define CV 544.0110137581285

#if 0
   // Perfect data
   #define PHOTOFRAME_1_PATH "/home/emalis/Tmp/debug/prediction/photoframe_augmentedpro_00_model_128x128.pgm"
   #define PHOTOFRAME_2_PATH "/home/emalis/Tmp/debug/prediction/photoframe_augmentedpro_00_model_128x128.pgm"
   #define IMAGE_PATH "/home/emalis/Tmp/debug/prediction/image_photoframe_centered_1.pgm"

   #define FU  128.0
   #define FV  128.0
   #define CU  959.5
   #define CV  539.5

   #define NBI  1
   #define PHOTOFRAME_TYPE 0
   #define PHOTOFRAME_BORD 16

   #define MODEL_SIZX 1.0
   #define MODEL_SIZY 1.0
#endif

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_multi_plane_single_photoframe)
{
   // ---------- ALLOCATE GLOBAL DATA -------------------------------------
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Sint n_model_tracked[NBP];

   Rox_Double score = 0.0;
   Rox_Double score_threshold = 0.80; // 0.89

   // Define timer to measure performances
   Rox_Timer timer = NULL;
   Rox_Double time = 0.0;

   // ----------- ALLOCATE VISUAL DATA ------------------------------------
   // Define the image for processing data
   Rox_Camera camera = NULL;
   Rox_MatUT3 K = NULL;
   Rox_Sint rows = 0, cols = 0;

   // Define pose matrix
   Rox_MatSE3 pose[NBP];

   // ----------- ALLOCATE MODEL DATA -------------------------------------

   // Real size of the target (in meters)
   Rox_Image model_image[NBP] = {NULL};

   // Define ome multiplane model per photoframe
   Rox_Model_Multi_Plane model[NBP] = {NULL};

   // Define odometry object
   Rox_Odometry_Multi_Plane odometry[NBP] = {NULL};
   Rox_Odometry_Multi_Plane_Params odometry_params[NBP] = {NULL};

   Rox_Sint tracked[NBP] ;

   // Define rox photoframe identification object
   Rox_Ident_PhotoFrame_SE3 ident = NULL;

   // Define the camera containing the first image
   sprintf(filename, IMAGE_PATH, 0);

   rox_log("Reading image file %s \n", filename);

   error = rox_camera_new_read_pgm ( &camera, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_camera_set_pinhole_params(camera, FU, FV, CU, CV);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_matrix_new ( &K, 3, 3 );
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
   for ( Rox_Sint k = 0; k < NBP; k++)
   {
      n_model_tracked[k] = 0;
      tracked[k] = 0;

      // Crete a pose per model
      error = rox_matse3_new(&pose[k]);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      // Define filename of the model
      sprintf(filename, MODEL_PATH, k+1);
      rox_log("Reading model file %s \n", filename);

      // Load model
      error = rox_image_new_read_pgm ( &model_image[k], filename );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      // Define the 2D model with texture and size in meters
      error = rox_model_multi_plane_new ( &model[k] );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      Rox_Point3D_Double_Struct vertices[4];

      error = rox_rectangle3d_create_centered_plane_xright_ydown ( vertices, MODEL_SIZX, MODEL_SIZY );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      error = rox_model_multi_plane_append_plane ( model[k], model_image[k], vertices );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      // Add photoframe
      Rox_Model_Single_Plane model_single_plane = NULL;

      // Define the 2D model with texture and size in meters
      error = rox_model_single_plane_new ( &model_single_plane );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      error = rox_model_single_plane_set_template_xright_ydown ( model_single_plane, model_image[k], MODEL_SIZX, MODEL_SIZY );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      error = rox_ident_photoframe_se3_addframe_model ( ident, model_single_plane, PHOTOFRAME_BORD );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      error = rox_odometry_multi_plane_params_new ( &odometry_params[k] );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      // Create odometry
      error = rox_odometry_multi_plane_new ( &odometry[k], odometry_params[k], model[k] );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      error = rox_odometry_multi_plane_set_score_threshold ( odometry[k], score_threshold );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   }

   for (Rox_Sint i = 0; i < NBI; i++)
   {
      sprintf(filename, IMAGE_PATH, i);
      rox_log("Reading image %s \n", filename);

      error = rox_camera_read_pgm(camera, filename);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      if(error) goto function_terminate;

      // Make the detection using the current image
      error = rox_ident_photoframe_se3_make ( ident, camera );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      if(error) goto function_terminate;

      for ( Rox_Sint k = 0; k < NBP; k++)
      {
         // Check odometry status
         if (tracked[k] < 1)
         {
            Rox_Sint identified = 0;

            error = rox_ident_photoframe_se3_get_result ( &identified, &score, pose[k], ident, k );
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

            if (identified)
            {
               printf ("photoframe %d identified with the following pose:\n", k); 
               rox_log("identification score = %f\n", score);
               rox_matse3_print(pose[k]);

               error = rox_odometry_multi_plane_set_pose (odometry[k], pose[k]);
               ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

               error = rox_timer_start(timer);
               ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

               error = rox_odometry_multi_plane_make ( odometry[k], model[k], camera);
               ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 

               error = rox_timer_stop(timer);
               ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

               error = rox_timer_get_elapsed_ms(&time, timer);
               ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

               rox_log("time spent for odometry of template [%d] = %f\n", k, time);

               if (error)
               {
                  tracked[k] = 0;
               }
               else
               {
                  tracked[k] = 1;
               }
            }
         }
         else
         {
            error = rox_timer_start(timer);
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

            // Make tracking
            error = rox_odometry_multi_plane_make(odometry[k], model[k], camera);
            // If the score is below the threshold the odometry return an error
            if (error)
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

            rox_log("time for odometry of template [%d] = %f\n", k, time);
         }

         if(tracked[k] == 1)
         {
             error = rox_odometry_multi_plane_get_score(&score, odometry[k]);
             ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
             if(error) goto function_terminate;

             ROX_TEST_CHECK_SUPERIOR_OR_EQUAL(score, score_threshold);

             error = rox_odometry_multi_plane_get_pose(pose[k], odometry[k]);
             ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
             if(error) goto function_terminate;

             n_model_tracked[k]++;

             rox_log("odometry score[%d] = %f\n", k, score);
             rox_matse3_print(pose[k]);
         }
      }
   }

   for ( Rox_Sint k = 0; k < NBP; k++)
   {
      rox_log("n_model_tracked[%d] = %d\n", k, n_model_tracked[k]);
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

   // ROX_TEST_CHECK_SMALL((double)(n_model_tracked[0] - 179), 1.1);
   // ROX_TEST_CHECK_SMALL((double)(n_model_tracked[1] - 136), 1.1);
   // ROX_TEST_CHECK_SMALL((double)(n_model_tracked[2] - 145), 1.1);
   // ROX_TEST_CHECK_SMALL((double)(n_model_tracked[3] - 201), 1.1);

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

   for (Rox_Sint k = 0; k < NBP; k++)
   {
      error = rox_matse3_del(&pose[k]);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_image_del(&model_image[k]);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_model_multi_plane_del(&model[k]);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_odometry_multi_plane_params_del(&odometry_params[k]);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_odometry_multi_plane_del(&odometry[k]);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }
}

#ifdef newtest

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_multi_plane_photoframe)
{
   // ---------- ALLOCATE GLOBAL DATA -------------------------------------
   Rox_ErrorCode error;
   Rox_Char filename[FILENAME_MAX];

   Rox_Double score = 0.0;
   Rox_Double score_threshold = 0.8; // 0.89

   // Define timer to measure performances
   Rox_Timer timer = NULL;
   Rox_Double time = 0.0;

   // ----------- ALLOCATE VISUAL DATA ------------------------------------
   // Define the image for processing data
   Rox_Camera camera = NULL;
   Rox_MatUT3 K = NULL;
   Rox_Sint rows = 0, cols = 0;

   Rox_Sint identified = 0;

   // Define pose matrix
   Rox_MatSE3 pose = NULL;
   Rox_MatSE3 *pTo = (Rox_MatSE3 *) rox_memory_allocate( sizeof ( Rox_MatSE3 ), NBP );

   // ----------- ALLOCATE MODEL DATA -------------------------------------

   double        * models_vertices = NULL;
   int           * models_border = NULL;

   Rox_Point3D_Double_Struct o_points[4], p_points[4];

   Rox_Double model[4*NBP][3] = MODEL;

   Rox_Double shift_x[NBP] = SHIFT_X;
   Rox_Double shift_y[NBP] = SHIFT_Y;

   // Real size of the target (in meters)
   Rox_Image model_image[NBP] = {NULL};

   // Define a multiplane model for all photoframes
   Rox_Model_Multi_Plane model_multi_plane = {NULL};

   // Define odometry object
   Rox_Odometry_Multi_Plane odometry = {NULL};
   Rox_Odometry_Multi_Plane_Params odometry_params = {NULL};

   Rox_Sint tracked;
   Rox_Sint best_photoframe_id;

   // Define rox photoframe identification object
   Rox_Ident_Multi_PhotoFrame_SE3 ident = NULL;

   // Define the camera containing the first image
   sprintf(filename, IMAGE_PATH, 0);

   // Crete a unique pose for all models (the main frame is in the center of the first model )
   error = rox_matse3_new(&pose);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;
      
   error = rox_camera_new_read_pgm ( &camera, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_camera_set_pinhole_params ( camera, FU, FV, CU, CV );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_matrix_new ( &K, 3, 3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_camera_get_intrinsic_parameters ( K, camera );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_camera_get_size(&rows, &cols, camera); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define database objects
   error = rox_ident_multi_photoframe_se3_new ( &ident, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_ident_multi_photoframe_se3_set_type(ident, PHOTOFRAME_TYPE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_ident_multi_photoframe_se3_set_score_threshold ( ident, score_threshold );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define timer object
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   models_vertices = (double *) rox_memory_allocate(sizeof(double), 3*4*NBP);
   models_border = (int *) rox_memory_allocate(sizeof(int), NBP);

   for (Rox_Sint k = 0; k<NBP; k++)
   {
      models_vertices[k*3*4+0 ] = model[4*k+0][0] + shift_x[k];
      models_vertices[k*3*4+1 ] = model[4*k+0][1] + shift_y[k];
      models_vertices[k*3*4+2 ] = model[4*k+0][2];

      models_vertices[k*3*4+3 ] = model[4*k+1][0] + shift_x[k];
      models_vertices[k*3*4+4 ] = model[4*k+1][1] + shift_y[k];
      models_vertices[k*3*4+5 ] = model[4*k+1][2];
      
      models_vertices[k*3*4+6 ] = model[4*k+2][0] + shift_x[k];
      models_vertices[k*3*4+7 ] = model[4*k+2][1] + shift_y[k];
      models_vertices[k*3*4+8 ] = model[4*k+2][2];

      models_vertices[k*3*4+9 ] = model[4*k+3][0] + shift_x[k];
      models_vertices[k*3*4+10] = model[4*k+3][1] + shift_y[k];
      models_vertices[k*3*4+11] = model[4*k+3][2];
   }

   error = rox_model_multi_plane_new ( &model_multi_plane ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   tracked = 0;

   //==========================================================================================================

   // The corners of the main photoframe

   // Set the first 4 model vertices to o_points
   error = rox_vector_point3d_double_set_data ( o_points, models_vertices, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Process each photoframe
   for ( int k = 0; k < NBP; k++ )
   {
      // Create multi plane model
      sprintf(filename, MODEL_PATH, k+1);
      rox_log("read file %s\n", filename);

      // Load model for identification
      error = rox_image_new_read_pgm ( &model_image[k], filename );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;
      // Vertices :
      // p_points contains the vertices of the current photoframe

      // Set the 4 model vertices to p_points
      error = rox_vector_point3d_double_set_data ( p_points, &models_vertices[k*12], 4 );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_matse3_new( &pTo[k] );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute the poses between the photoframe F_i and the global object frame Fo
      error = rox_matse3_from_photoframe_vertices_vector ( pTo[k], p_points, o_points  );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_model_multi_plane_append_plane ( model_multi_plane, model_image[k], p_points );
      ROX_ERROR_CHECK_TERMINATE ( error );

      models_border[k] = PHOTOFRAME_BORD;
   }

   // Set the
   error = rox_ident_multi_photoframe_se3_add_model ( ident, model_multi_plane, models_border[0] );
   ROX_ERROR_CHECK_TERMINATE ( error );

//==========================================================================

   error = rox_odometry_multi_plane_params_new ( &odometry_params );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Create odometry
   error = rox_odometry_multi_plane_new ( &odometry, odometry_params, model_multi_plane );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_odometry_multi_plane_set_score_threshold ( odometry, score_threshold );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   for (Rox_Sint i = 0; i < NBI; i++)
   {
      rox_log("image %d \n", i);
      sprintf(filename, IMAGE_PATH, i);

      error = rox_camera_read_pgm ( camera, filename );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      // Make the detection using the current image
      error = rox_ident_multi_photoframe_se3_make_optimal ( ident, camera );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      error = rox_ident_multi_photoframe_se3_get_best_result ( &identified, &best_photoframe_id, &score, pose, ident );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      rox_log("Best photoframe %d pose cTo: \n", best_photoframe_id);
      rox_matse3_print(pose);

      error = rox_ident_multi_photoframe_se3_get_optimal_result ( &identified, &score, pose, ident );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      rox_log("Optimal pose cTo: \n");
      rox_matse3_print(pose);

      // Check odometry status
      if (tracked < 1)
      {
         if (identified)
         {

            rox_matse3_print(pose);

            error = rox_odometry_multi_plane_set_pose ( odometry, pose );
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

            error = rox_odometry_multi_plane_make ( odometry, model_multi_plane, camera);
            rox_error_print ( error );

            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
            if (error)
            {
               tracked = 0;
            }
            else
            {
               tracked = 1;
            }
         }
      }
      else
      {
         error = rox_timer_start(timer);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

            // Make tracking
            error = rox_odometry_multi_plane_make(odometry, model_multi_plane, camera);
            // If the score is below the threshold the odometry return an error
            if (error)
            {
              tracked = 0;
            }
            else
            {
              tracked = 1;
            }

            error = rox_timer_stop(timer);
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

            error = rox_timer_get_elapsed_ms(&time, timer);
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

            rox_log("time for odometry of multiplane = %f\n", time);
         }

         if(tracked == 1)
         {
             error = rox_odometry_multi_plane_get_score(&score, odometry);
             ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
             if(error) goto function_terminate;

             ROX_TEST_CHECK_SUPERIOR_OR_EQUAL(score, score_threshold);

             error = rox_odometry_multi_plane_get_pose(pose, odometry);
             ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
             if(error) goto function_terminate;

             rox_log("score = %f\n", score);
             rox_matse3_print(pose);
         }
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

   // ROX_TEST_CHECK_SMALL((double)(n_model_tracked[0] - 179), 1.1);
   // ROX_TEST_CHECK_SMALL((double)(n_model_tracked[1] - 136), 1.1);
   // ROX_TEST_CHECK_SMALL((double)(n_model_tracked[2] - 145), 1.1);
   // ROX_TEST_CHECK_SMALL((double)(n_model_tracked[3] - 201), 1.1);

function_terminate:

   // Delete objects and free memory
   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_camera_del(&camera);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del(&K);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ident_multi_photoframe_se3_del(&ident);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      
   error = rox_matse3_del ( &pose );
   ROX_TEST_CHECK_EQUAL (error, ROX_ERROR_NONE);

   error = rox_model_multi_plane_del ( &model_multi_plane );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_multi_plane_params_del ( &odometry_params );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_multi_plane_del ( &odometry );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Sint k = 0; k < NBP; k++)
   {
      error = rox_image_del ( &model_image[k] );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }
}

#endif

ROX_TEST_SUITE_END()
