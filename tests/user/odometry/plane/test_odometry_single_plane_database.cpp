//==============================================================================
//
//    OPENROX   : File test_odometry_single_plane_database.cpp
//
//    Contents  : Tests for odometry_single_plane_database.c
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

ROX_TEST_SUITE_BEGIN(odometry_single_plane_database)

// Model and database paths
#define mod_file_01 ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/toy_128x128.pgm"
#define rdi_file_01 ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/toy_512x512.rdi"

#define mod_file_02 ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/gormiti_128x178.pgm"
#define rdi_file_02 ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/gormiti_517x719.rdi"

// File paths
#define seq_file    ROX_DATA_HOME"/regression_tests/openrox/identification/database/images/toytest%03d.pgm"

// Define the number of model to be detected
#define nb_models 2
#define nb_images 45

// Define the camera intrinsic parameters
#define PX 559.279
#define PY 559.279
#define U0 329.890
#define V0 242.164

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_single_plane_database)
{
   // ---------- ALLOCATE GLOBAL DATA -------------------------------------
   Rox_ErrorCode error;
   Rox_Char filename[FILENAME_MAX] ;

   Rox_Sint i = 0, k = 0;
   Rox_Double score;

   Rox_Sint n_model_tracked[nb_models] = {0, 0};

   Rox_Char mod_file[nb_models][1024] = {mod_file_01, mod_file_02};
   Rox_Char rdi_file[nb_models][1024] = {rdi_file_01, rdi_file_02};

   // Define timer to measure performances
   Rox_Timer timer = 0;

   // ----------- ALLOCATE VISUAL DATA ------------------------------------
   // Define the image for processing data
   Rox_Camera camera = 0;

   // Define pose matrix
   Rox_MatSE3 pose = 0;

   // ----------- ALLOCATE MODEL DATA -------------------------------------

   // Real size of the target (in meters)
   Rox_Double  model_sizex[nb_models] = {0.2, 0.14};
   Rox_Double  model_sizey[nb_models] = {0.2, 0.2};

   //Rox_Double  model_sizex[nb_models] = {512.0, 517.0};
   //Rox_Double  model_sizey[nb_models] = {512.0, 719.0};

   Rox_Image model_image[nb_models];

   // Define the 2D model with texture and size in meters
   Rox_Model_Single_Plane model[nb_models];

   // Define odometry object
   Rox_Odometry_Single_Plane odometry[nb_models];
   Rox_Odometry_Single_Plane_Params params[nb_models];

   // Define rox database identification object
   Rox_Ident_Database_SE3 ident = 0;
   Rox_Database_Item item = 0;
   Rox_Database database = 0;
   Rox_Matrix K = 0;

   //init in case of fail, in order for cleanup to be done without seg faults...
   for (k = 0; k < nb_models; k++)
   {
      model[k] = NULL;
      params[k] = NULL;
      model_image[k] = NULL;
      odometry[k] = NULL;
   }

   // Define database objects
   error = rox_ident_database_se3_new(&ident, nb_models);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

   error = rox_database_new(&database);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

   error = rox_database_item_new(&item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

   error = rox_matse3_new(&pose);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

   // Define timer object
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

   rox_log("Initialize models \n");
   // Initialize the odometry and ident
   for(k = 0; k < nb_models; k++)
   {
      // Load model
      sprintf(filename,  "%s", mod_file[k]);

      error = rox_image_new_read_pgm(&model_image[k], filename);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

      // Define the 2D model with texture and size in meters
      error = rox_model_single_plane_new(&model[k]);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

      error = rox_model_single_plane_set_template_xright_ydown(model[k], model_image[k], model_sizex[k], model_sizey[k]);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

      // Load db item
      sprintf(filename, "%s", rdi_file[k]);

      error = rox_database_item_load(item, filename);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

      // Add item to the database
      error = rox_database_add_item(database, item, model_sizex[k], model_sizey[k]);
      // error = rox_database_add_item_default_size(database, item);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

      error = rox_odometry_single_plane_params_new(&params[k]);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

      // Create odometry
      error = rox_odometry_single_plane_new(&odometry[k], params[k], model[k]);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;
   }

   // Compile database
   error = rox_database_compile(database);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

   error = rox_ident_database_se3_set_database(ident, database);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

   // Define the camera containing the first image
   sprintf(filename, seq_file, 0);
   
   error = rox_camera_new_read_pgm(&camera, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

   error = rox_camera_set_pinhole_params(camera, PX, PY, U0, V0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

   error = rox_matrix_new(&K, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

   error = rox_camera_get_intrinsic_parameters(K, camera);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

   rox_log("Start odometry \n");
   for (i = 0; i < nb_images; i++)
   {
      sprintf(filename, seq_file, i);
      error = rox_camera_read_pgm(camera, filename);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

      // Make the detection using the current image
      error = rox_ident_database_se3_make(ident, camera);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

      for(k = 0; k < 1/*nb_models*/; k++)
      {
         Rox_Sint is_identified = 0;

         error = rox_ident_database_se3_getresult(&is_identified, pose, ident, k);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

         if(is_identified)
         {
            Rox_Double sizex = model_sizex[k];
            error = rox_ident_database_se3_get_result_force_sizex(&is_identified, pose, ident, sizex, k);
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;
            // rox_matse3_print(pose);

            Rox_Double sizey = model_sizey[k];
            error = rox_ident_database_se3_get_result_force_sizey(&is_identified, pose, ident, sizey, k);
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;
            // rox_matse3_print(pose);

            error = rox_odometry_single_plane_set_pose(odometry[k], pose);
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

             error = rox_odometry_single_plane_make(odometry[k], camera);
             if(!error)
             {
                  error = rox_odometry_single_plane_get_score(&score, odometry[k]);
                  ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

                  ROX_TEST_CHECK_SUPERIOR_OR_EQUAL(score, 0.89);

                  error = rox_odometry_single_plane_get_pose(pose, odometry[k]);
                  ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

                  // rox_matse3_print(pose);
                  n_model_tracked[k]++;
             }
         }
      }
   }

function_return:

   // Delete objects and free memory
   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_camera_del(&camera);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del(&pose);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del(&K);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ident_database_se3_del(&ident);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_item_del(&item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_del ( &database );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE );

   for(k = 0; k < nb_models; k++)
   {
       error = rox_image_del(&model_image[k] );
       ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

       error = rox_model_single_plane_del ( &model[k] );
       ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

       error = rox_odometry_single_plane_params_del ( &params[k] );
       ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

       error = rox_odometry_single_plane_del ( &odometry[k] );
       ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }
}

ROX_TEST_SUITE_END()
