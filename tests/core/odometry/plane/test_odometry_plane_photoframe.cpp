//==============================================================================
//
//    OPENROX   : File test_odometry_plane_photoframe.cpp
//
//    Contents  : Tests for odometry_plane_photoframe.c
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

ROX_TEST_SUITE_BEGIN(odometry_plane_photoframe)

#if 1
// Model and database paths
#define PHOTOFRAME_1_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/photoframe/models/photoframe_1.pgm"
#define PHOTOFRAME_2_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/photoframe/models/photoframe_2.pgm"
#define PHOTOFRAME_3_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/photoframe/models/photoframe_3.pgm"
#define PHOTOFRAME_4_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/photoframe/models/photoframe_4.pgm"

// File paths
#define IMAGE_PATH        ROX_DATA_HOME"/regression_tests/openrox/identification/photoframe/images/image_000.pgm"

// Define the number of model to be detected
#define nb_models 4
#define nb_images 401
#define NBI 401

// Define the camera intrinsic parameters
#define FU 617.55
#define FV 616.98
#define CU 314.23
#define CV 246.51
#endif

// #define mod_file2  "/home/emalis/Tmp/debug/photoframe/squares/photoframe_texture_128_00.pgm"
// #define seq_file2 "/home/emalis/Tmp/debug/photoframe/squares/capturephoto_0.pgm"
// #define FU 1388.518897760294
// #define FV 1380.624743164325
// #define CU 975.5068896278956
// #define CV 544.0110137581285

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

#if 0
// PERFECT DATA
// Model and database paths
#define PHOTOFRAME_1_PATH "/home/emalis/Tmp/debug/prediction/photoframe_augmentedpro_00_model_128x128.pgm"

// File paths
#define IMAGE_PATH "/home/emalis/Tmp/debug/prediction/image_photoframe_centered_1.pgm"

// Define the number of model to be detected
#define nb_models 1
#define nb_images 1
#define NBI 1

// Define the camera intrinsic parameters
#define FU  128.0
#define FV  128.0
#define CU  959.5
#define CV  539.5
#endif

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_plane_photoframe)
{
   Rox_Double data[16] = {0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0};

   // Test repeatability  of odometry

   // ---------- ALLOCATE GLOBAL DATA -------------------------------------
   Rox_ErrorCode error;
   Rox_Char filename[FILENAME_MAX];

   //Rox_Sint n_model_tracked = 0;

   Rox_Double score = 0.0;

   //Rox_Char mod_file[FILENAME_MAX];

   // Define timer to measure performances
   Rox_Timer timer = NULL;

   // ----------- ALLOCATE VISUAL DATA ------------------------------------
   // Define the image for processing data
   Rox_Camera camera = NULL;

   Rox_Sint cols = 0;
   Rox_Sint rows = 0;

   // Define pose matrix
   Rox_MatSE3 pose = NULL;
   Rox_Sint identified = 0;

   // ----------- ALLOCATE MODEL DATA -------------------------------------

   // Real size of the target (in meters)
   Rox_Double  model_sizex = 1.0;//0.025333;
   Rox_Double  model_sizey = 1.0;//0.025333;

   Rox_Image model_image = NULL;

   // Define the 2D model with texture and size in meters
   Rox_Model_Single_Plane model_single_plane = NULL;

   // Define odometry object
   Rox_Odometry_Single_Plane odometry = NULL;
   Rox_Odometry_Single_Plane_Params params = NULL;
   //Rox_Sint tracked = 0;

   // Define rox photoframe identification object
   Rox_Ident_PhotoFrame_SE3 ident = NULL;
   Rox_Sint white_border = 16;

   Rox_Matrix K = NULL;

   Rox_Double score_threshold = 0.89;

   sprintf(filename, "%s", PHOTOFRAME_1_PATH);
   rox_log("read file %s\n", filename);

   // Load model for identification
   error = rox_image_new_read_pgm(&model_image, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define the 2D model with texture and size in meters
   error = rox_model_single_plane_new ( &model_single_plane );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_model_single_plane_set_template_xright_ydown ( model_single_plane, model_image, model_sizex, model_sizey );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);

   error = rox_camera_new_read_pgm(&camera, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_camera_set_pinhole_params(camera, FU, FV, CU, CV);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_camera_get_size(&rows, &cols, camera);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Initialize the identification
   error = rox_ident_photoframe_se3_new(&ident, cols, rows);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_ident_photoframe_se3_set_type(ident, 0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_ident_photoframe_se3_set_score_threshold(ident, score_threshold);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Add photoframe
   error = rox_ident_photoframe_se3_addframe_model(ident, model_single_plane, white_border);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define timer object
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Initialize the odometry

   // Crete a pose per model
   error = rox_matse3_new(&pose);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_odometry_single_plane_params_new(&params);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_odometry_single_plane_params_set_init_pyr(params, 0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Create odometry single plane
   error = rox_odometry_single_plane_new(&odometry, params, model_single_plane);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;
  
   error = rox_odometry_single_plane_set_score_thresh(odometry, score_threshold);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_matrix_new(&K, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_camera_get_intrinsic_parameters(K, camera);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Make the detection using the current image
   error = rox_ident_photoframe_se3_make(ident, camera);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_ident_photoframe_se3_get_pose(&identified, pose, ident, 0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   rox_log("pose after ident \n");
   rox_array2d_double_print(pose); 

   rox_log("forced pose  \n");

   rox_matse3_set_data(pose, data);
   rox_array2d_double_print(pose); 

   if (identified)
   {
      error = rox_ident_photoframe_se3_get_score( &score, ident, 0 );                     
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      rox_log("photoframe identified with score = %16.16f \n",score);
      error = rox_odometry_single_plane_set_pose(odometry, pose);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;
            
      for (Rox_Sint k =0; k<1; k++)
      {
         error = rox_odometry_single_plane_make(odometry, camera);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;
                     
         error = rox_odometry_single_plane_get_score(&score, odometry);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

         rox_log("photoframe tracked using single_plane odometry with score = %16.16f \n",score);
         ROX_TEST_CHECK_SUPERIOR_OR_EQUAL(score, 0.80);

         error = rox_odometry_single_plane_get_pose(pose, odometry);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

         rox_log("pose after odometry \n");
         rox_array2d_double_print(pose);
      }
   }

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

   error = rox_matse3_del(&pose);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del(&model_image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_model_single_plane_del(&model_single_plane);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_single_plane_params_del(&params);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_single_plane_del(&odometry);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
