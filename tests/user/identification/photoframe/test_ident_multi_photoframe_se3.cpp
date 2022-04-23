//==============================================================================
//
//    OPENROX   : File test_ident_multi_photoframe_se3.cpp
//
//    Contents  : Tests for ident_multi_photoframe_se3.c
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
#ifndef WIN32
   // Include needed to create directory
   #include <sys/types.h>
   #include <sys/stat.h>
   #include <unistd.h>
#endif

   #include <generated/dynvec_point3d_double.h>
   #include <generated/dynvec_point3d_double_struct.h>
   #include <generated/dynvec_ehid_point.h>
   #include <system/time/timer.h>

   #include <baseproc/image/draw/color.h>
   #include <baseproc/image/draw/image_rgba_draw_projection_model_single_plane.h>
   #include <baseproc/image/draw/image_rgba_draw_projection_model_multi_plane.h>
   #include <baseproc/image/image_rgba.h>
   #include <baseproc/image/draw/image_rgba_draw.h>
   #include <baseproc/geometry/point/dynvec_point3d_tools.h>
   #include <baseproc/geometry/point/point3d_tools.h>

   #include <inout/image/ppm/ppmfile.h>
   #include <inout/geometry/point/dynvec_point3d_print.h>

   #include <user/identification/photoframe/ident_multi_photoframe_se3.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN ( ident_multi_photoframe_se3 )

#ifdef ANDROID
   #define RESULT_PATH "/storage/emulated/0/Documents/Robocortex/Tests/Results/"
#else
   #define RESULT_PATH "./results/"
#endif

// TEST STANDARD IS PERFECT DATA

#define PERFECT_DATA

#if 0

// Image  512 x  512 =  262144 pixels
//#define IMAGE_PATH "/home/emalis/Tmp/debug/protoframe/gears/capture_photoframe.pgm"
//#define MODEL_PATH "/home/emalis/Tmp/protoframe/gears/debug/gears.pgm"

//#define IMAGE_PATH "/home/emalis/Tmp/debug/photoframe/squares/capturephoto_0.pgm"
//#define MODEL_PATH "/home/emalis/Tmp/debug/photoframe/squares/photoframe_texture_128_00.pgm"

// Photoframe de type 1, white border 20
#define MODEL_PATH    ROX_DATA_HOME"/regression_tests/openrox/identification/photoframe/models/photoframe_1.pgm"
#define IMAGE_PATH         ROX_DATA_HOME"/regression_tests/openrox/identification/photoframe/images/image_000.pgm"
// #define IMAGE_PATH      ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_3840x2160.pgm"

#define FU 617.55
#define FV 616.98
#define CU 314.23
#define CV 246.51

#define WHITE_BORDER 20
#define TYPE_BORDER 1

#define SIZEX 0.2
#define SIZEY 0.2

// Define the camera intrinsic parameters
// #define FU 1388.518897760294
// #define FV 1380.624743164325
// #define CU 975.5068896278956
// #define CV 544.0110137581285

//#define MODEL_PATH "/mnt/shared/Data/rox_datasets/regression_tests/openrox/identification/photoframe/models/photoframe_1.pgm"
//#define IMAGE_PATH "/mnt/shared/Data/rox_datasets/regression_tests/openrox/identification/photoframe/images/image_346.pgm"
#endif

#ifdef PERFECT_DATA
   // Perfect data
   #define MODEL_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/photoframe/synthetic/photoframe_%02d_128x128.pgm"
   #define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/photoframe/synthetic/image_photoframe_centered_%d.pgm"

   #define FU 128.0 
   #define FV 128.0 
   #define CU 958.5 
   #define CV 539.5

   #define WHITE_BORDER 16
   #define TYPE_BORDER 0

   #define SIZEX 1.0
   #define SIZEY 1.0

   #define NBP 3
   #define NBI 1
   
   // Define the shifts between the photoframes
   #define SHIFT_X { 0.0, 2.0, 4.0 } 
   #define SHIFT_Y { 0.0, 0.0, 0.0 }

   #define IMG_INI 1
#endif

#if 0
   // Debug AugmentedPro
   #define MODEL_PATH "F:/Debug/7f/photoframe_09_128x128.pgm"
   #define IMAGE_PATH "F:/Debug/7f/capture/camerasnapshot_0.pgm"

   #define FU  598.2895
   #define FV  595.5077
   #define CU  316.558
   #define CV  231.9193

   #define WHITE_BORDER 16
   #define TYPE_BORDER 0

   #define SIZEX 0.10*128/192
   #define SIZEY 0.10*128/192
#endif


//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

// const type * pointer : is a pointer to a constant (i.e. we cannot do *pointer = NULL)

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_ident_photoframe_se3_new_del)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ident_Multi_PhotoFrame_SE3 ident = NULL;

   error = rox_ident_multi_photoframe_se3_new(&ident, 640, 480);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ident_multi_photoframe_se3_del(&ident);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_ident_multi_photoframe_se3_init_single )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Ident_Multi_PhotoFrame_SE3 ident = NULL;
   Rox_Camera camera = NULL;
   Rox_Image_RGBA image_display = NULL;
   Rox_Image_RGBA image_draw = NULL;
   Rox_Image model_image_identify[NBP] = {NULL};
   Rox_Model_Single_Plane model_single_plane[NBP] = {NULL};

   Rox_Double sizex = SIZEX;
   Rox_Double sizey = SIZEY;
   Rox_Sint white_border = WHITE_BORDER;

   // Define pose matrix
   Rox_MatSE3 pose = NULL;
   Rox_Matrix K = NULL;

   Rox_Sint cols = 0;
   Rox_Sint rows = 0;

   // Define timer object
   Rox_Timer timer = 0;
   Rox_Double time = 0.0;

   Rox_Double score = 0;
   Rox_Sint identified = 0;
   Rox_Sint count = 0;

#ifndef WIN32
   struct stat st = {0};

   if (stat(RESULT_PATH, &st) == -1) {
       mkdir(RESULT_PATH, 0700);
   }
#endif

   // Define timer object
   error = rox_timer_new ( &timer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Crete a pose per model
   error = rox_matse3_new ( &pose );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_matrix_new ( &K, 3, 3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   sprintf(filename, IMAGE_PATH, IMG_INI);
   rox_log("read file %s\n", filename);

   error = rox_camera_new_read_pgm ( &camera, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_camera_set_pinhole_params(camera, FU, FV, CU, CV);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_camera_get_intrinsic_parameters ( K, camera );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_image_rgba_new_read_pgm ( &image_display, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_image_rgba_new_read_pgm ( &image_draw, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_camera_get_size ( &rows, &cols, camera );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Define identification multi photoframe
   error = rox_ident_multi_photoframe_se3_new ( &ident, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Set the type of photoframe : 0 = white border then black; 1 = black border then white
   error = rox_ident_multi_photoframe_se3_set_type ( ident, TYPE_BORDER );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_ident_multi_photoframe_se3_set_score_threshold ( ident, 0.80 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   for ( Rox_Sint k = 0; k < NBP; k++ )
   {
      // Create multi plane model
      sprintf(filename, MODEL_PATH, k);
      rox_log("read file %s\n", filename);

      // Load model for identification
      error = rox_image_new_read_pgm ( &model_image_identify[k], filename );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      // Define the 2D model with texture and size in meters
      error = rox_model_single_plane_new ( &model_single_plane[k] ); 
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      error = rox_model_single_plane_set_template_xright_ydown ( model_single_plane[k], model_image_identify[k], sizex, sizey );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      // Add photoframe
      // error = rox_ident_multi_photoframe_se3_addframe(ident, model_image_identify[k], sizex, sizey, white_border);
      // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;
        
      error = rox_ident_multi_photoframe_se3_addframe_model ( ident, model_single_plane[k], white_border );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;
   }

   //Rox_Double side_min = 10;
   //Rox_Double side_max = 1080;

   // NO photoframes
   //Rox_Double side_min = 60;
   //Rox_Double side_max = 70;

   //Rox_Double side_min = 50;
   //Rox_Double side_max = 60;

   //Rox_Double side_min = 40;
   //Rox_Double side_max = 50;

   //Rox_Double side_min = 30;
   //Rox_Double side_max = 40;

   //error = rox_ident_multi_photoframe_se3_set_side_bounds ( ident, side_min, side_max );
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //Rox_Double area_min = 10;//30 * 30;
   //Rox_Double area_max = 100000;//40 * 40;

   //error = rox_ident_multi_photoframe_se3_set_area_bounds ( ident, area_min, area_max );
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ident_multi_photoframe_se3_getcountframes(&count, ident);
   rox_log("Searching %d photoframes\n", count);

   error = rox_timer_start(timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_ident_multi_photoframe_se3_make_optimal ( ident, camera );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_timer_stop(timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_timer_get_elapsed_ms(&time, timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log("time to identify %d photoframes on a %d x %d image = %f\n", count, cols, rows, time);

   for ( Rox_Sint k = 0; k < NBP; k++ )
   {
      error = rox_ident_multi_photoframe_se3_get_pose ( &identified, pose, ident, k );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      // Draw results
      if (identified == 1)
      {
         Rox_Uint color = ROX_MAKERGBA(255, 0, 0, 255);
         
         error = rox_ident_multi_photoframe_se3_get_score ( &score, ident, k );
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

         rox_log("Photoframe %d found with score = %f\n", k, score);
         rox_log("pose: \n");
         rox_matse3_print(pose);

         error = rox_model_single_plane_set_template_xright_ydown ( model_single_plane[k], model_image_identify[k], sizex, sizey );
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

         error = rox_image_rgba_draw_projection_model_single_plane ( image_display, K, pose, model_single_plane[k], color );
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

         error = rox_image_rgba_draw_projection_frame ( image_display, K, pose, sizex/2.0 );
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;
      }
   }

   error = rox_image_rgba_save_ppm(RESULT_PATH"./result_ident_multi_photoframe_init_single.ppm", image_display);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;


   error = rox_ident_multi_photoframe_se3_get_optimal_result ( &identified, &score, pose, ident );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;
   
   rox_matse3_print(pose);

function_terminate:

   for ( Rox_Sint k = 0; k < NBP; k++ )
   {
      error = rox_image_del ( &model_image_identify[k] );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_model_single_plane_del ( &model_single_plane[k] );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 
   }

   error = rox_ident_multi_photoframe_se3_del ( &ident );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_camera_del ( &camera );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_del ( &image_display );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_del ( &image_draw );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_del ( &timer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del ( &pose );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del(&K);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_ident_multi_photoframe_se3_multi )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Ident_Multi_PhotoFrame_SE3 ident = NULL;
   Rox_Camera camera = NULL;
   Rox_Image_RGBA image_draw = NULL;
   Rox_Image_RGBA image_display = NULL;
   Rox_Image model_image_identify[NBP] = {NULL};
   Rox_Model_Multi_Plane model_multi_plane = NULL;

   Rox_Double sizex = SIZEX;
   Rox_Double sizey = SIZEY;
   Rox_Sint white_border = WHITE_BORDER;

   double shift_x[NBP] = SHIFT_X;
   double shift_y[NBP] = SHIFT_Y;

   // Define pose matrix cTo
   Rox_MatSE3 cTo = NULL;
   Rox_Matrix K = NULL;

   Rox_Sint cols = 0;
   Rox_Sint rows = 0;

   // Define timer object
   Rox_Timer timer = 0;
   Rox_Double time = 0.0;
   
   Rox_Sint identified = 0;
   Rox_Sint count = 0;

   Rox_Sint best_photoframe_id;
   Rox_Double best_score = 0.0;
   Rox_Double optimal_score = 0.0;
   Rox_Double score_threshold = 0.8; // 0.89
   
   // Define timer object
   error = rox_timer_new ( &timer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Crete a pose per model
   error = rox_matse3_new ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_matrix_new ( &K, 3, 3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   sprintf(filename, IMAGE_PATH, IMG_INI);
   rox_log("read file %s\n", filename);

   error = rox_camera_new_read_pgm ( &camera, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_camera_set_pinhole_params(camera, FU, FV, CU, CV);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_camera_get_intrinsic_parameters ( K, camera );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_image_rgba_new_read_pgm ( &image_display, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_image_rgba_new_read_pgm ( &image_draw, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_camera_get_size ( &rows, &cols, camera );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Define identification multi photoframe
   error = rox_ident_multi_photoframe_se3_new ( &ident, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Set the type of photoframe : 0 = white border then black; 1 = black border then white
   error = rox_ident_multi_photoframe_se3_set_type ( ident, TYPE_BORDER );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_ident_multi_photoframe_se3_set_score_threshold ( ident, score_threshold );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Define the 2D model with texture and size in meters
   error = rox_model_multi_plane_new ( &model_multi_plane ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   Rox_DynVec_Point3D_Double vertices_dynvec[NBP];

   for ( Rox_Sint k = 0; k < NBP; k++ )
   {
      // Create multi plane model
      sprintf(filename, MODEL_PATH, k);
      rox_log("read file %s\n", filename);

      // Load model for identification
      error = rox_image_new_read_pgm ( &model_image_identify[k], filename );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      Rox_Point3D_Double_Struct pt_move;

      error = rox_dynvec_point3d_double_new_rectangle ( &vertices_dynvec[k], SIZEX, SIZEY );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;
   
      pt_move.X = shift_x[k];
      pt_move.Y = shift_y[k];
      pt_move.Z = 0.0;

      error = rox_dynvec_point3d_double_shift ( vertices_dynvec[k], &pt_move );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      error = rox_dynvec_point3d_double_print ( vertices_dynvec[k] );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      Rox_Point3D_Double_Struct vertices[4];

      // Add plane to the multi model
      error = rox_rectangle3d_create_centered_plane_xright_ydown ( vertices, sizex, sizey );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      error = rox_model_multi_plane_append_plane ( model_multi_plane, model_image_identify[k], vertices_dynvec[k]->data );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;
   }

   error = rox_ident_multi_photoframe_se3_add_model ( ident, model_multi_plane, white_border );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_ident_multi_photoframe_se3_getcountframes ( &count, ident );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log("Searching %d photoframes\n", count);

   error = rox_timer_start(timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_ident_multi_photoframe_se3_make_optimal ( ident, camera );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_timer_stop(timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_timer_get_elapsed_ms(&time, timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log("time to identify %d photoframes on a %d x %d image = %f\n", count, cols, rows, time);

   for ( Rox_Sint k = 0; k < NBP; k++ )
   {
      error = rox_ident_multi_photoframe_se3_get_pose_object ( &identified, cTo, ident, k );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      // Draw results
      if (identified == 1)
      {
         Rox_Uint color = ROX_MAKERGBA(255, 0, 0, 255);
         Rox_Double score = 0;
         
         error = rox_ident_multi_photoframe_se3_get_score ( &score, ident, k );
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

         rox_log("Photoframe %d found with score = %f\n", k, score);
         rox_log("pose cTo: \n");
         rox_matse3_print(cTo);

         error = rox_image_rgba_copy(image_draw, image_display);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

         error = rox_image_rgba_draw_projection_model_multi_plane ( image_draw, K, cTo, model_multi_plane, color );
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

         sprintf(filename, RESULT_PATH"./result_ident_multi_photoframe_init_multi_%02d.ppm", k);
         rox_log("write file %s\n", filename);

         error = rox_image_rgba_save_ppm(filename, image_draw);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;
      }
   }

   error = rox_ident_multi_photoframe_se3_get_best_result ( &identified, &best_photoframe_id, &best_score, cTo, ident );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;
   
   printf ("best_photoframe_id = %d\n", best_photoframe_id);
   printf ("best_score = %f\n", best_score);
   
   ROX_TEST_CHECK_EQUAL ( best_photoframe_id, 0 ); if (error) goto function_terminate;
   ROX_TEST_CHECK_CLOSE ( best_score, 0.99999, 1e-5 ); if (error) goto function_terminate;

   // Draw results
   if (identified == 1)
   {
      Rox_Uint color = ROX_MAKERGBA(0, 255, 0, 255);

      rox_log("Best photoframe %d found with score = %f\n", best_photoframe_id, best_score);
      rox_log("Best pose cTo: \n");
      rox_matse3_print(cTo);

      error = rox_image_rgba_copy(image_draw, image_display);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      error = rox_image_rgba_draw_projection_model_multi_plane ( image_draw, K, cTo, model_multi_plane, color );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      error = rox_image_rgba_save_ppm(RESULT_PATH"./result_ident_multi_photoframe_init_best.ppm", image_draw);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;
   }

   error = rox_ident_multi_photoframe_se3_get_optimal_result ( &identified, &optimal_score, cTo, ident );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;
   
   printf ("optimal_score = %f\n", optimal_score);
   ROX_TEST_CHECK_CLOSE ( optimal_score, 0.99999, 1e-5 ); if (error) goto function_terminate;

   // Draw results
   if (identified == 1)
   {
      Rox_Uint color = ROX_MAKERGBA(0, 0, 255, 255);

      rox_log("Optimal score = %f\n", optimal_score);
      rox_log("Optimal pose cTo: \n");
      rox_matse3_print(cTo);

      error = rox_image_rgba_copy(image_draw, image_display);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      error = rox_image_rgba_draw_projection_model_multi_plane ( image_draw, K, cTo, model_multi_plane, color );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      error = rox_image_rgba_save_ppm(RESULT_PATH"./result_ident_multi_photoframe_init_optimal.ppm", image_draw);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;
   } 

function_terminate:

   for ( Rox_Sint k = 0; k < NBP; k++ )
   {
      error = rox_image_del ( &model_image_identify[k] );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }
      
   error = rox_model_multi_plane_del ( &model_multi_plane );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ident_multi_photoframe_se3_del ( &ident );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_camera_del(&camera);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_del(&image_display);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_del ( &timer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
