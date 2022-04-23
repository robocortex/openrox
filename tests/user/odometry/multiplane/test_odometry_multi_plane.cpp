//==============================================================================
//
//    OPENROX   : File test_odometry_multi_plane.cpp
//
//    Contents  : Tests for odometry_multi_plane.c
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
   #include <api/openrox.h>
   #include <baseproc/array/error/l2_error.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN ( odometry_multi_plane )

// Download the following archive and uncompress it in the "seq" directory:
// http://public.robocortex.com/download/sequences/cube.zip

#define seq_file ROX_DATA_HOME"/regression_tests/openrox/cube/images/cube%4.4d.pgm"

#ifdef ANDROID
   #define RESULT_PATH "/storage/emulated/0/Documents/Robocortex/Tests/Results/result_cube_%4.4d.ppm"
#else
   #define RESULT_PATH "./results/result_cube_%4.4d.ppm"
#endif

// ROX_DATA_HOME path is defined in cmake/tests.cmake
// Define the paths for the model textures
#define mod_file_L ROX_DATA_HOME"/regression_tests/openrox/cube/models/left.pgm"
#define mod_file_F ROX_DATA_HOME"/regression_tests/openrox/cube/models/front.pgm"
#define mod_file_T ROX_DATA_HOME"/regression_tests/openrox/cube/models/top.pgm"
#define mod_file_B ROX_DATA_HOME"/regression_tests/openrox/cube/models/back.pgm"
#define mod_file_R ROX_DATA_HOME"/regression_tests/openrox/cube/models/right.pgm"

#define PX 559.279
#define PY 559.279
#define U0 329.890
#define V0 242.164

#define IMG_REF_PATH ROX_DATA_HOME"/regression_tests/openrox/plane_trans/image_plane3D_trans_000.pgm"
#define IMG_CUR_PATH ROX_DATA_HOME"/regression_tests/openrox/plane_trans/image_plane3D_trans_010.pgm"

#define mod_file ROX_DATA_HOME"/regression_tests/openrox/multi_plane/model/subimage_%02d.pgm"

#define FU 512.0
#define FV 512.0
#define CU 255.5
#define CV 255.5

// ====== INTERNAL TYPESDEFS ===================================================

// ====== INTERNAL DATATYPES ===================================================

// ====== INTERNAL VARIABLES ===================================================

// ====== INTERNAL FUNCTDEFS ===================================================

// ====== INTERNAL FUNCTIONS ===================================================

// ====== EXPORTED FUNCTIONS ===================================================

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_odometry_multi_plane_non_regression )
{
   Rox_Error error;
   Rox_Char filename[1024];

   Rox_Image source = NULL;
   Rox_Image_RGBA image_rgba = NULL;
   Rox_MatSE3 pose = NULL;
   Rox_Matrix K = NULL;

   Rox_Model_Multi_Plane model = NULL;
   Rox_Camera camera = NULL;

   Rox_Ident_Multi_Plane identifier = NULL;
   Rox_Odometry_Multi_Plane odometry = NULL;
   Rox_Odometry_Multi_Plane_Params params = NULL;
   Rox_Timer timer = NULL;

   Rox_Uint tracked = 0, idtemplate = 0, idimg = 0;
   Rox_Double time = 0.0;

   Rox_Uint red = ROX_MAKERGBA(255, 0 , 0, 255);

   const Rox_Uint count_templates = 5;

   Rox_Point3D_Double_Struct vertices[][4] =
   {
      { {-1, -1,  1}, {-1, -1, -1}, {-1,  1, -1}, {-1,  1,  1} },
      { {-1, -1, -1}, { 1, -1, -1}, { 1,  1, -1}, {-1,  1, -1} },
      { {-1, -1,  1}, { 1, -1,  1}, { 1, -1, -1}, {-1, -1, -1} },
      { { 1, -1,  1}, {-1, -1,  1}, {-1,  1,  1}, { 1,  1,  1} },
      { { 1, -1, -1}, { 1, -1,  1}, { 1,  1,  1}, { 1,  1, -1} }
   };

   const char * names[] =
   {
       mod_file_L, mod_file_F, mod_file_T, mod_file_B, mod_file_R
   };

#ifndef WIN32
   struct stat st = {0};

   if (stat(RESULT_PATH, &st) == -1) {
       mkdir(RESULT_PATH, 0700);
   }
#endif

   // Creating model
   error = rox_model_multi_plane_new ( &model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_ident_multi_plane_new ( &identifier );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   for (idtemplate = 0; idtemplate < count_templates; idtemplate++)
   {
      error = rox_image_new_read_pgm ( &source, names[idtemplate] );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      error = rox_model_multi_plane_append_plane ( model, source, vertices[idtemplate] );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      rox_image_del(&source);
   }

   error = rox_ident_multi_plane_set_model(identifier, model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_matse3_new(&pose);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_odometry_multi_plane_params_new(&params);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_odometry_multi_plane_new ( &odometry, params, model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_image_rgba_new(&image_rgba, 640, 480);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_camera_new(&camera, 640, 480);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_camera_set_pinhole_params(camera, PX, PY, U0, V0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_matrix_new(&K, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_matrix_build_calibration_matrix(K, PX, PY, U0, V0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   tracked = 0;

   for (idimg = 0; idimg < 400; idimg++)
   {
      // Read image from sequence
      sprintf(filename, seq_file, idimg);
      error = rox_camera_read_pgm(camera, filename);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      error = rox_timer_start(timer);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      // Identification
      if (!tracked)
      {
          error = rox_ident_multi_plane_make(identifier, model, camera);
          if (!error)
          {
             error = rox_ident_multi_plane_get_pose(pose, identifier);
             ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

// Rox_Double pose_ok[16] = { -0.7070913868245143, 0.7069913161052160, -0.0136032948361383, -0.0221729310901230,
// -0.7071126586803339, -0.7068512133565399, 0.0187896277349088, -0.0363681000330559, 
// 0.0036685981808591, 0.0229050459113873, 0.9997309139259352, 6.0032356621092431, 
// 0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 1.0000000000000000 };

//             rox_matse3_set_data(pose, pose_ok);


             error = rox_odometry_multi_plane_set_pose(odometry, pose);
             ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;
        
             tracked = 1;

          }
      }

      // Odometry
      if (tracked)
      {
         error = rox_odometry_multi_plane_make ( odometry, model, camera );
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

         if (!error)
         {
            error = rox_odometry_multi_plane_get_pose ( pose, odometry );
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

            rox_matse3_print(pose);
            tracked = 1;
         }
         else
         {
             rox_log("tracking failed !!!\n");
             tracked = 0;
         }
      }

      error = rox_timer_stop(timer);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      error = rox_timer_get_elapsed_ms(&time, timer);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      rox_log("Time to track: %f ms \n", time);
      // Display results
      if (tracked)
      {
         error = rox_image_rgba_read_pgm ( image_rgba, filename );
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

         error = rox_image_rgba_draw_projection_model_multi_plane ( image_rgba, K, pose, model, red );
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

         // Save image
         sprintf(filename, RESULT_PATH, idimg);
         error = rox_image_rgba_save_ppm ( filename, image_rgba );
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;
      }
   }

function_terminate:
   rox_error_print(error);

   // Delete objects and free Memory
   error = rox_model_multi_plane_del(&model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_camera_del(&camera);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ident_multi_plane_del(&identifier);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_odometry_multi_plane_del(&odometry);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del(&pose);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matrix_del(&K);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_del(&image_rgba);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_odometry_multi_plane_non_regression_single )
{
   Rox_Error error;
   Rox_Char filename[1024];
   Rox_Double l2_error = 0.0;

   Rox_Double cTo_grt_data[16] = {  1.0000000000000000, 0.0000000000000000, 0.0000000000000000, -0.078125000000000, 
                                    0.0000000000000000, 1.0000000000000000, 0.0000000000000000,  0.0000000000000000, 
                                    0.0000000000000000, 0.0000000000000000, 1.0000000000000000,  1.0000000000000000,
                                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,  1.0000000000000000 };

   Rox_Image source = NULL;
   Rox_Image_RGBA image_rgba = NULL;
   Rox_MatSE3 cTo = NULL;
   Rox_MatSE3 cTo_grt = NULL;
   Rox_Matrix K = NULL;

   Rox_Model_Multi_Plane model = NULL;
   Rox_Camera camera = NULL;

   Rox_Ident_Multi_Plane identifier = NULL;
   Rox_Odometry_Multi_Plane odometry = NULL;
   Rox_Odometry_Multi_Plane_Params params = NULL;
   Rox_Timer timer = NULL;

   Rox_Sint tracked = 0, idtemplate = 0, idimg = 0;
   Rox_Double time = 0.0;

   Rox_Uint red = ROX_MAKERGBA(255, 0 , 0, 255);

   const Rox_Sint count_templates = 8;

   Rox_Point3D_Double_Struct vertices[][4] =
{
{ { -0.50000000, -0.50000000, 0.00000000 }, { -0.25000000, -0.50000000, 0.00000000 }, { -0.25000000, -0.25000000, 0.00000000 }, { -0.50000000, -0.25000000, 0.00000000 }  },
{ { -0.25000000, -0.50000000, 0.00000000 }, { 0.00000000, -0.50000000, 0.00000000 }, { 0.00000000, -0.25000000, 0.00000000 }, { -0.25000000, -0.25000000, 0.00000000 }  },
{ { 0.00000000, -0.50000000, 0.00000000 }, { 0.25000000, -0.50000000, 0.00000000 }, { 0.25000000, -0.25000000, 0.00000000 }, { 0.00000000, -0.25000000, 0.00000000 }  },
{ { 0.25000000, -0.50000000, 0.00000000 }, { 0.50000000, -0.50000000, 0.00000000 }, { 0.50000000, -0.25000000, 0.00000000 }, { 0.25000000, -0.25000000, 0.00000000 }  },
{ { -0.50000000, -0.25000000, 0.00000000 }, { -0.25000000, -0.25000000, 0.00000000 }, { -0.25000000, 0.00000000, 0.00000000 }, { -0.50000000, 0.00000000, 0.00000000 }  },
{ { -0.25000000, -0.25000000, 0.00000000 }, { 0.00000000, -0.25000000, 0.00000000 }, { 0.00000000, 0.00000000, 0.00000000 }, { -0.25000000, 0.00000000, 0.00000000 }  },
{ { 0.00000000, -0.25000000, 0.00000000 }, { 0.25000000, -0.25000000, 0.00000000 }, { 0.25000000, 0.00000000, 0.00000000 }, { 0.00000000, 0.00000000, 0.00000000 }  },
{ { 0.25000000, -0.25000000, 0.00000000 }, { 0.50000000, -0.25000000, 0.00000000 }, { 0.50000000, 0.00000000, 0.00000000 }, { 0.25000000, 0.00000000, 0.00000000 }  },
{ { -0.50000000, 0.00000000, 0.00000000 }, { -0.25000000, 0.00000000, 0.00000000 }, { -0.25000000, 0.25000000, 0.00000000 }, { -0.50000000, 0.25000000, 0.00000000 }  },
{ { -0.25000000, 0.00000000, 0.00000000 }, { 0.00000000, 0.00000000, 0.00000000 }, { 0.00000000, 0.25000000, 0.00000000 }, { -0.25000000, 0.25000000, 0.00000000 }  },
{ { 0.00000000, 0.00000000, 0.00000000 }, { 0.25000000, 0.00000000, 0.00000000 }, { 0.25000000, 0.25000000, 0.00000000 }, { 0.00000000, 0.25000000, 0.00000000 }  },
{ { 0.25000000, 0.00000000, 0.00000000 }, { 0.50000000, 0.00000000, 0.00000000 }, { 0.50000000, 0.25000000, 0.00000000 }, { 0.25000000, 0.25000000, 0.00000000 }  },
{ { -0.50000000, 0.25000000, 0.00000000 }, { -0.25000000, 0.25000000, 0.00000000 }, { -0.25000000, 0.50000000, 0.00000000 }, { -0.50000000, 0.50000000, 0.00000000 }  },
{ { -0.25000000, 0.25000000, 0.00000000 }, { 0.00000000, 0.25000000, 0.00000000 }, { 0.00000000, 0.50000000, 0.00000000 }, { -0.25000000, 0.50000000, 0.00000000 }  },
{ { 0.00000000, 0.25000000, 0.00000000 }, { 0.25000000, 0.25000000, 0.00000000 }, { 0.25000000, 0.50000000, 0.00000000 }, { 0.00000000, 0.50000000, 0.00000000 }  },
{ { 0.25000000, 0.25000000, 0.00000000 }, { 0.50000000, 0.25000000, 0.00000000 }, { 0.50000000, 0.50000000, 0.00000000 }, { 0.25000000, 0.50000000, 0.00000000 }  }
};

   // Creating model
   error = rox_model_multi_plane_new ( &model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   for (idtemplate = 0; idtemplate < count_templates; idtemplate++)
   {
      sprintf(filename, mod_file, idtemplate+1);
      rox_log("read file %s\n", filename);

      error = rox_image_new_read_pgm ( &source, filename );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      error = rox_model_multi_plane_append_plane ( model, source, vertices[idtemplate] );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      rox_image_del(&source);
   }

   error = rox_ident_multi_plane_new ( &identifier );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   //error = rox_ident_multi_plane_set_model ( identifier, model);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_matse3_new ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_matse3_new ( &cTo_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_matse3_set_data ( cTo_grt, cTo_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_multi_plane_params_new(&params);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_odometry_multi_plane_new(&odometry, params, model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   sprintf(filename, IMG_REF_PATH);

   error = rox_image_rgba_new_read_pgm ( &image_rgba, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_camera_new_read_pgm ( &camera, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_camera_set_pinhole_params(camera, FU, FV, CU, CV);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_matrix_new ( &K, 3, 3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_matrix_build_calibration_matrix ( K, FU, FV, CU, CV );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   tracked = 0;

   for (idimg = 0; idimg < 1; idimg++)
   {
      // Read image from sequence
      sprintf(filename, IMG_CUR_PATH);
      error = rox_camera_read_pgm(camera, filename);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      error = rox_timer_start(timer);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      // Identification
      if (!tracked)
      {
          //error = rox_ident_multi_plane_make ( identifier, model, camera );
          if (!error)
          {
             error = rox_ident_multi_plane_get_pose ( cTo, identifier );
             ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

             Rox_Double cTo_init_data[16] = {   1,     0,     0,     0,
                                                0,     1,     0,     0,
                                                0,     0,     1,     1,
                                                0,     0,     0,     1 };
             rox_matse3_set_data(cTo, cTo_init_data);

             error = rox_odometry_multi_plane_set_pose ( odometry, cTo );
             ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;
        
             tracked = 1;

             rox_log("identification of image %d succeed !!!\n", idimg);

             rox_log("setting the following pose to the odometry\n");
             rox_matse3_print(cTo);
          }
      }
      error = rox_timer_stop(timer);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      error = rox_timer_get_elapsed_ms(&time, timer);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      rox_log("Time to ident: %f ms \n", time);

      error = rox_timer_start(timer);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      // Odometry
      if (tracked)
      {
         error = rox_odometry_multi_plane_make ( odometry, model, camera );
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

         if (!error)
         {
            Rox_Double score = 0.0;
            error = rox_odometry_multi_plane_get_result ( &tracked, &score, cTo, odometry );
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

            rox_matse3_print(cTo);
            if (tracked == 0)
            {
               rox_log("tracking failed !!!\n");
            }
            else
            {
               rox_log("tracking succeed with score = %f\n", score);
            }
         }
         else
         {
             rox_log("tracking failed !!!\n");
             tracked = 0;
         }
      }

      error = rox_timer_stop(timer);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      error = rox_timer_get_elapsed_ms(&time, timer);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      rox_log("Time to track: %f ms \n", time);
      // Display results
      if (tracked)
      {
         error = rox_image_rgba_read_pgm(image_rgba, filename);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

         error = rox_image_rgba_draw_projection_model_multi_plane(image_rgba, K, cTo, model, red);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

         // Save image
         sprintf(filename, RESULT_PATH, idimg);
         error = rox_image_rgba_save_ppm(filename, image_rgba);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;
      }
   }


   error = rox_array2d_double_difference_l2_norm ( &l2_error, cTo_grt, cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error cTo = %12.12f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-4);

function_terminate:
   rox_error_print(error);

   // Delete objects and free Memory
   error = rox_model_multi_plane_del(&model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_camera_del(&camera);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ident_multi_plane_del(&identifier);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_odometry_multi_plane_del(&odometry);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matrix_del ( &K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_del(&image_rgba);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}




ROX_TEST_SUITE_END()
