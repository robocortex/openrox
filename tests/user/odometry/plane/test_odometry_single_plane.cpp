//==============================================================================
//
//    OPENROX   : File test_odometry_single_plane.cpp
//
//    Contents  : Tests for odometry_single_plane.c
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
   #include <baseproc/array/error/l2_error.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(odometry_single_plane)

// ROX_DATA path is defined in cmake/tests.cmake
//#define IMG_REF_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D000.pgm"
//#define IMG_CUR_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D001.pgm"


#define IMG_REF_PATH ROX_DATA_HOME"/regression_tests/openrox/plane_trans/image_plane3D_trans_000.pgm"
#define IMG_CUR_PATH ROX_DATA_HOME"/regression_tests/openrox/plane_trans/image_plane3D_trans_001.pgm"


#define seq_path ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D%03d.pgm"
#define mod_file ROX_DATA_HOME"/regression_tests/openrox/plane/model.pgm"

// ====== INTERNAL TYPESDEFS ===================================================

// ====== INTERNAL DATATYPES ===================================================

// ====== INTERNAL VARIABLES ===================================================

Rox_Sint i = 0;

// ====== INTERNAL FUNCTDEFS ===================================================

// ====== INTERNAL FUNCTIONS ===================================================

// This function is used to simulate an image grabber
//Rox_ErrorCode grab_image_buffer(Rox_Uchar* buffer, Rox_Sint *cols, Rox_Sint *rows);

// This function is used to simulate an image reader
//Rox_ErrorCode read_image_model(Rox_Uchar* buffer, Rox_Sint *cols, Rox_Sint *rows);

class RoxTestOdometrySinglePlane : public rox::OpenROXTest
{
public:
   Rox_ErrorCode grab_image_buffer(Rox_Uchar* buffer, Rox_Sint *cols, Rox_Sint *rows)
   {
      Rox_ErrorCode error = ROX_ERROR_NONE;
      Rox_Char filename[FILENAME_MAX];
      Rox_Image image = NULL;

      // sprintf(filename, seq_path, i);
      sprintf(filename, seq_path, i);
      ROX_TEST_MESSAGE("read file %s\n", filename);

      error = rox_image_new_read_pgm(&image, filename);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      error = rox_image_get_data(buffer, image);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      error = rox_image_get_cols(cols, image);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      error = rox_image_get_rows(rows, image);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   function_terminate:
      error = rox_image_del(&image);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      return error;
   }


   Rox_ErrorCode read_image_model(Rox_Uchar* buffer, Rox_Sint *cols, Rox_Sint *rows)
   {
      Rox_ErrorCode error = ROX_ERROR_NONE;
      Rox_Image image = NULL;
      Rox_Char filename[FILENAME_MAX];

      sprintf(filename, "%s", mod_file);
      ROX_TEST_MESSAGE("read file %s\n", filename);

      error = rox_image_new_read_pgm(&image, filename);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      error = rox_image_get_data(buffer, image);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      error = rox_image_get_cols(cols, image);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      if (error) goto function_terminate;

      error = rox_image_get_rows(rows, image);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      if (error) goto function_terminate;

   function_terminate:
      error = rox_image_del(&image);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      return error;
   }

};

// ====== EXPORTED FUNCTIONS ===================================================

ROX_TEST_CASE_DECLARE(RoxTestOdometrySinglePlane, test_odometry_single_plane_inputs_outputs)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Single_Plane odometry = NULL;

   // Define the 2D model with texture and size in meters
   Rox_Model_Single_Plane model = NULL;

   // Define odometry parameters
   Rox_Odometry_Single_Plane_Params params = NULL;

   // Error since params and model have not been allocated
   error = rox_odometry_single_plane_new(&odometry, params, model);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Allocate params
   error = rox_odometry_single_plane_params_new ( &params );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Error since model has not been allocated
   error = rox_odometry_single_plane_new ( &odometry, params, model );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Allocate model
   error = rox_model_single_plane_new(&model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Error since model has not been completely defined
   error = rox_odometry_single_plane_new(&odometry, params, model);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_odometry_single_plane_params_del(&params);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_model_single_plane_del(&model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( RoxTestOdometrySinglePlane, test_odometry_single_plane )
{
   Rox_ErrorCode error;
   Rox_Double score = 0.0;
   Rox_Double l2_error = 0.0;

   Rox_Double cTo_grt_data[16] = {  1.0000000000000000, 0.0000000000000000, 0.0000000000000000, -0.0078125000000000, 
                                    0.0000000000000000, 1.0000000000000000, 0.0000000000000000,  0.0000000000000000, 
                                    0.0000000000000000, 0.0000000000000000, 1.0000000000000000,  1.0000000000000000,
                                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,  1.0000000000000000 };

   // Real size of the target (in meters)
   Rox_Double model_sizex = 1.0;
   Rox_Double model_sizey = 1.0;

   // Define the 2D model with texture and size in meters
   Rox_Model_Single_Plane model = NULL;

   // ----------- ALLOCATE VISUAL DATA ------------------------------------

   // Define objects for the 2d model
   Rox_Image Ir_uchar = NULL;

   // Define objects for the current image
   Rox_Camera camera  = NULL;
   Rox_Matrix K = NULL;
   Rox_Image Ic_uchar = NULL;

   // Define pose object
   Rox_MatSE3 cTo_grt = NULL;
   Rox_MatSE3 cTo = NULL;
   Rox_Double tra[3] = {0.0, 0.0, 1.0};

   // Define odometry parameters
   Rox_Odometry_Single_Plane_Params params = NULL;

   // Define odometry object
   Rox_Odometry_Single_Plane odometry = NULL;

   // Define timer object
   Rox_Timer timer = 0;
   Rox_Double time = 0.0;

   // Define intrinsic parameters
   Rox_Double FU = 512.0;
   Rox_Double FV = 512.0;
   Rox_Double CU = 255.5;
   Rox_Double CV = 255.5;

   // Define the resolution : 512 x 512 pixels
   Rox_Sint cols = 512;
   Rox_Sint rows = 512;

   // Read the image reference from disk
   // Define the image reference
   error = rox_image_new_read_pgm ( &Ir_uchar, IMG_REF_PATH );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define the 2D model with texture and size in meters :
   // in this case 0.2 m x 0.2 m
   error = rox_model_single_plane_new ( &model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_model_single_plane_set_template_xright_ydown ( model, Ir_uchar, model_sizex, model_sizey );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define the odometry parameters
   error = rox_odometry_single_plane_params_new ( &params );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_odometry_single_plane_params_set_usecase ( params, Rox_Odometry_Single_Plane_UseCase_Affine_Light );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;
   
   error = rox_matse3_new ( &cTo_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_matse3_set_data ( cTo_grt, cTo_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_new ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_matse3_set_translation ( cTo, tra );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define the odometry object
   error = rox_odometry_single_plane_new ( &odometry, params, model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Create and read new current image
   error = rox_image_new_read_pgm ( &Ic_uchar, IMG_CUR_PATH );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_image_get_size ( &rows, &cols, Ic_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define the camera with a given image resolution
   error = rox_camera_new ( &camera, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_camera_set_pinhole_params(camera, FU, FV, CU, CV);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define the intrinsic parameters
   error = rox_matrix_new(&K, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_camera_get_intrinsic_parameters ( K, camera );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Get the pointer to the image of the camera
   error = rox_camera_set_image ( camera, Ic_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define timer object
   error = rox_timer_new ( &timer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_odometry_single_plane_set_pose ( odometry, cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_timer_start(timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Tracking the target
   error = rox_odometry_single_plane_make(odometry, camera);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 

   error = rox_timer_stop(timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_timer_get_elapsed_ms(&time, timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   ROX_TEST_MESSAGE("time for odometry on a %d x %d image = %f\n", cols, rows, time);

   // Get a copy of the cTo (position and orientation) of the camera
   error = rox_odometry_single_plane_get_pose ( cTo, odometry );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Display quality measure
   error = rox_odometry_single_plane_get_score ( &score, odometry );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   ROX_TEST_MESSAGE("score = %f \n", score);

   ROX_TEST_CHECK_CLOSE ( score, 1.0, 1e-2 );

   ROX_TEST_MESSAGE("pose cTo: \n");
   rox_matse3_print(cTo);


   error = rox_array2d_double_difference_l2_norm ( &l2_error, cTo_grt, cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error cTo = %f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);

function_terminate:

   // Delete objects and free Memory
   error = rox_image_del ( &Ir_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del ( &Ic_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_camera_del ( &camera );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_single_plane_del(&odometry);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_single_plane_params_del ( &params );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_model_single_plane_del ( &model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_del ( &timer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( RoxTestOdometrySinglePlane, test_odometry_single_plane_non_regression )
{
   Rox_ErrorCode error;
   Rox_Double score;

   // Real size of the target (in meters)
   Rox_Double  model_sizex = 0.2;
   Rox_Double  model_sizey = 0.2;

   // Define the 2D model with texture and size in meters
   Rox_Model_Single_Plane model = NULL;

   // ----------- ALLOCATE VISUAL DATA ------------------------------------

  // Define objects for the 2d model
   Rox_Image Im = NULL;

   // Define objects for the current image
   Rox_Camera camera  = NULL;
   Rox_Matrix K = NULL;
   Rox_Image image = NULL;

   // Define pose object
   Rox_MatSE3 cTo = NULL;

   // Define identification object
   Rox_Ident_Texture_SE3 identifier = NULL;

   // Define odometry parameters
   Rox_Odometry_Single_Plane_Params params = NULL;

   // Define odometry object
   Rox_Odometry_Single_Plane odometry = NULL;

   // Define tracking results
   Rox_Sint tracked = 0;

   // Define timer object
   Rox_Timer timer = 0;
   Rox_Double time = 0.0;

   // Define intrinsic parameters
   Rox_Double FU = 2560.0;
   Rox_Double FV = 2560.0;
   Rox_Double CU = 255.5;
   Rox_Double CV = 255.5;

   // Define the resolution : 512 x 512 pixels
   Rox_Sint cols = 512;
   Rox_Sint rows = 512;

   // Define the buffer containing the image data
   Rox_Uchar buffer[512*512];

   Rox_Sint n_tracked = 0;
   Rox_Sint is_identified = 0;

   // Reset counter
   i = 0;

   // Read the image model from disk
   // Replace this function with your own function to fill the buffer
   error = read_image_model ( buffer, &cols, &rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define the image model
   error = rox_image_new ( &Im, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Fill the Rox_Image with the data in the buffer
   error = rox_image_set_data ( Im, buffer, cols, Rox_Image_Format_Grays );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define the 2D model with texture and size in meters :
   // in this case 0.2 m x 0.2 m
   error = rox_model_single_plane_new ( &model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_model_single_plane_set_template_xright_ydown ( model, Im, model_sizex, model_sizey );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define the ident object
   error = rox_ident_texture_se3_new ( &identifier );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_ident_texture_se3_set_model ( identifier, model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define the odometry parameters
   error = rox_odometry_single_plane_params_new ( &params );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_odometry_single_plane_params_set_usecase ( params, Rox_Odometry_Single_Plane_UseCase_Robust_Light );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define the odometry object
   error = rox_matse3_new ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_odometry_single_plane_new ( &odometry, params, model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Replace this function with your own function to fill the buffer
   error = grab_image_buffer ( buffer, &cols, &rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Create new image
   error = rox_image_new ( &image, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Fill the Rox_Image with the data in the buffer
   error = rox_image_set_data(image, buffer, cols, Rox_Image_Format_Grays);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define the camera with a given image resolution
   error = rox_camera_new(&camera, cols, rows);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_camera_set_pinhole_params ( camera, FU, FV, CU, CV );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define the intrinsic parameters
   error = rox_matrix_new ( &K, 3, 3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_camera_get_intrinsic_parameters ( K, camera );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Get the pointer to the image of the camera
   error = rox_camera_set_image(camera, image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define timer object
   error = rox_timer_new ( &timer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   for (i = 0; i < 100; i++)
   {
      // Replace this function with your own function to fill the buffer
      error = grab_image_buffer ( buffer, &cols, &rows );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      // Fill the Rox_Image with the data in the buffer
      error = rox_image_set_data ( image, buffer, cols, Rox_Image_Format_Grays );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      // Get the pointer to the image of the camera
      error = rox_camera_set_image ( camera, image );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      if(tracked == 1)
      {
         error = rox_timer_start(timer);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

         // Tracking the target
         error = rox_odometry_single_plane_make ( odometry, camera );
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 
         if (error) tracked = 0;
         else tracked = 1;

         error = rox_timer_stop(timer);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
         if(error) goto function_terminate;

         error = rox_timer_get_elapsed_ms(&time, timer);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
         if(error) goto function_terminate;

         ROX_TEST_MESSAGE("time for odometry on a %d x %d image = %f\n", cols, rows, time);
      }
      else
      {
         // Identify the target
         error = rox_ident_texture_se3_make ( &is_identified, cTo, identifier, camera );
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
         if (error == 0)
         {
            error = rox_odometry_single_plane_set_pose(odometry, cTo);
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

            error = rox_odometry_single_plane_make(odometry, camera);
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 
            if(error) tracked = 0;
            else tracked = 1;
         }
      }

      // Get a copy of the cTo (position and orientation) of the camera
      error = rox_odometry_single_plane_get_pose ( cTo, odometry );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      // Display quality measure
      error = rox_odometry_single_plane_get_score ( &score, odometry );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      if (tracked == 1)
      {
         ROX_TEST_CHECK_SUPERIOR_OR_EQUAL(score, 0.98);
         //CU_ASSERT(time  <= 40.0);
         n_tracked++;
      }
   }

   ROX_TEST_CHECK_EQUAL(n_tracked, 100);

function_terminate:

   // Delete objects and free Memory
   error = rox_image_del ( &Im );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del ( &image );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_camera_del ( &camera );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_single_plane_del(&odometry);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_single_plane_params_del ( &params );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ident_texture_se3_del ( &identifier );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_model_single_plane_del ( &model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_del ( &timer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( RoxTestOdometrySinglePlane, test_odometry_single_plane_robust_light )
{
   Rox_ErrorCode error;
   Rox_Double score;

   // Real size of the target (in meters)
   Rox_Double  model_sizex = 0.2;
   Rox_Double  model_sizey = 0.2;

   // Define the 2D model with texture and size in meters
   Rox_Model_Single_Plane model = 0;

   // ----------- ALLOCATE VISUAL DATA ------------------------------------

  // Define objects for the 2d model
   Rox_Image Im = NULL;

   // Define objects for the current image
   Rox_Camera camera = NULL;
   Rox_Matrix K = NULL;
   Rox_Image image = NULL;

   // Define pose object
   Rox_MatSE3 cTo = NULL;

   // Define identification object
   Rox_Ident_Texture_SE3 identifier = NULL;

   // Define odometry parameters
   Rox_Odometry_Single_Plane_Params params = NULL;

   // Define odometry object
   Rox_Odometry_Single_Plane odometry = NULL;

   // Define tracking results
   Rox_Sint tracked = 0;

   // Define timer object
   Rox_Timer timer = 0;
   Rox_Double time;

   // Define intrinsic parameters
   Rox_Double FU = 2560.0;
   Rox_Double FV = 2560.0;
   Rox_Double CU = 255.5;
   Rox_Double CV = 255.5;

   // Define the resolution : 512 x 512 pixels
   Rox_Sint cols = 512;
   Rox_Sint rows = 512;

   // Define the buffer containing the image data
   Rox_Uchar buffer[512*512];

   Rox_Sint n_tracked = 0;
   Rox_Sint is_identified = 0;

   // Reset counter
   i = 0;
   
   // Read the image model from disk
   // Replace this function with your own function to fill the buffer
   error = read_image_model(buffer, &cols, &rows);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   if(error) goto function_terminate;

   // Define the image model
   error = rox_image_new(&Im, cols, rows);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   if(error) goto function_terminate;

   // Fill the Rox_Image with the data in the buffer
   error = rox_image_set_data(Im, buffer, cols, Rox_Image_Format_Grays);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   if(error) goto function_terminate;

   // Define the 2D model with texture and size in meters :
   // in this case 0.2 m x 0.2 m
   error = rox_model_single_plane_new(&model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   if(error) goto function_terminate;

   error = rox_model_single_plane_set_template_xright_ydown(model, Im, model_sizex, model_sizey);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   if(error) goto function_terminate;

   // Define the ident object
   error = rox_ident_texture_se3_new(&identifier);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   if(error) goto function_terminate;

   error = rox_ident_texture_se3_set_model ( identifier, model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define the odometry parameters
   error = rox_odometry_single_plane_params_new(&params);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_odometry_single_plane_params_set_usecase ( params, Rox_Odometry_Single_Plane_UseCase_Robust_Light);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define the odometry object
   error = rox_matse3_new ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_odometry_single_plane_new ( &odometry, params, model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Replace this function with your own function to fill the buffer
   error = grab_image_buffer ( buffer, &cols, &rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Create new image
   error = rox_image_new ( &image, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Fill the Rox_Image with the data in the buffer
   error = rox_image_set_data ( image, buffer, cols, Rox_Image_Format_Grays );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define the camera with a given image resolution
   error = rox_camera_new ( &camera, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   error = rox_camera_set_pinhole_params ( camera, FU, FV, CU, CV );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define the intrinsic parameters
   error = rox_matrix_new ( &K, 3, 3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   if(error) goto function_terminate;

   error = rox_camera_get_intrinsic_parameters(K, camera);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Get the pointer to the image of the camera
   error = rox_camera_set_image(camera, image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   // Define timer object
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

   for (i = 0; i < 100; i++)
   {
      // Replace this function with your own function to fill the buffer
      error = grab_image_buffer(buffer, &cols, &rows);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      // Fill the Rox_Image with the data in the buffer
      error = rox_image_set_data(image, buffer, cols, Rox_Image_Format_Grays);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      // Get the pointer to the image of the camera
      error = rox_camera_set_image(camera, image);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      if(tracked == 1)
      {
         error = rox_timer_start(timer);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

         // Tracking the target
         error = rox_odometry_single_plane_make(odometry, camera);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 
         if (error) tracked = 0;
         else tracked = 1;

         error = rox_timer_stop(timer);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

         error = rox_timer_get_elapsed_ms(&time, timer);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

         ROX_TEST_MESSAGE("time for odometry on a %d x %d image = %f\n", cols, rows, time);
      }
      else
      {
         // Identify the target
         error = rox_ident_texture_se3_make(&is_identified, cTo, identifier, camera);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
         if(error == 0)
         {
            error = rox_odometry_single_plane_set_pose(odometry, cTo);
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

            error = rox_odometry_single_plane_make(odometry, camera);
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
            if(error) tracked = 0;
            else tracked = 1;
         }
      }

      // Get a copy of the cTo (position and orientation) of the camera
      error = rox_odometry_single_plane_get_pose ( cTo, odometry ); 
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      // Display quality measure
      error = rox_odometry_single_plane_get_score ( &score, odometry );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_terminate;

      if (tracked == 1)
      {
         ROX_TEST_CHECK_SUPERIOR_OR_EQUAL ( score, 0.98 );
         //CU_ASSERT(time  <= 40.0);
         n_tracked++;
      }
   }

   ROX_TEST_CHECK_EQUAL(n_tracked, 100);

function_terminate:

   // Delete objects and free Memory
   error = rox_image_del ( &Im );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del ( &image );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_camera_del ( &camera );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matrix_del ( &K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matse3_del ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_odometry_single_plane_del ( &odometry );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_odometry_single_plane_params_del ( &params );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_ident_texture_se3_del ( &identifier );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_model_single_plane_del ( &model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_timer_del ( &timer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
