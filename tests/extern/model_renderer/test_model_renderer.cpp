//==============================================================================
//
//    OPENROX   : File test_model_renderer.cpp
//
//    Contents  : Tests for model_renderer.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//====== INCLUDED HEADERS   ====================================================

#include <openrox_tests.hpp>

extern "C"
{
   #include <math.h>
   #include <float.h>

   // Include needed to create directory
   #include <sys/types.h>
   #include <sys/stat.h>
   #include <unistd.h>

   #include "model_renderer/model_renderer.h"

   #include <objfile/objfile_edges.h>

   #include <array2d_float.h>

   #include <baseproc/image/roi/image_roi.h>
   #include <core/score/image_window_difference_score.h>

   #include <system/errors/errors.h>
   #include <system/time/timer.h>

   #include <inout/numeric/array2d_save.h>
   #include <inout/system/errors_print.h>
   #include <inout/image/ppm/ppmfile.h>
}

//====== INTERNAL MACROS    ====================================================

ROX_TEST_SUITE_BEGIN(test_model_renderer)

#ifdef ANDROID
   #define RESULT_PATH "/storage/emulated/0/Documents/Robocortex/Tests/Results/"
#else
   #define RESULT_PATH "./results/"
#endif

#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/pnp_keypoints_model/ref.pgm"
#define MODEL_PATH ROX_DATA_HOME"/regression_tests/openrox/pnp_keypoints_model/switch.obj"

#define IMAGE_REF_PATH ROX_DATA_HOME"/regression_tests/openrox/model_renderer/reference_image.pgm"
#define IMASK_REF_PATH ROX_DATA_HOME"/regression_tests/openrox/model_renderer/reference_imask.pgm"

#define FU 1415
#define FV 1431
#define CU 954
#define CV 559

#define MODEL_UNIT_TO_METERS 1.0/1000.0

#define SHOULD_CENTER_SCALE 0

#define POSE_INIT { -0.010706,  0.999802, -0.016746, -37.655368 * MODEL_UNIT_TO_METERS, \
                    -0.657489, -0.019656, -0.753208,  33.760876 * MODEL_UNIT_TO_METERS, \
                    -0.753388,  0.002946,  0.657570, 193.219878 * MODEL_UNIT_TO_METERS, \
                     0.000000,  0.000000,  0.000000,   1.000000}


// Centered and scaled 
// #define POSE_INIT {-0.0107061000000000, 0.9998020000000000, -0.0167459000000000, 0.0364667000000000, -0.6574890000000000, -0.0196561000000000, -0.7532080000000000, -0.0588530000000000, -0.7533879999999999, 0.0029463600000000, 0.6575700000000000, 1.8906300000000000, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 1.0000000000000000}
// #define SHIFT {-37.843102, -41.777825, -18.000000} // SHIFT = -CENTER
// #define SCALE 0.0107016

//====== INTERNAL TYPESDEFS ====================================================

//====== INTERNAL DATATYPES ====================================================

//====== INTERNAL VARIABLES ====================================================

//====== INTERNAL FUNCTDEFS ====================================================

//====== INTERNAL FUNCTIONS ====================================================

Rox_ErrorCode rox_array2d_uchar_depth_ogl_convert ( Rox_Array2D_Uchar Zogl_uchar, Rox_Array2D_Float Zogl )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uchar ** Zogl_uchar_data = NULL;
   Rox_Float ** Zogl_data = NULL;

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size ( &rows, &cols, Zogl_uchar );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &Zogl_uchar_data, Zogl_uchar );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_get_data_pointer_to_pointer( &Zogl_data, Zogl );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint r = 0; r < rows; r++ )
   {
      for ( Rox_Sint c = 0; c < cols ; c++ )
      {
         float depth = Zogl_data[r][c];
         unsigned char val   = (unsigned char) ( depth * 255.0f );
         // The rows are inverted for the OpenGL depth
         Zogl_uchar_data[ rows - 1 - r ][ c ] = val;
      }
   }

function_terminate:
   return error;
}

//====== EXPORTED FUNCTIONS ====================================================


ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_model_renderer_depth_map )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char modelpath[FILENAME_MAX];

   Rox_Sint cols = 1920, rows = 1080;
   Rox_Double cTo_data_double[16] = POSE_INIT;
   Rox_MatSE3 cTo = NULL;
   Rox_MatUT3 Kc = NULL;
   Rox_Imask Zm = NULL;
   Rox_Array2D_Float Z = NULL;
   Rox_Array2D_Float Zi = NULL;
   Rox_Array2D_Float Ziu = NULL;
   Rox_Array2D_Float Ziv = NULL;
   Rox_Array2D_Float Zogl = NULL;

   // Define timer to measure performances
   Rox_Timer timer = NULL;
   Rox_Double time = 0.0;

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_new ( &cTo );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   error = rox_matse3_set_data ( cTo, cTo_data_double );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   error = rox_matut3_new ( &Kc );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   error = rox_transformtools_build_calibration_matrix ( Kc, FU, FV, CU, CV );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   error = rox_imask_new ( &Zm, cols, rows );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   error = rox_array2d_float_new ( &Z, rows, cols );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   error = rox_array2d_float_new ( &Zi, rows, cols );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   error = rox_array2d_float_new ( &Ziu, rows, cols );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   error = rox_array2d_float_new ( &Ziv, rows, cols );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   error = rox_array2d_float_new ( &Zogl, rows, cols );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   //=======================================================================================

   sprintf ( modelpath, MODEL_PATH );
   ROX_TEST_MESSAGE( "reading file : %s\n", modelpath );

#ifdef INIT_MODEL_FROM_BUFFERS
   const Rox_Sint ignore_material = 0;

   Rox_Sint n_vertices = 0;
   Rox_Sint n_uniq_vertices = 0;
   Rox_Sint n_faces = 0;
   Rox_Sint n_colors = 0;
   Rox_Sint texture_data_size = 0;

   // Create a new model
   rox_model model = NULL;

   error = rox_model_new ( &model );                                                                                 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Load model from file
   error = rox_model_load_file ( model, &n_vertices, &n_uniq_vertices, &n_faces, &n_colors, &texture_data_size, modelpath, ignore_material );                                                                       
   ROX_ERROR_CHECK_TERMINATE ( error );

   ROX_TEST_MESSAGE("n_vertices = %d", n_vertices);
   ROX_TEST_MESSAGE("n_uniq_vertices = %d", n_uniq_vertices);
   ROX_TEST_MESSAGE("n_faces = %d", n_faces);
   ROX_TEST_MESSAGE("n_colors = %d", n_colors);
   ROX_TEST_MESSAGE("texture_data_size = %d\n", texture_data_size);

   //=======================================================================================
#endif

   Rox_Model_Renderer model_renderer = NULL;

   error = rox_model_renderer_new ( &model_renderer, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

#ifdef INITI_MODEL_FROM_BUFFERS
   error = rox_model_renderer_set_model_geometry ( model_renderer, model, MODEL_UNIT_TO_METERS );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
#else
   error = rox_model_renderer_read_model_geometry ( model_renderer, modelpath, MODEL_UNIT_TO_METERS );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
#endif

   rox_timer_start(timer);

   error = rox_model_renderer_make ( model_renderer, Kc, cTo );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);   

   // Display elapsed time
   rox_timer_stop(timer);
   rox_timer_get_elapsed_ms(&time, timer);

   ROX_TEST_MESSAGE("mean time to render a (%d x %d) image = %f (ms)", cols, rows, time );

   error = rox_model_renderer_get_depth_map ( Z, Zm, model_renderer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_timer_start(timer);

   error = rox_model_renderer_get_depth_map_inverse ( Zi, Ziu, Ziv, Zm, model_renderer );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);
   
   // Display elapsed time
   rox_timer_stop(timer);
   rox_timer_get_elapsed_ms(&time, timer);

   ROX_TEST_MESSAGE("mean time to get depth map %f (ms)", time);

   error = rox_model_renderer_get_depth_map_ogl ( Zogl, model_renderer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Image Zogl_uchar = NULL;
   error = rox_image_new ( &Zogl_uchar, cols, rows );                                                   
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_depth_ogl_convert ( Zogl_uchar, Zogl );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //rox_image_save_pgm (RESULT_PATH"/Zogl.pgm", Zogl_uchar );
   //rox_imask_save_pgm (RESULT_PATH"/Zm.pgm", Zm );

   //rox_array2d_float_save ( RESULT_PATH"/Z.txt", Z );
   //rox_array2d_float_save ( RESULT_PATH"/Zi.txt", Zi );
   //rox_array2d_float_save ( RESULT_PATH"/Ziu.txt", Ziu );
   //rox_array2d_float_save ( RESULT_PATH"/Ziv.txt", Ziv );
   //rox_array2d_float_save ( RESULT_PATH"/Zogl.txt", Zogl );

   ROX_TEST_MESSAGE("extract reference image and mask" );

   //=======================================================================================

   Rox_Image image = NULL;
   error = rox_image_new_read_pgm(&image, IMAGE_PATH);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Image image_ref = NULL;
   Rox_Imask imask_ref = NULL;
   Rox_Rect_Sint_Struct roi;

   // Extract the image_ref and imask_ref from depth map
   error = rox_image_new_roi ( &image_ref, &imask_ref, &roi, image, Zm );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //=======================================================================================

   ROX_TEST_MESSAGE("Comparing with references");
   // Rox_Double score = 0; 

   Rox_Sint ref_rows=0, ref_cols = 0;
   error = rox_image_get_size(&ref_rows, &ref_cols, image_ref);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Rox_Rect_Sint_Struct window;
   // window.x = 0;   window.y = 0;
   // window.height = ref_rows;   window.width = ref_cols;

   Rox_Image image_ref_ref = NULL;
   error = rox_image_new_read_pgm(&image_ref_ref, IMAGE_REF_PATH);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   //error = rox_image_window_nomask_difference_get_score(&score, &window, image_ref, image_ref_ref);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   //revert ref, perfom AND operation and check if result is empty
   //if not there's a difference between the masks
   Rox_Imask imask_ref_ref = NULL;
   error = rox_imask_new_read_pgm(&imask_ref_ref, IMASK_REF_PATH);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   Rox_Imask mask_and_result = NULL;
   error = rox_imask_new(&mask_and_result, ref_cols, ref_rows);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   error = rox_imask_set_not(imask_ref_ref);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);
   
   error = rox_imask_set_and(mask_and_result, imask_ref_ref, imask_ref);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   Rox_Sint is_empty = 0;
   error = rox_imask_count_valid(&is_empty, mask_and_result);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);
   ROX_TEST_CHECK_EQUAL(is_empty, 0);

   //error = rox_image_window_difference_get_score(&score, &window, imask_ref, imask_ref_ref);
   //ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);


   //error = rox_imask_save_pgm (RESULT_PATH"/reference_imask.pgm", imask_ref );
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //error = rox_image_save_pgm (RESULT_PATH"/reference_image.pgm", image_ref );
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   
   error = rox_matse3_del ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_del ( &Kc );  
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_del ( &Zm );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Z );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Zi );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Ziu );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Ziv );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Zogl );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_model_renderer_del ( &model_renderer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del ( &image_ref );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_del ( &imask_ref );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del ( &Zogl_uchar );                                                   
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del ( &image );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

#ifdef INIT_MODEL_FROM_BUFFERS
   rox_model_del ( &model );
#endif

}

#define INIT_MODEL_EDGES_FROM_BUFFERS

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_model_renderer_visible_edges )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char modelpath[FILENAME_MAX];

   Rox_Sint rows = 1080;
   Rox_Sint cols = 1920;

   Rox_Double cTo_data_double[16] = POSE_INIT;
   Rox_MatSE3 cTo = NULL;
   Rox_Array2D_Uint visible_edges_ogl = NULL;

   // Define timer to measure performances
   Rox_Timer timer = NULL;
   Rox_Double time = 0.0;

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_new ( &cTo );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   error = rox_matse3_set_data ( cTo, cTo_data_double );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   error = rox_array2d_uint_new ( &visible_edges_ogl, rows, cols );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   //=======================================================================================

   sprintf ( modelpath, MODEL_PATH );
   ROX_TEST_MESSAGE( "reading file : %s\n", modelpath );

#ifdef INIT_MODEL_EDGES_FROM_BUFFERS
   Rox_ObjFile_Edges objfile_edges = NULL;

   Rox_Point3D_Float_Struct * vertices = NULL;
   Rox_Triangle_Index_Struct * triangles = NULL;
   Rox_Contour_Edge * contours = NULL;
   Rox_Uint vertices_nbr = 0, triangles_nbr = 0, contours_nbr = 0;

   // Load object
   error = rox_objfile_edges_new ( &objfile_edges );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   error = rox_objfile_edges_load_obj ( objfile_edges, (char*) MODEL_PATH );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   error = rox_objfile_edges_scale_to_meters ( objfile_edges, MODEL_UNIT_TO_METERS );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   error = rox_objfile_edges_process ( objfile_edges, SHOULD_CENTER_SCALE );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);
   
   error = rox_objfile_edges_get_data ( &vertices, &vertices_nbr, &triangles, &triangles_nbr, &contours, &contours_nbr, objfile_edges );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   ROX_TEST_MESSAGE("vertices nbr = %d", vertices_nbr);
   ROX_TEST_MESSAGE("triangles nbr = %d", triangles_nbr);
   ROX_TEST_MESSAGE("contours nbr = %d", contours_nbr);

   //=======================================================================================
#endif

   Rox_Model_Renderer model_renderer = NULL;

   error = rox_model_renderer_new ( &model_renderer, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_model_renderer_setedges_intrinsics ( model_renderer, FU, FV, CU, CV );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

#ifdef INIT_MODEL_EDGES_FROM_BUFFERS
   Rox_Float min_angle_hard_contour = 60.0 * M_PI / 180.0;

   Rox_Uint * indices = new unsigned int[contours_nbr * 2];
   for (size_t i = 0; i < contours_nbr * 2; i+=2)
   {
      indices[i  ] = i / 2;
      indices[i+1] = i / 2;
   };

   error = rox_model_renderer_set_model_edges_from_buffers (
            model_renderer,
            (float*) &vertices[0],
            vertices_nbr,
            (unsigned int*) triangles, 
            triangles_nbr,
            (float*) &contours->pt1, 
            contours_nbr, 
            indices,
            min_angle_hard_contour 
           );
   rox_log("error = %d \n", error);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
#else
   error = rox_model_renderer_read_model_edges ( model_renderer, modelpath, MODEL_UNIT_TO_METERS );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
#endif

   rox_timer_start(timer);

   error = rox_model_renderer_make_visible_edges ( model_renderer, cTo );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);   

   // Display elapsed time
   rox_timer_stop(timer);
   rox_timer_get_elapsed_ms(&time, timer);

   ROX_TEST_MESSAGE("mean time to render a (%d x %d) image = %f (ms)", cols, rows, time );

   rox_timer_start(timer);

   error = rox_model_renderer_get_visible_edges_ogl ( visible_edges_ogl, model_renderer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Display elapsed time
   rox_timer_stop(timer);
   rox_timer_get_elapsed_ms(&time, timer);

   ROX_TEST_MESSAGE("mean time to get visible edges %f (ms)", time);


   rox_image_rgba_save_ppm ( RESULT_PATH"test_visible_edges_ogl.ppm", visible_edges_ogl );


   error = rox_matse3_del ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del ( &visible_edges_ogl );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_model_renderer_del ( &model_renderer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );


#ifdef INIT_MODEL_EDGES_FROM_BUFFERS
   rox_objfile_edges_del ( &objfile_edges );
#endif

}

ROX_TEST_SUITE_END()
