//==============================================================================
//
//    OPENROX   : File test_quad_detection.cpp
//
//    Contents  : Tests for quad_detection.c
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
   #include <math.h>

   #include <system/time/timer.h>

   #include <baseproc/image/draw/color.h>
   #include <baseproc/image/draw/draw_polygon.h>
   #include <baseproc/image/image.h>
   #include <baseproc/image/image_rgba.h>
   #include <inout/image/ppm/ppmfile.h>

   #include <core/features/detectors/quad/quad_detection.h>

   #include <inout/geometry/point/point2d_print.h>
   #include <inout/system/print.h>

}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(quad_detection)

#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/detection/quad/image_quad_centered_1.pgm"
// #define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/photoframe/images/image_000.pgm"
// #define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/photoframe/images/image_346.pgm"
// #define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_3840x2160.pgm"
// #define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_1920x1080.pgm"
// #define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_640x480.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_quaddetector_new_del)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_QuadDetector quad_detector = NULL;
   Rox_Sint cols  = 1920;
   Rox_Sint rows = 1080;

   error = rox_quaddetector_new ( &quad_detector, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_quaddetector_del ( &quad_detector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_quaddetector_process_image)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   // Define timer to measure performances
   Rox_Timer timer = NULL;
   Rox_Double time = 0.0, total_time = 0.0;

   Rox_QuadDetector quad_detector = NULL;

   Rox_Image image = NULL;
   Rox_Imask  mask = NULL;

   Rox_Sint cols = 0;
   Rox_Sint rows = 0;
   Rox_Sint nbtests = 1;

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);

   error = rox_image_new_read_pgm(&image, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_cols(&cols, image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_rows(&rows, image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New current mask
   error = rox_array2d_uint_new(&mask, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_quaddetector_new ( &quad_detector, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_quaddetector_set_quad_color(quad_detector, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //error = rox_quaddetector_set_side_bounds(quad_detector, 40, 40);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_timer_start(timer);

   total_time = 0.0;
   for (Rox_Sint i = 0; i < nbtests; i++)
   {
      error = rox_quaddetector_process_image(quad_detector, image, mask);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }

   rox_log("time to detect quads in a (%d x %d) image = %f (ms)\n", cols, rows, total_time/nbtests);

   Rox_Sint detected_quads = 0;

   error = rox_quaddetector_get_quad_count(&detected_quads, quad_detector);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("detected quads = %d\n", detected_quads);

   Rox_Image_RGBA image_display = NULL;

   error = rox_image_rgba_new_read_pgm ( &image_display, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for ( Rox_Sint i=0; i < detected_quads; i++)
   {
      Rox_Uint color = ROX_MAKERGBA(255, 0, 0, 255);
      Rox_Point2D_Double_Struct pts[4];

      error = rox_quaddetector_get_points(pts, quad_detector, i);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // error = rox_vector_point2d_double_print(pts, 4);
      // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_image_rgba_draw_polygon(image_display, pts, 4, color);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }  

   // error = rox_quaddetector_save_segment2d ("./test_quad_segments.txt", quad_detector);
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_image_rgba_save_ppm("./test_quad_detection_process_image.ppm", image_display);
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_quaddetector_del ( &quad_detector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del ( &image );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_del ( &image_display );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_del ( &mask );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_quaddetector_process_image_ac)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   // Define timer to measure performances
   Rox_Timer timer = NULL;
   Rox_Double time = 0.0, total_time = 0.0;

   Rox_QuadDetector quad_detector = NULL;

   Rox_Image image = NULL;
   Rox_Imask  mask = NULL;

   Rox_Sint cols = 0;
   Rox_Sint rows = 0;
   Rox_Sint nbtests = 1;

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   
   error = rox_image_new_read_pgm(&image, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_cols(&cols, image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_rows(&rows, image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New current mask
   error = rox_imask_new(&mask, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_quaddetector_new ( &quad_detector, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_quaddetector_set_quad_color(quad_detector, 0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //error = rox_quaddetector_set_side_bounds(quad_detector, 40, 40);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_timer_start(timer);

   total_time = 0.0;
   for (Rox_Sint i = 0; i < nbtests; i++)
   {
      error = rox_quaddetector_process_image_ac(quad_detector, image, mask);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }

   rox_log("time to detect quads in a (%d x %d) image = %f (ms)\n", cols, rows, total_time/nbtests);

   Rox_Sint detected_quads = 0;

   error = rox_quaddetector_get_quad_count(&detected_quads, quad_detector);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("detected quads = %d\n", detected_quads);

   Rox_Array2D_Uint image_display = NULL;

   error = rox_image_rgba_new_read_pgm(&image_display, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for(Rox_Sint i=0; i<detected_quads; i++)
   {
      Rox_Uint color = ROX_MAKERGBA(255, 0, 0, 255);
      Rox_Point2D_Double_Struct pts[4];

      error = rox_quaddetector_get_points(pts, quad_detector, i);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_image_rgba_draw_polygon(image_display, pts, 4, color);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }  

   //error = rox_image_rgba_save_ppm("./test_quad_detection_process_image_ac.ppm", image_display);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_quaddetector_del ( &quad_detector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_del ( &image_display );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del ( &image );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_imask_del ( &mask );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_quaddetector_compute_quads)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // Rox_DynVec_Quad quadlist = NULL;
   // Rox_DynVec_Quad_Segment2D seglist = NULL; 
   Rox_Double side_min = 0;
   Rox_Double side_max = 10e12;
   Rox_Double area_min = 0;
   Rox_Double area_max = 10e12;

   Rox_DynVec_Quad dynvec_quad = NULL;
   Rox_DynVec_Quad_Segment2D dynvec_quad_segment2d = NULL;

   error = rox_dynvec_quad_new(&dynvec_quad, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_quad_segment2d_new ( &dynvec_quad_segment2d, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_quad_computequads ( dynvec_quad, dynvec_quad_segment2d, side_min, side_max, area_min, area_max ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_quad_segment2d_del ( &dynvec_quad_segment2d );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_quad_del ( &dynvec_quad );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_quaddetector_get_quad_count)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_quaddetector_get_quad_SL3)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
