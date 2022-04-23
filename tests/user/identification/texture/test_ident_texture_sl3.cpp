//==============================================================================
//
//    OPENROX   : File test_ident_texture_sl3.cpp
//
//    Contents  : Tests for ident_texture_sl3.c
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
   #include <system/time/timer.h>

   #include <baseproc/geometry/rectangle/rectangle_struct.h>
   #include <baseproc/image/draw/color.h>
   #include <baseproc/image/image.h>
   #include <baseproc/image/image_rgba.h>
   #include <baseproc/image/draw/image_rgba_draw.h>
   #include <baseproc/array/error/l2_error.h>

   #include <inout/image/ppm/ppmfile.h>
   #include <inout/system/print.h>

   #include <user/identification/texture/identification.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(ident_texture_sl3)

#ifdef ANDROID
   #define RESULT_PATH "/storage/emulated/0/Documents/Robocortex/Tests/Results/"
#else
   #define RESULT_PATH "./"
#endif

// Model  128 x  128  
#define MODEL_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/model.pgm"
// Image  512 x  512 =  262144 pixels 
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D099.pgm"

// #define MODEL_PATH "/home/emalis/Services/verallia/Lecture_num_moules_exemples/Ebaucheur/pgm/5002795-C01_4032x3024.pgm"
// #define IMAGE_PATH "/home/emalis/Services/verallia/Lecture_num_moules_exemples/Ebaucheur/pgm/5002795-C01_4032x3024.pgm"

// #define MODEL_PATH "/home/emalis/Services/verallia/Lecture_num_moules_exemples/Ebaucheur/pgm/5002795-C01_2016x1512.pgm"
// #define IMAGE_PATH "/home/emalis/Services/verallia/Lecture_num_moules_exemples/Ebaucheur/pgm/5002795-C01_2016x1512.pgm"

// #define MODEL_PATH "/home/emalis/Services/verallia/Lecture_num_moules_exemples/Ebaucheur/pgm/5002795-C01_1008x0756.pgm"
// #define IMAGE_PATH "/home/emalis/Services/verallia/Lecture_num_moules_exemples/Ebaucheur/pgm/5002795-C01_1008x0756.pgm"

// #define MODEL_PATH "/home/emalis/Services/verallia/Lecture_num_moules_exemples/Ebaucheur/pgm/5002795-C01_0504x0378.pgm"
// #define IMAGE_PATH "/home/emalis/Services/verallia/Lecture_num_moules_exemples/Ebaucheur/pgm/5002795-C01_0504x0378.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

// const type * pointer : is a pointer to a constant (i.e. we cannot do *pointer = NULL)

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_ident_texture_sl3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Rect_Sint_Struct window;
   Rox_Uint red = ROX_MAKERGBA(255, 0 , 0, 255);
   Rox_Double H_grt[9] = {1.0988242786356330, -0.5442113147251797, 155.9614431748840104, 0.8703625669909365, 0.9706908084718749, -8.2009703121634647, 0.0001802938572460, -0.0003205448394573, 0.6965555098534626};
   Rox_Double time = 0.0;
   Rox_Sint cols = 0, rows = 0;
   Rox_Char filename[FILENAME_MAX];
   Rox_Double l2_error = 0.0;

   Rox_Timer  timer = NULL;
   Rox_Image  model    = NULL;
   Rox_Image  current  = NULL;
   Rox_Image_RGBA display = NULL;
   Rox_Identification  identifier = NULL;
   Rox_MatSL3 homography = NULL;
   Rox_MatSL3 homography_grt = NULL; 

   error = rox_matsl3_new ( &homography_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( homography_grt, H_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create a MatSL3 homography
   error = rox_matsl3_new(&homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   sprintf(filename, "%s", MODEL_PATH);
   rox_log("read file %s\n", filename);

   error = rox_image_new_read_pgm(&model, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_cols(&cols, model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_rows(&rows, model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);

   error = rox_image_new_read_pgm(&current, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_new_read_pgm(&display, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_identification_new(&identifier, model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("Starting identification\n");

   // Identification and display elapsed time
   error = rox_timer_start(timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_identification_make ( identifier, current );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_stop(timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_get_elapsed_ms(&time, timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("Time to identification %f\n", time);

   // Draw results
   error = rox_identification_get_homography(homography, identifier);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matsl3_print(homography);

   rox_log("Target identified, saving the result in identification.ppm \n");

   // Define model window
   window.x = 0; window.y = 0;
   window.width = cols; window.height = rows;

   error = rox_image_rgba_draw_warp_rectangle(display, homography, &window, red);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_save_ppm ( RESULT_PATH"./identification.ppm", display );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_difference_l2_norm ( &l2_error, homography_grt, homography );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);

   // Delete objects and free memory
   rox_timer_del(&timer);
   rox_image_del(&model);
   rox_image_del(&current);
   rox_image_rgba_del(&display);
   rox_identification_del(&identifier);
   rox_matsl3_del(&homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
