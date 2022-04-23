//==============================================================================
//
//    OPENROX   : File test_multi_ident_texture_sl3.cpp
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
#ifndef WIN32
   // Include needed to create directory
   #include <sys/types.h>
   #include <sys/stat.h>
   #include <unistd.h>
#endif

   #include <system/time/timer.h>

   #include <baseproc/geometry/rectangle/rectangle_struct.h>
   #include <baseproc/image/draw/color.h>
   #include <baseproc/image/image.h>
   #include <baseproc/image/image_rgba.h>
   #include <baseproc/image/draw/image_rgba_draw.h>
   #include <inout/image/ppm/ppmfile.h>
   #include <inout/system/print.h>

   #include <user/identification/texture/multiident_sl3.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(multi_ident_texture_sl3)

#ifdef ANDROID
   #define RESULT_PATH "/storage/emulated/0/Documents/Robocortex/Tests/Results/"
#else
   #define RESULT_PATH "./results/"
#endif

// #define MODEL_PATH "/home/emalis/Services/verallia/Lecture_num_moules_exemples/Ebaucheur/pgm/5002795-C01_4032x3024.pgm"
// #define IMAGE_PATH "/home/emalis/Services/verallia/Lecture_num_moules_exemples/Ebaucheur/pgm/5002795-C01_4032x3024.pgm"

// #define MODEL_PATH "/home/emalis/Services/verallia/Lecture_num_moules_exemples/Ebaucheur/pgm/5002795-C01_2016x1512.pgm"
// #define IMAGE_PATH "/home/emalis/Services/verallia/Lecture_num_moules_exemples/Ebaucheur/pgm/5002795-C01_2016x1512.pgm"

// #define MODEL_PATH "/home/emalis/Services/verallia/Lecture_num_moules_exemples/Ebaucheur/pgm/5002795-C01_1008x0756.pgm"
// #define IMAGE_PATH "/home/emalis/Services/verallia/Lecture_num_moules_exemples/Ebaucheur/pgm/5002795-C01_1008x0756.pgm"

// #define NBM 44
// #define MODEL_PATH "/home/emalis/Services/verallia/Lecture_num_moules_exemples/Ebaucheur/pgm/image_%02d.pgm"
// #define IMAGE_PATH "/home/emalis/Services/verallia/Lecture_num_moules_exemples/Ebaucheur/pgm/image_43.pgm"


#define NBM 44
#define MODEL_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/database/images/toytest%03d.pgm"
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/database/images/toytest000.pgm"


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

   Rox_Timer  timer = 0;
   Rox_Image  model    = 0;
   Rox_Image  current  = 0;
   Rox_Image_RGBA display = 0;
   Rox_Multi_Ident_SL3  identifier = 0;

   Rox_MatSL3 homography = 0;
   Rox_Uint red = ROX_MAKERGBA(255, 0 , 0, 255);
   Rox_Double time;
   Rox_Rect_Sint_Struct window;
   Rox_Sint cols, rows;
   Rox_Char filename[FILENAME_MAX];
   
#ifndef WIN32
   struct stat st = {0};

   if (stat(RESULT_PATH, &st) == -1) {
       mkdir(RESULT_PATH, 0700);
   }
#endif

   // Create a MatSL3 homography
   error = rox_matsl3_new(&homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_multi_ident_sl3_new ( &identifier );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Read Model image

   sprintf(filename, MODEL_PATH, 0);

   error = rox_image_new_read_pgm ( &model, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_cols(&cols, model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_rows(&rows, model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //error = rox_multi_ident_sl3_add_model ( identifier, model );
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Add further models
   for (int k = 0; k < NBM; k++ )
   {
      sprintf(filename, MODEL_PATH, k);

      error = rox_image_new_read_pgm ( &model, filename );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_multi_ident_sl3_add_model ( identifier, model );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_image_del ( &model );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }

   // Read Current image

   sprintf(filename, "%s", IMAGE_PATH);

   error = rox_image_new_read_pgm(&current, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_new_read_pgm(&display, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Compile model images
   error = rox_multi_ident_sl3_compile ( identifier );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Identification and display elapsed time
   error = rox_timer_start(timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_multi_ident_sl3_make ( identifier, current );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_stop(timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_get_elapsed_ms(&time, timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("Time to identification %f\n", time);

   Rox_Uint detected_id = 100;

   // Draw results
   error = rox_multi_ident_sl3_get_homography ( homography, &detected_id, identifier );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("Target %d identified, saving the result in identification.ppm \n", detected_id);

   // Define model window
   window.x = 0; window.y = 0;
   window.width = cols; window.height = rows;

   error = rox_image_rgba_draw_warp_rectangle(display, homography, &window, red);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_save_ppm ( RESULT_PATH"./identification.ppm", display );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Delete objects and free memory
   rox_timer_del(&timer);
   rox_image_del(&model);
   rox_image_del(&current);
   rox_image_rgba_del(&display);
   rox_multi_ident_sl3_del(&identifier);
   rox_matsl3_del(&homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
