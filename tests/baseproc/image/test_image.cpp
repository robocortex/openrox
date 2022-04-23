//==============================================================================
//
//    OPENROX   : File test_image.cpp
//
//    Contents  : Tests for image.c
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
   #include <inout/numeric/array2d_save.h>
   #include <inout/numeric/array2d_print.h>
   #include <baseproc/image/image.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(image)

// Image  512 x  512 =  262144 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/plane/image_plane3D000.pgm"
// Image 1280 x  720 =  921600 pixels
//#define IMAGE_PATH ROX_DATA_HOME"/model_based/switch/shield/sequence_1/ppm/switch_shield_seq1_img0001.ppm"
//#define IMAGE_PATH ROX_DATA_HOME"/model_based/switch/shield/sequence_1/pgm/switch_shield_seq1_img0001.pgm"
// Image 1920 x 1080 = 2073600 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/model_based/switch/lumia/sequence_1/ppm/switch_lumia_seq1_img0001.ppm"
// Image 4096 x 2160 = 8847360 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/projects/apra/deformation/img005.ppm"
// Image  517 x  719 =  371723 pixels
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/gormiti_517x719.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

// const type * pointer : is a pointer to a constant (i.e. we cannot do *pointer = NULL)

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Image image1 = NULL;
   Rox_Image image2 = NULL;
   Rox_Sint cols1 = 640, rows1 = 480;
   Rox_Sint cols1_get = 0, rows1_get = 0;
   Rox_Sint cols2 = 512, rows2 = 256;
   Rox_Sint cols2_get = 0, rows2_get = 0;

   error = rox_image_new(&image1, cols1, rows1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_new(&image2, cols2, rows2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_cols(&cols1_get, image1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_rows(&rows1_get, image1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_cols(&cols2_get, image2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_rows(&rows2_get, image2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("rows1_get = %d\n", rows1_get);
   rox_log("cols1_get = %d\n", cols1_get);

   rox_log("rows2_get = %d\n", rows2_get);
   rox_log("cols2_get = %d\n", cols2_get);

   rox_log("rows1 = %d\n", rows1);
   rox_log("cols1 = %d\n", cols1);

   rox_log("rows2 = %d\n", rows2);
   rox_log("cols2 = %d\n", cols2);

   ROX_TEST_CHECK_EQUAL(rows1_get, rows1);
   ROX_TEST_CHECK_EQUAL(cols1_get, cols1);

   ROX_TEST_CHECK_EQUAL(rows2_get, rows2);
   ROX_TEST_CHECK_EQUAL(cols2_get, cols2);

   error = rox_image_del(&image1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del(&image2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_image_get_data)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Image image = NULL;
   Rox_Sint cols = 1920, rows = 1080;
   Rox_Uchar * image_data = NULL;

   image_data = (Rox_Uchar *) rox_memory_allocate(sizeof(Rox_Uchar), rows*cols);

   error = rox_image_new(&image, cols, rows);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_data(image_data, image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del(&image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_memory_delete(image_data);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_image_readpgm)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Image image = NULL;
   Rox_Char filename[FILENAME_MAX];

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);

   error = rox_image_new_read_pgm(&image, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //error = rox_array2d_uchar_save("test_image_readpgm.txt", image);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //error = rox_image_save_pgm("test_image_readpgm.pgm", image);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del(&image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_image_copy)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Sint rows = 0, cols = 0;
   Rox_Image image = NULL;
   Rox_Image image_copy = NULL;

   // Define timer to measure performances
   Rox_Timer timer = NULL;

   Rox_Double time = 0.0, total_time = 0.0;
   Rox_Uint nbcopy = 1000;

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);

   error = rox_image_new_read_pgm(&image, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_size(&rows, &cols, image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_new(&image_copy, cols, rows);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   total_time = 0.0;
   for (Rox_Uint i = 0; i < nbcopy; i++)
   {
      rox_timer_start(timer);
      error = rox_image_copy(image_copy, image);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to copy a (%d x %d) image = %f (ms)\n", cols, rows, total_time/nbcopy);
   rox_log("total time = %f \n", total_time);

   error = rox_image_del(&image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del(&image_copy);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_timer_del(&timer);

}


ROX_TEST_SUITE_END()
