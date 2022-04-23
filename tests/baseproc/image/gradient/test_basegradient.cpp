//==============================================================================
//
//    OPENROX   : File test_basegradient.cpp
//
//    Contents  : Tests for basegradient.c
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
   #include <baseproc/array/fill/fillval.h>
   #include <baseproc/array/fill/fillzero.h>
   #include <baseproc/array/conversion/array2d_float_from_uchar.h>
   #include <baseproc/array/error/l2_error.h>

   #include <baseproc/image/gradient/basegradient.h>
   #include <baseproc/image/imask/imask.h>   
   #include <inout/image/pgm/pgmfile.h>
   #include <inout/numeric/array2d_save.h>
   #include <inout/numeric/array2d_print.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(basegradient)

#define TEST_IMG ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D000.pgm"
// #define TEST_IMG ROX_DATA_HOME"/../../Data/database/test_database_1280x0720.pgm"
// #define TEST_IMG ROX_DATA_HOME"/../../Data/database/test_database_1920x1080.pgm"
// #define TEST_IMG ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_3840x2160.pgm"
// #define TEST_IMG ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_1920x1080.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

Rox_Float I_data[10*12] =  {  132, 225,  23, 215, 133, 123,  72, 152, 166, 246, 214, 110,
                              114, 207, 112,  68,  52,  66,  75, 180,  58, 190, 125, 129,
                              158, 252, 145, 239,  15,  29, 185, 135, 152,  20, 243, 210,
                              220, 233, 146, 251,  69,  79,  16, 240, 116, 132,  28, 217,
                              204,  92, 250, 107, 235,  30,  51, 249, 183,  72, 150, 212,
                               76, 197,  45,  19, 215, 121, 138, 228, 183, 107,  67, 227,
                               65, 232,  47, 198, 132, 179, 128, 135,  18, 230, 101,  22,
                                4,  76,  40, 100, 194, 238, 145, 123, 118, 170, 243, 165,
                               73, 204,  78,  14, 255, 239,  21,  57,  90,  25, 186, 180,
                              100, 125,  83, 138, 158, 127,   9,  88, 121,  13, 207, 230 };

Rox_Float Iu_grt_data[10*12] = {   93.0000,  -54.5000,   -5.0000,   55.0000,  -46.0000,  -30.5000,   14.5000,   47.0000,   47.0000,   24.0000,  -68.0000, -104.0000,
                               93.0000,   -1.0000,  -69.5000,  -30.0000,   -1.0000,   11.5000,   57.0000,   -8.5000,    5.0000,   33.5000,  -30.5000,    4.0000,
                               94.0000,   -6.5000,   -6.5000,  -65.0000, -105.0000,   85.0000,   53.0000,  -16.5000,  -57.5000,   45.5000,   95.0000,  -33.0000,
                               13.0000,  -37.0000,    9.0000,  -38.5000,  -86.0000,  -26.5000,   80.5000,   50.0000,  -54.0000,  -44.0000,   42.5000,  189.0000,
                             -112.0000,   23.0000,    7.5000,   -7.5000,  -38.5000,  -92.0000,  109.5000,   66.0000,  -88.5000,  -16.5000,   70.0000,   62.0000,
                              121.0000,  -15.5000,  -89.0000,   85.0000,   51.0000,  -38.5000,   53.5000,   22.5000,  -60.5000,  -58.0000,   60.0000,  160.0000,
                              167.0000,   -9.0000,  -17.0000,   42.5000,   -9.5000,   -2.0000,  -22.0000,  -55.0000,   47.5000,   41.5000, -104.0000,  -79.0000,
                               72.0000,   18.0000,   12.0000,   77.0000,   69.0000,  -24.5000,  -57.5000,  -13.5000,   23.5000,   62.5000,   -2.5000,  -78.0000,
                              131.0000,    2.5000,  -95.0000,   88.5000,  112.5000, -117.0000,  -91.0000,   34.5000,  -16.0000,   48.0000,   77.5000,   -6.0000,
                               25.0000,   -8.5000,    6.5000,   37.5000,   -5.5000,  -74.5000,  -19.5000,   56.0000,  -37.5000,   43.0000,  108.5000,   23.0000};


Rox_Float Iv_grt_data[10*12] =  { -18.0000,  -18.0000,   89.0000, -147.0000,  -81.0000,  -57.0000,    3.0000,   28.0000, -108.0000,  -56.0000,  -89.0000,   19.0000,
                               13.0000,   13.5000,   61.0000,   12.0000,  -59.0000,  -47.0000,   56.5000,   -8.5000,   -7.0000, -113.0000,   14.5000,   50.0000,
                               53.0000,   13.0000,   17.0000,   91.5000,    8.5000,    6.5000,  -29.5000,   30.0000,   29.0000,  -29.0000,  -48.5000,   44.0000,
                               23.0000,  -80.0000,   52.5000,  -66.0000,  110.0000,    0.5000,  -67.0000,   57.0000,   15.5000,   26.0000,  -46.5000,    1.0000,
                              -72.0000,  -18.0000,  -50.5000, -116.0000,   73.0000,   21.0000,   61.0000,   -6.0000,   33.5000,  -12.5000,   19.5000,    5.0000,
                              -69.5000,   70.0000, -101.5000,   45.5000,  -51.5000,   74.5000,   38.5000,  -57.0000,  -82.5000,   79.0000,  -24.5000,  -95.0000,
                              -36.0000,  -60.5000,   -2.5000,   40.5000,  -10.5000,   58.5000,    3.5000,  -52.5000,  -32.5000,   31.5000,   88.0000,  -31.0000,
                                4.0000,  -14.0000,   15.5000,  -92.0000,   61.5000,   30.0000,  -53.5000,  -39.0000,   36.0000, -102.5000,   42.5000,   79.0000,
                               48.0000,   24.5000,   21.5000,   19.0000,  -18.0000,  -55.5000,  -68.0000,  -17.5000,    1.5000,  -78.5000,  -18.0000,   32.5000,
                               27.0000,  -79.0000,    5.0000,  124.0000,  -97.0000, -112.0000,  -12.0000,   31.0000,   31.0000,  -12.0000,   21.0000,   50.0000};

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_float_basegradient_nomask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Sint rows = 10;
   Rox_Sint cols = 12;

   Rox_Float l2_error = 0.0;

   Rox_Array2D_Float image_float = NULL;

   Rox_Array2D_Float Iu = NULL;
   Rox_Array2D_Float Iv = NULL;
   
   Rox_Array2D_Float Iu_grt = NULL;
   Rox_Array2D_Float Iv_grt = NULL;

   error = rox_array2d_float_new ( &Iu_grt, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride ( Iu_grt, Iu_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new ( &Iv_grt, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride ( Iv_grt, Iv_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new ( &Iu, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_fillzero(Iu);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new ( &Iv, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_fillzero(Iv);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new ( &image_float, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride ( image_float, I_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_basegradient_nomask ( Iu, Iv, image_float);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Iu_sub_grt = NULL;
   error = rox_array2d_float_new_subarray2d ( &Iu_sub_grt, Iu_grt, 1, 1, rows-2, cols-2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Iv_sub_grt = NULL;
   error = rox_array2d_float_new_subarray2d ( &Iv_sub_grt, Iv_grt, 1, 1, rows-2, cols-2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Iu_sub = NULL;
   error = rox_array2d_float_new_subarray2d ( &Iu_sub, Iu, 1, 1, rows-2, cols-2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Iv_sub = NULL;
   error = rox_array2d_float_new_subarray2d ( &Iv_sub, Iv, 1, 1, rows-2, cols-2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_difference_l2_norm ( &l2_error, Iu_sub_grt, Iu_sub );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("l2_error Iu = %0.12f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 10);

   error = rox_array2d_float_difference_l2_norm ( &l2_error, Iv_sub_grt, Iv_sub );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   rox_log("l2_error Iv = %0.12f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 10);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_float_basegradient_nomask_perf)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Array2D_Uchar image_uchar = NULL;
   Rox_Array2D_Float image_float = NULL;

   Rox_Array2D_Float Iu = NULL;
   Rox_Array2D_Float Iv = NULL;

#ifdef DEBUG
   // Small number of tests for slow debug (e.g. with valgrind)
   Rox_Sint nb_tests = 1;
#else 
   // High number of tests for measuring average performance in release
   Rox_Sint nb_tests = 1000;
#endif

   Rox_Timer timer = NULL ;
   Rox_Double time = 0.0, total_time = 0.0;

   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", TEST_IMG);
   rox_log("read file %s\n", filename);

   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Sint rows = 0, cols = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&Iu, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&Iv, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&image_float, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_from_uchar(image_float, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test the image gradient without mask
   for(Rox_Sint i = 0; i < nb_tests; ++i)
   {
      rox_timer_start(timer);

      error = rox_array2d_float_basegradient_nomask(Iu, Iv, image_float);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to compute the base gradient of a (%d x %d) image with no mask = %f (ms)\n", cols, rows, total_time/nb_tests);

   // Save result
   // error = rox_array2d_float_save("test_Iu.txt", Iu);
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_array2d_float_save("test_Iv.txt", Iv);
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&Iu);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&Iv);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_float);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_array2d_float_basegradient )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Sint rows = 10;
   Rox_Sint cols = 12;

   Rox_Float l2_error = 0.0;

   Rox_Array2D_Float I = NULL;

   Rox_Array2D_Float Iu = NULL;
   Rox_Array2D_Float Iv = NULL;
   
   Rox_Array2D_Float Iu_grt = NULL;
   Rox_Array2D_Float Iv_grt = NULL;
   
   Rox_Imask Im = NULL;
   Rox_Imask Gm = NULL;

   error = rox_array2d_float_new ( &Iu_grt, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride ( Iu_grt, Iu_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new ( &Iv_grt, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride ( Iv_grt, Iv_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new ( &Iu, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_fillzero(Iu);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new ( &Iv, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_fillzero(Iv);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_new ( &Gm, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_new ( &Im, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_set_ones ( Im );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new ( &I, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride ( I, I_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_basegradient ( Iu, Iv, Gm, I, Im );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Iu_sub_grt = NULL;
   error = rox_array2d_float_new_subarray2d ( &Iu_sub_grt, Iu_grt, 1, 1, rows-2, cols-2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Iv_sub_grt = NULL;
   error = rox_array2d_float_new_subarray2d ( &Iv_sub_grt, Iv_grt, 1, 1, rows-2, cols-2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Iu_sub = NULL;
   error = rox_array2d_float_new_subarray2d ( &Iu_sub, Iu, 1, 1, rows-2, cols-2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Iv_sub = NULL;
   error = rox_array2d_float_new_subarray2d ( &Iv_sub, Iv, 1, 1, rows-2, cols-2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_basegradient ( Iu, Iv, Gm, I, Im);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_difference_l2_norm ( &l2_error, Iu_sub_grt, Iu_sub );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("l2_error Iu = %0.12f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 10);

   error = rox_array2d_float_difference_l2_norm ( &l2_error, Iv_sub_grt, Iv_sub );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   rox_log("l2_error Iv = %0.12f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 10);

   // rox_array2d_float_print(Iu);
   // rox_array2d_float_print(Iv);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_float_basegradient_perf )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Array2D_Uchar image_uchar = NULL;
   Rox_Array2D_Float image_float = NULL;

   Rox_Array2D_Float Iu = NULL;
   Rox_Array2D_Float Iv = NULL;
   Rox_Imask image_mask = NULL;
   Rox_Imask gradient_mask = NULL;

#ifdef DEBUG
   // Small number of tests for slow debug (e.g. with valgrind)
   Rox_Sint nb_tests = 1;
#else 
   // High number of tests for measuring average performance in release
   Rox_Sint nb_tests = 1000;
#endif

   Rox_Timer timer = NULL ;
   Rox_Double time = 0.0, total_time = 0.0;

   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", TEST_IMG);
   rox_log("read file %s\n", filename);

   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Sint rows = 0, cols = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&Iu, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&Iv, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&image_float, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_from_uchar(image_float, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_new ( &gradient_mask, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_new ( &image_mask, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_set_ones ( image_mask );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test the image gradient with mask
   for(Rox_Sint i = 0; i < nb_tests; ++i)
   {
      rox_timer_start(timer);

      error = rox_array2d_float_basegradient ( Iu, Iv, gradient_mask, image_float, image_mask);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to compute the base gradient of a (%d x %d) image = %f (ms)\n", cols, rows, total_time/nb_tests);

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del ( &image_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Iu );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Iv );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_float);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&gradient_mask);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&image_mask);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()

// Matlab code
// I = readPGM('/home/emalis/data/plane/image_plane3D000.pgm');
// [Iu, Iv]= gradient(I);
// Iu_rox = load('test_Iu.txt');
// Iv_rox = load('test_Iv.txt');
// DIu = abs(Iu-Iu_rox);
// DIv = abs(Iv-Iv_rox);
// norm(DIu(2:end-1,2:end-1))
// norm(DIv(2:end-1,2:end-1))
