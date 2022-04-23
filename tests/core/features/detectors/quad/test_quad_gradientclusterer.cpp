//==============================================================================
//
//    OPENROX   : File test_quad_gradientclusterer.cpp
//
//    Contents  : Tests for quad_gradientclusterer.c
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
   #include <string.h>

   #include <generated/array2d_uchar.h>

   #include <system/time/timer.h>
   #include <baseproc/image/imask/imask.h>

	 #include <core/features/detectors/quad/quad_gradientclusterer.h>
   #include <inout/numeric/array_save.h>
   #include <inout/numeric/array2d_save.h>
   #include <inout/image/pgm/pgmfile.h>
   #include <inout/system/print.h>

}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(quad_gradientclusterer)

#define TEST_IMG     ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_3840x2160.pgm"
// #define TEST_IMG     ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_1920x1080.pgm"
// #define TEST_IMG     ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_640x480.pgm"
//#define TEST_IMG ROX_DATA_HOME"/regression_tests/openrox/detection/quad/image_quad_centered_1.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_gradientclusterer_new)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_gradientclusterer_del)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_gradientclusterer_reset)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_gradientclusterer_buildgradients_nreg)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   Rox_Uchar image_gray_buffer[18*32] = {
   208, 203, 178,  31, 208,  20, 176,  22, 103,  62, 140, 209,  58,  57, 133,  28, 206, 209,  16,  14,   9,  99, 109, 184, 163, 234,  62,  61,  82,  94,  89,  69, 
   231, 245,  81, 128,  63,  14, 191, 102,  20, 103,  76, 203,  44,  30,  60, 167, 148, 209, 102, 189, 144, 149,  80, 248, 245,   1, 234,  31,  31, 195,  39,  51,  
    33, 168, 243, 245, 237, 136, 115,  67,  62,  25, 190, 165,  59,  76, 125, 127,  47, 185, 135,  69, 225,  65,  42, 136,  62, 118,  69, 155, 240, 161, 150, 210, 
   233,  10,   9,  87,  90, 199,  22, 205,  32,  34,  49,  97, 112,  82, 160, 199,  62,  39, 107, 108, 171,  75,  46,  83, 173, 109, 196, 115, 165, 197,  67, 110, 
   162, 217, 112, 150,  51, 239,  59, 111,  47, 241, 176, 207,  80, 109, 174, 183, 227, 169, 168, 140,  49, 158, 108,  27,  74, 118,  49, 117, 123, 238,  12, 227,  
    25, 239,  98,  58,  65,  34, 233, 233,  62, 244,  47, 136, 236, 130, 101, 231,   8, 133, 161, 241,  95,  68,  25, 156, 172, 197,  74, 169, 164, 249, 193, 100,  
    72, 174, 196, 192, 158, 146,  39,  47, 107, 147,  94,  90, 110,  22,  94, 228, 125, 249,  75, 107, 118, 211, 153, 199, 178,  83,  24, 197, 139,  49,  62, 197, 
   140, 194, 203,  66, 121, 120, 211,  68,  13,  16, 160, 240,  48,  67, 252,  86,  43, 166, 111, 251, 251, 251, 121, 108,  18, 201, 147,  90, 166,  36, 113, 102, 
   245, 190,  48, 130,  90,   4, 138,  38, 231,  60, 199, 224, 231, 205,  10, 179, 250, 205,   4,  77,  40, 187, 178,  24,  65, 121, 175, 169, 139, 178, 176, 207, 
   247, 101, 125, 179, 212,  86, 255,  35, 241,  91,  21, 141, 250,   8, 226,  51, 182, 116, 251, 179, 219,  88, 179,  68,  58,  10, 140, 107, 184,  24,  92, 193, 
    41, 168, 114, 228, 150,  42,  20, 222, 126, 210, 237, 159, 112, 237, 233,   8, 128, 111,  43, 170, 165, 149, 163,  40, 171,  45, 109, 215, 134, 134, 188,  97, 
   248,  44, 165, 245, 141, 203, 113, 148, 125,   4, 198, 150,  29, 187, 204, 190, 121, 211,  28, 138,  96,  28,   9,  72, 216, 185, 165, 213, 254, 136, 101,  56, 
   245, 181, 181, 140, 234,  80,  28, 141,  87,  11, 125,  53,  66, 125,  26, 128,  16,  22,  95, 179,  49, 232,  18, 113,  88, 121, 166,  66,  56, 220, 175, 202, 
   124,   9, 193,  36,  73, 135, 246,  37, 230,  44, 112,  77, 105, 148,  67, 123, 174,  34,  51, 170, 110, 225,  82, 135, 200,  39, 174, 157,  27, 124, 180, 243, 
   205,  71,  71,  39, 194,  43,   2, 218,  95, 166, 114, 121, 152,  61,  86, 231,  11,  45, 125,  46, 123, 209, 136, 117, 173,  87, 163, 149,  28, 101, 113,  84,  
    37,  12, 174,  66, 193, 154, 198, 159,  29, 187,  79,  59,  67, 118, 174, 156,  19, 100,  87,  33,  31,  67, 167, 224,   2, 155, 242, 138,  17, 172,   5, 172, 
   108,  25, 168, 215,  98,  68, 209,  90, 199, 166, 130, 216, 154, 246,  35, 158, 134, 213, 243, 255, 151, 152, 104, 133, 154,  49,  54, 222, 104, 190,  85, 112, 
   234, 210,  42,  65, 145, 167, 222, 131, 100, 115, 131,  50, 182, 140, 184, 220,  25, 205, 235,  44,  58,   6, 210, 241,  99, 189, 181,  68, 115, 133, 109, 213};
   

  // Rox_Sint image_gradient_u_buffer[18*32] = {
  //  -240,    0,   -8, -420, -144, -180,  -28,  592, -468, -788,  372,  616, -188, -324,  108,  596,  -32, -672,  440, -172, -460,   96,  -64,  416,  168,  -16,  -88,  404,  616, -856, -780,   64,
  //  1680,  816, -512, -700,  300,  -12, -260,  -48,   40,  712, -372, -652,  108,  712,  324,  -28, -564, -572,   52, -232,  108,  320,  444, -200, -212,  436, -244,    8,  600,  -32, -516, -784, 
  //  -400, -444, -212,  -16,  468,  132,   -8,  588,  -24, -712, -240,  100, -236,  196, -148, -392,  344,  160,  540,  -12, -744,  696,  216, -912,  240,  348, -544,  308,  196,  -32,  476,  232,  
  //   992,  344, -628, -204,  220,   12,    0,   60,  -76,  -28,  312,  240, -516, -156,  152,    4,  564,  -76,   32, -432, -204,  140,  368,  -20, -564,  832,   40, -932,  104,  592, -136, -424, 
  // -1576, -620,  360,  564,  244,    0, -756,   76,  -24, -840,  696,  100,  -12,  296,    4,  432, -204, -140,  520,  116, -936,   36,  820, -272,  100,  148, -288, -644, -156,  504, -192, -736,  
  //   584,  436, -376,  -96,  868,  -56,  -16,   64, -760,  148,  480,   24, -216, -148,  316,  456,  -92, -428, -208,  -64,  340, -384,   24,  368, -704, -100,  -76,  480,  624,  152, -256,-1224, 
  //  -512,  -44, -612, -588,  388,  -84, -396, -200,  276,  132, -308,  156,  592, -128,   36,  272,   72, -204,  212,  628, -536, -500, -180,  276,  -64, -256,  124,  360,  368, -692,   24, 1224, 
  // -1072,  -44, -276, -396,  448,  228, -112, -564,  156,  448,   44,  200,  108, -580, -200,   36,  244,  624,   96, -656, -476,  152,  380, -192, -348,   48,  320, -116,  300,   60, -948,  -96,  
  //   480,  -40,  164,  404, -316, -180, -380,  -80,  832,   68, -556,   12, -200, -200,  124,  520, -172, -356,  720, -180, -364,  328,  212, -628,  204,  132, -100,  164, -116,   64, -288,  -48, 
  // -1376, -732,  -36,  168,  748,  -20, -724,  608,  -56, -440,  680,  540, -492, -560,   32,  604,  236, -924, -104,  356,  404, -292, -144,  716,   88, -240,  236, -592, -868,  776,   80,-1152,  
  //   616, -128, -712, -444,  392,  204, -152,  436, -232, -480,  268,  212,   76,  172,  220, -604, -492,  756,  472, -660, -432,  192,  408, -172, -364,  144, -100,  136,  128,  352,  -20, -696, 
  //  -536,   88, -156,    8,  384, -528,  -96,  240, -172,  -44, -452, -160,  360,  480,  440,    4, -676, -820,  700,  840, -388, -456,  380,   12,   40, -260, -340,  516,   52,  208, -276,-1192,  
  //   144, -112, -580,    4,  172, -500,  552,  908, -484, -152,  368, -512,   -8, -136,    0,  580, -524, -296,    8, -416,  688,  204, -616,  396,  660,  340, -572, -200,  -72, -660,   48,  528, 
  //  -480, -744, -368,  224,  744,  580, -180,    0, -408,  -72, -212, -264,  580,  228,  152, -308, -112,  -60, -576,   20,  424, -332, -360,  432,  512, -112, -444, -312,  156,   56,  344, 1352,  
  //   480, -504,  -24,  672, -252,  108, -604, -504,  648,  152, -520, -416,  700,  356, -472, -360,  592,  636, -388, -796,    8,  732,  160,   96, -208, -732, -264,  628,  564, -196, -628, -640, 
  //  -328, -404,    4, -276, -352,  456,  300, -412,  204,  840, -264, -444, -188,  396,  544, -844, -664,  504,    4, -364,  160,  416,  184, -456,  188,  764, -140, -744,   88,  340,  344,  952, 
  //  -256, -112,  716,  492, -388, -420,  440,  -64, -740,  656,  284, -488,  128,   56, -260, -356,  -72,  568,  372,  132, -400, -276,  388,  -72, -492, -132,  764,  352, -344,   68,  136,    0, 
  //  1520,  172, -500,  392,  504,  320, -232, -132,  388, -244, -488, -412,  356,  564, -440,   80, -188, -172,  264, -208, -324,  100,  680,  260, -704, -796,  788,  732, -692, -224,  580,  640};

  // Rox_Sint image_gradient_v_buffer[18*32] = {
  // -1400,   520,   232,  -488,  -328,   400,     8,   -64, -1272,   952,  1728,  -536,  -808,    56,  1264,   488,    16,  -576,   216, -1352,    96,  -216,   544,   800,  -688,    40,   216,  -272,  -576,  -304,  1072,   224,
  //  -184,  -264,  -628,  -468,  -224,   144,    88,   164,    84,   608,   160,    -4,  -356,   -52,   164,  -308,  -824,    68,     8,   168,   168,  -116,   768,   164,  -560,   236,  -196,  -220,  -292,  -640,   532,   616,
  //   156,  -188,  -316,  -304,   180,  -384,   204,  -124,   312,  -240,  -428,   444,   464,  -180,  -404,  -352,  -372,   776,   124,   756,   -76,   444,  -256,   368,   -76,    16,   320,   300,  -620,  -196,     4,   184,
  //   332,  -256,   156,   316,   736,    92,   604,  -656,    92,  -656,   -36,   280,   -36,   504,    64,   656,   888,   108,   588,    88,   716,  -104,    56,   500,   696,   360,   496,   616,  -456,   264,    80,  -404,
  //  -124,  -328,   -32,   -76,    76,   572,     8,   556,    12,  -128,   188,    40,   -28,   340,   -20,   504,   432,  -152,    80,  -392,   448,   152,   -76,  -192,   312,  -332,  -620,  -448,   792,    72,   352,   -48,
  //     0,   532,   576,  -440,  -576,  -296,  -660,    64,  -936,   364,    36,  -640,    92,   -36,  -332,    -4,  -492,   272,  -556,   -36,   -44,   364,  -580,  -636,   -32,  -800,  -436,  -388,   568,   136,  -628,   352,
  //   656,  -172,   176,   -72,  -124,  -492,   160,  -588,  -468,   328,  -168,  -108,     8,   216,  -424,  -300,  -844,    36,   208,   340,  -384,  -476,   152,  -120,  -408,   236,  -260,   632,  -856,   308,  -948,  -384,
  //  -548,   -52,  -544,   724,   448,    20,   352,    36,   472,   592,   408,   344,   264,  -448,   192,  -360,   440,  -604,   288,   -96,  -520,    76,   308,   468,  -596,   736,  -208,   512,  -404,    28,   352,  -284,
  //    -8,  -160,  -696,    80,  -132,   380,  -380,  -232,   792,  -444,   -96,   192,   244,  -408,   264,  -176,   832,  -184,  -716,  -384,   296,   496,  -148,   -28,   760,   408,   472,   324,    -4,  -844,   712,   184,
  //   240,   308,   152,  -568,  -696,   140,  -312,   368,   204,  -696,  -344,   128,  -144,   404,   228,   500,  -896,   180,   216,   -68,  -264,  -136,  -400,    60,    56,  -508,    68,  -508,    40,  -264,   328,     4,
  //    56,   476,   876,   356,   716,    -8,   208,   620,  -160,   504,   236,  -628,  -464,   224,   576,   632,   -24,  -280,    80,   524,   564,  -268,   400,   256,  -304,   208,  -324,  -368,   784,   552,   216,   196,
  //    24,  -212,    40,   -80,   488,  -300,  -216,   404,   256,   152,   584,   252,  -140,   168,  -448,   -52,   736,   -84,  -316,  -548,   -72,   572,   -60,  -452,   508,   572,   704,   100,   368,  -100,  -644,   -32,
  //  -108,   -80,  -940,  -292,  -724,    68,   384,   -16,   144,  -252,   116,   -12,    12,   208,  -240,   -80,  -552,   484,   208,  -792,  -612,    20,  -488,  -720,   -68,  -248,    80,  -352,  -748,  -248,  -900,   372,
  //   -24,   144,  -416,   700,   252,   276,   860,  -880,  -552,   252,  -248,  -636,  -152,    72,   340,  -400,  -600,   716,   332,   320,   -48,  -360,   480,   416,   180,  -452,  -892,  -144,   -64,   492,   400,  -184,
  //   -36,    40,   304,   412,  -196,  -684,  -320,  -204,  -732,   408,   180,   356,     0,  -412,   168,   -20,  -368,  -572,   196,     8,  -188,  -256,   560,   288,  -328,   -36,   548,   268,   116,   200,   400,   200,
  //  -388,  -756,     4,   -16,  -176,  -152,  -704,   892,  -264,  -496,   240,   308,   168,  -264,  -132,   -52,  -128,  -716,  -196,    44,   732,  -364,  -276,  -136,  -444,  -420,   156,   608,  -120,  -300,   144,   464,
  //  -624,   300,   -48,  -204,   620,   652,   484,   120,   764,   304,  -320,    80,  -288,   624,  -120,  -360,   804,   116,   128,   376,   284,  -108,   -32,   388,   684,  -504,  -876,   424,   600,  -356,    36,  -120,
  //  -368,  1408,   200, -1024,     0,   760,  1480,  -584,  1344,  1672,  -456,   128,  -304,   584,   712,   224,  1584,    -8,   104,  -224,  -576,   -72,   176,   512,   840,    88,  -488,   136,   272,  -560,  -312,   328};

   Rox_Array2D_Uint Imagval = NULL;
   Rox_Uint * Imag = NULL;
   Rox_Float * Itheta = NULL;
   Rox_Array2D_Uchar image_gray = NULL;
   Rox_Array2D_Uint image_mask = NULL;

   Rox_Sint rows = 18;
   Rox_Sint cols = 32;

   error = rox_array2d_uchar_new(&image_gray, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_set_buffer_no_stride(image_gray, image_gray_buffer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //error = rox_array2d_uchar_save_pgm("test.pgm", image_gray);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_new(&Imagval, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Imag = (Rox_Uint *) rox_memory_allocate( sizeof(*Imag), rows*cols );
   Itheta = (Rox_Float *) rox_memory_allocate( sizeof(*Itheta), rows*cols );

   memset(Imag, 0, sizeof(*Imag) * rows*cols );
   memset(Itheta, 0, sizeof(*Itheta) * rows*cols );

   error = rox_array2d_uint_new(&image_mask, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_set_ones(image_mask);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_gradientclusterer_buildgradients(Imagval, Imag, Itheta, image_gray, image_mask);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   memset(Imag, 0, sizeof(*Imag) * rows*cols );
   memset(Itheta, 0, sizeof(*Itheta) * rows*cols );

   error = rox_gradientclusterer_buildgradients(Imagval, Imag, Itheta, image_gray, image_mask);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del ( &image_gray );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_uint_del ( &Imagval );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_uint_del ( &image_mask );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   free ( Imag );
   free ( Itheta );

}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_gradientclusterer_buildgradients_perf)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Sint rows = 0;
   Rox_Sint cols = 0;

   Rox_Double time = 0.0, total_time = 0.0;

#ifdef DEBUG
   // Small number of tests for slow debug (e.g. with valgrind)
   Rox_Sint nb_tests = 1;
#else 
   // High number of tests for measuring average performance in release
   Rox_Sint nb_tests = 100;
#endif

   Rox_Array2D_Uint Imagval = NULL;
   Rox_Uint * Imag = NULL;
   Rox_Float * Itheta = NULL;
   Rox_Array2D_Uchar image_gray = NULL;
   Rox_Array2D_Uint image_mask = NULL;

   Rox_Timer timer = NULL ;

   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", TEST_IMG);
   rox_log("read file %s\n", filename);

   error = rox_array2d_uchar_new_pgm(&image_gray, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_gray);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_new(&Imagval, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Imag = (Rox_Uint *) rox_memory_allocate( sizeof(*Imag), rows*cols );
   Itheta = (Rox_Float *) rox_memory_allocate( sizeof(*Itheta), rows*cols );

   error = rox_array2d_uint_new(&image_mask, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_set_ones(image_mask);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   for(Rox_Sint i = 0; i < nb_tests; ++i)
   {
      memset(Imag, 0, sizeof(*Imag) * rows*cols );
      memset(Itheta, 0, sizeof(*Itheta) * rows*cols );

      rox_timer_start(timer);

      error = rox_gradientclusterer_buildgradients(Imagval, Imag, Itheta, image_gray, image_mask);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to build gradients of a (%d x %d) image = %f (ms)\n", cols, rows, total_time/nb_tests);

   total_time = 0;
   for(Rox_Sint i = 0; i < nb_tests; ++i)
   {
      memset(Imag, 0, sizeof(*Imag) * rows*cols );
      memset(Itheta, 0, sizeof(*Itheta) * rows*cols );

      rox_timer_start(timer);

      error = rox_gradientclusterer_buildgradients(Imagval, Imag, Itheta, image_gray, image_mask);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to build refactor gradients of a (%d x %d) image = %f (ms)\n", cols, rows, total_time/nb_tests);
   
   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del ( &image_gray );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_uint_del ( &Imagval );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_uint_del ( &image_mask );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   free ( Imag );
   free ( Itheta );

}  

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_gradientclusterer_edgecost)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_gradientclusterer_edgesort)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_gradientclusterer_computeedges)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_gradientclusterer_getrepresentative)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_gradientclusterer_connectnodes)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_gradientclusterer_cluster)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_gradientclusterer_makegroups)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_gradientclusterer_make)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_gradientclusterer_make_ac)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()

// gradient base  : 2/255
// gradient sobel : 8/255