//==============================================================================
//
//    OPENROX   : File test_znccrosscor9x9.cpp
//
//    Contents  : Tests for znccrosscor9x9.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//====== INCLUDED HEADERS   ================================================

#include <openrox_tests.hpp>

extern "C"
{
   #include <float.h>

   #include <system/time/timer.h>

   #include <baseproc/array/fill/fillval.h>
   #include <baseproc/array/conversion/array2d_float_from_uchar.h>
	 #include <baseproc/array/crosscor/zncrosscor9x9.h>

   #include <inout/image/pgm/pgmfile.h>
   #include <inout/system/print.h>
}

//====== INTERNAL MACROS    ================================================

ROX_TEST_SUITE_BEGIN(znccrosscor9x9)

#define SIZE 9

#define TEST_IMG ROX_DATA_HOME"/regression_tests/openrox/plane/model_corkes_032x032.pgm"

//====== INTERNAL TYPESDEFS ================================================

//====== INTERNAL DATATYPES ================================================

//====== INTERNAL VARIABLES ================================================

//====== INTERNAL FUNCTDEFS ================================================

//====== INTERNAL FUNCTIONS ================================================

//====== EXPORTED FUNCTIONS ================================================


ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_array2d_uchar_zncc_11x11 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double zncc = 0.0;

   Rox_Uchar Ir_data[SIZE*SIZE] = { 208,   247,   203,   101,   178,   125,    31,   179,   208,
                                    231,    41,   245,   168,    81,   114,   128,   228,    63,
                                     33,   248,   168,    44,   243,   165,   245,   245,   237,
                                    233,   245,    10,   181,     9,   181,    87,   140,    90,
                                    162,   124,   217,     9,   112,   193,   150,    36,    51,
                                     25,   205,   239,    71,    98,    71,    58,    39,    65,
                                     72,    37,   174,    12,   196,   174,   192,    66,   158,
                                    140,   108,   194,    25,   203,   168,    66,   215,   121,
                                    245,   234,   190,   210,    48,    42,   130,    65,    90 };

   Rox_Uchar Ic_data[SIZE*SIZE] = { 208,   247,   203,   101,   178,   125,    31,   179,   208,
                                    231,    41,   245,   168,    81,   114,   128,   228,    63,
                                     33,   248,   168,    44,   243,   165,   245,   245,   237,
                                    233,   245,    10,   181,     9,   181,    87,   140,    90,
                                    162,   124,   217,     9,   112,   193,   150,    36,    51,
                                     25,   205,   239,    71,    98,    71,    58,    39,    65,
                                     72,    37,   174,    12,   196,   174,   192,    66,   158,
                                    140,   108,   194,    25,   203,   168,    66,   215,   121,
                                    245,   234,   190,   210,    48,    42,   130,    65,    90 };

   const Rox_Sint cj = 4;
   const Rox_Sint ci = 4;

   Rox_Slint ref_sum = 0;
   Rox_Slint ref_sumsq = 0; 

   Rox_Array2D_Uchar Ir = NULL;
   Rox_Array2D_Uchar Ic = NULL;

   error = rox_array2d_uchar_new ( &Ir, SIZE, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new ( &Ic, SIZE, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_set_buffer_no_stride ( Ir, Ir_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_set_buffer_no_stride ( Ic, Ic_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for ( Rox_Sint i = 0; i < SIZE; i++)
   {
      for ( Rox_Sint j = 0; j < SIZE; j++)
      {
         Rox_Slint value = (Rox_Slint) Ir_data[i*SIZE+j];
         ref_sum += value;
         ref_sumsq += (value * value);
         rox_log("%ld ", value);
      }
      rox_log("\n");
   }

   rox_log("ref_sum = %ld \n", ref_sum);
   rox_log("ref_sumsq = %ld \n", ref_sumsq);

   error = rox_array2d_uchar_zncc_9x9 ( &zncc, Ir, ref_sum, ref_sumsq, Ic, cj, ci );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   rox_log("zncc = %f \n", zncc);
   ROX_TEST_CHECK_CLOSE (zncc, 1.0, 1e-12);

   error = rox_array2d_uchar_del ( &Ir );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del ( &Ic );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}


ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_array2d_uchar_zncc_9x9 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double zncc = 0.0;

   Rox_Uchar Ir_data[SIZE*SIZE] = { 208,   247,   203,   101,   178,   125,    31,   179,   208,
                                    231,    41,   245,   168,    81,   114,   128,   228,    63,
                                     33,   248,   168,    44,   243,   165,   245,   245,   237,
                                    233,   245,    10,   181,     9,   181,    87,   140,    90,
                                    162,   124,   217,     9,   112,   193,   150,    36,    51,
                                     25,   205,   239,    71,    98,    71,    58,    39,    65,
                                     72,    37,   174,    12,   196,   174,   192,    66,   158,
                                    140,   108,   194,    25,   203,   168,    66,   215,   121,
                                    245,   234,   190,   210,    48,    42,   130,    65,    90 };

   Rox_Uchar Ic_data[SIZE*SIZE] = { 208,   247,   203,   101,   178,   125,    31,   179,   208,
                                    231,    41,   245,   168,    81,   114,   128,   228,    63,
                                     33,   248,   168,    44,   243,   165,   245,   245,   237,
                                    233,   245,    10,   181,     9,   181,    87,   140,    90,
                                    162,   124,   217,     9,   112,   193,   150,    36,    51,
                                     25,   205,   239,    71,    98,    71,    58,    39,    65,
                                     72,    37,   174,    12,   196,   174,   192,    66,   158,
                                    140,   108,   194,    25,   203,   168,    66,   215,   121,
                                    245,   234,   190,   210,    48,    42,   130,    65,    90 };

   const Rox_Sint cj = 4;
   const Rox_Sint ci = 4;

   Rox_Slint ref_sum = 0;
   Rox_Slint ref_sumsq = 0; 

   Rox_Array2D_Uchar Ir = NULL;
   Rox_Array2D_Uchar Ic = NULL;

   error = rox_array2d_uchar_new ( &Ir, SIZE, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new ( &Ic, SIZE, SIZE);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_set_buffer_no_stride ( Ir, Ir_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_set_buffer_no_stride ( Ic, Ic_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for ( Rox_Sint i = 0; i < SIZE; i++)
   {
      for ( Rox_Sint j = 0; j < SIZE; j++)
      {
         Rox_Slint value = (Rox_Sint) Ir_data[i*SIZE+j];
         ref_sum += value;
         ref_sumsq += (value * value);
      }
   }

   rox_log("ref_sum = %ld \n", ref_sum);
   rox_log("ref_sumsq = %ld \n", ref_sumsq);

   error = rox_array2d_uchar_zncc_9x9 ( &zncc, Ir, ref_sum, ref_sumsq, Ic, cj, ci );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE);

   rox_log("zncc = %f \n", zncc);
   ROX_TEST_CHECK_CLOSE (zncc, 1.0, 1e-12);

   error = rox_array2d_uchar_del ( &Ir );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del ( &Ic );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}


ROX_TEST_SUITE_END()
