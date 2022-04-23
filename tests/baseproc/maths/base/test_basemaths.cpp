//==============================================================================
//
//    OPENROX   : File test_basemaths.cpp
//
//    Contents  : Tests for basemaths.c
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
   #include <baseproc/maths/base/basemaths.h>
   #include <system/memory/datatypes.h>
   #include <system/errors/errors.h>
   #include <inout/system/print.h>   
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN ( basemaths )

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_angle_isinrange )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint inrange = 0;
   Rox_Double angle_range = 0.436332312999;

   // ======================================

   Rox_Double angle_model = ROX_PI; 
   Rox_Double angle_measure = -ROX_PI; 

   error = rox_angle_isinrange ( &inrange, angle_model, angle_measure, angle_range );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_CHECK_EQUAL (  inrange, 1);

   // ======================================

   angle_model = -ROX_PI; 
   angle_measure = ROX_PI; 

   error = rox_angle_isinrange ( &inrange, angle_model, angle_measure, angle_range );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_CHECK_EQUAL (  inrange, 1);

   // ======================================

   angle_model = 0; 
   angle_measure = ROX_PI; 

   error = rox_angle_isinrange ( &inrange, angle_model, angle_measure, angle_range );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_CHECK_EQUAL (  inrange, 1);

   // ======================================

   angle_model = 0; 
   angle_measure = -ROX_PI; 

   error = rox_angle_isinrange ( &inrange, angle_model, angle_measure, angle_range );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_CHECK_EQUAL (  inrange, 1);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_popcount_slow)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_convert)
{
   // Rox_ErrorCode error = ROX_ERROR_NONE;

   // The result of this test show that the max difference between float and uint cast is 1 greylevel

   Rox_Uint d = 0, maxd = 0;
   unsigned long int totd = 0;

   for (Rox_Uint r = 0; r < 256; ++r)
   {
      for (Rox_Uint g = 0; g < 256; ++g)
      {
         for (Rox_Uint b = 0; b < 256; ++b)
         {
            Rox_Uchar r_uchar = (Rox_Uchar) r;
            Rox_Uchar g_uchar = (Rox_Uchar) g;
            Rox_Uchar b_uchar = (Rox_Uchar) b;

            Rox_Uint r_uint =  54 * r;
            Rox_Uint g_uint = 183 * g;
            Rox_Uint b_uint =  18 * b;

            Rox_Uchar I_char_from_float = (Rox_Uchar) ((r_uchar * 0.212671f) + (g_uchar * 0.715160f) + (b_uchar * 0.072169f)) ;

            Rox_Uchar I_char_from_uint = (Rox_Uchar) ((r_uint + g_uint + b_uint) >> 8);

            d = abs(I_char_from_float - I_char_from_uint);
            if(d > maxd) maxd = d;

            if(d>1)
            {
               rox_log("d = %d\n",d);
               rox_log("max diff = %d \n", maxd);
               rox_log("[I_char_from_float, Rox_Char I_char_from_uint] = [%d, %d]\n", I_char_from_float, I_char_from_uint);
            }
            totd += (unsigned long int) d;
         }
      }
   }

   rox_log("sum of values = %lu \n", totd);
   rox_log("max diff = %d \n", maxd);
   
   ROX_TEST_CHECK_EQUAL(maxd, 1u);
   ROX_TEST_CHECK_EQUAL(totd, 8323216u);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_fast_atan2)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double time = 0.0, total_time = 0.0;
   Rox_Timer timer = NULL;
   Rox_Sint nbtests = 10000000;

   Rox_Double a  = 0.0;
   // Rox_Double av = 0.0;
   
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_timer_start(timer);

   for (Rox_Sint i = 0; i < nbtests; i++)
   {  
      Rox_Double x = rox_rand();
      Rox_Double y = rox_rand();

      // fast atan2 is 10 time faster than atan2
      a = fast_atan2( y, x );
      ROX_UNUSED(a);
      //av = atan2( y, x );
   }
      
   rox_timer_stop(timer);
   rox_timer_get_elapsed_ms(&time, timer);
   total_time += time;

   rox_log("total time to compute fast_atan2 = %f (ms)\n", total_time);
   rox_log("mean time to compute fast_atan2 = %f (ms)\n", total_time/nbtests);

   rox_log("fast_atan2(0, 0) = %f \n", fast_atan2( 0, 0 ));
   rox_log("atan2(0, 0) = %f \n", atan2( 0, 0 ));

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_fast_atan2f2)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double time = 0.0, total_time = 0.0;
   Rox_Timer timer = NULL;
   Rox_Sint nbtests = 10000000;

   Rox_Double a  = 0.0;
   // Rox_Double av = 0.0;
   
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_timer_start(timer);

   for (Rox_Sint i = 0; i < nbtests; i++)
   {  
      Rox_Double x = rox_rand();
      Rox_Double y = rox_rand();

      // fast atan2 is 10 time faster than atan2
      a = fast_atan2f2( y, x );
      ROX_UNUSED(a);
      //av = atan2( y, x );
   }
      
   rox_timer_stop(timer);
   rox_timer_get_elapsed_ms(&time, timer);
   total_time += time;

   rox_log("total time to compute fast_atan2f2 = %f (ms)\n", total_time);
   rox_log("mean time to compute fast_atan2f2 = %f (ms)\n", total_time/nbtests);

   rox_log("fast_atan2f2(0, 0) = %f \n", fast_atan2f2( 0, 0 ));
   rox_log("atan2(0, 0) = %f \n", atan2( 0, 0 ));

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
