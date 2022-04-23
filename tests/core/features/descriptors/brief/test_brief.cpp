//==============================================================================
//
//    OPENROX   : File test_brief.cpp
//
//    Contents  : Tests for brief.c
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
   #include <limits.h>

   #include <system/time/timer.h>
   #include <generated/dynvec_brief_point.h>
   #include <generated/dynvec_segment_point.h>

   #include <baseproc/geometry/point/point2d_struct.h>
   
   #include <core/features/descriptors/brief/brief.h>
   #include <core/features/descriptors/brief/brief_point_struct.h>
   #include <core/features/detectors/segment/segmentpoint.h>
   #include <core/features/detectors/segment/segmentpoint_struct.h>
   #include <core/features/detectors/segment/fastst.h>

   #include <inout/image/pgm/pgmfile.h>
   #include <inout/image/ppm/ppmfile.h>
   #include <baseproc/image/draw/color.h>
   #include <baseproc/image/draw/draw_points.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(brief)

#define TEST_IMG_0 ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D000.pgm"
#define TEST_IMG_1 ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D010.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_brief_points_compute)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Timer timer = NULL;
   Rox_Double time = 0.0, total_time = 0.0;
   Rox_DynVec_Brief_Point dynvec_briefs_0;
   Rox_DynVec_Brief_Point dynvec_briefs_1;
   Rox_DynVec_Segment_Point dynvec_points_0;
   Rox_DynVec_Segment_Point dynvec_points_1;
   Rox_Array2D_Uchar image_0 = NULL;
   Rox_Array2D_Uchar image_1 = NULL;
   Rox_Array2D_Uint image_rgba_0 = NULL;
   Rox_Array2D_Uint image_rgba_1 = NULL;
   Rox_Char filename[FILENAME_MAX];
   Rox_Uint nb_briefs_0 = 0, nb_points_0 = 0, nb_briefs_1 = 0, nb_points_1 = 0;
   Rox_Brief_Point_Struct * brief_0 = NULL;
   Rox_Brief_Point_Struct * brief_1 = NULL;

   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_brief_point_new(&dynvec_briefs_0, 100);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_brief_point_new(&dynvec_briefs_1, 100);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_segment_point_new(&dynvec_points_0, 100);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_segment_point_new(&dynvec_points_1, 100);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", TEST_IMG_0);
   rox_log("read file %s\n", filename);

   ROX_TEST_MESSAGE("read file");
   ROX_TEST_MESSAGE(filename);

   error = rox_array2d_uchar_new_pgm(&image_0, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", TEST_IMG_1);
   error = rox_array2d_uchar_new_pgm(&image_1, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", TEST_IMG_0);

   error = rox_array2d_uint_rgba_new_pgm(&image_rgba_0, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", TEST_IMG_1);
   error = rox_array2d_uint_rgba_new_pgm(&image_rgba_1, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Compute points with FAST method
   error = rox_fastst_detector(dynvec_points_0, image_0, 20, 0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_fastst_detector(dynvec_points_1, image_1, 20, 0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Compute descriptors with BRIEF method
   error = rox_timer_start(timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_brief_points_compute(dynvec_briefs_0, image_0, dynvec_points_0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_brief_points_compute(dynvec_briefs_1, image_1, dynvec_points_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_stop(timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_get_elapsed_ms(&time, timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   total_time += time;
   rox_log("total_time: %.4f\n", total_time);

   // Get the number of BRIEF descriptors
   error = rox_dynvec_brief_point_get_used(&nb_briefs_0, dynvec_briefs_0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_brief_point_get_used(&nb_briefs_1, dynvec_briefs_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Get the number of FAST points
   error = rox_dynvec_segment_point_get_used(&nb_points_0, dynvec_points_0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_segment_point_get_used(&nb_points_1, dynvec_points_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_brief_point_get_data_pointer ( &brief_0, dynvec_briefs_0 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_brief_point_get_data_pointer ( &brief_1, dynvec_briefs_1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("nb_briefs_0: %d nb_points_0: %d \n", nb_briefs_0, nb_points_0);
   rox_log("nb_briefs_1: %d nb_points_1: %d \n", nb_briefs_1, nb_points_1);

    ROX_TEST_CHECK_EQUAL(nb_briefs_0, 2782u);
    ROX_TEST_CHECK_EQUAL(nb_briefs_1, 2718u);

    //TODO TODO problem mismatch results between Linux and Windows
    ROX_TEST_MESSAGE("TODO problem mismatch results between Linux and Windows");
    //ROX_TEST_CHECK_EQUAL(nb_points_0, 2925);
    //ROX_TEST_CHECK_EQUAL(nb_points_1, 2836);

   for (Rox_Uint i = 0; i < nb_briefs_0; ++i)
   {
      Rox_Point2D_Double_Struct pt_brief;

      pt_brief.u = brief_0[i].u;
      pt_brief.v = brief_0[i].v;

      error = rox_image_rgba_draw_2d_points(image_rgba_0, &pt_brief, 1, ROX_MAKERGBA(255,0,0,255));
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }


   for (Rox_Uint i = 0; i < nb_briefs_1; ++i)
   {
      Rox_Point2D_Double_Struct pt_brief;

      pt_brief.u = brief_1[i].u;
      pt_brief.v = brief_1[i].v;

      error = rox_image_rgba_draw_2d_points(image_rgba_1, &pt_brief, 1, ROX_MAKERGBA(255,0,0,255));
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }

   Rox_Sint selected_point_0 = 1368;
   Rox_Sint selected_point_1 = -1;

   {
      Rox_Point2D_Double_Struct pt_brief_0;

      pt_brief_0.u = brief_0[selected_point_0].u;
      pt_brief_0.v = brief_0[selected_point_0].v;

      error = rox_image_rgba_draw_2d_points(image_rgba_0, &pt_brief_0, 1, ROX_MAKERGBA(0,255,0,255));
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }

   // Search selected point into image 1
   Rox_Sint best_score = INT_MAX;
   for (Rox_Uint i = 0; i < nb_briefs_1; ++i)
   {
      Rox_Sint score = 0;
      for (Rox_Uint k = 0; k < 32; ++k)
      {
         score += (brief_1[i].description[k]-brief_0[selected_point_0].description[k])*(brief_1[i].description[k]-brief_0[selected_point_0].description[k]);
      }
      if (score < best_score)
      {
         best_score = score;
         selected_point_1 = i;
      }
   }
   ROX_TEST_CHECK_EQUAL(selected_point_1, 1338);

   {
      Rox_Point2D_Double_Struct pt_brief_1;

      pt_brief_1.u = brief_1[selected_point_1].u;
      pt_brief_1.v = brief_1[selected_point_1].v;

      error = rox_image_rgba_draw_2d_points(image_rgba_1, &pt_brief_1, 1, ROX_MAKERGBA(0,255,0,255));
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }

   //error = rox_image_rgba_save_ppm("./test_brief_0.ppm", image_rgba_0);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //error = rox_image_rgba_save_ppm("./test_brief_1.ppm", image_rgba_1);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // reset data
   error = rox_dynvec_brief_point_reset(dynvec_briefs_0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_brief_point_reset(dynvec_briefs_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_brief_point_del(&dynvec_briefs_0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_brief_point_del(&dynvec_briefs_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_segment_point_del(&dynvec_points_0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_segment_point_del(&dynvec_points_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&image_rgba_0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&image_rgba_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}

ROX_TEST_SUITE_END()
