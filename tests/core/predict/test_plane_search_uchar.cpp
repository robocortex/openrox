//==============================================================================
//
//    OPENROX   : File test_plane_search_uchar.cpp
//
//    Contents  : Tests for plane_search_uchar.c
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
   #include <baseproc/array/fill/fillval.h>
   #include <baseproc/maths/linalg/matsl3.h>
   #include <baseproc/maths/linalg/matso3.h>
   #include <baseproc/maths/linalg/matse3.h>
   #include <baseproc/geometry/transforms/transform_tools.h>

   #include <core/predict/plane_search_uchar.h>
   
   #include <inout/image/pgm/pgmfile.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(plane_search)

#define TEST_IMG ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D000.pgm"

#define IMAGE_REF TEST_IMG
#define IMAGE_CUR TEST_IMG

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_plane_search_uchar_new_del)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Plane_Search_Uchar plane_search_uchar = NULL;
   Rox_Sint model_rows = 64;
   Rox_Sint model_cols  = 64;
   Rox_Sint search_radius = 32;

   error = rox_plane_search_uchar_new ( &plane_search_uchar, model_rows, model_cols, search_radius );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_uchar_del ( &plane_search_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_plane_search_uchar_set_model)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Plane_Search_Uchar plane_search_uchar = NULL;
   Rox_Array2D_Uchar image_uchar = NULL;
   Rox_Array2D_Uchar image_model = NULL;
   Rox_Array2D_Uint  imask_model = NULL;
   Rox_Sint cols = 0, rows = 0;
   Rox_Sint cols_model = 80, rows_model = 40;
   // Displacement of the template relative to the reference image
   Rox_Sint tu_model = 220, tv_model = 280;
   Rox_Sint search_radius = 64;

   sprintf(filename, "%s", TEST_IMG);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define image model
   error = rox_array2d_uchar_new_subarray2d(&image_model, image_uchar, tv_model, tu_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define imask model
   error = rox_array2d_uint_new(&imask_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(imask_model, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define the new search
   error = rox_plane_search_uchar_new(&plane_search_uchar, rows_model, cols_model, search_radius);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_uchar_set_model(plane_search_uchar, image_model, imask_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_uchar_del(&plane_search_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&imask_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_plane_search_uchar_make)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Plane_Search_Uchar plane_search_uchar = NULL;
   Rox_Array2D_Uchar image_uchar = NULL;
   Rox_Array2D_Uchar image_model = NULL;
   Rox_Array2D_Uint  imask_model = NULL;
   Rox_Sint cols = 0, rows = 0;
   Rox_Sint cols_model = 80, rows_model = 40;
   // Displacement of the template relative to the reference image
   Rox_Sint tu_model = 220, tv_model = 280;
   Rox_Sint search_radius = 76;
   Rox_MatSL3 c_H_t = NULL;

   sprintf(filename, "%s", TEST_IMG);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define imask model
   error = rox_imask_new ( &imask_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(imask_model, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define the new search
   error = rox_plane_search_uchar_new(&plane_search_uchar, rows_model, cols_model, search_radius);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   if(0)
   {
      // If we set the model with this function the homography r_G_t is set to the identity I
      error = rox_plane_search_uchar_set_model(plane_search_uchar, image_model, imask_model);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_array2d_uchar_del(&image_model);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_array2d_uint_del(&imask_model);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }
   else
   {
      rox_log("new model definition\n");

      Rox_MatSL3 r_H_t = NULL;

      error = rox_matsl3_new(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_transformtools_homography_shift(r_H_t, tu_model, tv_model);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_plane_search_uchar_set_model_warp(plane_search_uchar, image_uchar, r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_matsl3_del(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }

   // Define estimated homography c_H_t to warp the current image into the template
   error = rox_matsl3_new(&c_H_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_homography_shift(c_H_t, tu_model, tv_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_uchar_make(plane_search_uchar, image_uchar, c_H_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_uchar_del(&plane_search_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_del ( &imask_model );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&c_H_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_plane_search_uchar_get_results)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Plane_Search_Uchar plane_search_uchar = NULL;
   Rox_Array2D_Uchar image_uchar = NULL;
   Rox_Array2D_Uchar image_model = NULL;
   Rox_Array2D_Uint  imask_model = NULL;
   Rox_Sint cols = 0, rows = 0;
   Rox_Sint cols_model = 80, rows_model = 40;
   // Displacement of the template relative to the reference image
   Rox_Sint tu_model = 220, tv_model = 280;
   Rox_Sint search_radius = 64;
   Rox_MatSL3 c_H_t = NULL;

   sprintf(filename, "%s", TEST_IMG);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define image model
   error = rox_array2d_uchar_new_subarray2d(&image_model, image_uchar, tv_model, tu_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define imask model
   error = rox_array2d_uint_new(&imask_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(imask_model, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define the new search
   error = rox_plane_search_uchar_new(&plane_search_uchar, rows_model, cols_model, search_radius);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   if(0)
   {
      // If we set the model with this function the homography r_G_t is set to the identity I
      error = rox_plane_search_uchar_set_model(plane_search_uchar, image_model, imask_model);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }
   else
   {
      rox_log("new model definition\n");

      Rox_MatSL3 r_H_t = NULL;

      error = rox_matsl3_new(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_transformtools_homography_shift(r_H_t, tu_model, tv_model);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_plane_search_uchar_set_model_warp(plane_search_uchar, image_uchar, r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_matsl3_del(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }

   error = rox_matsl3_new(&c_H_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_homography_shift(c_H_t, tu_model-1, tv_model-2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_uchar_make(plane_search_uchar, image_uchar, c_H_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double score = 0.0, tu = 0.0, tv = 0.0;

   error = rox_plane_search_uchar_get_results(&score, &tu, &tv, plane_search_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("score = %f\n", score);
   rox_log("tu = %f\n", tu);
   rox_log("tv = %f\n", tv);

   error = rox_plane_search_uchar_del(&plane_search_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&imask_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&c_H_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_plane_search_uchar_get_shift)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Plane_Search_Uchar plane_search_uchar = NULL;
   Rox_Array2D_Uchar image_uchar = NULL;
   Rox_Array2D_Uchar image_model = NULL;
   Rox_Array2D_Uint  imask_model = NULL;
   Rox_Sint cols = 0, rows = 0;
   Rox_Sint cols_model = 80, rows_model = 40;
   Rox_Sint tu_model = 220, tv_model = 280;
   Rox_Sint search_radius = 64;
   Rox_MatSL3 c_H_t = NULL;

   sprintf(filename, "%s", TEST_IMG);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define image model
   error = rox_array2d_uchar_new_subarray2d(&image_model, image_uchar, tv_model, tu_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define imask model
   error = rox_array2d_uint_new(&imask_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(imask_model, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define the new search
   error = rox_plane_search_uchar_new(&plane_search_uchar, rows_model, cols_model, search_radius);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   if(0)
   {
      // If we set the model with this function the homography r_G_t is set to the identity I
      error = rox_plane_search_uchar_set_model(plane_search_uchar, image_model, imask_model);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }
   else
   {
      rox_log("new model definition\n");

      Rox_MatSL3 r_H_t = NULL;

      error = rox_matsl3_new(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_transformtools_homography_shift(r_H_t, tu_model, tv_model);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_plane_search_uchar_set_model_warp(plane_search_uchar, image_uchar, r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_matsl3_del(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }

   error = rox_matsl3_new(&c_H_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_homography_shift(c_H_t, tu_model, tv_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_uchar_make(plane_search_uchar, image_uchar, c_H_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double tu = 0.0, tv = 0.0;

   error = rox_plane_search_uchar_get_shift(&tu, &tv, plane_search_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("tu = %f\n", tu);
   rox_log("tv = %f\n", tv);

   error = rox_plane_search_uchar_del(&plane_search_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&imask_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&c_H_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_plane_search_uchar_get_score)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Plane_Search_Uchar plane_search_uchar = NULL;
   Rox_Array2D_Uchar image_uchar = NULL;
   Rox_Array2D_Uchar image_model = NULL;
   Rox_Array2D_Uint  imask_model = NULL;
   Rox_Sint cols = 0, rows = 0;
   Rox_Sint cols_model = 80, rows_model = 40;
   Rox_Sint tu_model = 220, tv_model = 280;
   Rox_Sint search_radius = 64;
   Rox_MatSL3 c_H_t = NULL;

   sprintf(filename, "%s", TEST_IMG);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define image model
   error = rox_array2d_uchar_new_subarray2d(&image_model, image_uchar, tv_model, tu_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define imask model
   error = rox_array2d_uint_new(&imask_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(imask_model, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define the new search
   error = rox_plane_search_uchar_new(&plane_search_uchar, rows_model, cols_model, search_radius);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   if(0)
   {
      // If we set the model with this function the homography r_G_t is set to the identity I
      error = rox_plane_search_uchar_set_model(plane_search_uchar, image_model, imask_model);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }
   else
   {
      rox_log("new model definition\n");

      Rox_MatSL3 r_H_t = NULL;

      error = rox_matsl3_new(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_transformtools_homography_shift(r_H_t, tu_model, tv_model);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_plane_search_uchar_set_model_warp(plane_search_uchar, image_uchar, r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_matsl3_del(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }

   error = rox_matsl3_new(&c_H_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_homography_shift(c_H_t, tu_model, tv_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_uchar_make(plane_search_uchar, image_uchar, c_H_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double score = 0.0;

   error = rox_plane_search_uchar_get_score(&score, plane_search_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("score = %f\n", score);

   error = rox_plane_search_uchar_del(&plane_search_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&imask_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&c_H_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

// -------------------------------------------------------------------------------------------------------------------

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_plane_search_uchar_update_homography)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Plane_Search_Uchar plane_search_uchar = NULL;
   Rox_Array2D_Uchar image_ref_uchar = NULL;

   Rox_Array2D_Uchar image_cur_uchar = NULL;

   Rox_Array2D_Uchar image_model = NULL;
   Rox_Array2D_Uint  imask_model = NULL;
   Rox_Sint cols = 0, rows = 0;
   Rox_Sint cols_model = 80, rows_model = 40;
   Rox_Sint tu_model = 220, tv_model = 280;
   // Rox_Sint tu_init = 230, tv_init = 260;
   Rox_Sint search_radius = 64;
   Rox_MatSL3 c_H_t = NULL;
   Rox_MatSL3 homography = NULL;

   sprintf(filename, "%s", IMAGE_REF);
   //sprintf(filename, "%s", IMAGE_REF);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_ref_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_ref_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_REF);
   //sprintf(filename, "%s", IMAGE_CUR);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_cur_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_cur_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define image model (a part of the reference image)
   error = rox_array2d_uchar_new_subarray2d(&image_model, image_ref_uchar, tv_model, tu_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define imask model
   error = rox_array2d_uint_new(&imask_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(imask_model, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define the new search
   error = rox_plane_search_uchar_new(&plane_search_uchar, rows_model, cols_model, search_radius);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   if(0)
   {
      // If we set the model with this function the homography r_G_t is set to the identity I
      error = rox_plane_search_uchar_set_model(plane_search_uchar, image_model, imask_model);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }
   else
   {
      rox_log("new model definition\n");

      Rox_MatSL3 r_H_t = NULL;

      error = rox_matsl3_new(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_transformtools_homography_shift(r_H_t, tu_model, tv_model);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      rox_matsl3_print(r_H_t);

      error = rox_plane_search_uchar_set_model_warp(plane_search_uchar, image_ref_uchar, r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_matsl3_del(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }

   error = rox_matsl3_new(&c_H_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_homography_shift(c_H_t, tu_model, tv_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_uchar_make(plane_search_uchar, image_cur_uchar, c_H_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double score = 0.0, tu = 0.0, tv = 0.0;

   error = rox_plane_search_uchar_get_results(&score, &tu, &tv, plane_search_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("score = %f\n", score);
   rox_log("tu = %f\n", tu);
   rox_log("tv = %f\n", tv);

   //ROX_TEST_CHECK_EQUAL(tu, tu_model-tu_init);
   //ROX_TEST_CHECK_EQUAL(tv, tv_model-tv_init);

   error = rox_matsl3_new(&homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_uchar_update_homography(homography, plane_search_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_print(homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_uchar_del(&plane_search_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_ref_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_cur_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&imask_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&c_H_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_plane_search_update_pose_translation)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Plane_Search_Uchar plane_search_uchar = NULL;
   Rox_Array2D_Uchar image_ref_uchar = NULL;
   Rox_Array2D_Uchar image_cur_uchar = NULL;
   Rox_Array2D_Uchar image_model = NULL;
   Rox_Array2D_Uint  imask_model = NULL;
   Rox_Sint cols = 0, rows = 0;
   Rox_Sint cols_model = 80, rows_model = 40;
   Rox_Sint tu_model = 220, tv_model = 280;
   // Rox_Sint tu_init = 230, tv_init = 260;
   Rox_Sint search_radius = 64;
   Rox_MatSE3 pose = NULL;
   Rox_MatSL3 c_H_t = NULL;
   Rox_Array2D_Double intrinsics = NULL;

   sprintf(filename, "%s", TEST_IMG);
   // sprintf(filename, "%s", IMAGE_REF);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_ref_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_ref_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", TEST_IMG);
   //sprintf(filename, "%s", IMAGE_CUR);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_cur_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_cur_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define image model
   error = rox_array2d_uchar_new_subarray2d(&image_model, image_ref_uchar, tv_model, tu_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define imask model
   error = rox_array2d_uint_new(&imask_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(imask_model, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define the new search
   error = rox_plane_search_uchar_new(&plane_search_uchar, rows_model, cols_model, search_radius);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   if(0)
   {
      // If we set the model with this function the homography r_G_t is set to the identity I
      error = rox_plane_search_uchar_set_model(plane_search_uchar, image_model, imask_model);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }
   else
   {
      rox_log("new model definition\n");

      Rox_MatSL3 r_H_t = NULL;

      error = rox_matsl3_new(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_transformtools_homography_shift(r_H_t, tu_model, tv_model);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_plane_search_uchar_set_model_warp(plane_search_uchar, image_ref_uchar, r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_matsl3_del(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }

   error = rox_matsl3_new(&c_H_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_homography_shift(c_H_t, tu_model, tv_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_uchar_make(plane_search_uchar, image_cur_uchar, c_H_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double score = 0.0, tu = 0.0, tv = 0.0;

   error = rox_plane_search_uchar_get_results(&score, &tu, &tv, plane_search_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("score = %f\n", score);
   rox_log("tu = %f\n", tu);
   rox_log("tv = %f\n", tv);

   //ROX_TEST_CHECK_EQUAL(tu, -37);
   //ROX_TEST_CHECK_EQUAL(tv, -53);

   error = rox_matse3_new(&pose);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&intrinsics, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_calibration_matrix(intrinsics, 500.0, 500.0, cols/2, rows/2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Update pose translation
   error = rox_plane_search_uchar_update_pose_translation(pose, intrinsics, plane_search_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_print(pose);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del(&pose);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_uchar_del(&plane_search_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_ref_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_cur_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&imask_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&c_H_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&intrinsics);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_plane_search_uchar_update_pose_rotation)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Plane_Search_Uchar plane_search_uchar = NULL;
   Rox_Array2D_Uchar image_ref_uchar = NULL;
   Rox_Array2D_Uchar image_cur_uchar = NULL;

   Rox_Array2D_Uchar image_model = NULL;
   Rox_Array2D_Uint  imask_model = NULL;
   Rox_Sint cols = 0, rows = 0;
   Rox_Sint cols_model = 80, rows_model = 40;
   Rox_Sint tu_model = 220, tv_model = 280;
   // Rox_Sint tu_init = 230, tv_init = 260;
   Rox_Sint search_radius = 64;
   Rox_MatSO3 rotation = NULL;
   Rox_MatSL3 c_H_t = NULL;
   Rox_Array2D_Double intrinsics = NULL;

   sprintf(filename, "%s", TEST_IMG);
   //sprintf(filename, "%s", IMAGE_REF);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_ref_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_ref_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", TEST_IMG);
   // sprintf(filename, "%s", IMAGE_CUR);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_cur_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_cur_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define image model
   error = rox_array2d_uchar_new_subarray2d(&image_model, image_ref_uchar, tv_model, tu_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define imask model
   error = rox_array2d_uint_new(&imask_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(imask_model, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define the new search
   error = rox_plane_search_uchar_new(&plane_search_uchar, rows_model, cols_model, search_radius);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   if(0)
   {
      // If we set the model with this function the homography r_G_t is set to the identity I
      error = rox_plane_search_uchar_set_model(plane_search_uchar, image_model, imask_model);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }
   else
   {
      rox_log("new model definition\n");

      Rox_MatSL3 r_H_t = NULL;

      error = rox_matsl3_new(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_transformtools_homography_shift(r_H_t, tu_model, tv_model);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_plane_search_uchar_set_model_warp(plane_search_uchar, image_ref_uchar, r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_matsl3_del(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }

   error = rox_matsl3_new(&c_H_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_homography_shift(c_H_t, tu_model, tv_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_uchar_make(plane_search_uchar, image_cur_uchar, c_H_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double score = 0.0, tu = 0.0, tv = 0.0;

   error = rox_plane_search_uchar_get_results(&score, &tu, &tv, plane_search_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("score = %f\n", score);
   rox_log("tu = %f\n", tu);
   rox_log("tv = %f\n", tv);

   //ROX_TEST_CHECK_EQUAL(tu, -37);
   //ROX_TEST_CHECK_EQUAL(tv, -53);

   error = rox_matso3_new(&rotation);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&intrinsics, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_calibration_matrix(intrinsics, 500.0, 500.0, cols/2, rows/2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Update pose rotation
   error = rox_plane_search_uchar_update_pose_rotation(rotation, intrinsics, plane_search_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matso3_print(rotation);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Delete

   error = rox_matso3_del(&rotation);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_uchar_del(&plane_search_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_ref_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_cur_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&imask_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&c_H_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&intrinsics);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
