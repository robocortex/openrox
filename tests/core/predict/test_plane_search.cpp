//==============================================================================
//
//    OPENROX   : File test_plane_search.cpp
//
//    Contents  : Tests for plane_search.c
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
   #include <baseproc/array/rotate/rotate90.h>
   #include <baseproc/maths/linalg/matsl3.h>
   #include <baseproc/maths/linalg/matso3.h>
   #include <baseproc/maths/linalg/matse3.h>
   #include <baseproc/geometry/transforms/transform_tools.h>
   #include <baseproc/array/conversion/array2d_float_from_uchar.h>

   #include <core/predict/plane_search.h>
   
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

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_plane_search_new_del)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Plane_Search plane_search = NULL;
   Rox_Sint model_rows = 64;
   Rox_Sint model_cols  = 64;
   Rox_Sint search_radius = 32;

   error = rox_plane_search_new(&plane_search, model_rows, model_cols, search_radius);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_del(&plane_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_plane_search_set_model)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Plane_Search plane_search = NULL;
   Rox_Array2D_Uchar image_uchar = NULL;
   Rox_Array2D_Float image_float = NULL;
   Rox_Array2D_Float image_model = NULL;
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
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_uchar_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Create the float image
   error = rox_array2d_float_new(&image_float, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Convert image to float
   error = rox_array2d_float_from_uchar_normalize(image_float, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Define image model
   error = rox_array2d_float_new_subarray2d(&image_model, image_float, tv_model, tu_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Define imask model
   error = rox_array2d_uint_new(&imask_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_uint_fillval(imask_model, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Define the new search
   error = rox_plane_search_new(&plane_search, rows_model, cols_model, search_radius);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_plane_search_set_model(plane_search, image_model, imask_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

function_terminate:

   error = rox_plane_search_del(&plane_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_uchar_del(&image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_float_del(&image_float);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_float_del(&image_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_uint_del(&imask_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_plane_search_make)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Plane_Search plane_search = NULL;
   Rox_Array2D_Uchar image_uchar = NULL;
   Rox_Array2D_Float image_float = NULL;
   Rox_Array2D_Float image_model = NULL;
   Rox_Array2D_Uint  imask_model = NULL;
   Rox_Sint cols = 0, rows = 0;
   Rox_Sint cols_model = 80, rows_model = 40;
   // Displacement of the template relative to the reference image
   Rox_Sint tu_model = 220, tv_model = 280;
   Rox_Sint search_radius = 76;
   Rox_MatSL3 c_G_t = NULL;

   sprintf(filename, "%s", TEST_IMG);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_uchar_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Create the float image
   error = rox_array2d_float_new(&image_float, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Convert image to float
   error = rox_array2d_float_from_uchar_normalize(image_float, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Define image model
   error = rox_array2d_float_new_subarray2d(&image_model, image_float, tv_model, tu_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Define imask model
   error = rox_array2d_uint_new(&imask_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_uint_fillval(imask_model, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Define the new search
   error = rox_plane_search_new(&plane_search, rows_model, cols_model, search_radius);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   if(0)
   {
      // If we set the model with this function the homography r_G_t is set to the identity I
      error = rox_plane_search_set_model(plane_search, image_model, imask_model);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;
   }
   else
   {
      rox_log("new model definition\n");

      Rox_MatSL3 r_H_t = NULL;

      error = rox_matsl3_new(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      error = rox_transformtools_homography_shift(r_H_t, tu_model, tv_model);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      error = rox_plane_search_set_model_warp(plane_search, image_float, r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

      error = rox_matsl3_del(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;
   }

   // Define estimated homography c_G_t to warp the current image into the template
   error = rox_matsl3_new(&c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_transformtools_homography_shift(c_G_t, tu_model, tv_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_plane_search_make(plane_search, image_float, c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

function_terminate:

   error = rox_plane_search_del(&plane_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_uchar_del(&image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_float_del(&image_float);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_float_del(&image_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_uint_del(&imask_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_matsl3_del(&c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_plane_search_get_results)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Plane_Search plane_search = NULL;
   Rox_Array2D_Uchar image_uchar = NULL;
   Rox_Array2D_Float image_float = NULL;
   Rox_Array2D_Float image_model = NULL;
   Rox_Array2D_Uint  imask_model = NULL;
   Rox_Sint cols = 0, rows = 0;
   Rox_Sint cols_model = 80, rows_model = 40;
   // Displacement of the template relative to the reference image
   Rox_Sint tu_model = 220, tv_model = 280;
   Rox_Sint search_radius = 64;
   Rox_MatSL3 c_G_t = NULL;

   sprintf(filename, "%s", TEST_IMG);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the float image
   error = rox_array2d_float_new(&image_float, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Convert image to float
   error = rox_array2d_float_from_uchar_normalize(image_float, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define image model
   error = rox_array2d_float_new_subarray2d(&image_model, image_float, tv_model, tu_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define imask model
   error = rox_array2d_uint_new(&imask_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(imask_model, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define the new search
   error = rox_plane_search_new(&plane_search, rows_model, cols_model, search_radius);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   if(0)
   {
      // If we set the model with this function the homography r_G_t is set to the identity I
      error = rox_plane_search_set_model(plane_search, image_model, imask_model);
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

      error = rox_plane_search_set_model_warp(plane_search, image_float, r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_matsl3_del(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }

   error = rox_matsl3_new(&c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_homography_shift(c_G_t, tu_model-1, tv_model-2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_make(plane_search, image_float, c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double score = 0.0, tu = 0.0, tv = 0.0;

   error = rox_plane_search_get_results(&score, &tu, &tv, plane_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("score = %f\n", score);
   rox_log("tu = %f\n", tu);
   rox_log("tv = %f\n", tv);

   error = rox_plane_search_del(&plane_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_float);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&imask_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_plane_search_get_shift)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Plane_Search plane_search = NULL;
   Rox_Array2D_Uchar image_uchar = NULL;
   Rox_Array2D_Float image_float = NULL;
   Rox_Array2D_Float image_model = NULL;
   Rox_Array2D_Uint  imask_model = NULL;
   Rox_Sint cols = 0, rows = 0;
   Rox_Sint cols_model = 80, rows_model = 40;
   Rox_Sint tu_model = 220, tv_model = 280;
   Rox_Sint search_radius = 64;
   Rox_MatSL3 c_G_t = NULL;

   sprintf(filename, "%s", TEST_IMG);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the float image
   error = rox_array2d_float_new(&image_float, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Convert image to float
   error = rox_array2d_float_from_uchar_normalize(image_float, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define image model
   error = rox_array2d_float_new_subarray2d(&image_model, image_float, tv_model, tu_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define imask model
   error = rox_array2d_uint_new(&imask_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(imask_model, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define the new search
   error = rox_plane_search_new(&plane_search, rows_model, cols_model, search_radius);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   if(0)
   {
      // If we set the model with this function the homography r_G_t is set to the identity I
      error = rox_plane_search_set_model(plane_search, image_model, imask_model);
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

      error = rox_plane_search_set_model_warp(plane_search, image_float, r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_matsl3_del(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }

   error = rox_matsl3_new(&c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_homography_shift(c_G_t, tu_model, tv_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_make(plane_search, image_float, c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double tu = 0.0, tv = 0.0;

   error = rox_plane_search_get_shift(&tu, &tv, plane_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("tu = %f\n", tu);
   rox_log("tv = %f\n", tv);

   error = rox_plane_search_del(&plane_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_float);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&imask_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_plane_search_get_score)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Plane_Search plane_search = NULL;
   Rox_Array2D_Uchar image_uchar = NULL;
   Rox_Array2D_Float image_float = NULL;
   Rox_Array2D_Float image_model = NULL;
   Rox_Array2D_Uint  imask_model = NULL;
   Rox_Sint cols = 0, rows = 0;
   Rox_Sint cols_model = 80, rows_model = 40;
   Rox_Sint tu_model = 220, tv_model = 280;
   Rox_Sint search_radius = 64;
   Rox_MatSL3 c_G_t = NULL;

   sprintf(filename, "%s", TEST_IMG);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the float image
   error = rox_array2d_float_new(&image_float, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Convert image to float
   error = rox_array2d_float_from_uchar_normalize(image_float, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define image model
   error = rox_array2d_float_new_subarray2d(&image_model, image_float, tv_model, tu_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define imask model
   error = rox_array2d_uint_new(&imask_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(imask_model, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define the new search
   error = rox_plane_search_new(&plane_search, rows_model, cols_model, search_radius);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   if(0)
   {
      // If we set the model with this function the homography r_G_t is set to the identity I
      error = rox_plane_search_set_model(plane_search, image_model, imask_model);
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

      error = rox_plane_search_set_model_warp(plane_search, image_float, r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_matsl3_del(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }

   error = rox_matsl3_new(&c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_homography_shift(c_G_t, tu_model, tv_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_make(plane_search, image_float, c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double score = 0.0;

   error = rox_plane_search_get_score(&score, plane_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("score = %f\n", score);

   error = rox_plane_search_del(&plane_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_float);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&imask_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

// -------------------------------------------------------------------------------------------------------------------

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_plane_search_update_homography)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Plane_Search plane_search = NULL;
   Rox_Array2D_Uchar image_ref_uchar = NULL;
   Rox_Array2D_Float image_ref_float = NULL;

   Rox_Array2D_Uchar image_cur_uchar = NULL;
   Rox_Array2D_Float image_cur_float = NULL;

   Rox_Array2D_Float image_model = NULL;
   Rox_Array2D_Uint  imask_model = NULL;
   Rox_Sint cols = 0, rows = 0;
   Rox_Sint cols_model = 80, rows_model = 40;
   Rox_Sint tu_model = 220, tv_model = 280;
   // Rox_Sint tu_init = 230, tv_init = 260;
   Rox_Sint search_radius = 64;
   Rox_MatSL3 c_G_t = NULL;
   Rox_MatSL3 homography = NULL;

   sprintf(filename, "%s", IMAGE_REF);
   //sprintf(filename, "%s", IMAGE_REF);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_ref_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_ref_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the float image
   error = rox_array2d_float_new(&image_ref_float, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Convert image to float
   error = rox_array2d_float_from_uchar_normalize(image_ref_float, image_ref_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_REF);
   //sprintf(filename, "%s", IMAGE_CUR);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_cur_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_cur_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the float image
   error = rox_array2d_float_new(&image_cur_float, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Convert image to float
   error = rox_array2d_float_from_uchar_normalize(image_cur_float, image_cur_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );


   // Define image model (a part of the reference image)
   error = rox_array2d_float_new_subarray2d(&image_model, image_ref_float, tv_model, tu_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define imask model
   error = rox_array2d_uint_new(&imask_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(imask_model, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define the new search
   error = rox_plane_search_new(&plane_search, rows_model, cols_model, search_radius);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   if(0)
   {
      // If we set the model with this function the homography r_G_t is set to the identity I
      error = rox_plane_search_set_model(plane_search, image_model, imask_model);
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

      error = rox_plane_search_set_model_warp(plane_search, image_ref_float, r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_matsl3_del(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }

   error = rox_matsl3_new(&c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_homography_shift(c_G_t, tu_model, tv_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_make(plane_search, image_cur_float, c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double score = 0.0, tu = 0.0, tv = 0.0;

   error = rox_plane_search_get_results(&score, &tu, &tv, plane_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("score = %f\n", score);
   rox_log("tu = %f\n", tu);
   rox_log("tv = %f\n", tv);

   //ROX_TEST_CHECK_EQUAL(tu, tu_model-tu_init);
   //ROX_TEST_CHECK_EQUAL(tv, tv_model-tv_init);

   error = rox_matsl3_new(&homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_update_homography(homography, plane_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_print(homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_del(&plane_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_ref_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_ref_float);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_cur_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_cur_float);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&imask_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_plane_search_update_pose_translation)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Plane_Search plane_search = NULL;
   Rox_Array2D_Uchar image_ref_uchar = NULL;
   Rox_Array2D_Float image_ref_float = NULL;
   Rox_Array2D_Uchar image_cur_uchar = NULL;
   Rox_Array2D_Float image_cur_float = NULL;
   Rox_Array2D_Float image_model = NULL;
   Rox_Array2D_Uint  imask_model = NULL;
   Rox_Sint cols = 0, rows = 0;
   Rox_Sint cols_model = 80, rows_model = 40;
   Rox_Sint tu_model = 220, tv_model = 280;
   // Rox_Sint tu_init = 230, tv_init = 260;
   Rox_Sint search_radius = 64;
   Rox_MatSE3 pose = NULL;
   Rox_MatSL3 c_G_t = NULL;
   Rox_Array2D_Double intrinsics = NULL;

   sprintf(filename, "%s", TEST_IMG);
   // sprintf(filename, "%s", IMAGE_REF);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_ref_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_ref_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the float image
   error = rox_array2d_float_new(&image_ref_float, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Convert image to float
   error = rox_array2d_float_from_uchar_normalize(image_ref_float, image_ref_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", TEST_IMG);
   //sprintf(filename, "%s", IMAGE_CUR);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_cur_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_cur_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the float image
   error = rox_array2d_float_new(&image_cur_float, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Convert image to float
   error = rox_array2d_float_from_uchar_normalize(image_cur_float, image_cur_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define image model
   error = rox_array2d_float_new_subarray2d(&image_model, image_ref_float, tv_model, tu_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define imask model
   error = rox_array2d_uint_new(&imask_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(imask_model, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define the new search
   error = rox_plane_search_new(&plane_search, rows_model, cols_model, search_radius);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   if(0)
   {
      // If we set the model with this function the homography r_G_t is set to the identity I
      error = rox_plane_search_set_model(plane_search, image_model, imask_model);
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

      error = rox_plane_search_set_model_warp(plane_search, image_ref_float, r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_matsl3_del(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }

   error = rox_matsl3_new(&c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_homography_shift(c_G_t, tu_model, tv_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_make(plane_search, image_cur_float, c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double score = 0.0, tu = 0.0, tv = 0.0;

   error = rox_plane_search_get_results(&score, &tu, &tv, plane_search);
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
   error = rox_plane_search_update_pose_translation(pose, intrinsics, plane_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_print(pose);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del(&pose);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_del(&plane_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_ref_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_ref_float);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_cur_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_cur_float);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&imask_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&intrinsics);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_plane_search_update_pose_rotation)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Plane_Search plane_search = NULL;
   Rox_Array2D_Uchar image_ref_uchar = NULL;
   Rox_Array2D_Float image_ref_float = NULL;
   Rox_Array2D_Uchar image_cur_uchar = NULL;
   Rox_Array2D_Float image_cur_float = NULL;

   Rox_Array2D_Float image_model = NULL;
   Rox_Array2D_Uint  imask_model = NULL;
   Rox_Sint cols = 0, rows = 0;
   Rox_Sint cols_model = 80, rows_model = 40;
   Rox_Sint tu_model = 220, tv_model = 280;
   // Rox_Sint tu_init = 230, tv_init = 260;
   Rox_Sint search_radius = 64;
   Rox_MatSO3 rotation = NULL;
   Rox_MatSL3 c_G_t = NULL;
   Rox_Array2D_Double intrinsics = NULL;

   sprintf(filename, "%s", TEST_IMG);
   //sprintf(filename, "%s", IMAGE_REF);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_ref_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_ref_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the ref float image
   error = rox_array2d_float_new(&image_ref_float, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Convert ref image to float
   error = rox_array2d_float_from_uchar_normalize(image_ref_float, image_ref_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", TEST_IMG);
   // sprintf(filename, "%s", IMAGE_CUR);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_cur_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_cur_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the float image
   error = rox_array2d_float_new(&image_cur_float, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Convert image to float
   error = rox_array2d_float_from_uchar_normalize(image_cur_float, image_cur_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define image model
   error = rox_array2d_float_new_subarray2d(&image_model, image_ref_float, tv_model, tu_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define imask model
   error = rox_array2d_uint_new(&imask_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(imask_model, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define the new search
   error = rox_plane_search_new(&plane_search, rows_model, cols_model, search_radius);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   if(0)
   {
      // If we set the model with this function the homography r_G_t is set to the identity I
      error = rox_plane_search_set_model(plane_search, image_model, imask_model);
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

      error = rox_plane_search_set_model_warp(plane_search, image_ref_float, r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_matsl3_del(&r_H_t);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }

   error = rox_matsl3_new(&c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_homography_shift(c_G_t, tu_model, tv_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_make(plane_search, image_cur_float, c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double score = 0.0, tu = 0.0, tv = 0.0;

   error = rox_plane_search_get_results(&score, &tu, &tv, plane_search);
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
   error = rox_plane_search_update_pose_rotation(rotation, intrinsics, plane_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matso3_print(rotation);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Delete

   error = rox_matso3_del(&rotation);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_del(&plane_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_ref_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_ref_float);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_cur_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_cur_float);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&imask_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&intrinsics);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

#ifdef EZIO_DEBUG
#define IMAGE_MOD_DEBUG "/home/emalis/Tmp/debug/prediction/photoframe_augmentedpro_00_model_128x128.pgm"
#define IMAGE_CUR_DEBUG "/home/emalis/Tmp/debug/prediction/image_photoframe_centered_1.pgm"

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_plane_search_debug)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Plane_Search plane_search = NULL;
   Rox_Array2D_Uchar image_cur_uchar = NULL;
   Rox_Array2D_Float image_cur_float = NULL;
   Rox_Array2D_Uchar image_model_uchar = NULL;
   //Rox_Array2D_Uchar image_model_uchar_rotate90 = NULL;
   Rox_Array2D_Float image_model = NULL;
   Rox_Array2D_Uint  imask_model = NULL;
   Rox_Sint cols = 0, rows = 0;
   Rox_Sint cols_model = 0, rows_model = 0;
   Rox_Uint tu_model = 0, tv_model = 0;
   Rox_Double score = 0.0, tu = 0.0, tv = 0.0;
   Rox_Double tra[3] = {0.0, 0.0, 1.0};

   Rox_Sint search_radius = 32;
   Rox_MatSE3 c_T_o = NULL;
   Rox_MatSL3 c_G_t = NULL;
   Rox_MatSL3 c_G_o = NULL;
   Rox_MatSL3 t_G_o = NULL;

   Rox_Array2D_Double intrinsics = NULL;

   sprintf(filename, "%s", IMAGE_MOD_DEBUG);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_model_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_uchar_get_size(&rows_model, &cols_model, image_model_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   //error = rox_array2d_uchar_new(&image_model_uchar_rotate90, rows_model, cols_model);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   //error = rox_array2d_uchar_rotate90(image_model_uchar_rotate90, image_model_uchar);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Create the float image
   error = rox_array2d_float_new(&image_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Convert image to float
   error = rox_array2d_float_from_uchar_normalize(image_model, image_model_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   sprintf(filename, "%s", IMAGE_CUR_DEBUG);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_cur_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_uchar_get_size(&rows, &cols, image_cur_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Create the float image
   error = rox_array2d_float_new(&image_cur_float, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Convert image to float
   error = rox_array2d_float_from_uchar_normalize(image_cur_float, image_cur_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Define imask model
   error = rox_array2d_uint_new(&imask_model, rows_model, cols_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_uint_fillval(imask_model, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Define the new search
   error = rox_plane_search_new(&plane_search, rows_model, cols_model, search_radius);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // If we set the model with this function the homography r_G_t is set to the identity I
   error = rox_plane_search_set_model(plane_search, image_model, imask_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Build model_to_image matrix c_G_o
   error = rox_matsl3_new(&c_G_o);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;
   
   // Build model_to_image matrix t_G_o
   error = rox_matsl3_new(&t_G_o);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Compute image_to_image matrix c_G_t
   error = rox_matsl3_new(&c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   if(1)
   {
      error = rox_matsl3_mulmatinv ( c_G_t, c_G_o, t_G_o );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;
   }
   else
   {
      Rox_Double data[9] = {0.5, 0.0, 927.75, 0.0, 0.5, 507.75, 0.0, 0.0, 1.0};
      error = rox_matsl3_set_data(c_G_t, data);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;
   }

   error = rox_plane_search_make ( plane_search, image_cur_float, c_G_t );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;


   error = rox_plane_search_get_results(&score, &tu, &tv, plane_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   rox_log("score = %f\n", score);
   rox_log("tu = %f\n", tu);
   rox_log("tv = %f\n", tv);

   //ROX_TEST_CHECK_EQUAL(tu, -37);
   //ROX_TEST_CHECK_EQUAL(tv, -53);

   error = rox_matse3_new(&c_T_o);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;


   error =  rox_matse3_set_translation ( c_T_o, tra );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_array2d_double_new(&intrinsics, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_transformtools_build_calibration_matrix(intrinsics, 128, 128, (cols-1)/2, (rows-1)/2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   // Update pose translation
   error = rox_plane_search_update_pose_translation(c_T_o, intrinsics, plane_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

   error = rox_matse3_print(c_T_o);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if (error) goto function_terminate;

function_terminate:

   error = rox_matse3_del(&c_T_o);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_plane_search_del(&plane_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_model_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_cur_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_cur_float);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&imask_model);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&c_G_t);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&intrinsics);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

#endif

ROX_TEST_SUITE_END()
