//==============================================================================
//
//    OPENROX   : File test_unwrap_model.cpp
//
//    Contents  : Tests for unwrap_model.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== INCLUDED HEADERS   =====================================================

#include <openrox_tests.hpp>

extern "C"
{
   #include <baseproc/geometry/rectangle/rectangle.h>
   #include <baseproc/geometry/point/point3d_struct.h>
	#include <core/virtualview/unwrap_model.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN(unwrap_model)

#define TEXTURE_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/photoframe/models/photoframe_1.pgm"

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_unwrap_model)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Image unwrapped_image = NULL;
   Rox_Imask unwrapped_imask = NULL;
   Rox_Plane3D_Double_Struct plane;
   Rox_MatSE3 newpose = NULL; 
   Rox_MatUT3 newcalib = NULL; 
   Rox_Image model_image = NULL; 
   Rox_Point3D_Double_Struct vertices[4]; 
   Rox_Uint basesize = 32;

   Rox_Double sizex = 1.0; Rox_Double sizey = 1.0;
   
   error = rox_rectangle3d_create_centered_plane_xright_ydown ( vertices, sizex, sizey );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", TEXTURE_PATH);
   rox_log("read file %s\n", filename);

   // Load model for identification
   error = rox_image_new_read_pgm ( &model_image, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_new ( &newpose );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_new ( &newcalib );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Thi function creates a new image and mask
   error = rox_unwrap_model ( &unwrapped_image, &unwrapped_imask, &plane, newpose, newcalib, model_image, vertices, basesize );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // rox_matse3_print ( newpose );
   // rox_matut3_print ( newcalib );

   error = rox_matse3_del ( &newpose );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_del ( &newcalib );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del ( &model_image );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del ( &unwrapped_image );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_del ( &unwrapped_imask );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
