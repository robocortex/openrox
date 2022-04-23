//==============================================================================
//
//    OPENROX   : File test_ellipse3d.cpp
//
//    Contents  : Tests for ellipse3d.c
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
   #include <generated/objset_ellipse3d.h>
   #include <baseproc/geometry/ellipse/ellipse3d.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(ellipse3d)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_ellipse3d_new_del)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ellipse3D ellipse3d = NULL;
   
   error = rox_ellipse3d_new(&ellipse3d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_ellipse3d_del(&ellipse3d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_ellipse3d_objset)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ellipse3D ellipse3d = NULL;
   Rox_ObjSet_Ellipse3D objset_ellipse3d = NULL;

   error = rox_ellipse3d_new(&ellipse3d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_objset_ellipse3d_new(&objset_ellipse3d, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Add ellipse3d to objset_ellipse3d : do NOT delete ellipse3d, it will be deleted by objset_ellipse3d_del
   error = rox_objset_ellipse3d_append(objset_ellipse3d, ellipse3d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // The ellipse3d sould ne set to NULL afer append
   ellipse3d = NULL;
   
   Rox_Ellipse3D * e3d = NULL;
   error = rox_objset_ellipse3d_get_data_pointer( &e3d, objset_ellipse3d );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("print objset after append \n");
   rox_ellipse3d_print(e3d[0]);
   
   rox_log("print before del \n");
   rox_ellipse3d_print(ellipse3d);

   //error = rox_ellipse3d_del(&ellipse3d);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   
   rox_log("print objset before del \n");
   //rox_ellipse3d_print(rox_objset_ellipse3d_get_data(objset_ellipse3d)[0]);

   error = rox_objset_ellipse3d_del(&objset_ellipse3d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("print ellipse after objset del \n");
   rox_ellipse3d_print(ellipse3d);
}


ROX_TEST_SUITE_END()
