//==============================================================================
//
//    OPENROX   : File test_point3d_inverse_projection_from_point2d.cpp
//
//    Contents  : Tests for point3d_inverse_projection_from_point2d.c
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
	#include <baseproc/geometry/point/point3d_inverse_projection_from_point2d.h>
   #include <baseproc/geometry/point/point2d_struct.h>
   #include <baseproc/geometry/point/point3d_struct.h>
   #include <baseproc/geometry/rectangle/rectangle.h>
   #include <baseproc/geometry/transforms/transform_tools.h>
   #include <inout/geometry/point/point3d_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN ( point3d_inverse_projection_from_point2d )

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_point3d_inverse_projection_from_point2d )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double cTo_data[16] = { 1.0, 0.0, 0.0, 0.0,
                               0.0, 1.0, 0.0, 0.0,
                               0.0, 0.0, 1.0, 1.0,
                               0.0, 0.0, 0.0, 1.0};
   Rox_Point3D_Double mo = NULL;
   Rox_Plane3D_Double_Struct plane3d_o;
   Rox_MatUT3 Kc = NULL; 
   Rox_MatSE3 cTo = NULL; 
   Rox_Point2D_Double pc = NULL; 
   Rox_Sint nbpts = 4;

   error = rox_matse3_new ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matse3_set_data ( cTo, cTo_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_new ( &Kc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_calibration_matrix ( Kc, 2560.0, 2560.0, 255.5,255.5 ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   plane3d_o.a = 0.0;
   plane3d_o.b = 0.0;
   plane3d_o.c = 1.0;
   plane3d_o.d = 0.0;

   pc = ( Rox_Point2D_Double ) rox_memory_allocate(sizeof(*pc), nbpts);
   if (!pc) 
   { error = ROX_ERROR_NULL_POINTER; ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); }

   error = rox_rectangle2d_create_image_coordinates ( pc, 512, 512 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   mo = ( Rox_Point3D_Double ) rox_memory_allocate(sizeof(*mo), nbpts);
   if (!mo) 
   { error = ROX_ERROR_NULL_POINTER; ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); }

   error = rox_point3d_double_coplanar_inverse_projection_from_point2d_double ( mo, &plane3d_o, Kc, cTo, pc, nbpts );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_vector_point3d_double_print ( mo, 4 );


   error = rox_matse3_del ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_del ( &Kc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_memory_delete(mo);
   rox_memory_delete(pc);

}

ROX_TEST_SUITE_END()
