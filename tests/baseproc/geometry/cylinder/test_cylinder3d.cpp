//==============================================================================
//
//    OPENROX   : File test_cylinder3d.cpp
//
//    Contents  : Tests for cylinder3d.c
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
   #include <generated/objset_cylinder3d.h>
   #include <baseproc/geometry/cylinder/cylinder3d.h>
   #include <baseproc/geometry/segment/segment3d.h>
   #include <baseproc/geometry/segment/segment3d_struct.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(cylinder3d)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_cylinder3d_new_del)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Cylinder3D cylinder3d = NULL;
   
   error = rox_cylinder3d_new(&cylinder3d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_cylinder3d_del(&cylinder3d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_cylinder3d_objset)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Cylinder3D cylinder3d = NULL;
   Rox_ObjSet_Cylinder3D objset_cylinder3d = NULL;

   error = rox_cylinder3d_new(&cylinder3d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_objset_cylinder3d_new(&objset_cylinder3d, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Add cylinder3d to objset_cylinder3d : do NOT delete cylinder3d, it will be deleted by objset_cylinder3d_del
   error = rox_objset_cylinder3d_append(objset_cylinder3d, cylinder3d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // The cylinder3d sould ne set to NULL afer append
   cylinder3d = NULL;
   
   // ROX_TEST_MESSAGE("print objset after append \n");
   // rox_cylinder3d__print(rox_objset_cylinder3d_get_data(objset_cylinder3d)[0]);
   
   // ROX_TEST_MESSAGE("print before del \n");
   // rox_cylinder3d__print(cylinder3d);

   //error = rox_cylinder3d_del(&cylinder3d);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   //ROX_TEST_MESSAGE("print after del \n");
   //rox_cylinder3d__print(cylinder3d);
   
   // ROX_TEST_MESSAGE("print objset before del \n");
   //rox_cylinder3d__print(rox_objset_cylinder3d_get_data(objset_cylinder3d)[0]);

   error = rox_objset_cylinder3d_del(&objset_cylinder3d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // ROX_TEST_MESSAGE("print cylinder after objset del \n");
   // rox_cylinder3d__print(cylinder3d);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_cylinder3d_get_tangent_segments)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Segment3D_Struct segment3d_1;
   Rox_Segment3D_Struct segment3d_2; 
   Rox_MatSE3 cam_T_obj = NULL; 
   Rox_MatSE3 obj_T_cyl = NULL; 
   Rox_Double data_obj_T_cyl[16] = {0.999973367931024, -0.007228337043990, 0.001007259779402, -0.002450874586940, 0.007216298107468, 0.999908029094757, 0.011482961005906, 0.002080908184369, -0.001090169853222,  -0.011475386504057, 0.999933561310087, -0.011669245527233, 0.0, 0.0, 0.0, 1.000000000000000};
   Rox_Double data_cam_T_obj[16] = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, -1.0, 0.0, 2.0, 0.0, 0.0, 0.0, 1.0};
   Rox_Cylinder3D cylinder3d = NULL;
   
   error = rox_matse3_new(&cam_T_obj);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matse3_set_data(cam_T_obj, data_cam_T_obj);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matse3_new(&obj_T_cyl);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matse3_set_data(obj_T_cyl, data_obj_T_cyl);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_cylinder3d_new(&cylinder3d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_cylinder3d_set(cylinder3d, 0.1, 0.2, 0.2, obj_T_cyl);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // rox_cylinder3d__print(cylinder3d);
   
   error = rox_cylinder3d_get_tangent_segments(&segment3d_1, &segment3d_2, cam_T_obj, cylinder3d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // ROX_TEST_MESSAGE("Display segments\n");
   // rox_segment3d_print(&segment3d_1);
   // rox_segment3d_print(&segment3d_2);

   ROX_TEST_CHECK_SMALL(segment3d_1.points[0].X - 0.102032515766595, 1e-12);
   ROX_TEST_CHECK_SMALL(segment3d_1.points[0].Y - 0.111822583612272, 1e-12);
   ROX_TEST_CHECK_SMALL(segment3d_1.points[0].Z - 1.976409952294365, 1e-12);

   ROX_TEST_CHECK_SMALL(segment3d_1.points[1].X - 0.101831063810715, 1e-12);
   ROX_TEST_CHECK_SMALL(segment3d_1.points[1].Y + 0.088164128649745, 1e-12);
   ROX_TEST_CHECK_SMALL(segment3d_1.points[1].Z - 1.974113360093184, 1e-12);

   ROX_TEST_CHECK_SMALL(segment3d_2.points[0].X + 0.096977738019807, 1e-12);
   ROX_TEST_CHECK_SMALL(segment3d_2.points[0].Y - 0.111962427837392, 1e-12);
   ROX_TEST_CHECK_SMALL(segment3d_2.points[0].Z - 1.981689086329897, 1e-12);

   ROX_TEST_CHECK_SMALL(segment3d_2.points[1].X + 0.097179189975687, 1e-12);
   ROX_TEST_CHECK_SMALL(segment3d_2.points[1].Y + 0.088024284424626, 1e-12);
   ROX_TEST_CHECK_SMALL(segment3d_2.points[1].Z - 1.979392494128715, 1e-12);

   error = rox_matse3_del(&cam_T_obj);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del(&obj_T_cyl);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_cylinder3d_del(&cylinder3d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
