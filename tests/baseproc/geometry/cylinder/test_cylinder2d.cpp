//==============================================================================
//
//    OPENROX   : File test_cylinder2d.cpp
//
//    Contents  : Tests for cylinder2d.c
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
   #include <baseproc/geometry/segment/segment2d_struct.h>
   #include <baseproc/geometry/cylinder/cylinder2d_struct.h>
   #include <baseproc/geometry/cylinder/cylinder2d.h>
   #include <baseproc/geometry/cylinder/cylinder3d.h>
   #include <baseproc/geometry/cylinder/cylinder_project.h>
   #include <baseproc/geometry/cylinder/cylinder_transform.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(cylinder2d)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_cylinder2d_new_del)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Cylinder2D cylinder2d = NULL;
   
   error = rox_cylinder2d_new(&cylinder2d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_cylinder2d_del(&cylinder2d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_cylinder2d_project_cylinder3d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSE3 cam_T_obj = NULL; 
   Rox_MatSE3 obj_T_cyl = NULL; 
   Rox_Double data_obj_T_cyl[16] = {0.999973367931024, -0.007228337043990, 0.001007259779402, -0.002450874586940, 0.007216298107468, 0.999908029094757, 0.011482961005906, 0.002080908184369, -0.001090169853222,  -0.011475386504057, 0.999933561310087, -0.011669245527233, 0.0, 0.0, 0.0, 1.000000000000000};
   Rox_Double data_cam_T_obj[16] = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, -1.0, 0.0, 2.0, 0.0, 0.0, 0.0, 1.0};
   Rox_Double data_K_cam[9] = {1000.0, 0.0, 320.0, 0.0, 1000.0, 240.0, 0.0, 0.0, 1.0};
   Rox_Cylinder3D cylinder3d_cam = NULL;
   Rox_Cylinder3D cylinder3d_obj = NULL;
   Rox_Cylinder2D cylinder2d = NULL;
   Rox_Array2D_Double K_cam = NULL;
   
   error = rox_array2d_double_new(&K_cam, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_set_buffer_no_stride(K_cam, data_K_cam);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matse3_new(&cam_T_obj);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matse3_set_data(cam_T_obj, data_cam_T_obj);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matse3_new(&obj_T_cyl);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matse3_set_data(obj_T_cyl, data_obj_T_cyl);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_cylinder2d_new(&cylinder2d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_cylinder3d_new(&cylinder3d_cam);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_cylinder3d_new(&cylinder3d_obj);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_cylinder3d_set(cylinder3d_obj, 0.1, 0.2, 0.2, obj_T_cyl);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // ROX_TEST_MESSAGE("Display 3D cylinder in F_obj\n");
   // rox_cylinder3d__print(cylinder3d_obj);
   
   // Change the object frame to the camera frame for the cylinder
   error = rox_cylinder3d_transform(cylinder3d_cam, cam_T_obj, cylinder3d_obj);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // ROX_TEST_MESSAGE("Display 3D cylinder in F_cam\n");
   // rox_cylinder3d__print(cylinder3d_cam);

   error = rox_cylinder2d_project_cylinder3d(cylinder2d, K_cam, cylinder3d_cam);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // ROX_TEST_MESSAGE("Display projected cylinder\n");
   // rox_cylinder2d_print(cylinder2d);

   error = rox_cylinder2d_transform_project_cylinder3d(cylinder2d, K_cam, cam_T_obj, cylinder3d_obj);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // ROX_TEST_MESSAGE("Display tranformed and projected cylinder\n");
   // rox_cylinder2d_print(cylinder2d);

   // Check results
   ROX_TEST_CHECK_SMALL(cylinder2d->s1->points[0].u - 371.6251780902783, 1e-12);
   ROX_TEST_CHECK_SMALL(cylinder2d->s1->points[0].v - 296.5786381931845, 1e-12);

   ROX_TEST_CHECK_SMALL(cylinder2d->s1->points[1].u - 371.5831896329945, 1e-12);
   ROX_TEST_CHECK_SMALL(cylinder2d->s1->points[1].v - 195.3398855243126, 1e-12);

   ROX_TEST_CHECK_SMALL(cylinder2d->s2->points[0].u - 271.0630912342509, 1e-12);
   ROX_TEST_CHECK_SMALL(cylinder2d->s2->points[0].v - 296.4984833441995, 1e-12);

   ROX_TEST_CHECK_SMALL(cylinder2d->s2->points[1].u - 270.9045374962566, 1e-12);
   ROX_TEST_CHECK_SMALL(cylinder2d->s2->points[1].v - 195.5296462496833, 1e-12);

   error = rox_array2d_double_del(&K_cam);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del(&cam_T_obj);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del(&obj_T_cyl);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_cylinder2d_del(&cylinder2d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_cylinder3d_del(&cylinder3d_cam);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_cylinder3d_del(&cylinder3d_obj);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
