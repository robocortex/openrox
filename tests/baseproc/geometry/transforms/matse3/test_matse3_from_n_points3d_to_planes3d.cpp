//==============================================================================
//
//    OPENROX   : File test_matse3_from_n_points3d_to_planes3d.cpp
//
//    Contents  : Tests for matse3_from_n_points3d_to_planes3d.c
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
   #include <baseproc/geometry/plane/plane_struct.h>
   #include <baseproc/geometry/point/point3d_struct.h>
   #include <baseproc/geometry/transforms/matse3/matse3_from_n_points3d_to_planes3d.h>
   #include <baseproc/maths/maths_macros.h>
   #include <baseproc/maths/linalg/matse3.h>
   #include <inout/numeric/objset_array2d_print.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(matse3_from_n_points3d_to_planes3d)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

// Unitary test
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_unit_matse3_from_n_points3d_to_planes3d_double)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_matse3_from_n_points3d_to_planes3d_double(NULL, NULL, NULL, 0);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NULL_POINTER );

   ROX_TEST_MESSAGE("This test is not completed yet !!! \n");
}

// Non-regression test
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_matse3_from_n_points3d_to_planes3d_double)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint nb_used = 0;
   // Define timer to measure performances
   Rox_Timer timer = NULL;
   Rox_Double time = 0.0;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   // The 3D references planes 
   Rox_Plane3D_Double_Struct p[9];
   {
      p[0].a = -0.861171873375060; p[1].a = -0.861171873375060; p[2].a = -0.861171873375060; p[3].a =  0.210689969485484; p[4].a =  0.210689969485484; p[5].a =  0.210689969485484; p[6].a =  0.371717113252038; p[7].a =  0.371717113252038; p[8].a =  0.371717113252038;
      p[0].b =  0.124883168695917; p[1].b =  0.124883168695917; p[2].b =  0.124883168695917; p[3].b =  0.921733238442047; p[4].b =  0.921733238442047; p[5].b =  0.921733238442047; p[6].b = -0.835646846707646; p[7].b = -0.835646846707646; p[8].b = -0.835646846707646;
      p[0].c = -0.492734409884430; p[1].c = -0.492734409884430; p[2].c = -0.492734409884430; p[3].c =  0.325603399720184; p[4].c =  0.325603399720184; p[5].c =  0.325603399720184; p[6].c =  0.404376971776509; p[7].c =  0.404376971776509; p[8].c =  0.404376971776509;
      p[0].d =  1.347014021244461; p[1].d =  1.347014021244461; p[2].d =  1.347014021244461; p[3].d = -0.327438340965535; p[4].d = -0.327438340965535; p[5].d = -0.327438340965535; p[6].d =  0.388972160994222; p[7].d =  0.388972160994222; p[8].d =  0.388972160994222;
   };

   // 3D corresponding current points with 
   Rox_Point3D_Double_Struct m[9];
   { // Output points from Matlab
      m[0].X = 0.459063722873588; m[1].X = 1.206059057396911; m[2].X = 1.149652817056281; m[3].X = 4.210733678502117; m[4].X = 1.101384602688918; m[5].X = 0.774538933776166; m[6].X =  0.707542946612367; m[7].X = -0.377024680923748; m[8].X = 0.296400427313728;
      m[0].Y = 2.799748441327810; m[1].Y = 1.234885316051811; m[2].Y = 2.007705701570053; m[3].Y = 1.517380792978901; m[4].Y = 0.577813076950107; m[5].Y = 0.619046103670954; m[6].Y =  1.937802531967625; m[7].Y =  0.998200345954548; m[8].Y = 1.352544208991433;
      m[0].Z = 1.266841041976037; m[1].Z = 0.720218523469000; m[2].Z = 3.217269147115803; m[3].Z = 1.265675226506815; m[4].Z = 0.449338561517633; m[5].Z = 1.616106459308457; m[6].Z = -0.360395961267678; m[7].Z =  2.128812590729227; m[8].Z = 0.335640741844283;
   }

   // Rox_Double T_grt[16] = {0.712953265165042, 0.346005675900491, -0.609899757284056, 0.014293351630246, -0.583992135789088, 0.774434594619146, -0.243319222408708, 0.241999346633081, 0.388137639281486, 0.529651895967269, 0.754200266554760, -0.357676970393795, 0.0, 0.0, 0.0, 1.0};
   
   Rox_ObjSet_Array2D_Double T = NULL;

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_objset_array2d_double_new(&T, 1);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

if(0)
{
   rox_timer_start(timer);

   error = rox_matse3_from_n_points3d_to_planes3d_double(T, p, m, 9);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   // Display elapsed time
   rox_timer_stop(timer);
   rox_timer_get_elapsed_ms(&time, timer);

   rox_log("mean time to solve the problem = %f (ms)\n", time);

   error = rox_objset_array2d_double_get_used(&nb_used, T);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("Found %d poses, the first is the best \n", nb_used);
   error = rox_objset_array2d_double_print(T);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );
}
   error = rox_objset_array2d_double_del(&T);
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );

}

// Performance test
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_perf_matse3_from_n_points3d_to_planes3d_double)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );
   ROX_TEST_CHECK_EQUAL( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_matse3_from_n_points3d_to_planes3d_float)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
