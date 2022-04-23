//==============================================================================
//
//    OPENROX   : File test_linsys_point_to_line_matse3.cpp
//
//    Contents  : Tests for linsys_point_to_line_matse3.c
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
   #include <baseproc/calculus/linsys/linsys_point_to_line_matse3.h>
   #include <baseproc/geometry/line/line3d_struct.h>
   #include <baseproc/geometry/line/line2d_struct.h>
   #include <baseproc/geometry/line/line_from_points.h>
   #include <baseproc/geometry/line/line_project.h>
   #include <inout/geometry/line/line2d_print.h>
   #include <inout/geometry/line/line3d_print.h>
   #include <baseproc/array/error/l2_error.h>
   #include <baseproc/array/inverse/svdinverse.h>
   #include <baseproc/array/multiply/mulmatmat.h>
   #include <baseproc/maths/linalg/matrix.h>
   #include <inout/numeric/array2d_save.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN ( linsys_point_to_line_matse3 )

#ifdef ANDROID
   #define RESULT_PATH "/storage/emulated/0/Documents/Robocortex/Tests/Results/"
#else
   #define RESULT_PATH "./"
#endif

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_linsys_point_to_line_matse3 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double l2_error = 0.0;

#if defined(ROX_USE_AVX) || defined(ROX_USE_SSE)
   // Ground truth when computation is with AVX vectorisation
   Rox_Double Lte_grt_data[6*1] = { 0.0,  0.0, 0.0,  0.0, 0.0, 0.0 };
   Rox_Double LtL_grt_data[6*6] = { 0.0,  0.0, 0.0,  0.0, 0.0, 0.0,                                    
                                    0.0,  1.0, 0.0, -1.0, 0.0, 0.0,                                    
                                    0.0,  0.0, 0.0,  0.0, 0.0, 0.0,                                          
                                    0.0, -1.0, 0.0,  1.0, 0.0, 0.0,                                      
                                    0.0,  0.0, 0.0,  0.0, 0.0, 0.0,                                    
                                    0.0,  0.0, 0.0,  0.0, 0.0, 0.0 }; 
#elif ROX_USE_NEON
   // Ground truth when computation is with NEON vectorisation
   Rox_Double Lte_grt_data[6*1] = {   -1207748.649630399886519,      53628.135098390259372,     38081.843632873235036,     -52112.507299648736080,  -1213166.693142144475132,     9499.711165036183957, };
   Rox_Double LtL_grt_data[6*6] = {  695430582.823000073432922,  -26945947.170727744698524, -19246889.948441371321678,   26334662.767303619533777, 698185244.746395230293274, -4002145.585729388054460,
                                     -26945947.170727744698524,  599652087.295029163360596,  -4674327.163945515640080, -601234360.982751011848450, -26854782.046274051070213,  4939378.010390540584922,
                                     -19246889.948441371321678,   -4674327.163945515640080,   4375429.391575847752392,    4707977.131689601577818, -19365840.581442464143038,   520119.278970292245504,
                                      26334662.767303619533777, -601234360.982751011848450,   4707977.131689601577818,  602830949.732856273651123,  26238041.133267775177956, -4915594.338493820279837,
                                     698185244.746395230293274,  -26854782.046274051070213, -19365840.581442464143038,   26238041.133267775177956, 700960618.040518164634705, -4031597.266650577075779,
                                      -4002145.585729388054460,    4939378.010390540584922,    520119.278970292245504,   -4915594.338493820279837,  -4031597.266650577075779,  2559661.724435267038643 }; 
#else
   // Ground truth when computation is with ANSI C
   // Rox_Double Lte_grt_data[8*1] = {  -1208193.115196403115988,      53656.530413264845265,     38075.530933013702452,     -52141.530185661737050,  -1213612.517704645870253,     9546.446496904649393};
   // Rox_Double LtL_grt_data[8*8] = { 695717673.129885077476501,  -26958337.964016653597355, -19244434.746542617678642,   26347297.642187539488077, 698473255.251480340957642, -4031859.960156377870589, 
   //                                  -26958337.964016653597355,  599821048.862501144409180,  -4689925.977472108788788, -601404873.291644454002380, -26866740.795861084014177,  4942578.470288009382784, 
   //                                  -19244434.746542617678642,   -4689925.977472108788788,   4377895.420797899365425,    4723821.103199382312596, -19363424.595922388136387,   519443.153673445631284, 
   //                                   26347297.642187539488077, -601404873.291644454002380,   4723821.103199382312596,  603003037.155323147773743,  26250240.065790329128504, -4918862.014753805473447, 
   //                                  698473255.251480340957642,  -26866740.795861084014177, -19363424.595922388136387,   26250240.065790329128504, 701249560.012359619140625, -4061357.429531683679670, 
   //                                   -4031859.960156377870589,    4942578.470288009382784,    519443.153673445631284,   -4918862.014753805473447,  -4061357.429531683679670,  2563215.459786897990853};

   Rox_Double Lte_grt_data[6*1] = {  -1213361.411007273010910, 53774.507158934145991, 38588.617429641242779, -52238.218780158189475, -1218830.819406633032486, 9771.278034221753842 };

   Rox_Double LtL_grt_data[6*6] = { 698329920.494538545608521, -27015729.023011006414890, -19503788.387211974710226, 26397301.902524508535862, 701110779.465198159217834, -4111332.430630113463849,                                                                                              
-27015729.023011006414890, 600313402.106603145599365, -4680709.752970546483994, -601898413.834343552589417, -26925030.076592445373535, 4991026.611825047060847,                                                                                              
-19503788.387211974710226, -4680709.752970546483994, 4404878.275738031603396, 4715352.775364154949784, -19625408.010943494737148, 527728.174067809479311,                                                                                                      
26397301.902524508535862, -601898413.834343552589417, 4715352.775364154949784, 603497824.619994878768921, 26301069.657152779400349, -4966880.803726577199996,                                                                                                  
701110779.465198159217834, -26925030.076592445373535, -19625408.010943494737148, 26301069.657152779400349, 703912617.375411033630371, -4141637.365882639773190,                                                                                              
-4111332.430630113463849, 4991026.611825047060847, 527728.174067809479311, -4966880.803726577199996, -4141637.365882639773190, 2573450.995604545343667 };

#endif

   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);

   // rox_array2d_float_print(Iu);
   Rox_Matrix Lte_grt = NULL;
   error = rox_matrix_new ( &Lte_grt, 6, 1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( Lte_grt, Lte_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Matrix LtL_grt = NULL;
   error = rox_matrix_new ( &LtL_grt, 6, 6 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( LtL_grt, LtL_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Matrix LtL = NULL;
   error = rox_matrix_new ( &LtL, 6, 6 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Matrix Lte = NULL;
   error = rox_matrix_new ( &Lte, 6, 1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Line3D_Planes_Struct line3D; 
   Rox_Point3D_Double_Struct pt1;
   Rox_Point3D_Double_Struct pt2;

   pt1.X = 0.0;
   pt1.Y = 0.0;
   pt1.Z = 1.0;

   pt2.X = 1.0;
   pt2.Y = 0.0;
   pt2.Z = 1.0;

   error =  rox_line3d_planes_from_2_point3d ( &line3D, &pt1, &pt2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_line3d_planes_print(&line3D);

   Rox_Line2D_Normal_Struct line2D; 

   error = rox_line3d_planes_project_meters ( &line2D, &line3D );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   rox_line2d_normal_print(&line2D);

   Rox_Double x = 0.0; 
   Rox_Double y = 1.0;

   Rox_Double e = 0.0;

   error = rox_linsys_point_to_line_matse3 ( LtL, Lte, &line3D, &line2D, x, y, e );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matrix_print ( Lte );
   rox_matrix_print ( LtL );
   
   rox_array2d_double_save (RESULT_PATH"/Lte_6.txt", Lte);
   rox_array2d_double_save (RESULT_PATH"/LtL_6.txt", LtL);

   error = rox_array2d_double_difference_l2_norm ( &l2_error, Lte_grt, Lte );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error Lte = %f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);
   
   error = rox_array2d_double_difference_l2_norm ( &l2_error, LtL_grt, LtL );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error LtL = %f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);

   // Compute update to solution   
   Rox_Matrix solution = NULL;
   error = rox_matrix_new ( &solution, 6, 1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Matrix LtL_inv = NULL;
   error = rox_matrix_new ( &LtL_inv, 6, 6 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //error = rox_array2d_double_svdinverse ( LtL_inv, LtL );
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //error = rox_array2d_double_mulmatmat ( solution, LtL_inv, Lte );
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   //rox_matrix_print(solution);

   //----- delete 

   error = rox_matrix_del ( &LtL );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &Lte );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &LtL_inv );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &solution );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &Lte_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &LtL_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
