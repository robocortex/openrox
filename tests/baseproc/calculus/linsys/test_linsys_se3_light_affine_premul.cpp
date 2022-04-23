//==============================================================================
//
//    OPENROX   : File test_linsys_se3_light_affine_premul.cpp
//
//    Contents  : Tests for linsys_se3_light_affine_premul.c
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
   #include <baseproc/array/conversion/array2d_float_from_uchar.h>
   #include <baseproc/array/fill/fillzero.h>
   #include <baseproc/image/gradient/basegradient.h>
   #include <baseproc/image/image.h>
   #include <baseproc/image/imask/imask.h>
   #include <baseproc/image/remap/remap_bilinear_omo_float_to_float/remap_bilinear_omo_float_to_float.h>
   #include <baseproc/array/mean/mean.h>
   #include <baseproc/array/substract/substract.h>
   #include <baseproc/array/inverse/svdinverse.h>
   #include <baseproc/array/multiply/mulmatmat.h>
 	#include <baseproc/calculus/linsys/linsys_se3_light_affine_premul.h>
   #include <baseproc/array/error/l2_error.h>
   #include <baseproc/geometry/transforms/transform_tools.h>
   #include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
   #include <inout/numeric/array2d_print.h>
   #include <inout/numeric/array2d_save.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN ( linsys_se3_light_affine_premul )

#ifdef ANDROID
   #define RESULT_PATH "/storage/emulated/0/Documents/Robocortex/Tests/Results/"
#else
   #define RESULT_PATH "./"
#endif

// A (0.2 x 0.2) m square seen at T = [ 1,0,0,0; 0,1,0,0; 0,0,1,1; 0,0,0,1 ]
// m = rox_geometry_rectangle_model(0.2,0.2);
// K = [2560, 0, 255.5; 0, 2560, 255.5; 0, 0, 1];
// T = eye(4);T(3,4)=1;
// [p, z] = rox_camera_perspective_projection(K, T, m);
#define IMG_REF_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D000.pgm"
#define IMG_CUR_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D001.pgm"

#define FU 2560.0
#define FV 2560.0
#define CU 255.5
#define CV 255.5

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_jacobian_se3_light_affine_premul)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double tra[3] = {0.002, 0.0, 1.0};
   Rox_Double l2_error = 0.0;

#if defined(ROX_USE_AVX) || defined(ROX_USE_SSE)
   // Ground truth when computation is with AVX vectorisation
   Rox_Double Lte_grt_data[8*1] = {  -1207748.656311035156250,     53628.135875463485718,     38081.847109876573086,     1515.627922978252172,   -5418.044004797935486,     9499.710580438375473,    111.523392230272293,    167.740457236766815 };
   Rox_Double LtL_grt_data[8*8] = { 695430590.468750000000000, -26945947.512207031250000, -19246891.794067382812500,  -611284.463554382324219, 2754662.218292236328125, -4002145.505493164062500, -37657.432540893554688, -45365.148193359375000, 
                                    -26945947.512207031250000, 599652088.156250000000000,  -4674327.042968750000000, -1582273.702094919979572,   91165.158680915832520,  4939380.654449462890625, -35930.940479755401611, -91664.258033752441406,
                                    -19246891.794067382812500,  -4674327.042968750000000,   4375429.626708984375000,    33649.967625141143799, -118950.661421775817871,   520119.298568725585938,  21563.002514123916626,  43154.187776803970337, 
                                      -611284.463554382324219,  -1582273.702094919979572,     33649.967625141143799,    14315.062901951183449,   -5456.510274454951286,    23783.667897338978946,    618.340940703637898,   1767.855509759858251,
                                      2754662.218292236328125,     91165.158680915832520,   -118950.661421775817871,    -5456.510274454951286,   20711.373694896697998,   -29451.683816790580750,    -36.884447470307350,    160.142502516508102,
                                     -4002145.505493164062500,   4939380.654449462890625,    520119.298568725585938,    23783.667897338978946,  -29451.683816790580750,  2559661.873901367187500,  -6032.367159366607666, -15410.097681522369385,
                                       -37657.432540893554688,    -35930.940479755401611,     21563.002514123916626,      618.340940703637898,     -36.884447470307350,    -6032.367159366607666,  68673.614273071289062, 125793.606018066406250,
                                       -45365.148193359375000,    -91664.258033752441406,     43154.187776803970337,     1767.855509759858251,     160.142502516508102,   -15410.097681522369385, 125793.606018066406250, 258572.000000000000000 };
#elif ROX_USE_NEON
   // Ground truth when computation is with NEON vectorisation
   Rox_Double Lte_grt_data[8*1] = {  -1208305.437868204899132,     53769.112199674338626,     38079.571895335386216,     1514.807289233475331,   -5419.970871315335899,     9549.329702427050506,    110.897232296982651,    165.949354246258736 };
   Rox_Double LtL_grt_data[8*8] = { 695907389.039999127388000, -26920392.657342728227377, -19260408.401747100055218,  -611444.869168334407732, 2756650.803636252414435, -4033304.039498863741755, -37444.034936661846587, -44642.050838470458984, 
                                    -26920392.657342728227377, 599751169.162597298622131,  -4684286.056578221730888, -1583531.176547982729971,   91649.730227276013466,  4943390.978239029645920, -35791.626370684229187, -91069.840484619140625,
                                    -19260408.401747100055218,  -4684286.056578221730888,   4378702.747760000638664,    33887.310492618547869, -119089.090713343437528,   519795.139508148713503,  21563.280893994782673,  43139.203493571360013,
                                      -611444.869168334407732,  -1583531.176547982729971,     33887.310492618547869,    14340.328446494615491,   -5462.675415818479451,    23745.404319925051823,    618.294729744144320,   1766.209170696551155,
                                      2756650.803636252414435,     91649.730227276013466,   -119089.090713343437528,    -5462.675415818479451,   20729.891155349716428,   -29521.371488151205995,    -31.992328638055909,    174.398924366340367,
                                     -4033304.039498863741755,   4943390.978239029645920,    519795.139508148713503,    23745.404319925051823,  -29521.371488151205995,  2563538.045517673715949,  -6036.838507171747551, -15434.864221495467064,
                                       -37444.034936661846587,    -35791.626370684229187,     21563.280893994782673,      618.294729744144320,     -31.992328638055909,    -6036.838507171747551,  68755.506493215449154, 125985.702693082392216, 
                                       -44642.050838470458984,    -91069.840484619140625,     43139.203493571360013,     1766.209170696551155,     174.398924366340367,   -15434.864221495467064, 125985.702693082392216, 257040.000000000000000 };
#else
   // Ground truth when computation is with ANSI C
   // Rox_Double Lte_grt_data[8*1] = {  -1208193.115197023376822,     53656.530320713500259,     38075.528953369124793,     1515.000208143758755,   -5419.402397068413848,     9546.446483428913780,    111.086553540594991,    166.224138319492340 };
   // Rox_Double LtL_grt_data[8*8] = { 695717673.082426905632019, -26958337.876568458974361, -19244433.790675126016140,  -611040.318076178780757, 2755582.070151477586478, -4031859.964987221639603, -37438.264319425201393, -44624.052106857299805, 
   //                                  -26958337.876568458974361, 599821048.703270435333252,  -4689925.923586901277304, -1583824.431406176881865,   91597.169857006359962,  4942578.476234426721931, -35791.390465661912458, -91068.838182449340820, 
   //                                  -19244433.790675126016140,  -4689925.923586901277304,   4377895.322986768558621,    33895.121893235467724, -118989.834706528243260,   519443.147438240877818,  21562.325841964317078,  43137.228908698656596, 
   //                                    -611040.318076178780757,  -1583824.431406176881865,     33895.121893235467724,    14339.434208908673099,   -5460.408185991606842,    23716.453363427022850,    618.273739527534644,   1766.169595093439966, 
   //                                    2755582.070151477586478,     91597.169857006359962,   -118989.834706528243260,    -5460.408185991606842,   20722.638264597764646,   -29497.468157749757665,    -31.894265253582631,    174.599957615086794, 
   //                                   -4031859.964987221639603,   4942578.476234426721931,    519443.147438240877818,    23716.453363427022850,  -29497.468157749757665,  2563215.458052732516080,  -6036.860285280889002, -15434.944021702860482, 
   //                                     -37438.264319425201393,    -35791.390465661912458,     21562.325841964317078,      618.273739527534644,     -31.894265253582631,    -6036.860285280889002,  68755.069458316080272, 125985.565309941768646, 
   //                                     -44624.052106857299805,    -91068.838182449340820,     43137.228908698656596,     1766.169595093439966,     174.599957615086794,   -15434.944021702860482, 125985.565309941768646, 257040.000000000000000};

Rox_Double Lte_grt_data[8*1] = { -1213361.411012915894389, 53774.507064938428812, 38588.615450278412027, 1536.288359240867976, -5469.408289658884314, 9771.278020620105963, 133.365762428523681, 217.754214555025101 };                                                                                                                       
   
Rox_Double LtL_grt_data[8*8] = { 698329920.437159657478333, -27015728.934457235038280, -19503787.429590836167336, -618427.116717528202571, 2780858.919861903879791, -4111332.435928887221962, -45748.629047875961987, -65670.345697402954102,                                                                                              
-27015728.934457235038280, 600313401.953453183174133, -4680709.699478698894382, -1585011.730021580355242, 90698.948129561453243, 4991026.619689689949155, -35676.847496971517103, -90763.646394729614258,                                                                                                 
-19503787.429590836167336, -4680709.699478698894382, 4404878.177618499845266, 34643.018556003851700, -121619.609123271991848, 527728.167868569726124, 22388.419733182621712, 45228.207079801359214,                                                                                                       
-618427.116717528202571, -1585011.730021580355242, 34643.018556003851700, 14399.057584654075981, -5533.298899592279668, 24145.805918449212186, 648.243071627955715, 1837.041736125946954,                                                                                                                 
2780858.919861903879791, 90698.948129561453243, -121619.609123271991848,-5533.298899592279668, 20978.938551390434441, -30304.934063714357762, -112.406149331447665, -29.188734768663608,                                                                                                                 
-4111332.435928887221962, 4991026.619689689949155, 527728.167868569726124, 24145.805918449212186, -30304.934063714357762, 2573450.994279577862471, -5716.849246028263224, -14671.688980520819314,                                                                                                         
-45748.629047875961987, -35676.847496971517103, 22388.419733182621712, 648.243071627955715, -112.406149331447665, -5716.849246028263224, 68827.859559982083738, 126174.678714901208878,                                                                                                                   
-65670.345697402954102, -90763.646394729614258, 45228.207079801359214, 1837.041736125946954, -29.188734768663608, -14671.688980520819314, 126174.678714901208878, 257550.000000000000000 };

#endif


   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);

   Rox_MatUT3 rKo = NULL;
   error = rox_matut3_new ( &rKo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_build_calibration_matrix ( rKo, FU, FV, CU, CV );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_MatSE3 cTo = NULL;
   error = rox_matse3_new ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_set_translation ( cTo, tra );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matse3_print(cTo);

	Rox_Image Ir_uchar = NULL;
   error = rox_image_new_read_pgm ( &Ir_uchar, IMG_REF_PATH );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Sint cols = 0, rows = 0;
   error = rox_image_get_size ( &rows, &cols, Ir_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Ir = NULL;
   error = rox_array2d_float_new ( &Ir, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_from_uchar_normalize ( Ir, Ir_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Image Ic_uchar = NULL;
   error = rox_image_new_read_pgm ( &Ic_uchar, IMG_REF_PATH );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Array2D_Float Ic = NULL;
   error = rox_array2d_float_new ( &Ic, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_from_uchar_normalize ( Ic, Ic_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Imask Gm = NULL;
   error = rox_imask_new ( &Gm, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_imask_set_ones ( Gm );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Uint Im = NULL;
   error = rox_imask_new ( &Im, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_set_ones ( Im );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_MatSL3 homography = NULL;
   error = rox_matsl3_new ( &homography );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_MatUT3 Kc    = rKo;
   Rox_MatSE3 c_T_o = cTo;
   Rox_MatSL3 t_G_o = rKo; // -> Kr
   Rox_MatSL3 c_G_t = homography; 
   Rox_MatSL3 c_G_o = NULL; // should be permanently stored in the odometry object ?

   error = rox_matsl3_new ( &c_G_o ) ;
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_model_to_image_homography ( c_G_o, Kc, c_T_o );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      
   error = rox_matsl3_mulmatinv ( c_G_t, c_G_o, t_G_o );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matsl3_del ( &c_G_o );

   Rox_Array2D_Float Iw = NULL;
   error = rox_array2d_float_new ( &Iw, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Warp the current image
   Rox_MeshGrid2D_Float grid = NULL;
   error = rox_meshgrid2d_float_new ( &grid, rows, cols ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Warp coordinates
   error = rox_warp_grid_sl3_float ( grid, homography ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // Remap with bilinear interpolation
   error = rox_remap_bilinear_omo_float_to_float ( Iw, Im, Ic, grid ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   //rox_imask_save_pgm("imask_warp.pgm", Im);
   //rox_array2d_float_save ( "Iw.txt", Iw);

   Rox_Array2D_Float Ia = NULL;
   error = rox_array2d_float_new ( &Ia, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_mean ( Ia, Ir, Iw );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Id = NULL;
   error = rox_array2d_float_new ( &Id, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_substract ( Id, Ir, Iw );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Iu = NULL;
   error = rox_array2d_float_new ( &Iu, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Iv = NULL;
   error = rox_array2d_float_new ( &Iv, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error =  rox_array2d_float_basegradient ( Iu, Iv, Gm, Ia, Im );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // rox_array2d_float_print(Iu);

   Rox_Matrix LtL = NULL;
   error = rox_matrix_new ( &LtL, 8, 8 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Matrix Lte = NULL;
   error = rox_matrix_new ( &Lte, 8, 1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Matrix LtL_grt = NULL;
   error = rox_matrix_new ( &LtL_grt, 8, 8 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( LtL_grt, LtL_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Matrix Lte_grt = NULL;
   error = rox_matrix_new ( &Lte_grt, 8, 1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( Lte_grt, Lte_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //rox_imask_save_pgm("imask_total.pgm", Gm);

   //rox_array2d_float_save ( "Iu.txt", Iu);
   //rox_array2d_float_save ( "Iv.txt", Iv);
   //rox_array2d_float_save ( "Ia.txt", Ia);
   //rox_array2d_float_save ( "Id.txt", Id);

   error = rox_jacobian_se3_light_affine_premul ( LtL, Lte, Iu, Iv, Ia, Id, Gm, cTo, rKo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matrix_print(Lte);
   rox_matrix_print(LtL);

   rox_array2d_double_save (RESULT_PATH"/Lte_4.txt", Lte);
   rox_array2d_double_save (RESULT_PATH"/LtL_4.txt", LtL);

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
   error = rox_matrix_new ( &solution, 8, 1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Matrix LtL_inv = NULL;
   error = rox_matrix_new ( &LtL_inv, 8, 8 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_svdinverse ( LtL_inv, LtL );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_mulmatmat ( solution, LtL_inv, Lte );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   rox_matrix_print(solution);

   //----- delete 

   error = rox_image_del ( &Ir_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_del ( &Im );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Iu );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Iv );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Ia );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Id );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &LtL );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &Lte );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_del ( &rKo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}

ROX_TEST_SUITE_END()
