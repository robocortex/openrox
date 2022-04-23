//==============================================================================
//
//    OPENROX   : File test_linsys_se3_z1_light_affine_premul.cpp
//
//    Contents  : Tests for linsys_se3_z1_light_affine_premul.c
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
   #include <baseproc/array/conversion/array2d_float_from_uchar.h>
   #include <baseproc/array/fill/fillzero.h>
   #include <baseproc/array/mean/mean.h>
   #include <baseproc/array/error/l2_error.h>
   #include <baseproc/array/substract/substract.h>
   #include <baseproc/array/inverse/svdinverse.h>
   #include <baseproc/array/multiply/mulmatmat.h>
   #include <baseproc/image/gradient/basegradient.h>
   #include <baseproc/image/image.h>
   #include <baseproc/image/imask/imask.h>
   #include <baseproc/image/remap/remap_bilinear_omo_float_to_float/remap_bilinear_omo_float_to_float.h>
	#include <baseproc/calculus/linsys/linsys_se3_z1_light_affine_premul.h>
   #include <baseproc/geometry/transforms/transform_tools.h>
   #include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
   #include <inout/numeric/array2d_save.h>
   #include <inout/numeric/array2d_print.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN ( linsys_se3_z1_light_affine_premul )

#ifdef ANDROID
   #define RESULT_PATH "/storage/emulated/0/Documents/Robocortex/Tests/Results/"
#else
   #define RESULT_PATH "./"
#endif

#define mod_file ROX_DATA_HOME"/regression_tests/openrox/plane/model.pgm"

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

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_linsys_se3_z1_light_affine_premul)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double tra[3] = {0.002, 0.0, 1.0};
   Rox_Double l2_error = 0.0;

#if defined(ROX_USE_AVX) || defined(ROX_USE_SSE)
   // Ground truth when computation is with AVX vectorisation
   Rox_Double Lte_grt_data[8*1] = {  -1207748.656311035156250,      53628.135875463485718,     38081.846836172044277,     -52112.507982254028320,  -1213166.705551147460938,     9499.716892123222351,    111.523392230272293,    167.740457236766815 };
   Rox_Double LtL_grt_data[8*8] = { 695430590.468750000000000,  -26945947.512207031250000, -19246891.743835449218750,   26334662.662597656250000, 698185251.140625000000000, -4002148.760803222656250, -37657.432540893554688, -45365.148193359375000, 
                                    -26945947.512207031250000,  599652088.156250000000000,  -4674330.132049560546875, -601234363.156250000000000, -26854781.645629882812500,  4939380.880088806152344, -35930.940479755401611, -91664.258033752441406, 
                                    -19246891.743835449218750,   -4674330.132049560546875,   4375429.752441406250000,    4707979.935638427734375, -19365842.347045898437500,   520119.377901077270508,  21563.002572774887085,  43154.188509702682495,
                                     26334662.662597656250000, -601234363.156250000000000,   4707979.935638427734375,  602830953.328125000000000,  26238040.864746093750000, -4915597.124526977539062,  36549.287902355194092,  93432.113384246826172,
                                    698185251.140625000000000,  -26854781.645629882812500, -19365842.347045898437500,   26238040.864746093750000, 700960628.187500000000000, -4031600.643310546875000, -37694.327266693115234, -45205.020912170410156,
                                     -4002148.760803222656250,    4939380.880088806152344,    520119.377901077270508,   -4915597.124526977539062,  -4031600.643310546875000,  2559661.982177734375000,  -6032.366896629333496, -15410.097501754760742,
                                       -37657.432540893554688,     -35930.940479755401611,     21563.002572774887085,      36549.287902355194092,    -37694.327266693115234,    -6032.366896629333496,  68673.614273071289062, 125793.606018066406250,
                                       -45365.148193359375000,     -91664.258033752441406,     43154.188509702682495,      93432.113384246826172,    -45205.020912170410156,   -15410.097501754760742, 125793.606018066406250, 258572.000000000000000 };
#elif ROX_USE_NEON
   // Ground truth when computation is with NEON vectorisation
   Rox_Double Lte_grt_data[8*1] = {  -1208300.562500000000000,      53769.046875000000000,     38079.524414062500000,     -52254.287109375000000,  -1213722.875000000000000,     9549.317871093750000,    110.896971583366394,    165.948266983032227 };
   Rox_Double LtL_grt_data[8*8] = { 695897648.000000000000000,  -26920449.500000000000000, -19260439.000000000000000,   26308950.000000000000000, 698656208.000000000000000, -4033281.812500000000000, -37443.868896484375000, -44641.646972656250000,
                                    -26920449.500000000000000,  599743280.000000000000000,  -4684302.250000000000000, -601326704.000000000000000, -26828828.500000000000000,  4943403.375000000000000, -35791.466796875000000, -91070.189453125000000,
                                    -19260439.000000000000000,   -4684302.250000000000000,   4378634.750000000000000,    4718204.500000000000000, -19379503.000000000000000,   519795.218750000000000,  21563.260742187500000,  43139.247070312500000,
                                     26308950.000000000000000, -601326704.000000000000000,   4718204.500000000000000,  602923840.000000000000000,  26211818.500000000000000, -4919661.750000000000000,  36409.580078125000000,  92835.873046875000000,
                                    698656208.000000000000000,  -26828828.500000000000000, -19379503.000000000000000,   26211818.500000000000000, 701431296.000000000000000, -4062824.000000000000000, -37475.921386718750000, -44467.169921875000000,
                                     -4033281.812500000000000,    4943403.375000000000000,    519795.218750000000000,   -4919661.750000000000000,  -4062824.000000000000000,  2563491.187500000000000,  -6036.845092773437500, -15434.827880859375000,
                                       -37443.868896484375000,     -35791.466796875000000,     21563.260742187500000,      36409.580078125000000,    -37475.921386718750000,    -6036.845092773437500,  68755.742187500000000, 125985.800781250000000,
                                       -44641.646972656250000,     -91070.189453125000000,     43139.247070312500000,      92835.873046875000000,    -44467.169921875000000,   -15434.827880859375000, 125985.800781250000000, 262144.000000000000000 };
#else
   // Ground truth when computation is with ANSI C
   // Rox_Double Lte_grt_data[8*1] = {  -1208193.115197023376822,      53656.530320713500259,     38075.528953369124793,     -52141.530313016955915,  -1213612.517652464099228,     9546.446483428913780,    111.086553540594991,    166.224138319492340 };
   // Rox_Double LtL_grt_data[8*8] = { 695717673.082426905632019,  -26958337.876568458974361, -19244433.790675126016140,   26347297.643741615116596, 698473255.247297048568726, -4031859.964987221639603, -37438.264319425201393, -44624.052106857299805,                                                                                           
   //                                  -26958337.876568458974361,  599821048.703270435333252,  -4689925.923586901277304, -601404873.027936697006226, -26866740.757707040756941,  4942578.476234426721931, -35791.390465661912458, -91068.838182449340820,                                                                                           
   //                                  -19244433.790675126016140,   -4689925.923586901277304,   4377895.322986768558621,    4723821.045745715498924, -19363423.642072763293982,   519443.147438240877818,  21562.325841964317078,  43137.228908698656596,                                                                                                   
   //                                   26347297.643741615116596, -601404873.027936697006226,   4723821.045745715498924,  603003037.080236315727234,  26250240.055895920842886, -4918862.017422907054424,  36409.664476832993387,  92835.007621305849170,                                                                                              
   //                                  698473255.247297048568726,  -26866740.757707040756941, -19363423.642072763293982,   26250240.055895920842886, 701249560.102618694305420, -4061357.434862137772143, -37470.159342031154665, -44449.452211073483340,                                                                                           
   //                                   -4031859.964987221639603,    4942578.476234426721931,    519443.147438240877818,   -4918862.017422907054424,  -4061357.434862137772143,  2563215.458052732516080,  -6036.860285280889002, -15434.944021702860482,                                                                                                    
   //                                     -37438.264319425201393,     -35791.390465661912458,     21562.325841964317078,      36409.664476832993387,    -37470.159342031154665,    -6036.860285280889002,  68755.069458316080272, 125985.565309941768646,                                                                                                               
   //                                     -44624.052106857299805,     -91068.838182449340820,     43137.228908698656596,      92835.007621305849170,    -44449.452211073483340,   -15434.944021702860482, 125985.565309941768646, 257040.000000000000000};


Rox_Double Lte_grt_data[8*1] = { -1213361.411012915894389, 53774.507064938428812, 38588.615450278412027, -52238.218908914182975, -1218830.819332751445472, 9771.278020620105963, 133.365762428523681, 217.754214555025101 };                                                                                                                       
   
Rox_Double LtL_grt_data[8*8] = { 698329920.437159657478333, -27015728.934457235038280, -19503787.429590836167336, 26397301.906275540590286, 701110779.452715873718262, -4111332.435928887221962, -45748.629047875961987, -65670.345697402954102,                                                                                           
-27015728.934457235038280, 600313401.953453183174133, -4680709.699478698894382, -601898413.575392365455627, -26925030.038576383143663, 4991026.619689689949155, -35676.847496971517103, -90763.646394729614258,                                                                                           
-19503787.429590836167336, -4680709.699478698894382, 4404878.177618499845266, 4715352.718187060207129, -19625407.056749235838652, 527728.167868569726124, 22388.419733182621712, 45228.207079801359214,                                                                                                   
26397301.906275540590286, -601898413.575392365455627, 4715352.718187060207129, 603497824.550158858299255, 26301069.648922186344862, -4966880.808918509632349, 36325.090835294373392, 92600.687955128814792,                                                                                               
701110779.452715873718262, -26925030.038576383143663, -19625407.056749235838652, 26301069.648922186344862, 703912617.464560747146606, -4141637.372110369615257, -45861.036000792446430, -65699.534514757222496,                                                                                           
-4111332.435928887221962, 4991026.619689689949155, 527728.167868569726124, -4966880.808918509632349, -4141637.372110369615257, 2573450.994279577862471, -5716.849246028263224, -14671.688980520819314,                                                                                                    
-45748.629047875961987, -35676.847496971517103, 22388.419733182621712, 36325.090835294373392, -45861.036000792446430, -5716.849246028263224, 68827.859559982083738, 126174.678714901208878,                                                                                                               
-65670.345697402954102, -90763.646394729614258, 45228.207079801359214, 92600.687955128814792, -65699.534514757222496, -14671.688980520819314, 126174.678714901208878, 257550.000000000000000 };
#endif

   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);

   Rox_MatUT3 rKo = NULL;
   error = rox_matut3_new ( &rKo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_build_calibration_matrix ( rKo, FU, FV, CU, CV );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_MatSE3 cTo_z0 = NULL;
   error = rox_matse3_new ( &cTo_z0 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_set_translation ( cTo_z0, tra );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_MatSE3 cTo = NULL;
   error = rox_matse3_new ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_updateZref ( cTo, cTo_z0, -1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matse3_print(cTo_z0);
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

   Rox_Imask Im = NULL;
   error = rox_imask_new ( &Im, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_set_ones ( Im );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_MatSL3 homography = NULL;
   error = rox_matsl3_new ( &homography );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_homography ( homography, cTo, rKo, rKo, 0, 0, -1, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

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

   Rox_Array2D_Float Iu = NULL;
   error = rox_array2d_float_new ( &Iu, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // Fill with zeros since borders may be ignored in gradient computation
   error = rox_array2d_float_fillzero ( Iu );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   Rox_Array2D_Float Iv = NULL;
   
   error = rox_array2d_float_new ( &Iv, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // Fill with zeros since borders may be ignored in gradient computation
   error = rox_array2d_float_fillzero ( Iv );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
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

   error =  rox_array2d_float_basegradient ( Iu, Iv, Gm, Ia, Im);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   //    error =  rox_array2d_float_basegradient_nomask ( Iu, Iv, Ia );
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
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

   error = rox_linsys_se3_z1_light_affine_premul ( LtL, Lte, Iu, Iv, Ia, Id, Gm, cTo, rKo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matrix_print(Lte);
   rox_matrix_print(LtL);

   rox_array2d_double_save ( RESULT_PATH"/Lte_5.txt", Lte );
   rox_array2d_double_save ( RESULT_PATH"/LtL_5.txt", LtL );

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
   
   error = rox_image_del ( &Ic_uchar );
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
