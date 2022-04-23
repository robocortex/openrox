//==============================================================================
//
//    OPENROX   : File test_linsys_texture_matse3_light_affine_model2d.cpp
//
//    Contents  : Tests for linsysn_texture_matse3_light_affine_model2d.c
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
   #include <baseproc/image/gradient/basegradient.h>
   #include <baseproc/image/image.h>
   #include <baseproc/image/imask/imask.h>
   #include <baseproc/image/remap/remap_bilinear_omo_float_to_float/remap_bilinear_omo_float_to_float.h>
	#include <baseproc/calculus/linsys/linsys_texture_matse3_light_affine_model2d.h>
   #include <baseproc/maths/linalg/matse3.h>
   #include <baseproc/maths/linalg/matut3.h>
   #include <baseproc/geometry/transforms/transform_tools.h>
   #include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
   #include <baseproc/array/mean/mean.h>
   #include <baseproc/array/fill/fillzero.h>
   #include <baseproc/array/substract/substract.h>
   #include <baseproc/array/conversion/array2d_float_from_uchar.h>
   #include <baseproc/array/inverse/svdinverse.h>
   #include <baseproc/array/multiply/mulmatmat.h>
   #include <baseproc/array/error/l2_error.h>
   #include <baseproc/geometry/pixelgrid/meshgrid2d.h>

   #include <inout/numeric/array2d_save.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN ( linsys_texture_matse3_light_affine_model2d )

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

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_jacobian_se3_zplane_light_affine_premul)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double tra[3] = {0.002, 0.0, 0.0};

   Rox_Double a = +0.0;
   Rox_Double b = +0.0; 
   Rox_Double c = +1.0;
   Rox_Double d = -1.0;

   Rox_Double l2_error = 0.0;

#if defined(ROX_USE_AVX) || defined(ROX_USE_SSE)
   // Ground truth when computation is with AVX vectorisation
   Rox_Double Lte_grt_data[8*1] = {  -1207748.582794116111472, 53628.131889921889524, 38081.844864415630582, -52112.504052175696415, -1213166.626688295975327, 9499.716187434500171, 111.523388095017594, 167.740458205342293 };
   Rox_Double LtL_grt_data[8*8] = { 695430508.986564993858337, -26945943.903525512665510, -19246889.311972144991159, 26334659.424327824264765, 698185170.754959464073181, -4002148.439984728582203, -37657.432397513613978, -45365.147265106497798, 
                                    -26945943.903525512665510, 599652018.572873353958130, -4674329.410743013024330 ,-601234292.034212470054626, -26854778.681744389235973, 4939380.249891038052738, -35930.943378070529434, -91664.260988742258633 ,
                                    -19246889.311972144991159, -4674329.410743013024330 ,4375429.235493646934628 ,4707979.415604049339890, -19365839.980930674821138, 520119.308852628164459, 21563.001731366384774, 43154.187252497620648 ,
                                    26334659.424327824264765 ,-601234292.034212470054626, 4707979.415604049339890 ,602830880.802516102790833, 26238037.616882327944040, -4915596.562976432032883, 36549.283852377026051, 93432.116499713651137 ,
                                    698185170.754959464073181, -26854778.681744389235973, -19365839.980930674821138, 26238037.616882327944040, 700960544.233276963233948, -4031600.121622863225639, -37694.317149479531508, -45205.005138828622876 ,
                                    -4002148.439984728582203 , 4939380.249891038052738 ,520119.308852628164459 ,-4915596.562976432032883, -4031600.121622863225639, 2559661.656310697551817, -6032.366506732785638, -15410.097301434201654 ,
                                    -37657.432397513613978   ,-35930.943378070529434 ,21563.001731366384774, 36549.283852377026051, -37694.317149479531508, -6032.366506732785638, 68673.615143318660557, 125793.607143566012383 ,
                                    -45365.147265106497798   ,-91664.260988742258633 ,43154.187252497620648, 93432.116499713651137, -45205.005138828622876, -15410.097301434201654, 125793.607143566012383, 256536.000000000000000 };
#elif ROX_USE_NEON
   // TODO : NOT the same number of pixels of SSE / AVX and ANSI case !!!
   // Ground truth when computation is with NEON vectorisation
   Rox_Double Lte_grt_data[8*1] = {  -1208305.365324838552624,      53769.109068820675020,     38079.575555088515102,     -52254.301474057887390,  -1213725.336702848551795,     9549.335301470091508,    110.897232296982651,    165.949354246258736 }; 
   Rox_Double LtL_grt_data[8*8] = { 695907305.583628535270691,  -26920389.376347567886114, -19260409.481515895575285,   26308944.367380529642105, 698663956.153710603713989, -4033307.057240697089583, -37444.030921201272577, -44642.047445899253944,
                                    -26920389.376347567886114,  599751097.893763065338135,  -4684288.386874402873218, -601334628.772845864295960, -26828739.654248028993607,  4943393.439087437465787, -35791.624180632912612, -91069.835357397008920, 
                                    -19260409.481515895575285,   -4684288.386874402873218,   4378702.603641927242279,    4718175.728627372533083, -19379498.612668927758932,   519795.162240891368128,  21563.280563413081836,  43139.202704892493784, 
                                     26308944.367380529642105, -601334628.772845864295959,   4718175.728627372533083,  602932500.213394045829773,  26211831.966675385832787, -4919648.040813517756760,  36409.918941883341176,  92836.044171604502480, 
                                    698663956.153710603713989,  -26828739.654248028993607, -19379498.612668927758932,   26211831.966675385832787, 701441336.820669054985046, -4062828.431254826486111, -37476.023401812519296, -44467.647964928008150,
                                     -4033307.057240697089583,    4943393.439087437465787,    519795.162240891368128,   -4919648.040813517756760,  -4062828.431254826486111,  2563537.918551901355386,  -6036.838339326944151, -15434.863986985481461, 
                                       -37444.030921201272577,     -35791.624180632912612,     21563.280563413081836,      36409.918941883341176,    -37476.023401812519296,    -6036.838339326944151,  68755.506493215449154, 125985.702693082392216,
                                       -44642.047445899253944,     -91069.835357397008920,     43139.202704892493784,      92836.044171604502480,    -44467.647964928008150,   -15434.863986985481461, 125985.702693082392216, 257040.000000000000000 };
#else
   // Ground truth when computation is with ANSI C
   // Rox_Double Lte_grt_data[8*1] = {-1208193.042665240820497, 53656.527377160346077, 38075.532651344794431, -52141.527153386996360, -1213612.445506399963051, 9546.452109686872063, 111.086553540594991, 166.224138319492340  };
   // Rox_Double LtL_grt_data[8*8] = { 695717589.964677095413208, -26958334.784165386110544, -19244434.935967415571213, 26347294.399264737963676, 698473171.856281042098999, -4031862.989898267667741, -37438.263172296610719, -44624.052329830854433, 
   //                                  -26958334.784165386110544, 599820977.571013569831848, -4689928.269562419503927, -601404801.771214723587036, -26866737.523441642522812, 4942580.947298077866435, -35791.389025627715455, -91068.834036789994570, 
   //                                  -19244434.935967415571213, -4689928.269562419503927, 4377895.182985313236713, 4723823.433453733101487, -19363424.830223076045513, 519443.170369837549515, 21562.325582868710626, 43137.228335367835825, 
   //                                  26347294.399264737963676 ,-601404801.771214723587036, 4723823.433453733101487, 603002965.639993667602539, 26250236.655305288732052, -4918864.474490384571254, 36409.662327627760533, 92835.003540967270965, 
   //                                  698473171.856281042098999, -26866737.523441642522812, -19363424.830223076045513, 26250236.655305288732052, 701249476.712515950202942, -4061360.462804576847702, -37470.157960145479592, -44449.452973437466426, 
   //                                  -4031862.989898267667741, 4942580.947298077866435, 519443.170369837549515, -4918864.474490384571254, -4061360.462804576847702, 2563215.333807330578566, -6036.860064595985023, -15434.943675362912472, 
   //                                  -37438.263172296610719, -35791.389025627715455, 21562.325582868710626, 36409.662327627760533, -37470.157960145479592, -6036.860064595985023, 68755.069458316080272, 125985.565309941768646, 
   //                                  -44624.052329830854433, -91068.834036789994570, 43137.228335367835825, 92835.003540967270965, -44449.452973437466426, -15434.943675362912472, 125985.565309941768646, 257040.000000000000000};

   Rox_Double Lte_grt_data[8*1] = {-1213361.338153963908553, 53774.504116222749872, 38588.619111423620780, -52238.215742357395357, -1218830.746864377986640, 9771.283662093655948, 133.365762428523681, 217.754214555025101 };

   Rox_Double LtL_grt_data[8*8] = { 698329836.974786281585693, -27015725.834483359009027, -19503788.539530083537102, 26397298.652024492621422, 701110695.724245786666870, -4111335.463295343797654, -45748.627408871230728, -65670.344646789162653,                                                                                           
-27015725.834483359009027, 600313330.755899786949158, -4680712.048799239099026, -601898342.251759529113770, -26925026.796394564211369, 4991029.084503060206771, -35676.846066444260941, -90763.642271064381930,                                                                                           
-19503788.539530083537102, -4680712.048799239099026, 4404878.033953381702304, 4715355.109423039481044, -19625408.209352977573872, 527728.190769124310464, 22388.419418504752684, 45228.206367479957407,                                                                                                   
26397298.652024492621422, -601898342.251759529113770, 4715355.109423039481044, 603497753.049694776535034, 26301066.240892596542835, -4966883.259537278674543, 36325.088691051227215, 92600.683906291291350,                                                                                               
701110695.724245786666870, -26925026.796394564211369, -19625408.209352977573872, 26301066.240892596542835, 703912533.724874138832092, -4141640.402314499486238, -45861.034054066964018, -65699.533978797466261,                                                                                           
-4111335.463295343797654, 4991029.084503060206771, 527728.190769124310464, -4966883.259537278674543, -4141640.402314499486238, 2573450.869515756145120, -5716.849003065657598, -14671.688576178101357,                                                                                                    
-45748.627408871230728, -35676.846066444260941, 22388.419418504752684, 36325.088691051227215, -45861.034054066964018, -5716.849003065657598, 68827.859559982083738, 126174.678714901208878,                                                                                                               
-65670.344646789162653, -90763.642271064381930, 45228.206367479957407, 92600.683906291291350, -65699.533978797466261, -14671.688576178101357, 126174.678714901208878, 257550.000000000000000 };

#endif

   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);

   Rox_Array2D_Double K = NULL; 
   error = rox_matut3_new ( &K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_build_calibration_matrix ( K, FU, FV, CU, CV );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Double cTr = NULL; 
   error = rox_matse3_new ( &cTr );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_set_translation ( cTr, tra );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Image Ir_uchar = NULL;
   error = rox_image_new_read_pgm ( &Ir_uchar, IMG_REF_PATH );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Sint cols = 0, rows = 0;
   error = rox_image_get_size ( &cols, &rows, Ir_uchar );
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

   error = rox_transformtools_build_homography ( homography, cTr, K, K, 0, 0, -1, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matsl3_print(homography);

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

   Rox_Array2D_Float Ia = NULL; 
   error = rox_array2d_float_new ( &Ia, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_mean ( Ia, Ir, Iw );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Id = NULL; 
   error = rox_array2d_float_new ( &Id, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_float_substract ( Id, Ir, Iw );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create gradients
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

   error =  rox_array2d_float_basegradient ( Iu, Iv, Gm, Ia, Im );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Matrix Lte_grt = NULL;
   error = rox_matrix_new ( &Lte_grt, 8, 1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( Lte_grt, Lte_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );  

   Rox_Matrix LtL_grt = NULL;
   error = rox_matrix_new ( &LtL_grt, 8, 8 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( LtL_grt, LtL_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create new outputs
   Rox_Matrix LtL = NULL; 
   error = rox_matrix_new ( &LtL, 8, 8 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Matrix Lte = NULL; 
   error = rox_matrix_new ( &Lte, 8, 1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_linsys_texture_matse3_light_affine_model2d ( LtL, Lte, K, cTr, a, b, c, d, Iu, Iv, Ia, Id, Gm );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matrix_print ( Lte );
   rox_matrix_print ( LtL );

   error = rox_array2d_double_difference_l2_norm ( &l2_error, LtL_grt, LtL );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error LtL = %f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);

   error = rox_array2d_double_difference_l2_norm ( &l2_error, Lte_grt, Lte );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error Lte = %f \n", l2_error);

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

}

ROX_TEST_SUITE_END()
