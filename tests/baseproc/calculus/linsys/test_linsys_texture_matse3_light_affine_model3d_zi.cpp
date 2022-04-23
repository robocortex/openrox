//==============================================================================
//
//    OPENROX   : File test_linsys_texture_matse3_light_affine_model3d_zi.cpp
//
//    Contents  : Tests for linsys_texture_matse3_light_affine_model3d_zi.c
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
   #include <baseproc/array/fill/fillval.h>
   #include <baseproc/array/error/l2_error.h>
   #include <baseproc/array/mean/mean.h>
   #include <baseproc/array/substract/substract.h>
   #include <baseproc/array/inverse/svdinverse.h>
   #include <baseproc/array/multiply/mulmatmat.h>
   #include <baseproc/maths/linalg/matse3.h>
   #include <baseproc/maths/linalg/matrix.h>
   #include <baseproc/image/gradient/basegradient.h>
   #include <baseproc/image/image.h>
   #include <baseproc/image/imask/imask.h>
   #include <baseproc/image/remap/remap_bilinear_omo_float_to_float/remap_bilinear_omo_float_to_float.h>
   #include <baseproc/calculus/linsys/linsys_texture_matse3_light_affine_model3d_zi.h>
   #include <baseproc/geometry/pixelgrid/warp_grid_matse3_z.h>
   #include <baseproc/geometry/pixelgrid/warp_grid_matse3_zi.h>
   #include <system/time/timer.h>
   #include <inout/numeric/array2d_print.h>
   #include <inout/numeric/ansi_array_print.h>
   #include <inout/numeric/array2d_save.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN ( linsys_texture_matse3_light_affine_model3d_zi )

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

// #define IMG_REF_PATH ROX_DATA_HOME"/devapps/model_based/switch/lumia/sequence_1/pgm/switch_lumia_seq1_img0000.pgm"
// #define IMG_CUR_PATH ROX_DATA_HOME"/devapps/model_based/switch/lumia/sequence_1/pgm/switch_lumia_seq1_img0001.pgm"
// #define FU 1415
// #define FV 1431
// #define CU 954
// #define CV 559

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_linsys_texture_matse3_light_affine_model3d_zi )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double l2_error = 0.0;

#ifdef ROX_USE_AVX
   // Ground truth when computation is with AVX vectorisation
   Rox_Double Lte_grt_data[8*1] = {  -1207748.655487060546875,      53628.135463237762451,     38081.841278135776520,     -52112.507410526275635,  -1213166.691162109375000,     9499.710679650306702,    111.523388452827930,    167.740456640720367 };

   Rox_Double LtL_grt_data[8*8] = { 695430589.187500000000000,  -26945946.957641601562500, -19246888.213745117187500,   26334662.424072265625000, 698185252.640625000000000, -4002145.502380371093750, -37657.435111999511719, -45365.145576477050781,
                                    -26945946.957641601562500,  599652089.296875000000000,  -4674326.875000000000000, -601234364.390625000000000, -26854781.596191406250000,  4939377.839416503906250, -35930.951557159423828, -91664.257667541503906,
                                    -19246888.213745117187500,   -4674326.875000000000000,   4375429.397460937500000,    4707976.867218017578125, -19365838.615966796875000,   520119.281620025634766,  21563.002219676971436,  43154.187518835067749,
                                     26334662.424072265625000, -601234364.390625000000000,   4707976.867218017578125,  602830954.562500000000000,  26238040.162109375000000, -4915594.132415771484375,  36549.278554916381836,  93432.122383117675781,
                                    698185252.640625000000000,  -26854781.596191406250000, -19365838.615966796875000,   26238040.162109375000000, 700960624.015625000000000, -4031596.935974121093750, -37694.328483581542969, -45205.022956848144531,
                                     -4002145.502380371093750,    4939377.839416503906250,    520119.281620025634766,   -4915594.132415771484375,  -4031596.935974121093750,  2559661.786621093750000,  -6032.366791486740112, -15410.097655296325684,
                                       -37657.435111999511719,     -35930.951557159423828,     21563.002219676971436,      36549.278554916381836,    -37694.328483581542969,    -6032.366791486740112,  68673.615665435791016, 125793.606994628906250,
                                       -45365.145576477050781,     -91664.257667541503906,     43154.187518835067749,      93432.122383117675781,    -45205.022956848144531,   -15410.097655296325684, 125793.606994628906250, 256536.000000000000000};
#elif ROX_USE_SSE
   // Ground truth when computation is with SSE vectorisation
   Rox_Double Lte_grt_data[8*1] = {  -1207748.656311035156250,      53628.135875463485718,     38081.840834140777588,     -52112.507019519805908,  -1213166.699935913085938,     9499.710330098867416,    111.523392230272293,    167.740457236766815 };
   Rox_Double LtL_grt_data[8*8] = { 695430590.468750000000000,  -26945947.512207031250000, -19246888.124389648437500,   26334663.435058593750000, 698185248.531250000000000, -4002145.428649902343750, -37657.432540893554688, -45365.148193359375000,
                                    -26945947.512207031250000,  599652088.156250000000000,  -4674327.300659179687500, -601234362.718750000000000, -26854781.874389648437500,  4939377.755699157714844, -35930.940479755401611, -91664.258033752441406,
                                    -19246888.124389648437500,   -4674327.300659179687500,   4375429.356201171875000,    4707977.160949707031250, -19365838.732055664062500,   520119.288879394531250,  21563.001874685287476,  43154.187577009201050,
                                     26334663.435058593750000, -601234362.718750000000000,   4707977.160949707031250,  602830954.085937500000000,  26238041.262695312500000, -4915594.067916870117188,  36549.292445182800293,  93432.130016326904297,
                                    698185248.531250000000000,  -26854781.874389648437500, -19365838.732055664062500,   26238041.262695312500000, 700960620.718750000000000, -4031596.946228027343750, -37694.328281402587891, -45205.024131774902344,
                                     -4002145.428649902343750,    4939377.755699157714844,    520119.288879394531250,   -4915594.067916870117188,  -4031596.946228027343750,  2559661.784179687500000,  -6032.366896152496338, -15410.097280979156494,
                                       -37657.432540893554688,     -35930.940479755401611,     21563.001874685287476,      36549.292445182800293,    -37694.328281402587891,    -6032.366896152496338,  68673.614273071289062, 125793.606018066406250,
                                       -45365.148193359375000,     -91664.258033752441406,     43154.187577009201050,      93432.130016326904297,    -45205.024131774902344,   -15410.097280979156494, 125793.606018066406250, 256536.000000000000000 };
#elif ROX_USE_NEON
   // Ground truth when computation is with NEON vectorisation
   Rox_Double Lte_grt_data[8 * 1] = { -1207748.652256011962891,     53628.135094031691551,     38081.842041997238994,     -52112.507814168930054,  -1213166.695793151855469,     9499.711172372102737,    111.523373540840112,   167.740370392799377 };
   Rox_Double LtL_grt_data[8*8] = { 695430583.220703125000000,  -26945947.684753417968750, -19246889.294296264648438,   26334663.345264434814453, 698185241.736328125000000, -4002145.575977325439453, -37657.432570755481720, -45365.148647308349609,
                                    -26945947.684753417968750,  599652085.148437500000000,  -4674327.413825988769531, -601234360.166015625000000, -26854782.013092041015625,  4939378.090474128723145, -35930.941703915596008, -91664.249553680419922,
                                    -19246889.294296264648438,   -4674327.413825988769531,   4375429.423400878906250,    4707977.307666778564453, -19365839.735168457031250,   520119.296621799468994,  21563.002072989940643,  43154.187677532434464,
                                     26334663.345264434814453, -601234360.166015625000000,   4707977.307666778564453,  602830950.539062500000000,  26238041.467956542968750, -4915594.373625278472900,  36549.291412234306335,  93432.124134778976440,
                                    698185241.736328125000000,  -26854782.013092041015625, -19365839.735168457031250,   26238041.467956542968750, 700960611.472656250000000, -4031597.277942657470703, -37694.329265356063843, -45205.024638414382935,
                                     -4002145.575977325439453,    4939378.090474128723145,    520119.296621799468994,   -4915594.373625278472900,  -4031597.277942657470703,  2559661.820022583007812,  -6032.366973754018545, -15410.097876310348511,
                                       -37657.432570755481720,     -35930.941703915596008,     21563.002072989940643,      36549.291412234306335,    -37694.329265356063843,    -6032.366973754018545,  68673.614438056945801, 125793.605728149414062,
                                       -45365.148647308349609,     -91664.249553680419922,     43154.187677532434464,      93432.124134778976440,    -45205.024638414382935,   -15410.097876310348511, 125793.605728149414062, 256536.000000000000000 };
#else
   // Ground truth when computation is with ANSI C
   // Rox_Double Lte_grt_data[8*1] = {  -1208193.115196403115988,      53656.530413264845265,     38075.530933013702452,     -52141.530185661737050,  -1213612.517704645870253,     9546.446496904649393,    111.086555865532773,    166.224138319492340 };
   // Rox_Double LtL_grt_data[8*8] = { 695717673.129885077476501,  -26958337.964016653597355, -19244434.746542617678642,   26347297.642187539488077, 698473255.251480340957642, -4031859.960156377870589, -37438.264669577569293, -44624.052524566650391,
   //                                  -26958337.964016653597355,  599821048.862501144409180,  -4689925.977472108788788, -601404873.291644454002380, -26866740.795861084014177,  4942578.470288009382784, -35791.390738664500532, -91068.838272094726562,
   //                                  -19244434.746542617678642,   -4689925.977472108788788,   4377895.420797899365425,    4723821.103199382312596, -19363424.595922388136387,   519443.153673445631284,  21562.325600060856232,  43137.228213179769227,
   //                                   26347297.642187539488077, -601404873.291644454002380,   4723821.103199382312596,  603003037.155323147773743,  26250240.065790329128504, -4918862.014753805473447,  36409.664451101576560,  92835.007801529922290,
   //                                  698473255.251480340957642,  -26866740.795861084014177, -19363424.595922388136387,   26250240.065790329128504, 701249560.012359619140625, -4061357.429531683679670, -37470.158980999462074, -44449.452656173714786,
   //                                   -4031859.960156377870589,    4942578.470288009382784,    519443.153673445631284,   -4918862.014753805473447,  -4061357.429531683679670,  2563215.459786897990853,  -6036.860260198418473, -15434.944014471024275,
   //                                     -37438.264669577569293,     -35791.390738664500532,     21562.325600060856232,      36409.664451101576560,    -37470.158980999462074,    -6036.860260198418473,  68755.069524216902209, 125985.565309941768646,
   //                                     -44624.052524566650391,     -91068.838272094726562,     43137.228213179769227,      92835.007801529922290,    -44449.452656173714786,   -15434.944014471024275, 125985.565309941768646, 257040.000000000000000 };

   Rox_Double Lte_grt_data[8*1] = {  -1213361.411007273010910, 53774.507158934145991, 38588.617429641242779, -52238.218780158189475, -1218830.819406633032486, 9771.278034221753842, 133.365764717407529, 217.754214555025101 };

   Rox_Double LtL_grt_data[8*8] = { 698329920.494538545608521, -27015729.023011006414890, -19503788.387211974710226, 26397301.902524508535862, 701110779.465198159217834, -4111332.430630113463849, -45748.629437288094778, -65670.346145629882812,                                                                                             
-27015729.023011006414890, 600313402.106603145599365, -4680709.752970546483994, -601898413.834343552589417, -26925030.076592445373535, 4991026.611825047060847, -35676.847760692493466, -90763.646469116210938,                                                                                             
-19503788.387211974710226, -4680709.752970546483994, 4404878.275738031603396, 4715352.775364154949784, -19625408.010943494737148, 527728.174067809479311, 22388.419493203360616, 45228.206391500549216,                                                                                                     
26397301.902524508535862, -601898413.834343552589417, 4715352.775364154949784, 603497824.619994878768921, 26301069.657152779400349, -4966880.803726577199996, 36325.090805433217611, 92600.688139821999357,                                                                                                 
701110779.465198159217834, -26925030.076592445373535, -19625408.010943494737148, 26301069.657152779400349, 703912617.375411033630371, -4141637.365882639773190, -45861.035630602840683, -65699.534963878919370,                                                                                             
-4111332.430630113463849, 4991026.611825047060847, 527728.174067809479311, -4966880.803726577199996, -4141637.365882639773190, 2573450.995604545343667, -5716.849220911642988, -14671.688972696661949,                                                                                                      
-45748.629437288094778, -35676.847760692493466, 22388.419493203360616, 36325.090805433217611, -45861.035630602840683, -5716.849220911642988, 68827.859625959739788, 126174.678714901208878,                                                                                                                
-65670.346145629882812, -90763.646469116210938, 45228.206391500549216, 92600.688139821999357, -65699.534963878919370, -14671.688972696661949, 126174.678714901208878, 257550.000000000000000 };

#endif

   Rox_Double tra[3] = {0.002, 0.0, 0.0};

   // Rox_Double tra[3] = {0.0, 0.0, 0.0};

   // Define timer to measure performances
   Rox_Double time_ms = 0.0, total_time = 0.0;

#ifdef DEBUG
   // Small number of tests for slow debug (e.g. with valgrind)
   Rox_Sint nb_tests = 1;
#else
   // High number of tests for measuring average performance in release
   Rox_Sint nb_tests = 1;//1000;
#endif

   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);

   Rox_Timer timer = NULL;

   // Define timer object
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_MatUT3 K = NULL;
   error = rox_matut3_new ( &K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_build_calibration_matrix ( K, FU, FV, CU, CV );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_matut3_build_calibration_matrix ( K, 1, 1, 0, 0 );
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_MatSE3 cTr = NULL;
   error = rox_matse3_new ( &cTr );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_set_translation ( cTr, tra );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

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

   // error = rox_array2d_float_from_uchar ( Ir, Ir_uchar );
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Image Ic_uchar = NULL;
   error = rox_image_new_read_pgm ( &Ic_uchar, IMG_REF_PATH );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Ic = NULL;
   error = rox_array2d_float_new ( &Ic, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_from_uchar_normalize ( Ic, Ic_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_array2d_float_from_uchar( Ic, Ic_uchar );
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

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

   Rox_Array2D_Float Zi  = NULL;
   error = rox_array2d_float_new ( &Zi, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_fillval ( Zi, 1.0 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Ziu = NULL;
   error = rox_array2d_float_new ( &Ziu, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_fillval ( Ziu, 0.0f );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Ziv = NULL;
   error = rox_array2d_float_new ( &Ziv, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_fillval ( Ziv, 0.0f );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Iw = NULL;
   error = rox_array2d_float_new ( &Iw, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Warp the current image
   Rox_MeshGrid2D_Float grid = NULL;
   error = rox_meshgrid2d_float_new ( &grid, rows, cols ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_warp_grid_float_matse3_z_float ( grid, Im, Zi, cTr, K, K );
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

   Rox_Array2D_Float Iu = NULL;
   error = rox_array2d_float_new ( &Iu, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Fill with zeros since borders may be ignored in gradient computation
   error = rox_array2d_float_fillzero ( Iu );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Iv = NULL;
   error = rox_array2d_float_new ( &Iv, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Fill with zeros since borders may be ignored in gradient computation
   error = rox_array2d_float_fillzero ( Iv );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error =  rox_array2d_float_basegradient ( Iu, Iv, Gm, Ia, Im );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // rox_array2d_float_print(Iu);
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

   Rox_Matrix LtL = NULL;
   error = rox_matrix_new ( &LtL, 8, 8 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Matrix Lte = NULL;
   error = rox_matrix_new ( &Lte, 8, 1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // rox_array2d_float_save("inter_Ir_zi.txt", Ir);
   // rox_array2d_float_save("inter_Iw_zi.txt", Iw);
   // rox_array2d_float_save("inter_Iu_zi.txt", Iu);
   // rox_array2d_float_save("inter_Iv_zi.txt", Iv);
   // rox_array2d_float_save("inter_Ia_zi.txt", Ia);
   // rox_array2d_float_save("inter_Id_zi.txt", Id);
   // rox_array2d_uint_save ("inter_Gm_zi.txt", Gm);

   for ( Rox_Sint i = 0; i < nb_tests; ++i )
   {
      rox_timer_start ( timer );

      error = rox_linsys_texture_matse3_light_affine_model3d_zi ( LtL, Lte, K, cTr, Zi, Ziu, Ziv, Iu, Iv, Id, Ia, Gm );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      rox_timer_stop ( timer );
      rox_timer_get_elapsed_ms ( &time_ms, timer );
      total_time += time_ms;
   }

   ROX_TEST_MESSAGE("mean time spent to compute interaction matrix of a " + std::to_string(cols) + " x " + std::to_string(rows) + " image = " + std::to_string(total_time/nb_tests));

   rox_matrix_print ( Lte );
   rox_matrix_print ( LtL );

   Rox_Sint valid_pixels = 0;
   error = rox_imask_count_valid ( &valid_pixels, Gm );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("valid_pixels = " + std::to_string(valid_pixels));

   error = rox_array2d_double_difference_l2_norm ( &l2_error, Lte_grt, Lte );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error Lte = " + std::to_string(l2_error));

   ROX_TEST_CHECK_CLOSE ( l2_error, 0.0, 1 );

   error = rox_array2d_double_difference_l2_norm ( &l2_error, LtL_grt, LtL );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error LtL = " + std::to_string(l2_error));

   ROX_TEST_CHECK_CLOSE ( l2_error, 0.0, 10 );

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

   rox_matrix_print ( solution );

   //----- delete

   error = rox_matrix_del ( &solution );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_del ( &K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del ( &cTr );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Zi );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Ziu );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Ziv );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del ( &Ir_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del ( &Ic_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_del ( &Im );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_del ( &Gm );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Ir );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Ic );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Iu );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Iv );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Ia );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Id );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Iw );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_meshgrid2d_float_del ( &grid ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &LtL );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &Lte );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &LtL_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &Lte_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &LtL_inv );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_del ( &timer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
