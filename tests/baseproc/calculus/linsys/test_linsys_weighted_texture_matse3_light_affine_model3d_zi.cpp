//==============================================================================
//
//    OPENROX   : File test_linsys_weighted_texture_matse3_light_affine_model3d_zi.cpp
//
//    Contents  : Tests for linsys_weighted_texture_matse3_light_affine_model3d_zi.c
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
   #include <baseproc/calculus/linsys/linsys_texture_matse3_model3d_zi.h>
   #include <baseproc/calculus/linsys/linsys_texture_matse3_light_affine_model3d_zi.h>
   #include <baseproc/calculus/linsys/linsys_weighted_texture_matse3_light_affine_model3d_zi.h>
   #include <baseproc/geometry/pixelgrid/warp_grid_matse3_z.h>
   #include <baseproc/geometry/pixelgrid/warp_grid_matse3_zi.h>
   #include <system/time/timer.h>
   #include <inout/numeric/array2d_print.h>
   #include <inout/numeric/ansi_array_print.h>
   #include <inout/numeric/array2d_save.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =====================================================
ROX_TEST_SUITE_BEGIN ( test_linsys_weighted_texture_matse3_light_affine_model3d_zi )

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

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_linsys_weighted_texture_matse3_light_affine_model3d_zi )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double l2_error = 0.0;

#if defined(ROX_USE_AVX) || defined(ROX_USE_SSE)
   // Ground truth when computation is with AVX or SSE vectorisation
   Rox_Double Lte_grt_data[8*1] = {  -1207748.655343435704708,      53628.134929729501891,     38081.843144755017420,     -52112.507076784218953,  -1213166.698898292612284,     9499.710575722745489,    111.523390312805844,    167.740458205342293 };
   Rox_Double LtL_grt_data[8*8] = { 695430592.164201617240906,  -26945947.080954633653164, -19246889.122074462473392,   26334662.659787125885487, 698185254.162421703338623, -4002145.406793540809304, -37657.433869173532003, -45365.147571563720703, 
                                    -26945947.080954633653164,  599652089.835969924926758,  -4674327.117135760374367, -601234363.532090663909912, -26854781.952161181718111,  4939377.775184858590364, -35930.945073196169687, -91664.265327453613281,
                                    -19246889.122074462473392,   -4674327.117135760374367,   4375429.473149051889777,    4707977.082953241653740, -19365839.747671257704496,   520119.292373985634185,  21563.001735185414873,  43154.187098818692903,
                                     26334662.659787125885487, -601234363.532090663909912,   4707977.082953241653740,  602830952.290926337242126,  26238041.021234769374132, -4915594.105702902190387,  36549.286004192836117,  93432.120811882181442,
                                    698185254.162421703338623,  -26854781.952161181718111, -19365839.747671257704496,   26238041.021234769374132, 700960627.532047033309937, -4031597.086494256276637, -37694.318240693108237, -45205.004911272830213, 
                                     -4002145.406793540809304,    4939377.775184858590364,    520119.292373985634185,   -4915594.105702902190387,  -4031597.086494256276637,  2559661.782032142393291,  -6032.366715085511714, -15410.097658146172762, 
                                       -37657.433869173532003,     -35930.945073196169687,     21563.001735185414873,      36549.286004192836117,    -37694.318240693108237,    -6032.366715085511714,  68673.615207268405356, 125793.607143566012383, 
                                       -45365.147571563720703,     -91664.265327453613281,     43154.187098818692903,      93432.120811882181442,    -45205.004911272830213,   -15410.097658146172762, 125793.607143566012383, 256536.000000000000000 };
#elif ROX_USE_NEON
   // Ground truth when computation is with NEON vectorisation
   Rox_Double Lte_grt_data[8*1] = {  -1207748.649630399886519,      53628.135098390259373,     38081.843632873235038,     -52112.507299648736080,  -1213166.693142144475132,     9499.711165036183957,    111.523369771498949,    167.740374743938446 };
   Rox_Double LtL_grt_data[8*8] = { 695430582.823000073432922,  -26945947.170727744698524, -19246889.948441371321678,   26334662.767303619533777, 698185244.746395230293274, -4002145.585729388054460, -37657.433869173270068, -45365.147571563720703, 
                                    -26945947.170727744698524,  599652087.295029163360596,  -4674327.163945515640080, -601234360.982751011848450, -26854782.046274051070213,  4939378.010390540584922, -35930.944689844603999, -91664.264240264892578, 
                                    -19246889.948441371321678,   -4674327.163945515640080,   4375429.391575847752392,    4707977.131689601577818, -19365840.581442464143038,   520119.278970292245504,  21563.001747933787556,  43154.187142013506673,
                                     26334662.767303619533777, -601234360.982751011848450,   4707977.131689601577818,  602830949.732856273651123,  26238041.133267775177956, -4915594.338493820279837,  36549.285620888680569,  93432.119725711163483, 
                                    698185244.746395230293274,  -26854782.046274051070213, -19365840.581442464143038,   26238041.133267775177956, 700960618.040518164634705, -4031597.266650577075779, -37694.318239082640503, -45205.004903923720121,
                                     -4002145.585729388054460,    4939378.010390540584922,    520119.278970292245504,   -4915594.338493820279837,  -4031597.266650577075779,  2559661.724435267038643,  -6032.366750403079095, -15410.097758151590824, 
                                       -37657.433869173270068,     -35930.944689844603999,     21563.001747933787557,      36549.285620888680569,    -37694.318239082640507,    -6032.366750403079095,  68673.615215291574714, 125793.607181131839752,
                                       -45365.147571563720703,     -91664.264240264892578,     43154.187142013506673,      93432.119725711163483,    -45205.004903923720121,   -15410.097758151590824, 125793.607181131839752, 256536.000000000000000 };
#else
   // Ground truth when computation is with ANSI C
   // Rox_Double Lte_grt_data[8*1] = {  -1208193.115196403115988,      53656.530413264845265,     38075.530933013702452,     -52141.530185661737050,  -1213612.517704645870253,     9546.446496904649393,    111.086555865532773,    166.224138319492340};
   // Rox_Double LtL_grt_data[8*8] = { 695717673.129885077476501,  -26958337.964016653597355, -19244434.746542617678642,   26347297.642187539488077, 698473255.251480340957642, -4031859.960156377870589, -37438.264669577569293, -44624.052524566650391, 
   //                                  -26958337.964016653597355,  599821048.862501144409180,  -4689925.977472108788788, -601404873.291644454002380, -26866740.795861084014177,  4942578.470288009382784, -35791.390738664500532, -91068.838272094726562, 
   //                                  -19244434.746542617678642,   -4689925.977472108788788,   4377895.420797899365425,    4723821.103199382312596, -19363424.595922388136387,   519443.153673445631284,  21562.325600060856232,  43137.228213179769227, 
   //                                   26347297.642187539488077, -601404873.291644454002380,   4723821.103199382312596,  603003037.155323147773743,  26250240.065790329128504, -4918862.014753805473447,  36409.664451101576560,  92835.007801529922290, 
   //                                  698473255.251480340957642,  -26866740.795861084014177, -19363424.595922388136387,   26250240.065790329128504, 701249560.012359619140625, -4061357.429531683679670, -37470.158980999462074, -44449.452656173714786, 
   //                                   -4031859.960156377870589,    4942578.470288009382784,    519443.153673445631284,   -4918862.014753805473447,  -4061357.429531683679670,  2563215.459786897990853,  -6036.860260198418473, -15434.944014471024275, 
   //                                     -37438.264669577569293,     -35791.390738664500532,     21562.325600060856232,      36409.664451101576560,    -37470.158980999462074,    -6036.860260198418473,  68755.069524216902209, 125985.565309941768646, 
   //                                     -44624.052524566650391,     -91068.838272094726562,     43137.228213179769227,      92835.007801529922290,    -44449.452656173714786,   -15434.944014471024275, 125985.565309941768646, 257040.000000000000000};

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
   //Rox_Double tra[3] = {0.0, 0.0, 0.0};

   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);

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

   // error = rox_array2d_float_from_uchar ( Ic, Ic_uchar );
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
   
   Rox_Array2D_Float weight = NULL;
   error = rox_array2d_float_new ( &weight, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_fillval ( weight, 1.0f );
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

   // rox_array2d_float_save("inter_Ir_wzi.txt", Ir);
   // rox_array2d_float_save("inter_Iw_wzi.txt", Iw);
   // rox_array2d_float_save("inter_Iu_wzi.txt", Iu);
   // rox_array2d_float_save("inter_Iv_wzi.txt", Iv);
   // rox_array2d_float_save("inter_Ia_wzi.txt", Ia);
   // rox_array2d_float_save("inter_Id_wzi.txt", Id);
   // rox_array2d_uint_save ("inter_Gm_wzi.txt", Gm);

   error = rox_linsys_weighted_texture_matse3_light_affine_model3d_zi ( LtL, Lte, K, cTr, Zi, Ziu, Ziv, Iu, Iv, Id, Ia, weight, Gm );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matrix_print(Lte);
   rox_matrix_print(LtL);
   
   rox_array2d_double_save (RESULT_PATH"/Lte_3.txt", Lte);
   rox_array2d_double_save (RESULT_PATH"/LtL_3.txt", LtL);

   Rox_Sint valid_pixels = 0;
   error = rox_imask_count_valid(&valid_pixels, Gm);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("valid_pixels = %d \n", valid_pixels);

   error = rox_array2d_double_difference_l2_norm ( &l2_error, Lte_grt, Lte );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error Lte = %0.12f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);
   
   error = rox_array2d_double_difference_l2_norm ( &l2_error, LtL_grt, LtL );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error LtL = %0.12f \n", l2_error);

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

   error = rox_matse3_del ( &cTr );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
