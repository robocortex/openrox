//==============================================================================
//
//    OPENROX   : File test_linsys_texture_matse3_model3d_zi.cpp
//
//    Contents  : Tests for linsys_texture_matse3_model3_zi.c
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
   #include <baseproc/geometry/pixelgrid/warp_grid_matse3_zi.h>
   #include <baseproc/geometry/pixelgrid/warp_grid_matse3_z.h>
   #include <system/time/timer.h>
   #include <inout/numeric/array2d_print.h>
   #include <inout/numeric/ansi_array_print.h>
   #include <inout/numeric/array2d_save.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN ( linsys_texture_matse3_model3d_zi )

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

#ifdef TEST_ROW
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_linsys_row_texture_matse3_model3d)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double Lk_grt_data[6] = { 18947.368421052630, 190350.877192982443, -43754.385964912275, -199101.754385964887, 27698.245614035091, 34280.701754385962 }; 
   Rox_Double Lk[6];
   Rox_Double ur = 520; 
   Rox_Double vr = 440; 
   Rox_Double Iu =  20; 
   Rox_Double Iv = 190; 

   Rox_Double zir = 1.0;
   Rox_Double ziur =  5.555555555555556e-04; 
   Rox_Double zivr = -1.851851851851852e-04;
   Rox_Matrix tau = NULL; 

   Rox_MatUT3 K = NULL;

   error = rox_matut3_new ( &K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_build_calibration_matrix ( K, 1000.0, 1000.0, 320.0, 240.0 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_new ( &tau, 3, 1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_set_value ( tau, 0, 0, -0.1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_set_value ( tau, 1, 0,  0.0 );   
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_set_value ( tau, 2, 0, 0.0 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_linsys_row_texture_matse3_model3d ( Lk, ur, vr, Iu, Iv, zir, ziur, zivr,K, tau );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("the row of the Jacobian is : %12.12f %12.12f %12.12f %12.12f %12.12f %12.12f \n", Lk[0], Lk[1], Lk[2], Lk[3], Lk[4], Lk[5] );

   ROX_TEST_CHECK_CLOSE (Lk[0], Lk_grt_data[0], 1e-12);
   ROX_TEST_CHECK_CLOSE (Lk[1], Lk_grt_data[1], 1e-12);
   ROX_TEST_CHECK_CLOSE (Lk[2], Lk_grt_data[2], 1e-12);
   ROX_TEST_CHECK_CLOSE (Lk[3], Lk_grt_data[3], 1e-12);
   ROX_TEST_CHECK_CLOSE (Lk[4], Lk_grt_data[4], 1e-12);
   ROX_TEST_CHECK_CLOSE (Lk[5], Lk_grt_data[5], 1e-12);
}
#endif

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_linsys_texture_matse3_model3d_zi )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double l2_error = 0.0;

#if defined(ROX_USE_AVX) || defined(ROX_USE_SSE)
   // Ground truth when computation is with AVX vectorisation
   Rox_Double Lte_grt_data[6*1] = {   -1207748.655343435704708,      53628.134929729501891,     38081.843144755017420,     -52112.507076784218953,  -1213166.698898292612284,     9499.710575722745489 };
   Rox_Double LtL_grt_data[6*6] = {  695430592.164201617240906,  -26945947.080954633653164, -19246889.122074462473392,   26334662.659787125885487, 698185254.162421703338623, -4002145.406793540809304,                                    
                                     -26945947.080954633653164,  599652089.835969924926758,  -4674327.117135760374367, -601234363.532090663909912, -26854781.952161181718111,  4939377.775184858590364,                                    
                                     -19246889.122074462473392,   -4674327.117135760374367,   4375429.473149051889777,    4707977.082953241653740, -19365839.747671257704496,   520119.292373985634185,                                          
                                      26334662.659787125885487, -601234363.532090663909912,   4707977.082953241653740,  602830952.290926337242126,  26238041.021234769374132, -4915594.105702902190387,                                      
                                     698185254.162421703338623,  -26854781.952161181718111, -19365839.747671257704496,   26238041.021234769374132, 700960627.532047033309937, -4031597.086494256276637,                                    
                                      -4002145.406793540809304,    4939377.775184858590364,    520119.292373985634185,   -4915594.105702902190387,  -4031597.086494256276637,  2559661.782032142393291 }; 
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

   Rox_Double tra[3] = {0.002, 0.0, 0.0};

   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);

   Rox_MatUT3 K = NULL;
   error = rox_matut3_new ( &K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_build_calibration_matrix ( K, FU, FV, CU, CV );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

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
   Rox_MeshGrid2D_Float meshgrid2d = NULL;
   error = rox_meshgrid2d_float_new ( &meshgrid2d, rows, cols ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_warp_grid_float_matse3_zi_float ( meshgrid2d, Im, Zi, cTr, K, K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Remap with bilinear interpolation
   error = rox_remap_bilinear_omo_float_to_float ( Iw, Im, Ic, meshgrid2d ); 
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

   // rox_array2d_float_save("inter_Ir.txt", Ir);
   // rox_array2d_float_save("inter_Iw.txt", Iw);
   // rox_array2d_float_save("inter_Iu.txt", Iu);
   // rox_array2d_float_save("inter_Iv.txt", Iv);
   // rox_array2d_float_save("inter_Ia.txt", Ia);
   // rox_array2d_float_save("inter_Id.txt", Id);
   // rox_array2d_uint_save ("inter_Gm.txt", Gm);

   error = rox_linsys_texture_matse3_model3d_zi ( LtL, Lte, Iu, Iv, Id, Zi, Ziu, Ziv, K, cTr, Gm );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matrix_print(Lte);
   rox_matrix_print(LtL);
   
   rox_array2d_double_save (RESULT_PATH"/Lte_6.txt", Lte);
   rox_array2d_double_save (RESULT_PATH"/LtL_6.txt", LtL);

   Rox_Sint valid_pixels = 0;
   error = rox_imask_count_valid(&valid_pixels, Gm);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("valid_pixels = %d \n", valid_pixels);

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

   error = rox_array2d_float_del ( &Ir );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_float_del ( &Ic );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_del ( &Im );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_del ( &Gm );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Iu );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Iv );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Iw );
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

   error = rox_matrix_del ( &LtL_inv );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &solution );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &Lte_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &LtL_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Zi );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_float_del ( &Ziu );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &Ziv );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_del ( &K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_meshgrid2d_float_del ( &meshgrid2d );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
