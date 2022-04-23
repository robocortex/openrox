//==============================================================================
//
//    OPENROX   : File test_vvs_se3_so3z_so3z.cpp
//
//    Contents  : Tests for vvs_se3_so3z_so3z.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

// ====== INCLUDED HEADERS   ===================================================

#include <openrox_tests.hpp>

extern "C"
{
   #include <generated/objset_matse3.h>   
   #include <generated/objset_matse3_struct.h>

   #include <baseproc/geometry/point/point3d_tools.h>
   #include <core/indirect/euclidean/vvs_se3_so3z_so3z.h>
   #include <inout/geometry/point/objset_dynvec_point2d_print.h>
   #include <inout/geometry/point/objset_dynvec_point3d_print.h>
}

// ====== INTERNAL MACROS    ===================================================

ROX_TEST_SUITE_BEGIN(vvs_se3_so3z_so3z)

// ====== INTERNAL TYPESDEFS ===================================================

// ====== INTERNAL DATATYPES ===================================================

// ====== INTERNAL VARIABLES ===================================================

// ====== INTERNAL FUNCTDEFS ===================================================

// ====== INTERNAL FUNCTIONS ===================================================

// ====== EXPORTED FUNCTIONS ===================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_vvs_se3_so3z_so3z)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Double model_size_x = 0.025333;//1.0; 
   Rox_Double model_size_y = 0.025333;//1.0;

   Rox_Double cTo_data[16] = { 0.999999990082018,  0.000018737298023, -0.000139588244254, -0.156384858100393, 
                              -0.000018727314191,  0.999999997266774,  0.000071524405530, -0.078990632966740, 
                               0.000139589584047, -0.000071521790707,  0.999999987699691,  1.007845834185510, 
                               0.0              ,  0.0              ,  0.0              ,  1.0              };

   Rox_Double oTb_1_data[16] = {1.0, -0.0, 0.0, -0.0, 0.0, 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};

   Rox_Double oTb_2_data[16] = { 0.999999999747778, -0.000022459852279,  0.0, 0.313171310637918, 
                                 0.000022459852279,  0.999999999747778,  0.0, 0.157590980709245, 
                                 0.0              ,  0.0              ,  1.0, 0.0              , 
                                 0.0              ,  0.0              ,  0.0, 1.0              };

   Rox_MatSE3 cTo = NULL; 
   Rox_MatSE3 oTb_1 = NULL; 
   Rox_MatSE3 oTb_2 = NULL; 
   Rox_ObjSet_MatSE3 oTb = NULL;                // Should be an objset since there are many possible planes
   
   Rox_Matrix Kc = NULL;

   Rox_ObjSet_DynVec_Point2D_Double pr = NULL;  // Should be a dynvec since there are many possible planes
   Rox_ObjSet_DynVec_Point3D_Double mb = NULL;  // Should be a dynvec since there are many possible planes
   
   Rox_DynVec_Point3D_Double mb_1 = NULL;
   Rox_DynVec_Point3D_Double mb_2 = NULL;

   Rox_DynVec_Point2D_Double pr_1 = NULL;
   Rox_DynVec_Point2D_Double pr_2 = NULL;

   // Define the models of the points on the planes
   error = rox_objset_dynvec_point3d_double_new ( &mb, 2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_new ( &mb_1, 4);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_append_rectangle(mb_1, model_size_x, model_size_y );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Append set of points mb_1 to objset mb
   error = rox_objset_dynvec_point3d_double_append( mb, mb_1 ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_new ( &mb_2, 4);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_append_rectangle(mb_2,  model_size_x, model_size_y );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Append set of points mb_2 to objset mb
   error = rox_objset_dynvec_point3d_double_append( mb, mb_2 ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Display sets for debug
   error = rox_objset_dynvec_point3d_double_print(mb);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define the camera intrinsic parameters
   error = rox_matrix_new(&Kc, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double fu = 192000.0/38.0; 
   Rox_Double fv = 192000.0/38.0;  
   Rox_Double cu = (1920.0-1.0)/2.0;
   Rox_Double cv = (1080.0-1.0)/2.0;

   error = rox_matrix_build_calibration_matrix(Kc, fu, fv, cu, cv);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define the input image points for each plane
   error = rox_objset_dynvec_point2d_double_new ( &pr, 2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point2d_double_new ( &pr_1, 4);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point2d_double_new ( &pr_2, 4);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Point2D_Double_Struct pts;

   // Add points of set 1 to the dynvec pr_1
   pts.u = 111.993238; pts.v = 79.996268;

   error = rox_dynvec_point2d_double_append( pr_1, &pts ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   pts.u = 238.997753; pts.v = 79.995502;

   error = rox_dynvec_point2d_double_append( pr_1, &pts ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   pts.u = 238.998838; pts.v = 206.996447;

   error = rox_dynvec_point2d_double_append( pr_1, &pts ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   pts.u = 111.994094; pts.v = 206.997659;

   error = rox_dynvec_point2d_double_append( pr_1, &pts ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Append set of points pr_1 to objset pr
   error = rox_objset_dynvec_point2d_double_append( pr, pr_1 ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Add points of set 1 to the dynvec pr_2
   pts.u = 1682.004355; pts.v = 870.004067;

   error = rox_dynvec_point2d_double_append( pr_2, &pts ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   pts.u = 1809.001625; pts.v = 870.004590;

   error = rox_dynvec_point2d_double_append( pr_2, &pts ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   pts.u = 1808.997857; pts.v = 996.999979;

   error = rox_dynvec_point2d_double_append( pr_2, &pts ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   pts.u = 1682.0010804; pts.v = 996.999437;

   error = rox_dynvec_point2d_double_append( pr_2, &pts ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Append set of points pr_2 to objset pr
   error = rox_objset_dynvec_point2d_double_append( pr, pr_2 ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Display sets for debug
   error = rox_objset_dynvec_point2d_double_print(pr);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Define and intit poses
   error = rox_matse3_new( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_set_data( cTo, cTo_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_new( &oTb_1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_set_data( oTb_1, oTb_1_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_new( &oTb_2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_set_data( oTb_2, oTb_2_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_objset_matse3_new( &oTb, 2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_objset_matse3_append( oTb, oTb_1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_objset_matse3_append( oTb, oTb_2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Compute optimal poses
   error = rox_vvs_points_pix_se3_so3z_r2 ( cTo, oTb, Kc, pr, mb);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Display results

   error = rox_matse3_print( cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matse3_print( oTb->data[1] );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Delete memory
   error = rox_objset_dynvec_point3d_double_del ( &mb );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Do not delete points, they are deleted in objset

   // error = rox_dynvec_point3d_double_del ( &mb_1 );
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_dynvec_point3d_double_del ( &mb_2 );
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_objset_dynvec_point2d_double_del( &pr );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Do not delete points, they are deleted in objset

   // error = rox_dynvec_point2d_double_del ( &pr_1 );
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_dynvec_point2d_double_del ( &pr_2 );
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Do not delete matrices, they are deleted in objset

   // error = rox_matse3_del( &oTb_1 );
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_matse3_del( &oTb_2 );
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_objset_matse3_del( &oTb );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matrix_del( &Kc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}

ROX_TEST_SUITE_END()
