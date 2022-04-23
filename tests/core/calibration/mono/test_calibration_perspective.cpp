//==============================================================================
//
//    OPENROX   : File test_calibration_perspective.cpp
//
//    Contents  : Tests for calibration_perspective.c
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
   #include <baseproc/maths/maths_macros.h>
   #include <baseproc/array/fill/fillunit.h>
   #include <baseproc/array/fill/fillval.h>
   #include <baseproc/geometry/point/point3d_tools.h>
   #include <baseproc/geometry/point/dynvec_point3d_tools.h>
   #include <baseproc/geometry/point/point2d_projection_from_point3d_transform.h>
   #include <baseproc/geometry/transforms/matsl3/sl3normalize.h>
   #include <baseproc/geometry/transforms/transform_tools.h>

   #include <core/calibration/mono/calibration_perspective.h>
   #include <core/calibration/mono/calibration_perspective_struct.h>
   
   #include <inout/numeric/array2d_print.h>
   #include <inout/system/errors_print.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(calibration_perspective)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//! Test if two calibration matrices are equal.
//! (no K1 value must differ by more than epsilon from those of K2)
//! \param [in] K1 the first  calibration matrix
//! \param [in] K2 the second calibration matrix
//! \param [in] epsilon tolerance between each values of the two matrices
//! \return An error code
Rox_ErrorCode equality(Rox_Array2D_Double K1, Rox_Array2D_Double K2, Rox_Double epsilon)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Double k = 0.0;
    Rox_Double rk = 0.0;
    
    if (!K1 || !K2) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    for (Rox_Uint i = 0; i<4; i++)
    {
       for (Rox_Uint j = 0; j<4; j++) 
       {
          rox_array2d_double_get_value(&k, K1, i, j);
          rox_array2d_double_get_value(&rk, K2, i, j);
       
          if (fabs(k-rk) > epsilon)
          {
             rox_log("theorical K = %f \n", k);
             //rox_array2d_double_print(*K1);
                
             rox_log("estimated K : %f \n", rk);
             //rox_array2d_double_print(*K2);
                
             error = ROX_ERROR_ALGORITHM_FAILURE;
             goto function_terminate;
          }
       }
    }
    
function_terminate:
    return error;
}

//! Build a pixel homography using the equation H = Kout*(R - 1/d*n^t*t)*Kin^-1
//! \param [out] homography the result homohgraphy
//! \param [in] pose the pose
//! \param [in] calibration the calibration matrice
//! \return An error code
Rox_ErrorCode rox_transformtools_build_homography_model_to_image(Rox_Array2D_Double homography, Rox_Array2D_Double pose, Rox_Array2D_Double calibration)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double px = 0.0, py = 0.0, u0 = 0.0, v0 = 0.0, s = 0.0;
   Rox_Double ** dh = NULL;
   Rox_Double ** dt = NULL;
   Rox_Double ** dk = NULL;

   if (!homography || !pose || !calibration) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(homography, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
    
   error = rox_array2d_double_check_size(calibration, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
    
   error = rox_array2d_double_check_size(pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );
    
   error = rox_array2d_double_get_data_pointer_to_pointer( &dh, homography);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer( &dt, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer( &dk, calibration);
   ROX_ERROR_CHECK_TERMINATE ( error );

   px = dk[0][0];
   py = dk[1][1];
   u0 = dk[0][2];
   v0 = dk[1][2];
   s  = dk[0][1];
    
   dh[0][0] = px * dt[0][0] + u0 * dt[2][0] + s * dt[1][0];
   dh[0][1] = px * dt[0][1] + u0 * dt[2][1] + s * dt[1][1];
   dh[0][2] = px * dt[0][3] + u0 * dt[2][3] + s * dt[1][3] ;
   dh[1][0] = py * dt[1][0] + v0 * dt[2][0];
   dh[1][1] = py * dt[1][1] + v0 * dt[2][1];
   dh[1][2] = py * dt[1][3] + v0 * dt[2][3];
   dh[2][0] = dt[2][0];
   dh[2][1] = dt[2][1];
   dh[2][2] = dt[2][3];
    
function_terminate:
   return error;
}

//! Initialize the calibration
//! \param  [out] K      The theorical calibration matrix
//! \param  [in]  ret    The calibration t use
//! \param  [in]  nbk    The number of parameters to use later
//! \return An error code
Rox_ErrorCode initialisation(Rox_Array2D_Double * K, Rox_Calibration_Mono_Perspective ret, Rox_Uint nbk)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Double cu = 320.0;
    Rox_Double cv = 240.0;
    Rox_Double f = 100.0;
    Rox_Double theta = 1.0*ROX_PI/2.0;
    Rox_Double s = 0.0;
    Rox_Double r = 1.0;
    Rox_DynVec_Point3D_Double M2 = NULL;
    Rox_Double t1 = 0;
    Rox_Double t2 = 0;
    Rox_Double t3 = 1;
    Rox_DynVec_Point2D_Double points = NULL;
    Rox_Float omega = 0;
    
    error = rox_dynvec_point2d_double_new(&points, 4);
    ROX_ERROR_CHECK_TERMINATE ( error );

    if (!ret) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
    
    if (nbk==0 || nbk>5) 
    { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
    
    error = rox_array2d_double_fillunit(ret->S); 
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_dynvec_point3d_double_new_rectangle(&M2, 0.5, 0.5); 
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_calibration_mono_perspective_set_model_points(ret, M2->data, 4); 
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    switch (nbk) {
        case 1:
            error = rox_array2d_double_set_value(ret->K, 0, 2, cu); 
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_array2d_double_set_value(ret->K, 1, 2, cv); 
            ROX_ERROR_CHECK_TERMINATE ( error );
            break;
        case 2:
            r=2;
            error = rox_array2d_double_set_value(ret->K, 0, 2, cu); 
            ROX_ERROR_CHECK_TERMINATE ( error );
            error = rox_array2d_double_set_value(ret->K, 1, 2, cv); 
            ROX_ERROR_CHECK_TERMINATE ( error );
            break;
        case 4:
            r=2;
            break;
        case 5:
            r=2;
            s = 0-f*cos(theta);
            break;
    }

    error = rox_array2d_double_new(K, 3, 3); 
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_array2d_double_fillunit(*K); 
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_array2d_double_set_value(*K, 0, 0, f); 
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_array2d_double_set_value(*K, 0, 1, s); 
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_array2d_double_set_value(*K, 0, 2, cu); 
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_array2d_double_set_value(*K, 1, 1, f*r); 
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_array2d_double_set_value(*K, 1, 2, cv); 
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    for ( Rox_Sint i=0; i<3; i++)
    {
        omega = (8+i+1)*ROX_PI/100;
        
        Rox_Array2D_Double T = NULL;
        
        error = rox_array2d_double_new(&T, 4,4); 
        ROX_ERROR_CHECK_TERMINATE ( error );
        
        error = rox_array2d_double_set_value(T, 0, 0, cos(omega)); 
        ROX_ERROR_CHECK_TERMINATE ( error );
        error = rox_array2d_double_set_value(T, 0, 1, 0); 
        ROX_ERROR_CHECK_TERMINATE ( error );
        error = rox_array2d_double_set_value(T, 0, 2, sin(omega)); 
        ROX_ERROR_CHECK_TERMINATE ( error );
        error = rox_array2d_double_set_value(T, 0, 3, t1); 
        ROX_ERROR_CHECK_TERMINATE ( error );
        error = rox_array2d_double_set_value(T, 1, 0, 0); 
        ROX_ERROR_CHECK_TERMINATE ( error );
        error = rox_array2d_double_set_value(T, 1, 1, 1); 
        ROX_ERROR_CHECK_TERMINATE ( error );
        error = rox_array2d_double_set_value(T, 1, 2, 0); 
        ROX_ERROR_CHECK_TERMINATE ( error );
        error = rox_array2d_double_set_value(T, 1, 3, t2); 
        ROX_ERROR_CHECK_TERMINATE ( error );
        error = rox_array2d_double_set_value(T, 2, 0, -sin(omega)); 
        ROX_ERROR_CHECK_TERMINATE ( error );
        error = rox_array2d_double_set_value(T, 2, 1, 0); 
        ROX_ERROR_CHECK_TERMINATE ( error );
        error = rox_array2d_double_set_value(T, 2, 2, cos(omega)); 
        ROX_ERROR_CHECK_TERMINATE ( error );
        error = rox_array2d_double_set_value(T, 2, 3, t3); 
        ROX_ERROR_CHECK_TERMINATE ( error );
        error = rox_array2d_double_set_value(T, 3, 0, 0); 
        ROX_ERROR_CHECK_TERMINATE ( error );
        error = rox_array2d_double_set_value(T, 3, 1, 0); 
        ROX_ERROR_CHECK_TERMINATE ( error );
        error = rox_array2d_double_set_value(T, 3, 2, 0); 
        ROX_ERROR_CHECK_TERMINATE ( error );
        error = rox_array2d_double_set_value(T, 3, 3, 1); 
        ROX_ERROR_CHECK_TERMINATE ( error );
        
        error = rox_point3d_double_transform_project(points->data, T, *K, M2->data, 4); 
        ROX_ERROR_CHECK_TERMINATE ( error );
        
        Rox_Array2D_Double homog;
        error = rox_array2d_double_new(&homog, 3, 3); 
        ROX_ERROR_CHECK_TERMINATE ( error );

        error = rox_transformtools_build_homography_model_to_image(homog, T, *K); 
        ROX_ERROR_CHECK_TERMINATE ( error );

        error = rox_matsl3_normalize(homog, homog); 
        ROX_ERROR_CHECK_TERMINATE ( error );
        
        // error = rox_calibration_mono_perspective_add_current_points(ret, points->data, 4); ROX_ERROR_CHECK_TERMINATE ( error );
        error = rox_calibration_mono_perspective_add_measure(ret, homog, points->data, 4); 
        ROX_ERROR_CHECK_TERMINATE ( error );

        error = rox_calibration_mono_perspective_set_model_points(ret, M2->data, 4); 
        ROX_ERROR_CHECK_TERMINATE ( error );
        
        rox_array2d_double_del(&T);
        rox_array2d_double_del(&homog);
    }
    
function_terminate :
    rox_dynvec_point3d_double_del(&M2);
    rox_dynvec_point2d_double_del(&points);
    // rox_array2d_double_del(K); 

    return error;
}

//=== EXPORTED FUNCTIONS ===================================================


//! Test the creation of a calibration object
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_new)
{
    // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Calibration_Mono_Perspective ret = NULL;
    
    // Test NULL pointers 
    error = rox_calibration_mono_perspective_new(NULL);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // Test correct values 
    error = rox_calibration_mono_perspective_new ( &ret );
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Free memory 
    error = rox_calibration_mono_perspective_del ( &ret );
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test the deletion of a calibration object
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_del)
{
    // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Calibration_Mono_Perspective ret = NULL;
    error = rox_calibration_mono_perspective_new(&ret);
    
    // Test NULL pointers 
    error = rox_calibration_mono_perspective_del(NULL);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // Test correct values 
    error = rox_calibration_mono_perspective_del(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test the adding of a homography and its current detected 2D-model to a calibration object
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_add_measure)
{
    // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Calibration_Mono_Perspective ret = NULL;
    Rox_Array2D_Double H = NULL;
    Rox_Array2D_Double H2 = NULL;
    Rox_DynVec_Point2D_Double points = NULL;
    Rox_Point2D_Double_Struct p1;
    Rox_Point2D_Double_Struct p2;
    Rox_Point2D_Double_Struct p3;
    Rox_Point2D_Double_Struct p4;
    Rox_Uint count = 4;
    
    error = rox_calibration_mono_perspective_new(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // homographie initialisation 
    rox_array2d_double_new(&H, 3, 3);
    rox_array2d_double_new(&H2, 3, 3);
    rox_array2d_double_set_value(H2, 0, 0, 3.1767e-01);
    rox_array2d_double_set_value(H2, 0, 1, -1.0957e-16);
    rox_array2d_double_set_value(H2, 0, 2, 1.5055e+01);
    rox_array2d_double_set_value(H2, 1, 0, -3.1502e+00);
    rox_array2d_double_set_value(H2, 1, 1, 4.7047e+00);
    rox_array2d_double_set_value(H2, 1, 2, 1.1291e+01);
    rox_array2d_double_set_value(H2, 2, 0, -1.3126e-02);
    rox_array2d_double_set_value(H2, 2, 1, 1.0312e-18);
    rox_array2d_double_set_value(H2, 2, 2, 4.7047e-02);
    
    // points initialisation 
    rox_dynvec_point2d_double_new(&points,4);
    p1.u = 277.8632; p1.v = 196.120;
    rox_dynvec_point2d_double_append(points,&p1);
    p2.u = 375.7983; p2.v = 181.8945;
    rox_dynvec_point2d_double_append(points,&p2);
    p3.u = 375.7983; p3.v = 298.1055;
    rox_dynvec_point2d_double_append(points,&p3);
    p4.u = 277.8632; p4.v = 283.8791;
    rox_dynvec_point2d_double_append(points,&p4);
    
    Rox_DynVec_Point2D_Double points2 = NULL;
    rox_dynvec_point2d_double_new(&points2,4);
    Rox_Point2D_Double_Struct p5;
    p5.u = 0; p1.v = 0;
    rox_dynvec_point2d_double_append(points2,&p5);
    rox_dynvec_point2d_double_append(points2,&p5);
    rox_dynvec_point2d_double_append(points2,&p5);
    rox_dynvec_point2d_double_append(points2,&p5);
    
    // Test NULL pointers 
    error = rox_calibration_mono_perspective_add_measure(NULL,NULL,NULL,0);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_add_measure(ret,NULL,NULL,0);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_add_measure(NULL,H,NULL,0);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_add_measure(NULL,NULL,points->data,0);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_add_measure(NULL,NULL,NULL,count);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_add_measure(ret,H,NULL,0);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_add_measure(ret,NULL,NULL,count);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_add_measure(NULL,H,points->data,0);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_add_measure(NULL,H,NULL,count);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_add_measure(NULL,NULL,points->data,count);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_add_measure(ret, H, NULL, count);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_add_measure(ret, NULL, points->data,count);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_add_measure(NULL, H, points->data,count);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
	error = rox_array2d_double_fillval(H, 0.0);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	// rox_array2d_double_print(H);

    // Test matrix with null determinant 
    error = rox_calibration_mono_perspective_add_measure(ret, H, points->data, count);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_DETERMINANT_NULL);
    
    // Test wrong numer of points 
    error = rox_calibration_mono_perspective_add_measure(ret,NULL,points->data,0);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);
    
    // Test correct values 
    error = rox_calibration_mono_perspective_add_measure(ret, H2 , points->data, count);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Free memory 
    error = rox_calibration_mono_perspective_del(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    error = rox_array2d_double_del(&H);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    error = rox_array2d_double_del(&H2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    error = rox_dynvec_point2d_double_del(&points);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    error = rox_dynvec_point2d_double_del(&points2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test the adding of a homography to a calibration object
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_add_homography)
{
   // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Calibration_Mono_Perspective ret = NULL;
    Rox_Array2D_Double H = NULL;
    Rox_Array2D_Double H2 = NULL;
    
    error = rox_calibration_mono_perspective_new(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Initialisation 
    rox_array2d_double_new(&H, 3, 3);
    rox_array2d_double_fillunit(H);
    rox_array2d_double_set_value(H, 0, 0, 0);
    rox_array2d_double_set_value(H, 1, 1, 0);
    rox_array2d_double_set_value(H, 2, 2, 0);
    rox_array2d_double_new(&H2, 3, 3);
    rox_array2d_double_set_value(H2, 0, 0, 3.1767e-01);
    rox_array2d_double_set_value(H2, 0, 1, -1.0957e-16);
    rox_array2d_double_set_value(H2, 0, 2, 1.5055e+01);
    rox_array2d_double_set_value(H2, 1, 0, -3.1502e+00);
    rox_array2d_double_set_value(H2, 1, 1, 4.7047e+00);
    rox_array2d_double_set_value(H2, 1, 2, 1.1291e+01);
    rox_array2d_double_set_value(H2, 2, 0, -1.3126e-02);
    rox_array2d_double_set_value(H2, 2, 1, 1.0312e-18);
    rox_array2d_double_set_value(H2, 2, 2, 4.7047e-02);
    
    // Test NULL pointers 
    error = rox_calibration_mono_perspective_add_homography(NULL,H);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_add_homography(ret,NULL);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_add_homography(NULL,NULL);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // Test matrix with null determinant 
    error = rox_calibration_mono_perspective_add_homography(ret, H);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_DETERMINANT_NULL);
    
    // Test correct values 
    error = rox_calibration_mono_perspective_add_homography(ret,H2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Free memory 
    error = rox_calibration_mono_perspective_del(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    error = rox_array2d_double_del(&H);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    error = rox_array2d_double_del(&H2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test the adding of 2D detected points of an image to a calibration object
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_add_current_points)
{
    // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Calibration_Mono_Perspective ret = NULL;
    Rox_Point2D_Double_Struct p1;
    Rox_Point2D_Double_Struct p2;
    Rox_Point2D_Double_Struct p3;
    Rox_Point2D_Double_Struct p4;
    Rox_Point2D_Double_Struct p5;
    Rox_Uint count = 4;
    Rox_DynVec_Point2D_Double points2 = NULL;
    
    error = rox_calibration_mono_perspective_new(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // points initialisation 
    Rox_DynVec_Point2D_Double points;
    rox_dynvec_point2d_double_new(&points,4);
    p1.u = 277.8632; p1.v = 196.120;
    rox_dynvec_point2d_double_append(points,&p1);
    p2.u = 375.7983; p2.v = 181.8945;
    rox_dynvec_point2d_double_append(points,&p2);
    p3.u = 375.7983; p3.v = 298.1055;
    rox_dynvec_point2d_double_append(points,&p3);
    p4.u = 277.8632; p4.v = 283.8791;
    rox_dynvec_point2d_double_append(points,&p4);
    rox_dynvec_point2d_double_new(&points2,4);
    p5.u = 0; p1.v = 0;
    rox_dynvec_point2d_double_append(points2,&p5);
    rox_dynvec_point2d_double_append(points2,&p5);
    rox_dynvec_point2d_double_append(points2,&p5);
    rox_dynvec_point2d_double_append(points2,&p5);
    
    // Test NULL pointers 
    error = rox_calibration_mono_perspective_add_current_points(NULL,NULL,0);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_add_current_points(ret,NULL,0);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_add_current_points(NULL,points->data,0);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_add_current_points(NULL,NULL,count);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_add_current_points(ret,NULL,count);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_add_current_points(NULL,points->data,count);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // Test wrong numer of points 
    error = rox_calibration_mono_perspective_add_current_points(ret, points->data, 1);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);
    
    // Test correct values 
    error = rox_calibration_mono_perspective_add_current_points(ret, points->data, count);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Free memory 
    error = rox_calibration_mono_perspective_del(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    error = rox_dynvec_point2d_double_del(&points);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    error = rox_dynvec_point2d_double_del(&points2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test the set the 3D coordinates of the calibration model
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_set_model_points)
{
    // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;

    Rox_Uint count = 4;

    Rox_Point3D_Double_Struct p1;
    Rox_Point3D_Double_Struct p2;
    Rox_Point3D_Double_Struct p3;
    Rox_Point3D_Double_Struct p4;
    Rox_Point3D_Double_Struct p5;
    
    Rox_DynVec_Point3D_Double points = NULL;
    Rox_Calibration_Mono_Perspective ret = NULL;
    Rox_DynVec_Point3D_Double points2 = NULL;

    error = rox_calibration_mono_perspective_new(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // points initialisation 
    rox_dynvec_point3d_double_new(&points,4);
    p1.X = 277.8632; p1.Y = 196.120; p1.Z=1;
    rox_dynvec_point3d_double_append(points,&p1);
    p2.Z=1; p2.X = 375.7983; p2.Y = 181.8945;
    rox_dynvec_point3d_double_append(points,&p2);
    p3.Z=1; p3.X = 375.7983; p3.Y = 298.1055;
    rox_dynvec_point3d_double_append(points,&p3);
    p4.Z=1; p4.X = 277.8632; p4.Y = 283.8791;
    rox_dynvec_point3d_double_append(points,&p4);

    rox_dynvec_point3d_double_new(&points2,4);
    p5.X = 0; p5.Y = 0; p5.Z=0;
    rox_dynvec_point3d_double_append(points2,&p5);
    rox_dynvec_point3d_double_append(points2,&p5);
    rox_dynvec_point3d_double_append(points2,&p5);
    rox_dynvec_point3d_double_append(points2,&p5);
    
    // Test NULL pointers 
    error = rox_calibration_mono_perspective_set_model_points(NULL, NULL, 0);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_set_model_points(ret, NULL, 0);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_set_model_points(NULL, points->data, 0);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_set_model_points(NULL,NULL,count);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_set_model_points(ret,NULL,count);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_set_model_points(NULL,points->data,count);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // Test bad number of points 
    error = rox_calibration_mono_perspective_set_model_points(ret, points->data, 1);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);
    
    // Test correct values 
    error = rox_calibration_mono_perspective_set_model_points(ret, points->data, count);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Free memory 
    error = rox_calibration_mono_perspective_del(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_dynvec_point3d_double_del(&points);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_dynvec_point3d_double_del(&points2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test the computation of the linear estimation of the intrinsic parameters using the homography set
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_process_linear)
{
    // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;
    
    // Rox_Double epsilon = 0.0000001;
    Rox_Double epsilon = 0.01;

    Rox_Calibration_Mono_Perspective ret = NULL;
    Rox_Calibration_Mono_Perspective ret2 = NULL;
    Rox_Array2D_Double K = NULL;


    error = rox_calibration_mono_perspective_new(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    error = rox_calibration_mono_perspective_new(&ret2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = initialisation(&K, ret, 4);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    // Test NULL pointers 
    error = rox_calibration_mono_perspective_process_linear(NULL, 4);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // Test without homographies 
    error = rox_calibration_mono_perspective_process_linear(ret2, 4);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);
    
    // Test with wrong numeber of method 
    error = rox_calibration_mono_perspective_process_linear(ret, 8);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_INVALID_VALUE);

    error = rox_array2d_double_del(&K);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_calibration_mono_perspective_del(&ret2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    // Test correct values 
    for ( Rox_Sint method = 1; method < 6; method++)
    {
        error = rox_calibration_mono_perspective_new(&ret2);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        
        error = initialisation(&K, ret2, method);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        
        error = rox_calibration_mono_perspective_process_linear(ret2, method);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        
        error = equality(K, ret2->K, epsilon);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

        error = rox_array2d_double_del(&K);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

        error = rox_calibration_mono_perspective_del(&ret2);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    }

    // Free memory 
    error = rox_calibration_mono_perspective_del(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test the refinement of the linear estimation of the intrinsic parameters using a non linear approach
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_process_nolinear)
{
    // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;
    
    Rox_Calibration_Mono_Perspective ret = NULL;
    Rox_Calibration_Mono_Perspective ret2 = NULL;
    Rox_Calibration_Mono_Perspective ret3 = NULL;
    Rox_Array2D_Double K = NULL;
    // Rox_Double epsilon = 0.0000001;
    Rox_Double epsilon = 0.01;
    Rox_Uint i, Kddl;
    
    error = rox_calibration_mono_perspective_new(&ret2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    error = rox_calibration_mono_perspective_new(&ret3);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Test NULL pointers 
    error = rox_calibration_mono_perspective_process_nolinear(NULL,4);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // Test without homographies 
    error = rox_calibration_mono_perspective_process_nolinear(ret2,4);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);
    
    // Test with wrong numeber of method 
    error = rox_calibration_mono_perspective_process_nolinear(ret3,8);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_INVALID_VALUE);
    
    // initialisation 
    
    for ( Rox_Sint method = 1; method<6; method++)
    {
        Kddl = method;
        error = rox_calibration_mono_perspective_new(&ret);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        
        error = initialisation(&K, ret, method);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        
        if(ret->valid_image == 1 && Kddl > 2) Kddl = 2;
        if(ret->valid_image == 2 && Kddl > 3) Kddl = 3;
        
        // Copy K init 
        error = rox_array2d_double_copy(ret->K_cpy, ret->K);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        
        // Check homographies 
        error = rox_calibration_mono_perspective_check_homographies(ret);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        
        // Init with linearized solution 
        error = rox_calibration_mono_perspective_process_linear(ret, Kddl);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        
        error = equality(K, ret->K, epsilon);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        
        // Compute poses 
        for(i = 0; i < ret->homographies->used; i++)
        {
            // Compute poses 
            error = rox_transformtools_build_pose_intermodel(ret->poses->data[i], ret->homographies->data[i], ret->K);
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
            
            error = rox_array2d_double_copy(ret->poses_cpy->data[i], ret->poses->data[i]);
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        }
        
        // Test correct values 
        error = rox_calibration_mono_perspective_process_nolinear ( ret, method );
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        
        error = equality(K, ret->K, epsilon);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        
        error = rox_calibration_mono_perspective_del(&ret);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

        error = rox_array2d_double_del(&K);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    }
    
    // Free memory 
    error = rox_calibration_mono_perspective_del(&ret2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_calibration_mono_perspective_del(&ret3);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test the refinement of the linear estimation of the intrinsic parameters using a non linear approach and normalized coordinates
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_process_nolinear_normalize)
{
    // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;
    
    Rox_Calibration_Mono_Perspective ret = NULL;
    Rox_Calibration_Mono_Perspective ret2 = NULL;
    Rox_Calibration_Mono_Perspective ret3 = NULL;
    Rox_Array2D_Double K = NULL;
    Rox_Uint method = 4;
        // Rox_Double epsilon = 0.0000001;
    Rox_Double epsilon = 0.01;
    Rox_Uint i, Kddl;
    
    error = rox_calibration_mono_perspective_new(&ret2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    error = rox_calibration_mono_perspective_new(&ret3);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Test NULL pointers 
    error = rox_calibration_mono_perspective_process_nolinear_normalize(NULL, method);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // Test without homographies 
    error = rox_calibration_mono_perspective_process_nolinear_normalize(ret2, method);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);
    
    // Test with wrong number of method 
    error = rox_calibration_mono_perspective_process_nolinear_normalize(ret3, 8);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_INVALID_VALUE);
    
    for ( Rox_Sint method = 1; method<6; method++)
    {
        Kddl = method;
        // initialisation 
        error = rox_calibration_mono_perspective_new(&ret);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

        error = initialisation(&K, ret, method);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        
        if(ret->valid_image == 1 && Kddl > 2) Kddl = 2;
        if(ret->valid_image == 2 && Kddl > 3) Kddl = 3;
        
        // Copy K init 
        error = rox_array2d_double_copy(ret->K_cpy, ret->K);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        // Check homographies 
        error = rox_calibration_mono_perspective_check_homographies(ret);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        
        // Init with linearized solution 
        error = rox_calibration_mono_perspective_process_linear(ret, Kddl);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

        // rox_array2d_double_print(K);
        // rox_array2d_double_print(ret->K);

        error = equality(K, ret->K, epsilon);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        
        // Compute poses 
        for(i = 0; i < ret->homographies->used; i++)
        {
            // Compute poses 
            error = rox_transformtools_build_pose_intermodel(ret->poses->data[i], ret->homographies->data[i], ret->K);
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

            error = rox_array2d_double_copy(ret->poses_cpy->data[i], ret->poses->data[i]);
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        }
        
        // Test correct values 
        error = rox_calibration_mono_perspective_process_nolinear_normalize(ret,method);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        
        error = equality(K, ret->K, epsilon);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        
        // Free memory 
        error = rox_calibration_mono_perspective_del(&ret);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

        error = rox_array2d_double_del(&K);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    }
    
    // Free memory 
    error = rox_calibration_mono_perspective_del(&ret2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_calibration_mono_perspective_del(&ret3);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test the initialization of all working buffers for the linear estimation
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_init_buffers)
{
    // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Calibration_Mono_Perspective ret = NULL;
    
    error = rox_calibration_mono_perspective_new(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Test NULL pointers 
    error = rox_calibration_mono_perspective_init_buffers(NULL);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // Test correct values 
    error = rox_calibration_mono_perspective_init_buffers(ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Free memory 
    error = rox_calibration_mono_perspective_del(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test the computation of the linear estimation of the fu parameter using the homography set
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_linear_fu)
{
   // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Calibration_Mono_Perspective ret = NULL;
    Rox_Calibration_Mono_Perspective ret2 = NULL;
    Rox_Array2D_Double K = NULL;
    
    error = rox_calibration_mono_perspective_new(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_calibration_mono_perspective_new(&ret2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    error = initialisation(&K, ret, 1);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Test NULL pointers 
    error = rox_calibration_mono_perspective_linear_fu(NULL);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // Test without homographies 
    error = rox_calibration_mono_perspective_linear_fu(ret2);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);
    
    // Test correct values 
    error = rox_calibration_mono_perspective_linear_fu(ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Free memory 
    error = rox_calibration_mono_perspective_del(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_calibration_mono_perspective_del(&ret2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_array2d_double_del(&K);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test the computation of the linear estimation of the fu-fv parameters using the homography set
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_linear_fu_fv)
{
    // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Calibration_Mono_Perspective ret = NULL;
    Rox_Calibration_Mono_Perspective ret2 = NULL;
    Rox_Array2D_Double K = NULL;
    
    error = rox_calibration_mono_perspective_new(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_calibration_mono_perspective_new(&ret2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = initialisation(&K, ret, 2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Test NULL pointers 
    error = rox_calibration_mono_perspective_linear_fu_fv(NULL);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // Test without homographies 
    error = rox_calibration_mono_perspective_linear_fu_fv(ret2);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);
    
    // Test correct values 
    error = rox_calibration_mono_perspective_linear_fu_fv(ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Free memory 
    error = rox_calibration_mono_perspective_del(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_calibration_mono_perspective_del(&ret2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_array2d_double_del(&K);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test the computation of the linear estimation of the fu-fv-c-cv parameters using the homography set
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_linear_fu_fv_cu_cv)
{
    // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Calibration_Mono_Perspective ret = NULL;
    Rox_Calibration_Mono_Perspective ret2 = NULL;
    Rox_Array2D_Double K = NULL;
    
    error = rox_calibration_mono_perspective_new(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_calibration_mono_perspective_new(&ret2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = initialisation(&K, ret, 4);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Test NULL pointers 
    error = rox_calibration_mono_perspective_linear_fu_fv_cu_cv(NULL);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // Test without homographies 
    error = rox_calibration_mono_perspective_linear_fu_fv_cu_cv(ret2);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);
    
    // Test correct values 
    error = rox_calibration_mono_perspective_linear_fu_fv_cu_cv(ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Free memory 
    error = rox_calibration_mono_perspective_del(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_calibration_mono_perspective_del(&ret2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_array2d_double_del(&K);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test the computation of the linear estimation of the fu-fv-cu-cv-s parameters using the homography set
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_linear_fu_fv_cu_cv_s)
{
    // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Calibration_Mono_Perspective ret = NULL;
    Rox_Calibration_Mono_Perspective ret2 = NULL;
    Rox_Array2D_Double K = NULL;
    
    error = rox_calibration_mono_perspective_new(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_calibration_mono_perspective_new(&ret2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = initialisation(&K, ret, 5);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Test NULL pointers 
    error = rox_calibration_mono_perspective_linear_fu_fv_cu_cv_s(NULL);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // Test without homographies 
    error = rox_calibration_mono_perspective_linear_fu_fv_cu_cv_s(ret2);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);
    
    // Test correct values 
    error = rox_calibration_mono_perspective_linear_fu_fv_cu_cv_s(ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Free memory 
    error = rox_calibration_mono_perspective_del(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_calibration_mono_perspective_del(&ret2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_array2d_double_del(&K);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test the computation of the linear estimation of the fu-cu-cv parameters using the homography set
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_linear_fu_cu_cv)
{
    // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Calibration_Mono_Perspective ret = NULL;
    Rox_Calibration_Mono_Perspective ret2 = NULL;
    Rox_Array2D_Double K = NULL;
    
    error = rox_calibration_mono_perspective_new(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    error = rox_calibration_mono_perspective_new(&ret2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = initialisation(&K, ret, 3);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Test NULL pointers 
    error = rox_calibration_mono_perspective_linear_fu_cu_cv(NULL);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // Test without homographies 
    error = rox_calibration_mono_perspective_linear_fu_cu_cv(ret2);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);
    
    // Test correct values 
    error = rox_calibration_mono_perspective_linear_fu_cu_cv(ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Free memory 
    error = rox_calibration_mono_perspective_del(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_calibration_mono_perspective_del(&ret2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_array2d_double_del(&K);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test the compute of the intrinsic parameters after the linear estimation: cholesky decomposition + un-normalization
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_update_linear_instrinsics)
{
    // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Calibration_Mono_Perspective ret = NULL;
    Rox_Array2D_Double K = NULL;
    
    error = rox_calibration_mono_perspective_new(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    // Create a new calibration matrix K
    error = initialisation(&K, ret, 4);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Test NULL pointers 
    error = rox_calibration_mono_perspective_update_linear_instrinsics(NULL);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // Test correct values 
    error = rox_calibration_mono_perspective_update_linear_instrinsics(ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Free memory 
    error = rox_calibration_mono_perspective_del(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_array2d_double_del(&K);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test the initialization the intrinsic parameters for the linear estimation
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_set_intrinsics)
{
    // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;
    
    Rox_Calibration_Mono_Perspective ret = NULL;
    Rox_Calibration_Mono_Perspective ret2 = NULL;
    Rox_Calibration_Mono_Perspective ret3 = NULL;
    Rox_Calibration_Mono_Perspective ret4 = NULL;
    Rox_Calibration_Mono_Perspective ret5 = NULL;
    
    error = rox_calibration_mono_perspective_new(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_calibration_mono_perspective_new(&ret2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_calibration_mono_perspective_new(&ret3);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_calibration_mono_perspective_new(&ret4);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    error = rox_calibration_mono_perspective_new(&ret5);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Ks initialisation 
    rox_array2d_double_del(&ret3->K);
    ret3->K = NULL;
    rox_array2d_double_new(&ret3->K, 4, 3);

    rox_array2d_double_del(&ret4->K);
    ret4->K = NULL;
    rox_array2d_double_new(&ret4->K, 3, 4);
    
    // Test NULL pointers 
    error = rox_calibration_mono_perspective_set_intrinsics(NULL,ret->K);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_set_intrinsics(ret2,NULL);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_set_intrinsics(NULL,NULL);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // Test array not the same size 
    error = rox_calibration_mono_perspective_set_intrinsics(ret3,ret->K);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

    error = rox_calibration_mono_perspective_set_intrinsics(ret4,ret->K);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);
    
    // Test correct values 
    error = rox_calibration_mono_perspective_set_intrinsics(ret2,ret->K);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Free memory 
    error = rox_calibration_mono_perspective_del(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    error = rox_calibration_mono_perspective_del(&ret2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    error = rox_calibration_mono_perspective_del(&ret3);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    error = rox_calibration_mono_perspective_del(&ret4);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    error = rox_calibration_mono_perspective_del(&ret5);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test getting a copy of the estimated intrinsic parameters
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_get_intrinsics)
{
    // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Calibration_Mono_Perspective ret = NULL;
    Rox_Calibration_Mono_Perspective ret2 = NULL;
    Rox_Calibration_Mono_Perspective ret3 = NULL;
    Rox_Calibration_Mono_Perspective ret4 = NULL;
    Rox_Array2D_Double K = NULL;
    
    error = rox_calibration_mono_perspective_new(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    error = rox_calibration_mono_perspective_new(&ret2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    error = rox_calibration_mono_perspective_new(&ret3);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    error = rox_calibration_mono_perspective_new(&ret4);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    rox_array2d_double_del(&ret3->K);
    ret3->K = NULL;
    rox_array2d_double_new(&ret3->K, 2, 3);

    rox_array2d_double_del(&ret4->K);
    ret4->K = NULL;
    rox_array2d_double_new(&ret4->K, 3, 2);
    
    error = initialisation(&K, ret, 4);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Test NULL pointers 
    error = rox_calibration_mono_perspective_get_intrinsics(0, 0);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    error = rox_calibration_mono_perspective_get_intrinsics(ret2->K, 0);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_get_intrinsics(0, ret);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // Test array not the same size 
    error = rox_calibration_mono_perspective_set_intrinsics(ret3,ret->K);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);

    error = rox_calibration_mono_perspective_set_intrinsics(ret4,ret->K);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_ARRAYS_NOT_MATCH);
    
    // Test correct values 
    error = rox_calibration_mono_perspective_get_intrinsics(ret2->K,ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Free memory 
    error = rox_calibration_mono_perspective_del(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    error = rox_calibration_mono_perspective_del(&ret2);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    error = rox_calibration_mono_perspective_del(&ret3);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    error = rox_calibration_mono_perspective_del(&ret4);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_array2d_double_del(&K);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test the computation of the linear and no linear estimations of the intrinsic parameters
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_compute_parameters)
{
    // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Calibration_Mono_Perspective ret = NULL;
        // Rox_Double epsilon = 0.0000001;
    Rox_Double epsilon = 0.01;
    Rox_Array2D_Double K = NULL;
    
    error = rox_calibration_mono_perspective_new(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    //! The test should be done as follows :
    //! 1) choose the 3D model points
    //! 2) choose the camera extrinsic parameters
    //! 3) choose the camera intrinsic parameters
    //! 4) project the 3D model points in the images using the camera intrinsic and extrinsic parameters
    //! 5) compute the homographies
    //! 6) calibrate
    //! 7) check the error
    
    //! Test NULL pointers 
    error = rox_calibration_mono_perspective_compute_parameters(NULL, 4);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // Test with wrong numeber of method 
    error = rox_calibration_mono_perspective_compute_parameters(ret,8);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_INVALID_VALUE);
    
    // Test correct values 
    for ( Rox_Sint method = 1; method<6; method++)
    {
        Rox_Calibration_Mono_Perspective ret2;
        error = rox_calibration_mono_perspective_new(&ret2);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        
        error = initialisation(&K, ret2, method);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        
        error = rox_calibration_mono_perspective_compute_parameters(ret2, method);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        
        error = equality(K, ret2->K, epsilon);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
        
        error = rox_calibration_mono_perspective_del(&ret2);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

        error = rox_array2d_double_del(&K);
        ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    }
    
    // Free memory 
    error = rox_calibration_mono_perspective_del(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test foror a given measure, the computation of the projection error using the estimated parameters
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_get_statistics)
{
    // Variables declaration and initialisation 
    Rox_Double min = 1, max = 1, mean = 1;
    Rox_Double median = 1, std = 1;
    Rox_Uint id = 0;
    
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Calibration_Mono_Perspective ret = NULL;
    Rox_Array2D_Double K = NULL;
    
    // Initialisation 
    error = rox_calibration_mono_perspective_new(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = initialisation(&K, ret, 4);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Test NULL pointers 
    error = rox_calibration_mono_perspective_get_statistics(0, &max, &mean, &median, &std, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, 0, &mean, &median, &std, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, &max, 0, &median, &std, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, &max, &mean, 0, &std, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, &max, &mean, &median, 0, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, &max, &mean, &median, &std, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_get_statistics(0, 0, &mean, &median, &std, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, &max, 0, &median, &std, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, &max, &mean, 0, &std, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, &max, &mean, &median, 0, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, &max, &mean, &median, &std, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, 0, 0, &median, &std, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, 0, &mean, 0, &std, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, 0, &mean, &median, 0, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, 0, &mean, &median, &std, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, &max, 0, 0, &std, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, &max, 0, &median, 0, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, &max, 0, &median, &std, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, &max, &mean, 0, 0, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, &max, &mean, 0, &std, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, &max, &mean, &median, 0, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_get_statistics(0, 0, 0, &median, &std, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, 0, &mean, 0, &std, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, 0, &mean, &median, 0, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, 0, &mean, &median, &std, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, 0, 0, 0, &std, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, 0, 0, &median, 0, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, 0, 0, &median, &std, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, &max, 0, 0, &std, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, &max, 0, 0, 0, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, &max, 0, 0, &std, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, &max, &mean, 0, 0, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, 0, &mean, 0, 0, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, &max, &mean, 0, 0, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_get_statistics(&min, &max, 0, 0, 0, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, 0, &mean, 0, 0, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, 0, 0, &median, 0, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, 0, 0, 0, &std, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(&min, 0, 0, 0, 0, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, &max, &mean, 0, 0, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, &max, 0, &median, 0, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, &max, 0, 0, &std, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, &max, 0, 0, 0, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, 0, &mean, &median, 0, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, 0, &mean, 0, &std, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, 0, &mean, 0, 0, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, 0, 0, &median, &std, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, 0, 0, &median, 0, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, 0, 0, 0, &std, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_get_statistics(&min, 0, 0, 0, 0, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, &max, 0, 0, 0, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, 0, &mean, 0, 0, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, 0, 0, &median, 0, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

    error = rox_calibration_mono_perspective_get_statistics(0, 0, 0, 0, 0, ret, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    error = rox_calibration_mono_perspective_get_statistics(0, 0, 0, 0, 0, NULL, id);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // initialisation 
    error = rox_calibration_mono_perspective_linear_fu_fv_cu_cv(ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Test correct values 
    error = rox_calibration_mono_perspective_get_statistics(&min, &max, &mean, &median, &std, ret, id);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Free memory 
    error = rox_calibration_mono_perspective_del(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_array2d_double_del(&K);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test the make of a copy of the estimated intrinsic parameters and the pose set
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_save_data)
{
    // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Calibration_Mono_Perspective ret = NULL;
    
    error = rox_calibration_mono_perspective_new(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Test NULL pointers 
    error = rox_calibration_mono_perspective_save_data(NULL);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // Test correct values 
    error = rox_calibration_mono_perspective_save_data(ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Free memory liberation 
    error = rox_calibration_mono_perspective_del(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test restoring the copied data to make a new calibration after the exclusion of the worst images
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_restore_data)
{
    // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;
    
    Rox_Calibration_Mono_Perspective ret = NULL;
    error = rox_calibration_mono_perspective_new(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Test NULL pointers 
    error = rox_calibration_mono_perspective_restore_data(NULL);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // Test correct values 
    error = rox_calibration_mono_perspective_restore_data(ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Free memory 
    error = rox_calibration_mono_perspective_del(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//! Test checking that the given homographies are correctly estimated by computing the reprojection error
//! \return An error code
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_calibration_mono_perspective_check_homographies)
{
    // Initialisation 
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Calibration_Mono_Perspective ret = NULL;
    Rox_Array2D_Double K = NULL;
    
    error = rox_calibration_mono_perspective_new(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = initialisation(&K, ret, 4);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Test NULL pointers 
    error = rox_calibration_mono_perspective_check_homographies(NULL);
    ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
    
    // Test correct values 
    error = rox_calibration_mono_perspective_check_homographies(ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
    
    // Free memory 
    error = rox_calibration_mono_perspective_del(&ret);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

    error = rox_array2d_double_del(&K);
    ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
