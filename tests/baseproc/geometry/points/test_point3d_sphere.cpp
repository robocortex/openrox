//==============================================================================
//
//    OPENROX   : File test_point3d_sphere.cpp
//
//    Contents  : Tests for point3d_from_sphere.c
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
   #include <baseproc/geometry/point/point3d_sphere.h>
   #include <inout/geometry/point/dynvec_point3d_save.h>
   #include <inout/geometry/point/dynvec_point3d_print.h>
   #include <inout/numeric/dynvec_serialize.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(pointsphere)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_dynvec_point3d_double_new_from_sphere)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint nb_subdivs = 3;
   Rox_Double maxangle = 60.0; // degrees
   Rox_DynVec_Point3D_Double dynvec_points3d = NULL;
   
   error = rox_dynvec_point3d_double_new_from_sphere(&dynvec_points3d, nb_subdivs, maxangle);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_dynvec_point3d_double_print ( dynvec_points3d );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_dynvec_point3d_double_save("test_point3d_sphere.txt", dynvec_points3d);
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_del ( &dynvec_points3d );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()

/* 

% Matlab/Octave code to print results

m = load('test_point3d_sphere.txt');
figure(1);hold on; grid on; axis equal;

for i = 1:size(m,1)
   cTo_centered(:,:,i) = rox_matse3_centered_vec_rot(m(i,:)', 0);
   oTc_centered(:,:,i) = inv(cTo_centered(:,:,i));
   otc_centered(:,i) = oTc_centered(1:3,4,i);  
   display_frame(oTc_centered(:,:,i)) ;
   plot3([0, otc_centered(1,i)], [0, otc_centered(2,i)], [0, otc_centered(3,i)], 'g--');
end;

*/