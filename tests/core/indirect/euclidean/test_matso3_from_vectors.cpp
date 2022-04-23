//==============================================================================
//
//    OPENROX   : File test_vec2rot.cpp
//
//    Contents  : Tests for vec2rot.c
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
   #include <baseproc/geometry/point/point3d_tools.h>
   #include <baseproc/geometry/point/dynvec_point3d_tools.h>
   #include <core/indirect/euclidean/matso3_from_vectors.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(vec2rot)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_matso3_from_vectors)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSO3 R = NULL;
   Rox_DynVec_Point3D_Double vref = NULL;
   Rox_DynVec_Point3D_Double vcur = NULL;
   
   Rox_Point3D_Double_Struct vr, vc;

   error = rox_matso3_new(&R);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_new(&vref, 10);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_dynvec_point3d_double_new(&vcur, 10);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   vr.X = 0.0; vr.Y = 1.0; vr.Z = 0.0;
   vc.X = 1.0; vc.Y = 0.0; vc.Z = 0.0;

   error = rox_dynvec_point3d_double_append(vref, &vr);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_dynvec_point3d_double_append(vcur, &vc);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      
   vr.X = -1.0; vr.Y = 0.0; vr.Z = 0.0;
   vc.X =  0.0; vc.Y = 1.0; vc.Z = 0.0;

   error = rox_dynvec_point3d_double_append(vref, &vr);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_dynvec_point3d_double_append(vcur, &vc);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
 
   vr.X = 0.0; vr.Y = 0.0; vr.Z = 1.0;
   vc.X = 0.0; vc.Y = 0.0; vc.Z = 1.0;

   error = rox_dynvec_point3d_double_append(vref, &vr);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_dynvec_point3d_double_append(vcur, &vc);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
 
   error = rox_matso3_from_vectors(R, vref, vcur);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   rox_matso3_print(R);

   error = rox_matso3_del(&R);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_del(&vref);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_dynvec_point3d_double_del(&vcur);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_matso3_from_vectors_real)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSO3 R = NULL;
   Rox_DynVec_Point3D_Double vref = NULL;
   Rox_DynVec_Point3D_Double vcur = NULL;
   
   Rox_Point3D_Double_Struct vr, vc;

   Rox_Float ai[3*15] = {0.9673, 0.0302, 0.2519, 0.9269, 0.2890, 0.2392, 0.9223, 0.2986, 0.2455, 0.9153, -0.3207, 0.2438, 0.8784, -0.1151, 0.4639, 0.9321, -0.0301, 0.3608, 0.8916, -0.0198, 0.4524, 0.9919, 0.1106, -0.0618, 0.9521, 0.2837,-0.1141, 0.9965,-0.0767,-0.0333, 0.9868,-0.0814, 0.1400, 0.9836,-0.0885, 0.1574, 0.9563, 0, 0.2925, 0.9836, -0.0600, 0.1700, 0.9982, -0.0393, 0.0458};
   Rox_Float ac[3*15] = {-0.0331, 0.2582, -0.9655, -0.2825, 0.2420, -0.9282, -0.2917, 0.2479, -0.9238, 0.3139, 0.2348, -0.9199, 0.1206, 0.4703, -0.8742, 0.0428, 0.3538, -0.9343, 0.0191, 0.4562, -0.8897, -0.1170, -0.0689, -0.9907, -0.2813, -0.1142, -0.9528, 0.0739, -0.0285, -0.9969, 0.0882, 0.1321, -0.9873, 0.0832, 0.1604, -0.9835, -0.0064, 0.2855, -0.9584, 0.0564, 0.1667, -0.9844, 0.0548, 0.0408, -0.9977};
      
   error = rox_matso3_new(&R);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_new(&vref, 15);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_dynvec_point3d_double_new(&vcur, 15);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for ( Rox_Sint k=0; k<15; k++)
   {
      vc.X = ai[3*k]; vc.Y = ai[3*k+1]; vc.Z = ai[3*k+2];
      vr.X = ac[3*k]; vr.Y = ac[3*k+1]; vr.Z = ac[3*k+2];

      error = rox_dynvec_point3d_double_append(vref, &vr);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      
      error = rox_dynvec_point3d_double_append(vcur, &vc);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }  
   
   // Normalize vectors to unit 
   error = rox_dynvec_point3d_double_normalize_unit(vref);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_normalize_unit(vcur);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Compute rotation from normalized vectors
   error = rox_matso3_from_vectors(R, vref, vcur);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   rox_matso3_print(R);

   error = rox_matso3_del(&R);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_double_del(&vref);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_dynvec_point3d_double_del(&vcur);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
