//==============================================================================
//
//    OPENROX   : File sl3from4points.c
//
//    Contents  : Implementation of sl3from4points module
//
//    Author(s) : R&D department leaded by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "sl3from4points.h"
#include "sl3normalize.h"
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/inverse/mat3x3inv.h>
#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/array/scale/scale.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_matsl3_from_4_points_float ( 
   Rox_MatSL3 homography, 
   const Rox_Point2D_Float source, 
   const Rox_Point2D_Float dest
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double He = NULL, Hep = NULL, Hei = NULL;

   if (!homography || !source || !dest) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(homography, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create internal buffers
   error = rox_array2d_double_new(&He, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&Hep, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&Hei, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dHe = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dHe, He);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dHep = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dHep, Hep);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Copy points in "aliases" for clearer code
   Rox_Double u1 = source[0].u; Rox_Double v1 = source[0].v;
   Rox_Double u2 = source[1].u; Rox_Double v2 = source[1].v;
   Rox_Double u3 = source[2].u; Rox_Double v3 = source[2].v;
   Rox_Double u4 = source[3].u; Rox_Double v4 = source[3].v;

   Rox_Double u1p = dest[0].u; Rox_Double v1p = dest[0].v;
   Rox_Double u2p = dest[1].u; Rox_Double v2p = dest[1].v;
   Rox_Double u3p = dest[2].u; Rox_Double v3p = dest[2].v;
   Rox_Double u4p = dest[3].u; Rox_Double v4p = dest[3].v;

   // 
   dHe[0][0] = (u2 - u1) * (u3 * v4 - v3 * u4) + (-u1 * v2 + v1 * u2) * (u4 - u3);
   dHe[0][1] = (u3 - u1) * (u2 * v4 - v2 * u4) + (-u1 * v3 + v1 * u3) * (u4 - u2);
   dHe[0][2] = (u4 - u1) * (u2 * v3 - v2 * u3) + (-u1 * v4 + v1 * u4) * (u3 - u2);
   dHe[1][0] = (u1 * v2 - v1 * u2) * (v3 - v4) + (v1 - v2) * (-u3 * v4 + v3 * u4);
   dHe[1][1] = (u1 * v3 - v1 * u3) * (v2 - v4) + (v1 - v3) * (-u2 * v4 + v2 * u4);
   dHe[1][2] = (u1 * v4 - v1 * u4) * (v2 - v3) + (v1 - v4) * (-u2 * v3 + v2 * u3);
   dHe[2][0] = (v1 - v2) * (u4 - u3) - (u2 - u1) * (v3 - v4);
   dHe[2][1] = (v1 - v3) * (u4 - u2) - (u3 - u1) * (v2 - v4);
   dHe[2][2] = (v1 - v4) * (u3 - u2) + (-u4 + u1) * (v2 - v3);

   // 
   dHep[0][0] = (u2p - u1p) * (u3p * v4p - v3p * u4p) + (-u1p * v2p + v1p * u2p) * (u4p - u3p);
   dHep[0][1] = (u3p - u1p) * (u2p * v4p - v2p * u4p) + (-u1p * v3p + v1p * u3p) * (u4p - u2p);
   dHep[0][2] = (u4p - u1p) * (u2p * v3p - v2p * u3p) - (u1p * v4p - v1p * u4p) * (u3p - u2p);
   dHep[1][0] = (u1p * v2p - v1p * u2p) * (v3p - v4p) + (v1p - v2p) * (-u3p * v4p + v3p * u4p);
   dHep[1][1] = (u1p * v3p - v1p * u3p) * (v2p - v4p) + (v1p - v3p) * (-u2p * v4p + v2p * u4p);
   dHep[1][2] = (u1p * v4p - v1p * u4p) * (v2p - v3p) + (v1p - v4p) * (-u2p * v3p + v2p * u3p);
   dHep[2][0] = (v1p - v2p) * (u4p - u3p) - (u2p - u1p) * (v3p - v4p);
   dHep[2][1] = (v1p - v3p) * (u4p - u2p) - (u3p - u1p) * (v2p - v4p);
   dHep[2][2] = (v1p - v4p) * (u3p - u2p) + (-u4p + u1p) * (v2p - v3p);

   error = rox_array2d_double_mat3x3_inverse(Hei, He); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(He, Hep, Hei);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matsl3_normalize(homography, He);
   ROX_ERROR_CHECK_TERMINATE(error)

   Rox_Double ** dH = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dH, homography );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute the sign of third coordinate of reprojected point
   if ( (dH[2][0]*u1+dH[2][1]*v1+dH[2][2]) < 0.0 )
   {
      error = rox_array2d_double_scale_inplace ( homography, -1.0 );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   rox_array2d_double_del(&He);
   rox_array2d_double_del(&Hei);
   rox_array2d_double_del(&Hep);

   return error;
}


Rox_ErrorCode rox_matsl3_from_4_points_double (
   Rox_MatSL3 homography, 
   const Rox_Point2D_Double source, 
   const Rox_Point2D_Double dest
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double He = NULL, Hep = NULL, Hei = NULL;

   if (!homography || !source || !dest) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(homography, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create internal buffers
   error = rox_array2d_double_new(&He, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Hep, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Hei, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dHe = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dHe, He );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dHep = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dHep, Hep );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Copy points in "aliases" for clearer code
   Rox_Double u1 = source[0].u; Rox_Double v1 = source[0].v;
   Rox_Double u2 = source[1].u; Rox_Double v2 = source[1].v;
   Rox_Double u3 = source[2].u; Rox_Double v3 = source[2].v;
   Rox_Double u4 = source[3].u; Rox_Double v4 = source[3].v;

   Rox_Double u1p = dest[0].u; Rox_Double v1p = dest[0].v;
   Rox_Double u2p = dest[1].u; Rox_Double v2p = dest[1].v;
   Rox_Double u3p = dest[2].u; Rox_Double v3p = dest[2].v;
   Rox_Double u4p = dest[3].u; Rox_Double v4p = dest[3].v;

   // 
   dHe[0][0] = (u2 - u1) * (u3 * v4 - v3 * u4) + (-u1 * v2 + v1 * u2) * (u4 - u3);
   dHe[0][1] = (u3 - u1) * (u2 * v4 - v2 * u4) + (-u1 * v3 + v1 * u3) * (u4 - u2);
   dHe[0][2] = (u4 - u1) * (u2 * v3 - v2 * u3) - (u1 * v4 - v1 * u4) * (u3 - u2);
   dHe[1][0] = (u1 * v2 - v1 * u2) * (v3 - v4) + (v1 - v2) * (-u3 * v4 + v3 * u4);
   dHe[1][1] = (u1 * v3 - v1 * u3) * (v2 - v4) + (v1 - v3) * (-u2 * v4 + v2 * u4);
   dHe[1][2] = (u1 * v4 - v1 * u4) * (v2 - v3) + (v1 - v4) * (-u2 * v3 + v2 * u3);
   dHe[2][0] = (v1 - v2) * (u4 - u3) - (u2 - u1) * (v3 - v4);
   dHe[2][1] = (v1 - v3) * (u4 - u2) - (u3 - u1) * (v2 - v4);
   dHe[2][2] = (v1 - v4) * (u3 - u2) + (-u4 + u1) * (v2 - v3);

   // 
   dHep[0][0] = (u2p - u1p) * (u3p * v4p - v3p * u4p) + (-u1p * v2p + v1p * u2p) * (u4p - u3p);
   dHep[0][1] = (u3p - u1p) * (u2p * v4p - v2p * u4p) + (-u1p * v3p + v1p * u3p) * (u4p - u2p);
   dHep[0][2] = (u4p - u1p) * (u2p * v3p - v2p * u3p) - (u1p * v4p - v1p * u4p) * (u3p - u2p);
   dHep[1][0] = (u1p * v2p - v1p * u2p) * (v3p - v4p) + (v1p - v2p) * (-u3p * v4p + v3p * u4p);
   dHep[1][1] = (u1p * v3p - v1p * u3p) * (v2p - v4p) + (v1p - v3p) * (-u2p * v4p + v2p * u4p);
   dHep[1][2] = (u1p * v4p - v1p * u4p) * (v2p - v3p) + (v1p - v4p) * (-u2p * v3p + v2p * u3p);
   dHep[2][0] = (v1p - v2p) * (u4p - u3p) - (u2p - u1p) * (v3p - v4p);
   dHep[2][1] = (v1p - v3p) * (u4p - u2p) - (u3p - u1p) * (v2p - v4p);
   dHep[2][2] = (v1p - v4p) * (u3p - u2p) + (-u4p + u1p) * (v2p - v3p);

   error = rox_array2d_double_mat3x3_inverse ( Hei, He ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmat ( He, Hep, Hei ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matsl3_normalize ( homography, He ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double **dH = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dH, homography );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute the sign of third coordinate of reprojected point
   if ( (dH[2][0]*u1+dH[2][1]*v1+dH[2][2]) < 0.0 )
   {
      error = rox_array2d_double_scale_inplace ( homography, -1.0 );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   rox_array2d_double_del(&He);
   rox_array2d_double_del(&Hei);
   rox_array2d_double_del(&Hep);

   return error;
}
