//==============================================================================
//
//    OPENROX   : File ellipse_transform.c
//
//    Contents  : Implementation of ellipse_transform module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ellipse_transform.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/ellipse/ellipse2d_struct.h>
#include <baseproc/geometry/ellipse/ellipse3d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_ellipse3d_transform(Rox_Ellipse3D ellipse3d_out, Rox_Array2D_Double matse3, Rox_Ellipse3D ellipse3d_inp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ellipse3d_out || !ellipse3d_inp || !matse3) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ellipse3d_out->a = ellipse3d_inp->a;
   ellipse3d_out->b = ellipse3d_inp->b;
   
   error = rox_matse3_mulmatmat(ellipse3d_out->Te, matse3, ellipse3d_inp->Te);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_ellipse2d_transform_pixels_to_meters(Rox_Ellipse2D ellipse2d_out, Rox_Array2D_Double matct2, Rox_Ellipse2D ellipse2d_inp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ellipse2d_out || !ellipse2d_inp || !matct2) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** K = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &K, matct2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ellipse2d_out->xc = (ellipse2d_inp->xc - K[0][2])/K[0][0];
   ellipse2d_out->yc = (ellipse2d_inp->yc - K[1][2])/K[1][1];
   ellipse2d_out->nxx = K[0][0]*K[0][0] * ellipse2d_inp->nxx;
   ellipse2d_out->nyy = K[1][1]*K[1][1] * ellipse2d_inp->nyy;
   ellipse2d_out->nxy = K[0][0]*K[1][1] * ellipse2d_inp->nxy;

function_terminate:
   return error;
}
