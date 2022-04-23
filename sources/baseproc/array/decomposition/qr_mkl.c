//==============================================================================
//
//    OPENROX   : File qr_mkl.c
//
//    Contents  : Implementation of qr module using MKL
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "qr.h"

#include <stdio.h>

#include <generated/array2d_uint.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillunit.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_qr ( 
   Rox_Array2D_Double Q, 
   Rox_Array2D_Double R, 
   const Rox_Array2D_Double M
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   double * mkl_M = NULL;
   double * ptr = NULL;

   if (!Q || !R || !M) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, M); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // TO BE DONE
   // https://www.netlib.org/lapack/explore-html/dd/d9a/group__double_g_ecomputational_ga3766ea903391b5cf9008132f7440ec7b.html

function_terminate:

   return error;
}

Rox_ErrorCode rox_array2d_double_qrp(Rox_Array2D_Double Q, Rox_Array2D_Double R, Rox_Array2D_Double P, const Rox_Array2D_Double M)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!Q || !R || !P || !M) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, M); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
