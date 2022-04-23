//==============================================================================
//
//    OPENROX   : File zncrosscor.c
//
//    Contents  : Implementation of zncrosscor module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "zncrosscor.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>


#include <inout/system/errors_print.h>
#include <inout/image/pgm/pgmfile.h>

Rox_ErrorCode rox_array2d_float_zncc_normalizedscore (
   Rox_Double * score,
   const Rox_Array2D_Float one,
   const Rox_Array2D_Float two,
   const Rox_Array2D_Uint mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double zncc = -1.0;
   
   if ( !score ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_float_zncc(&zncc, one, two, mask);
   if (error == ROX_ERROR_ZNCC_UNDEFINED)
   {
      *score = 0.0;
      error = ROX_ERROR_NONE;
   }
   else
   {
      ROX_ERROR_CHECK_TERMINATE ( error );
      *score = (1.0 + zncc) * 0.5;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_zncc_nomask_normalizedscore (
   Rox_Double * score, 
   const Rox_Array2D_Float one, 
   const Rox_Array2D_Float two
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double zncc = -1.0;

   if (!score) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_float_zncc_nomask ( &zncc, one, two ); 
   if (error == ROX_ERROR_ZNCC_UNDEFINED)
   {
      *score = 0.0;
      error = ROX_ERROR_NONE;
   }
   else
   {
      ROX_ERROR_CHECK_TERMINATE ( error );
      *score = (1.0 + zncc) * 0.5;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_zncc_normalizedscore (
   Rox_Double * score, 
   const Rox_Array2D_Uchar one, 
   const Rox_Array2D_Uchar two, 
   const Rox_Array2D_Uint mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double zncc = 0.0;
   
   if (!score) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_zncc ( &zncc, one, two, mask ); 
   if (error == ROX_ERROR_ZNCC_UNDEFINED)
   {
      *score = 0.0;
      error = ROX_ERROR_NONE;
   }
   else
   {
      ROX_ERROR_CHECK_TERMINATE(error);
      *score = (1.0 + zncc) * 0.5;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_zncc_nomask_normalizedscore (
   Rox_Double * score, 
   const Rox_Array2D_Uchar one, 
   const Rox_Array2D_Uchar two
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double zncc = 0.0;

   if (!score) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_zncc_nomask(&zncc, one, two);
   if (error == ROX_ERROR_ZNCC_UNDEFINED)
   {
      *score = 0.0;
      error = ROX_ERROR_NONE;
   }
   else
   {
      ROX_ERROR_CHECK_TERMINATE ( error );
      *score = (1.0 + zncc) * 0.5;
   }

function_terminate:
   return error;
}
