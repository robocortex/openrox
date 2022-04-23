//==============================================================================
//
//    OPENROX   : File zncrosscor9x9.c
//
//    Contents  : Implementation of zncrosscor9x9 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "zncrosscor9x9.h"

#include <baseproc/maths/maths_macros.h>
#include <float.h>

#include <system/arch/platform.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_uchar_zncc_9x9 (
   Rox_Double * score,
   const Rox_Array2D_Uchar Ir,
   const Rox_Slint ref_sum,
   const Rox_Slint ref_sumsq,
   const Rox_Array2D_Uchar Ic,
   const Rox_Sint cj,
   const Rox_Sint ci
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_uchar_check_size ( Ir, 9, 9 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** Ir_data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &Ir_data, Ir );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** Ic_data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &Ic_data, Ic );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size ( &rows, &cols, Ic );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (ci - 4 < 0)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (cj - 4 < 0)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (ci + 4 >= rows)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (cj + 4 >= cols)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Slint cur_sum = 0;
   Rox_Slint cur_sumsq = 0;
   Rox_Slint refcur_sum = 0;

   for (Rox_Sint k = 0; k < 9; k++)
   {
      Rox_Sint pi = ci - 4 + k;

      for (Rox_Sint l = 0; l < 9; l++)
      {
         Rox_Sint pj = cj - 4 + l;

         Rox_Slint ref = Ir_data[k][l];
         Rox_Slint val = Ic_data[pi][pj];

         cur_sum = cur_sum + val;
         cur_sumsq = cur_sumsq + val*val;
         refcur_sum = refcur_sum + val * ref;
      }
   }

   Rox_Double mean1 = ((double) ref_sum) / 81.0;
   Rox_Double mean2 = ((double) cur_sum) / 81.0;

   Rox_Double nom = ((Rox_Double) refcur_sum) - mean1 * mean2 * 81.0;

   Rox_Double denom1 = (Rox_Double) ref_sumsq - mean1 * mean1 * 81.0;
   Rox_Double denom2 = (Rox_Double) cur_sumsq - mean2 * mean2 * 81.0;

   *score = nom / (sqrt(denom1)*sqrt(denom2));

function_terminate:
   return error;
}
