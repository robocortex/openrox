//==============================================================================
//
//    OPENROX   : File pseudoinverse.c
//
//    Contents  : Implementation of pseudoinverse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "pseudoinverse.h"

#include <system/errors/errors.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/array/transpose/transpose.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_pseudoinverse(Rox_Array2D_Double out, Rox_Array2D_Double in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double in_t = NULL;
   Rox_Array2D_Double res = NULL;
   Rox_Array2D_Double resi = NULL;

   if (!out || !in) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Get size 
   Rox_Sint in_cols = 0, in_rows = 0, out_cols = 0, out_rows = 0;

   error = rox_array2d_double_get_size(&in_rows, &in_cols, in);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_size(&out_rows, &out_cols, out);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Check size 
   if ( (out_cols != in_rows) || (out_rows != in_cols))
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Allocate ressources 
   error = rox_array2d_double_new(&in_t, in_cols, in_rows);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&res, in_cols, in_cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&resi, in_cols, in_cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // in_t = trans(in) 
   error = rox_array2d_double_transpose(in_t, in);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // res = trans(in)*in 
   error = rox_array2d_double_mulmattransmat(res, in, in);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // resi = inv(res) 
   error = rox_array2d_double_svdinverse(resi, res);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // out = resi * t 
   error = rox_array2d_double_mulmatmat(out, resi, in_t);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:

   // Deallocate ressources 
   rox_array2d_double_del(&in_t);
   rox_array2d_double_del(&res);
   rox_array2d_double_del(&resi);

   return error;
}

