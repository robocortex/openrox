//==============================================================================
//
//    OPENROX   : File essentialposes.c
//
//    Contents  : API of essentialposes module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "essentialposes.h"

#include <baseproc/array/determinant/detgl3.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/decomposition/svd.h>
#include <baseproc/array/decomposition/svdsort.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/array/multiply/mulmatmattrans.h>
#include <baseproc/array/transpose/transpose.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_essential_possible_poses(Rox_Array2D_Double * poses, Rox_Array2D_Double essential)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double Vt,U,S,V,D;
   Rox_Array2D_Double buffer, subrot1, subrot2, subrot3, subrot4;
   Rox_Double detu, detv;
   Rox_Double **du;

   if (!poses || !essential) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   error = rox_array2d_double_check_size(essential, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   U = NULL;
   S = NULL;
   V = NULL;
   D = NULL;
   Vt = NULL;
   buffer = NULL;
   subrot1 = NULL;
   subrot2 = NULL;
   subrot3 = NULL;
   subrot4 = NULL;

   error = rox_array2d_double_new(&U, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&S, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&V, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&D, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&Vt, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&buffer, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new_subarray2d(&subrot1, poses[0], 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new_subarray2d(&subrot2, poses[1], 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new_subarray2d(&subrot3, poses[2], 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new_subarray2d(&subrot4, poses[3], 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(poses[0]);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_fillunit(poses[1]);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_fillunit(poses[2]);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_fillunit(poses[3]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer(&du, U);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute one possible basis
   error = rox_array2d_double_fillval(D, 0);
   error = rox_array2d_double_set_value(D, 0, 1, 1);
   error = rox_array2d_double_set_value(D, 1, 0, -1);
   error = rox_array2d_double_set_value(D, 2, 2, 1);

   // Compute raw svd
   error = rox_array2d_double_svd(U, S, V, essential);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_svd_sort(U, S, V); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Make determinant positive
   error = rox_array2d_double_detgl3(&detu, U); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_detgl3(&detv, V); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   rox_array2d_double_scale(U, U, 1.0/detu);
   rox_array2d_double_scale(V, V, 1.0/detv);

   // Get the two possible rotations
   rox_array2d_double_transpose(Vt, V);
   rox_array2d_double_mulmatmat(buffer, D, Vt);
   rox_array2d_double_mulmatmat(subrot1, U, buffer);
   rox_array2d_double_copy(subrot2, subrot1);
   rox_array2d_double_mulmattransmat(buffer, D, Vt);
   rox_array2d_double_mulmatmat(subrot3, U, buffer);
   rox_array2d_double_copy(subrot4, subrot3);

   // Put the two possible translations (opposite)
   rox_array2d_double_set_value(poses[0], 0, 3, du[0][2]);
   rox_array2d_double_set_value(poses[0], 1, 3, du[1][2]);
   rox_array2d_double_set_value(poses[0], 2, 3, du[2][2]);
   rox_array2d_double_set_value(poses[2], 0, 3, du[0][2]);
   rox_array2d_double_set_value(poses[2], 1, 3, du[1][2]);
   rox_array2d_double_set_value(poses[2], 2, 3, du[2][2]);
   rox_array2d_double_set_value(poses[1], 0, 3, -du[0][2]);
   rox_array2d_double_set_value(poses[1], 1, 3, -du[1][2]);
   rox_array2d_double_set_value(poses[1], 2, 3, -du[2][2]);
   rox_array2d_double_set_value(poses[3], 0, 3, -du[0][2]);
   rox_array2d_double_set_value(poses[3], 1, 3, -du[1][2]);
   rox_array2d_double_set_value(poses[3], 2, 3, -du[2][2]);

function_terminate:

   rox_array2d_double_del(&U);
   rox_array2d_double_del(&S);
   rox_array2d_double_del(&V);
   rox_array2d_double_del(&D);
   rox_array2d_double_del(&Vt);
   rox_array2d_double_del(&buffer);
   rox_array2d_double_del(&subrot1);
   rox_array2d_double_del(&subrot2);
   rox_array2d_double_del(&subrot3);
   rox_array2d_double_del(&subrot4);
   return error;
}
