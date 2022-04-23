//==============================================================================
//
//    OPENROX   : File sl3fromNpoints.c
//
//    Contents  : Implementation of sl3fromNpoints module
//
//    Author(s) : R&D department leaded by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "sl3fromNpoints.h"

#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/multiply/mulmatmattrans.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/inverse/mat3x3inv.h>
#include <baseproc/geometry/point/point2d_matsl3_transform.h>
#include <baseproc/array/decomposition/svd.h>
#include <baseproc/array/decomposition/svdsort.h>
#include <baseproc/geometry/transforms/matsl3/sl3normalize.h>
#include <inout/system/errors_print.h>

#include <float.h>
#include <baseproc/maths/maths_macros.h>

Rox_ErrorCode rox_matsl3_from_n_points_double (
    Rox_MatSL3 homography,
    Rox_Point2D_Double ref,
    Rox_Point2D_Double cur,
    Rox_Uint nbpoints
)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    Rox_Array2D_Double Cn = 0, U = 0, S = 0, V = 0, M = 0, iKc = 0, Vn = 0, VK = 0;
    Rox_Array2D_Double Kref = 0, Kcur = 0;
    Rox_Point2D_Double ref_norm = 0, cur_norm = 0;

    Rox_Uint row;
    Rox_Double **dC, **dVn, **dV, **dK;
    Rox_Double X, Y, u, v;
    Rox_Double sumx_ref = 0, sumy_ref = 0, sumx_cur = 0, sumy_cur = 0, sum_norm_ref = 0, sum_norm_cur = 0;
    Rox_Double meanx_ref = 0, meany_ref = 0, meanx_cur = 0, meany_cur = 0;
    Rox_Double scale_ref, scale_cur, mean_norm_ref, mean_norm_cur;

    if(!homography || !ref || !cur) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    error = rox_array2d_double_check_size(homography, 3, 3);
    ROX_ERROR_CHECK_TERMINATE(error)

    error = rox_array2d_double_new(&Kref, 3, 3);
    ROX_ERROR_CHECK_TERMINATE(error)

    error = rox_array2d_double_new(&Kcur, 3, 3);
    ROX_ERROR_CHECK_TERMINATE(error)

    error = rox_array2d_double_new(&iKc, 3, 3);
    ROX_ERROR_CHECK_TERMINATE(error)

    error = rox_array2d_double_new(&Cn, 3*nbpoints, 9);
    ROX_ERROR_CHECK_TERMINATE(error)

    error = rox_array2d_double_new(&M, 9, 9);
    ROX_ERROR_CHECK_TERMINATE(error)

    error = rox_array2d_double_new(&U, 9, 9);
    ROX_ERROR_CHECK_TERMINATE(error)

    error = rox_array2d_double_new(&V, 9, 9);
    ROX_ERROR_CHECK_TERMINATE(error)

    error = rox_array2d_double_new(&S, 9, 1);
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_array2d_double_new(&Vn, 3, 3);
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_array2d_double_new(&VK, 3, 3);
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_array2d_double_fillval(Cn, 0.0);
    ROX_ERROR_CHECK_TERMINATE ( error );

    ref_norm = (Rox_Point2D_Double ) rox_memory_allocate(sizeof(*ref_norm), nbpoints);
    if (!ref_norm) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    cur_norm = (Rox_Point2D_Double ) rox_memory_allocate(sizeof(*cur_norm), nbpoints);
    if(!cur_norm) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    error  = rox_array2d_double_get_data_pointer_to_pointer( &dC, Cn);
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error  = rox_array2d_double_get_data_pointer_to_pointer( &dV, V);
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_array2d_double_get_data_pointer_to_pointer( &dVn, Vn);
    ROX_ERROR_CHECK_TERMINATE ( error );

    // Normalization
    for (Rox_Uint i = 0; i < nbpoints; i++)
    {
        // Ref points
        sumx_ref += ref[i].u;
        sumy_ref += ref[i].v;

        // Cur points
        sumx_cur += cur[i].u;
        sumy_cur += cur[i].v;
    }

    meanx_cur = sumx_cur / (Rox_Double) nbpoints;
    meany_cur = sumy_cur / (Rox_Double) nbpoints;
    meanx_ref = sumx_ref / (Rox_Double) nbpoints;
    meany_ref = sumy_ref / (Rox_Double) nbpoints;

    for (Rox_Uint i = 0; i < nbpoints; i++)
    {
        sum_norm_ref += sqrt((ref[i].u - meanx_ref) * (ref[i].u - meanx_ref) + (ref[i].v - meany_ref) * (ref[i].v - meany_ref));
        sum_norm_cur += sqrt((cur[i].u - meanx_cur) * (cur[i].u - meanx_cur) + (cur[i].v - meany_cur) * (cur[i].v - meany_cur));
    }
    mean_norm_ref = sum_norm_ref / (Rox_Double) nbpoints;
    mean_norm_cur = sum_norm_cur / (Rox_Double) nbpoints;

    scale_ref = sqrt(2.0) / mean_norm_ref;
    scale_cur = sqrt(2.0) / mean_norm_cur;

    // Build Kref
    error = rox_array2d_double_fillunit(Kref);
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_array2d_double_get_data_pointer_to_pointer( &dK, Kref);
    ROX_ERROR_CHECK_TERMINATE ( error );

    dK[0][0] = scale_ref;
    dK[1][1] = scale_ref;
    dK[0][2] = -scale_ref * meanx_ref;
    dK[1][2] = -scale_ref * meany_ref;

    // Build Kcur
    error = rox_array2d_double_fillunit(Kcur);
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_array2d_double_get_data_pointer_to_pointer( &dK, Kcur);
    ROX_ERROR_CHECK_TERMINATE ( error );

    dK[0][0] = scale_cur;
    dK[1][1] = scale_cur;
    dK[0][2] = -scale_cur * meanx_cur;
    dK[1][2] = -scale_cur * meany_cur;

    // Compute normalized points
    error = rox_point2d_double_homography(ref_norm, ref, Kref, nbpoints);
    ROX_ERROR_CHECK_TERMINATE(error)

    error = rox_point2d_double_homography(cur_norm, cur, Kcur, nbpoints);
    ROX_ERROR_CHECK_TERMINATE(error)

    for (Rox_Uint i = 0; i < nbpoints; i++)
    {
        X = ref_norm[i].u;
        Y = ref_norm[i].v;
        u = cur_norm[i].u;
        v = cur_norm[i].v;

        row = 3*i;

        // 1st row
        dC[row][3] = -X;
        dC[row][4] = -Y;
        dC[row][5] = -1.0;

        dC[row][6] = v * X;
        dC[row][7] = v * Y;
        dC[row][8] = v;

        // 2nd row
        dC[row+1][0] = X;
        dC[row+1][1] = Y;
        dC[row+1][2] = 1.0;

        dC[row+1][6] = -u * X;
        dC[row+1][7] = -u * Y;
        dC[row+1][8] = -u;

        // 3rd row
        dC[row+2][0] = -v * X;
        dC[row+2][1] = -v * Y;
        dC[row+2][2] = -v;

        dC[row+2][3] = u * X;
        dC[row+2][4] = u * Y;
        dC[row+2][5] = u;
    }

    // M = Cn' * Cn
    error = rox_array2d_double_mulmattransmat(M, Cn, Cn);
    ROX_ERROR_CHECK_TERMINATE(error)

    // SVD
    error = rox_array2d_double_svd(U, S, V, M);
    ROX_ERROR_CHECK_TERMINATE(error)

    error = rox_array2d_double_svd_sort(U, S, V);
    ROX_ERROR_CHECK_TERMINATE(error)

    // Build G = inv(K_cur)*[Vn(1:3,9)' ; Vn(4:6,9)' ; Vn(7:9,9)']*K_ref;
    error = rox_array2d_double_mat3x3_inverse(iKc, Kcur);
    ROX_ERROR_CHECK_TERMINATE(error)

    dVn[0][0] = dV[0][8]; dVn[0][1] = dV[1][8]; dVn[0][2] = dV[2][8];
    dVn[1][0] = dV[3][8]; dVn[1][1] = dV[4][8]; dVn[1][2] = dV[5][8];
    dVn[2][0] = dV[6][8]; dVn[2][1] = dV[7][8]; dVn[2][2] = dV[8][8];

    error = rox_array2d_double_mulmatmat(VK, Vn, Kref);
    ROX_ERROR_CHECK_TERMINATE(error)

    error = rox_array2d_double_mulmatmat(homography, iKc, VK);
    ROX_ERROR_CHECK_TERMINATE(error)

    // Normalize G
    error = rox_matsl3_normalize ( homography, homography );
    ROX_ERROR_CHECK_TERMINATE(error)


function_terminate:
    rox_array2d_double_del(&U);
    rox_array2d_double_del(&S);
    rox_array2d_double_del(&V);
    rox_array2d_double_del(&Vn);
    rox_array2d_double_del(&VK);
    rox_array2d_double_del(&Cn);
    rox_array2d_double_del(&M);
    rox_array2d_double_del(&Kref);
    rox_array2d_double_del(&Kcur);
    rox_array2d_double_del(&iKc);
    rox_memory_delete(ref_norm);
    rox_memory_delete(cur_norm);

    return error;
}

ROX_API Rox_ErrorCode rox_matsl3_from_n_points3d_to_points2d_double (
   Rox_MatSL3 homography,
   Rox_Point3D_Double points3D_ref,
   Rox_Point2D_Double points2D_cur,
   Rox_Uint nbpoints
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Double points2D_ref = NULL;

   points2D_ref = (Rox_Point2D_Double ) rox_memory_allocate( sizeof( Rox_Point2D_Double_Struct ), nbpoints);
   if ( !points2D_ref ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint i = 0; i < nbpoints; i++ )
   {
      points2D_ref[i].u = points3D_ref[i].X;
      points2D_ref[i].v = points3D_ref[i].Y;
   }

   error = rox_matsl3_from_n_points_double ( homography, points2D_ref, points2D_cur, nbpoints );
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:

   rox_memory_delete(points2D_ref);

   return error;
}