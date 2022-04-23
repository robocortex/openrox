//==============================================================================
//
//    OPENROX   : File vvspointssl3.c
//
//    Contents  : API of vvspointssl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "vvspointssl3.h"

#include <baseproc/maths/maths_macros.h>
#include <float.h>
#include <string.h>

#include <generated/dynvec_point2d_float_struct.h>
#include <generated/dynvec_point3d_float_struct.h>

#include <baseproc/geometry/point/point2d_matsl3_transform.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/robust/tukey.h>
#include <baseproc/calculus/linsys/linsys_point2d_pix_matsl3.h>
#include <baseproc/maths/linalg/matsl3.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_point2d_double_sl3_distance (
   Rox_Array2D_Double dist,
   Rox_DynVec_Point2D_Float one,
   Rox_DynVec_Point2D_Float two
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
    
   if (!dist || !one || !two) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint size = one->used;
   if (two->used != size) 
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint rows = 0;

   error = rox_array2d_double_get_rows(&rows, dist); 
   if (rows != size) 
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double *dd = NULL;
   error = rox_array2d_double_get_data_pointer ( &dd, dist);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 0; i < size; i++)
   {
      Rox_Double du = one->data[i].u - two->data[i].u;
      Rox_Double dv = one->data[i].v - two->data[i].v;
      dd[i] = sqrt(du*du + dv*dv);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_point2d_double_sl3_error (
   Rox_Double * psqsum,
   Rox_Array2D_Double errors,
   Rox_DynVec_Point2D_Float one,
   Rox_DynVec_Point2D_Float two
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!errors || !one || !two) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint size = one->used;
   if (two->used != size) {error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0;

   error = rox_array2d_double_get_rows(&rows, errors); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (rows != 2*size) 
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double * de = NULL;
   error = rox_array2d_double_get_data_pointer ( &de, errors);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double sqsum = 0.0;
   for ( Rox_Uint i = 0; i < size; i++)
   {
      Rox_Double du = one->data[i].u - two->data[i].u;
      Rox_Double dv = one->data[i].v - two->data[i].v;
      de[i*2] = du;
      de[i*2+1] = dv;

      sqsum += sqrt(du*du + dv*dv);
   }

   *psqsum = sqsum;

function_terminate:
   return error;
}

Rox_ErrorCode rox_jacobian_sl3_flat_solver (
   Rox_MatSL3 homography,
   Rox_Matrix JtJ,
   Rox_Matrix Jtf
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double sol = NULL, iJtJ = NULL;

   if (!JtJ || !Jtf || !homography)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( homography );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_check_size(JtJ, 8, 8);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_check_size(Jtf, 8, 1);
   ROX_ERROR_CHECK_TERMINATE(error)


   error = rox_array2d_double_new(&sol, 8, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&iJtJ, 8, 8); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_svdinverse(iJtJ, JtJ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmat(sol, iJtJ, Jtf); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matsl3_update_left(homography, sol); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&iJtJ);
   rox_array2d_double_del(&sol);

   return error;
}

Rox_ErrorCode rox_points_float_refine_homography_vvs (
   Rox_MatSE3 homography,
   const Rox_DynVec_Point2D_Float measures,
   const Rox_DynVec_Point2D_Float references
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint nb_points, idpt, countvalid;
   Rox_Double sqsum1, sqsum2, delta, invcount;
   Rox_Double * ddist;
   Rox_Array2D_Double JtJ = NULL;
   Rox_Array2D_Double Jtf = NULL;
   Rox_Array2D_Double diff = NULL, subdiff = NULL;
   Rox_Array2D_Double weight = NULL;
   Rox_Array2D_Double work1 = NULL;
   Rox_Array2D_Double work2 = NULL;
   Rox_Array2D_Double dist = NULL, subdist = NULL;
   Rox_DynVec_Point3D_Float mreft = NULL;
   Rox_DynVec_Point2D_Float mrefp = NULL;


   const Rox_Uint max_iter = 10;


   if (!homography || !measures || !references) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   nb_points = measures->used;
   if (nb_points < 4) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (references->used != nb_points) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matsl3_check_size ( homography ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_float_new(&mreft, 100); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_float_new(&mrefp, 100); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&JtJ, 8, 8); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Jtf, 8, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&diff, nb_points*2, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&dist, nb_points, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate dynamic
   mreft->used = 0;
   mrefp->used = 0;
   rox_dynvec_point3d_float_usecells(mreft, nb_points);
   rox_dynvec_point2d_float_usecells(mrefp, nb_points);

   // Iterate minimization
   sqsum1 = 0.0;
   invcount = 1.0 / (Rox_Double)(references->used);

   error = rox_array2d_double_get_data_pointer ( &ddist, dist );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute distances
   error = rox_point2d_float_homography(mrefp->data, references->data, homography, references->used);
   ROX_ERROR_CHECK_TERMINATE ( error );


   error = rox_array2d_point2d_double_sl3_distance(dist, mrefp, measures); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Reorder
   countvalid = 0;
   for (idpt = 0; idpt < measures->used; idpt++)
   {
      // TODO: hardcoded threshold
      if (ddist[idpt] < 15.0)
      {
         references->data[countvalid] = references->data[idpt];
         measures->data[countvalid] = measures->data[idpt];
         countvalid++;
      }
   }

   if (countvalid <= 4)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   references->used = countvalid;
   measures->used = countvalid;
   mreft->used = countvalid;
   mrefp->used = countvalid;


   error = rox_array2d_double_new(&weight, countvalid, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&work1, countvalid, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&work2, countvalid, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&subdiff, countvalid * 2, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&subdist, countvalid, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint iter = 0; iter < max_iter; iter++)
   {
      // Re-Compute distances
      rox_point2d_float_homography(mrefp->data, references->data, homography, references->used);
      error = rox_array2d_point2d_double_sl3_distance(subdist, mrefp, measures);
      if (error) break;

      // Compute error
      error = rox_array2d_point2d_double_sl3_error(&sqsum2, subdiff, mrefp, measures);
      if (error) break;

      // Exit criteria
      delta = fabs(sqsum2 - sqsum1) * invcount;
      sqsum1 = sqsum2;
      if (delta < 1e-5 && iter > 0) break;

      // Linear algebra
      error = rox_array2d_double_tukey(weight, work1, work2, subdist);
      if (error) break;

      error = rox_jacobian_sl3_from_points_weighted_premul_float(JtJ, Jtf, subdiff, weight, mrefp);
      if (error) break;

      error = rox_jacobian_sl3_flat_solver(homography, JtJ, Jtf);
      if (error) break;
   }

   error = ROX_ERROR_NONE;
   // Destroy buffers

function_terminate:
   rox_array2d_double_del(&JtJ);
   rox_array2d_double_del(&Jtf);
   rox_array2d_double_del(&diff);
   rox_array2d_double_del(&dist);
   rox_array2d_double_del(&weight);
   rox_array2d_double_del(&work1);
   rox_array2d_double_del(&work2);
   rox_array2d_double_del(&subdiff);
   rox_array2d_double_del(&subdist);
   rox_dynvec_point2d_float_del(&mrefp);
   rox_dynvec_point3d_float_del(&mreft);

   return error;
}
