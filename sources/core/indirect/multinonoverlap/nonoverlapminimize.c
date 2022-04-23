//==============================================================================
//
//    OPENROX   : File nonoverlapminimize.c
//
//    Contents  : Implementation of nonoverlapminimize module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "nonoverlapminimize.h"

#include <float.h>
#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_point3d_double.h>
#include <generated/dynvec_point2d_double_struct.h>
#include <generated/dynvec_point3d_double_struct.h>
#include <generated/objset_dynvec_point2d_double_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmatmattrans.h>
#include <baseproc/array/add/add.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/point/point3d_matse3_transform.h>
#include <baseproc/geometry/line/line_from_points.h>
#include <baseproc/geometry/line/line_transform.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/array/robust/tukey.h>
#include <baseproc/array/scale/scale.h>

#include <core/indirect/euclidean/triangulate.h>
#include <core/indirect/multinonoverlap/p7p.h>
#include <core/indirect/multinonoverlap/nonoverlaperror.h>
#include <baseproc/calculus/linsys/linsys_generalized_geometric_weighted_premul.h>

#include <inout/system/errors_print.h>

#define ROX_NONOVERLAP_MINIMIZE_ITERATIONS 20

Rox_ErrorCode rox_nonoverlap_minimize_get_inliers(Rox_DynVec_Point2D_Double ar, Rox_DynVec_Point3D_Double br, Rox_DynVec_Point2D_Double ac, Rox_DynVec_Point3D_Double bc, Rox_Array2D_Double pose, Rox_Array2D_Double_Collection relativeposes, Rox_Array2D_Double_Collection calibrations, Rox_ObjSet_DynVec_Point2D_Double refs, Rox_ObjSet_DynVec_Point2D_Double curs)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint idcam, count_cameras, idpt;
   Rox_DynVec_Point2D_Double camref, camcur;
   Rox_Array2D_Double calib, cxTc0, c0Tcx, corpose;
   Rox_Double ** dt, **dk;
   Rox_Double dist;
   Rox_Double fu,fv,cu,cv;
   Rox_Double ifu,ifv,icu,icv;
   Rox_Point3D_Double_Struct pt, origin;
   Rox_Line3D_Plucker_Struct plucker, transformed_plucker;

   Rox_Point2D_Double_Struct ref;
   Rox_Point2D_Double_Struct cur, diff, p2d;
   Rox_Point3D_Double_Struct triangulated, transformed, p3d;

   if (!pose || !relativeposes || !calibrations || !refs || !curs) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   corpose = NULL;
   c0Tcx = NULL;

   error = rox_array2d_double_new(&corpose, 4, 4); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&c0Tcx, 4, 4); ROX_ERROR_CHECK_TERMINATE(error)

   count_cameras = rox_array2d_double_collection_get_count(relativeposes);

   origin.X = 0;
   origin.Y = 0;
   origin.Z = 0;

   for (idcam = 0; idcam < count_cameras; idcam++)
   {
      // Get information
      cxTc0 = rox_array2d_double_collection_get(relativeposes, idcam);
      calib = rox_array2d_double_collection_get(calibrations, idcam);
      error = rox_array2d_double_get_data_pointer_to_pointer( &dt, corpose); ROX_ERROR_CHECK_TERMINATE ( error );
      error = rox_array2d_double_get_data_pointer_to_pointer( &dk, calib); ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_svdinverse(c0Tcx, cxTc0); ROX_ERROR_CHECK_TERMINATE ( error );
      error = rox_transformtools_estimate_relativepose_from_general(corpose, cxTc0, pose); ROX_ERROR_CHECK_TERMINATE ( error );

      fu = dk[0][0];
      fv = dk[1][1];
      cu = dk[0][2];
      cv = dk[1][2];
      ifu = 1.0 / fu;
      ifv = 1.0 / fv;
      icu = - cu * ifu;
      icv = - cv * ifv;

      // Select list of points
      camcur = curs->data[idcam];
      camref = refs->data[idcam];

      for (idpt = 0; idpt < camcur->used; idpt++)
      {
         ref.u = ifu * camref->data[idpt].u + icu;
         ref.v = ifv * camref->data[idpt].v + icv;
         cur.u = ifu * camcur->data[idpt].u + icu;
         cur.v = ifv * camcur->data[idpt].v + icv;

         error = rox_pose_triangulate_oneway(&triangulated, &ref, &cur, corpose);
         if (error) continue;

         if (triangulated.Z < DBL_EPSILON) continue;

         transformed.X = dt[0][0] * triangulated.X + dt[0][1] * triangulated.Y + dt[0][2] * triangulated.Z + dt[0][3];
         transformed.Y = dt[1][0] * triangulated.X + dt[1][1] * triangulated.Y + dt[1][2] * triangulated.Z + dt[1][3];
         transformed.Z = dt[2][0] * triangulated.X + dt[2][1] * triangulated.Y + dt[2][2] * triangulated.Z + dt[2][3];

         diff.u = fu * (transformed.X / transformed.Z) + cu - camcur->data[idpt].u;
         diff.v = fv * (transformed.Y / transformed.Z) + cv - camcur->data[idpt].v;
         dist = sqrt(diff.u * diff.u + diff.v * diff.v);
         if (dist > 10.0) continue;

         pt.X = ref.u;
         pt.Y = ref.v;
         pt.Z = 1;

         rox_line3d_plucker_from_2_point3d(&plucker, &origin, &pt);
         rox_line3d_plucker_transform(&transformed_plucker, &plucker, c0Tcx);

         p2d.u = transformed_plucker.displacement[0] / transformed_plucker.displacement[2];
         p2d.v = transformed_plucker.displacement[1] / transformed_plucker.displacement[2];
         p3d.X = transformed_plucker.moment[0] / transformed_plucker.displacement[2];
         p3d.Y = transformed_plucker.moment[1] / transformed_plucker.displacement[2];
         p3d.Z = transformed_plucker.moment[2] / transformed_plucker.displacement[2];

         error = rox_dynvec_point2d_double_append(ar, &p2d); ROX_ERROR_CHECK_TERMINATE(error)
         error = rox_dynvec_point3d_double_append(br, &p3d); ROX_ERROR_CHECK_TERMINATE(error)


         pt.X = cur.u;
         pt.Y = cur.v;
         pt.Z = 1;

         rox_line3d_plucker_from_2_point3d(&plucker, &origin, &pt);
         rox_line3d_plucker_transform(&transformed_plucker, &plucker, c0Tcx);

         p2d.u = transformed_plucker.displacement[0] / transformed_plucker.displacement[2];
         p2d.v = transformed_plucker.displacement[1] / transformed_plucker.displacement[2];
         p3d.X = transformed_plucker.moment[0] / transformed_plucker.displacement[2];
         p3d.Y = transformed_plucker.moment[1] / transformed_plucker.displacement[2];
         p3d.Z = transformed_plucker.moment[2] / transformed_plucker.displacement[2];

         error = rox_dynvec_point2d_double_append(ac, &p2d); ROX_ERROR_CHECK_TERMINATE(error)
         error = rox_dynvec_point3d_double_append(bc, &p3d); ROX_ERROR_CHECK_TERMINATE(error)
      }

   }

function_terminate:
   rox_array2d_double_del(&corpose);
   rox_array2d_double_del(&c0Tcx);

   return error;
}

Rox_ErrorCode rox_nonoverlap_minimize(Rox_Array2D_Double pose, Rox_Array2D_Double_Collection relativeposes, Rox_Array2D_Double_Collection calibrations, Rox_ObjSet_DynVec_Point2D_Double refs, Rox_ObjSet_DynVec_Point2D_Double curs)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_Point2D_Double ar;
   Rox_DynVec_Point3D_Double br;
   Rox_DynVec_Point2D_Double ac;
   Rox_DynVec_Point3D_Double bc;
   Rox_Double ** dt;
   Rox_Double maxtrans, ctrans;
   Rox_Double normalizertrans;
   Rox_Uint iter, maxtransid, k, idpt, pos;
   Rox_Double *dd, *de, *dx, *drx;

   Rox_Array2D_Double E, vecerr;
   Rox_Array2D_Double JtJ, Jtf, invJtJ, x, rx;
   Rox_Array2D_Double dist, weight, weightbuf1, weightbuf2;

   Rox_Double sum = 0.0;


   E = NULL;
   vecerr = NULL;
   JtJ = NULL;
   invJtJ = NULL;
   Jtf = NULL;
   x = NULL;
   rx = NULL;
   dist = NULL;
   weight = NULL;
   weightbuf1 = NULL;
   weightbuf2 = NULL;
   ar = NULL;
   br = NULL;
   ac = NULL;
   bc = NULL;

   if (!pose || !relativeposes || !calibrations || !refs || !curs) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dynvec_point2d_double_new(&ar, 100); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_point2d_double_new(&ac, 100); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_point3d_double_new(&br, 100); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_point3d_double_new(&bc, 100); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&E, 3, 3);ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_nonoverlap_minimize_get_inliers(ar, br, ac, bc, pose, relativeposes, calibrations, refs, curs); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&vecerr, ar->used * 2, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&dist, ar->used, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&weight, ar->used, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&weightbuf1, ar->used, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&weightbuf2, ar->used, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&JtJ, 5, 5); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&Jtf, 5, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&invJtJ, 5, 5); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&x, 5, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&rx, 6, 1); ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dt, pose); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer ( &de, vecerr); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer ( &dd, dist); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer ( &dx, x); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer ( &drx, rx); ROX_ERROR_CHECK_TERMINATE ( error );

   for (iter = 0; iter < ROX_NONOVERLAP_MINIMIZE_ITERATIONS; iter++)
   {
      // Search for the biggest translation direction
      maxtrans = 0;
      maxtransid = 0;
      for (k = 0; k < 3; k++)
      {
         ctrans = fabs(dt[k][3]);
         if (ctrans > maxtrans)
         {
            maxtrans = ctrans;
            maxtransid = k;
         }
      }

      // We normalize using the translation, need to be non zero
      if (maxtrans < DBL_EPSILON)
      {
         error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
         ROX_ERROR_CHECK_TERMINATE(error)
      }

      // Keep the sign
      normalizertrans = maxtrans;
      if (dt[k][3] < 0.0)
      {
         normalizertrans = -maxtrans;
      }

      // Normalize
      for (k = 0; k < 3; k++)
      {
         dt[k][3] = dt[k][3] / normalizertrans;
      }

      error = rox_transformtools_build_essential(E, pose);
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_nonoverlap_geometric_error(vecerr, E, pose, ar, br, ac, bc);
      ROX_ERROR_CHECK_TERMINATE(error)

      sum = 0.0;
      for (idpt = 0; idpt < ac->used; idpt++)
      {
      	dd[idpt] = de[idpt*2]*de[idpt*2] + de[idpt*2+1]*de[idpt*2+1];
      	sum += dd[idpt];
      }

      error = rox_array2d_double_tukey(weight, weightbuf1, weightbuf2, dist);
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_jacobian_generalized_geometric_weighted_premul(JtJ, Jtf, vecerr, weight, pose, E, ar->data, br->data, ac->data, bc->data, ar->used, maxtransid);
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_array2d_double_svdinverse(invJtJ, JtJ);
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_array2d_double_mulmatmat(x, invJtJ, Jtf);
      ROX_ERROR_CHECK_TERMINATE(error)

      // Create a classic update vector for se(3) but ignoring the biggest translation (otherwise the solution is always t=0
      pos = 0;
      for (k = 0; k < 6; k++)
      {
         if (k == maxtransid)
         {
            drx[k] = 0;
         }
         else
         {
            drx[k] = dx[pos];
            pos++;
         }
      }

      error = rox_array2d_double_scale_inplace(rx, -1.0);
      ROX_ERROR_CHECK_TERMINATE( error );
      
      error = rox_matse3_update_left(pose, rx);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   rox_dynvec_point2d_double_del(&ac);
   rox_dynvec_point2d_double_del(&ar);
   rox_dynvec_point3d_double_del(&bc);
   rox_dynvec_point3d_double_del(&br);
   rox_array2d_double_del(&vecerr);
   rox_array2d_double_del(&dist);
   rox_array2d_double_del(&weight);
   rox_array2d_double_del(&weightbuf1);
   rox_array2d_double_del(&weightbuf2);
   rox_array2d_double_del(&JtJ);
   rox_array2d_double_del(&invJtJ);
   rox_array2d_double_del(&Jtf);
   rox_array2d_double_del(&x);
   rox_array2d_double_del(&rx);
   rox_array2d_double_del(&E);

   return error;
}
