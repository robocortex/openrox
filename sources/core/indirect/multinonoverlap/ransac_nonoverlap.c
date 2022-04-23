//==============================================================================
//
//    OPENROX   : File ransac_nonoverlap.c
//
//    Contents  : Implementation of ransac_nonoverlap module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ransac_nonoverlap.h"

#include <stdio.h>
#include <float.h>

#include <generated/dynvec_uint.h>
#include <generated/dynvec_uint_struct.h>
#include <generated/dynvec_point2d_double_struct.h>
#include <generated/objset_combination.h>
#include <generated/objset_combination_struct.h>
#include <generated/objset_dynvec_point2d_double_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmatmattrans.h>
#include <baseproc/array/add/add.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/geometry/line/line_from_points.h>
#include <baseproc/geometry/line/line_transform.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/maths/random/combination_struct.h>

#include <core/indirect/euclidean/triangulate.h>
#include <core/indirect/multinonoverlap/p7p.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_ransac_nonoverlap_check_consensus3d(Rox_Uint *card_consensus, Rox_Array2D_Double pose, Rox_Array2D_Double_Collection relativeposes, Rox_Array2D_Double_Collection calibrations, Rox_ObjSet_DynVec_Point2D_Double refs, Rox_ObjSet_DynVec_Point2D_Double curs)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Uint idcam, count_cameras, countvalid, idpt;
   Rox_DynVec_Point2D_Double camref, camcur;
   Rox_Array2D_Double calib, cxTc0, corpose;
   Rox_Double ** dt, **dk;
   Rox_Double dist;
   Rox_Double fu,fv,cu,cv;
   Rox_Double ifu,ifv,icu,icv;

   Rox_Point2D_Double_Struct ref;
   Rox_Point2D_Double_Struct cur, diff;
   Rox_Point3D_Double_Struct triangulated, transformed;

   if (!card_consensus || !pose || !relativeposes || !calibrations || !refs || !curs) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   corpose = NULL;

   error = rox_array2d_double_new(&corpose, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE(error)

   count_cameras = rox_array2d_double_collection_get_count(relativeposes);

   countvalid = 0;
   for (idcam = 0; idcam < count_cameras; idcam++)
   {
      // Get information
      cxTc0 = rox_array2d_double_collection_get(relativeposes, idcam);
      calib = rox_array2d_double_collection_get(calibrations, idcam);
      
      error = rox_array2d_double_get_data_pointer_to_pointer( &dt, corpose);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_data_pointer_to_pointer( &dk, calib);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_transformtools_estimate_relativepose_from_general(corpose, cxTc0, pose); 
      ROX_ERROR_CHECK_TERMINATE ( error );

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
         if (dist < 1.0)
         {
            countvalid++;
         }
      }
   }

   *card_consensus = countvalid;

function_terminate:
   rox_array2d_double_del(&corpose);

   return error;
}

Rox_ErrorCode rox_ransac_nonoverlap(Rox_Array2D_Double pose, Rox_Array2D_Double_Collection relativeposes, Rox_Array2D_Double_Collection calibrations, Rox_ObjSet_DynVec_Point2D_Double refs, Rox_ObjSet_DynVec_Point2D_Double curs)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint iter, count_cameras, idcam, loopcam, idpt, looppt, maxpts;
   Rox_Uint card_consensus;
   Rox_Uint max_consensus;
   Rox_Uint count_pairs;
   Rox_Uint ransac_max_iterations;
   Rox_Double q, p_q;
   Rox_ObjSet_Combination randpts_set;
   Rox_Combination randcam, randpt;
   Rox_DynVec_Point2D_Double camref, camcur;
   Rox_Point2D_Double_Struct ref1[3], cur1[3];
   Rox_Point2D_Double_Struct ref2[2], cur2[2];
   Rox_Point2D_Double_Struct ref3[2], cur3[2];
   Rox_Array2D_Double estcams[3], estpose, invrefpose, bufpose, curcalib;
   Rox_Double **dk = NULL;
   Rox_Double fu,fv,cu,cv;
   Rox_Double ifu,ifv,icu,icv;
   const Rox_Uint minsize_mss = 6;
   const Rox_Uint max_iterations = 40000;
   const Rox_Double log_probability_of_never_selecting_good_subset = log(1e-2);

   if (!pose || !relativeposes || !calibrations || !refs || !curs) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   error = rox_array2d_double_check_size(pose, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   count_cameras = rox_array2d_double_collection_get_count(relativeposes);
   
   if (count_cameras < 3) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (rox_array2d_double_collection_get_count(calibrations) != count_cameras) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (refs->used != count_cameras || curs->used != count_cameras) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   randcam = NULL;
   estpose = NULL;
   bufpose = NULL;
   invrefpose = NULL;
   randpts_set = NULL;

   // Constants
   max_consensus = 6;
   iter = 0;
   ransac_max_iterations = max_iterations;

   error = rox_array2d_double_new(&estpose, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE(error);
   
   error = rox_array2d_double_new(&bufpose, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE(error);
   
   error = rox_array2d_double_new(&invrefpose, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE(error);
   
   error = rox_combination_new(&randcam, count_cameras, 3); 
   ROX_ERROR_CHECK_TERMINATE(error);
   
   error = rox_objset_combination_new(&randpts_set, count_cameras); 
   ROX_ERROR_CHECK_TERMINATE(error);

   // Manage set of cameras measurements
   count_pairs = 0;
   for (idcam = 0; idcam < count_cameras; idcam++)
   {
      Rox_Combination toadd;

      if (curs->data[idcam]->used != refs->data[idcam]->used)
      {
         error = ROX_ERROR_INVALID_VALUE;
         ROX_ERROR_CHECK_TERMINATE(error);
      }

      if (curs->data[idcam]->used < 3)
      {
         error = ROX_ERROR_INVALID_VALUE;
         ROX_ERROR_CHECK_TERMINATE(error);
      }

      count_pairs += refs->data[idcam]->used;

      error = rox_combination_new(&toadd, refs->data[idcam]->used, 3); 
      ROX_ERROR_CHECK_TERMINATE(error);
      
      error = rox_objset_combination_append(randpts_set, toadd);
      if (error)
      {
         rox_combination_del(&toadd);
         ROX_ERROR_CHECK_TERMINATE(error);
      }
   }

   // Trials loop
   while (iter < max_iterations && iter < ransac_max_iterations)
   {
      // Update iter first, for "continue" sake
      iter++;

      // Select three cameras
      error = rox_combination_draw(randcam); 
      ROX_ERROR_CHECK_TERMINATE(error)

      // Extract points
      for (loopcam = 0; loopcam < 3; loopcam++)
      {
         idcam = randcam->draw->data[loopcam];

         curcalib = rox_array2d_double_collection_get(calibrations, idcam);
         error = rox_array2d_double_get_data_pointer_to_pointer( &dk, curcalib);
         fu = dk[0][0];
         fv = dk[1][1];
         cu = dk[0][2];
         cv = dk[1][2];
         ifu = 1.0 / fu;
         ifv = 1.0 / fv;
         icu = - cu * ifu;
         icv = - cv * ifv;

         // Save camera
         estcams[loopcam] = rox_array2d_double_collection_get(relativeposes, idcam);

         // Choose random points for this camera
         randpt = randpts_set->data[idcam];
         error = rox_combination_draw(randpt); 
         ROX_ERROR_CHECK_TERMINATE(error)

         // Select count of points
         maxpts = 2;
         if (loopcam == 0) maxpts = 3;

         // Select list of points
         camcur = curs->data[idcam];
         camref = refs->data[idcam];

         // Extract set
         for (looppt = 0; looppt < maxpts; looppt++)
         {
            idpt = randpt->draw->data[looppt];

            if (loopcam == 0)
            {
               ref1[looppt].u = ifu * camref->data[idpt].u + icu;
               ref1[looppt].v = ifv * camref->data[idpt].v + icv;
               cur1[looppt].u = ifu * camcur->data[idpt].u + icu;
               cur1[looppt].v = ifv * camcur->data[idpt].v + icv;
            }
            else if (loopcam == 1)
            {
               ref2[looppt].u = ifu * camref->data[idpt].u + icu;
               ref2[looppt].v = ifv * camref->data[idpt].v + icv;
               cur2[looppt].u = ifu * camcur->data[idpt].u + icu;
               cur2[looppt].v = ifv * camcur->data[idpt].v + icv;
            }
            else
            {
               ref3[looppt].u = ifu * camref->data[idpt].u + icu;
               ref3[looppt].v = ifv * camref->data[idpt].v + icv;
               cur3[looppt].u = ifu * camcur->data[idpt].u + icu;
               cur3[looppt].v = ifv * camcur->data[idpt].v + icv;
            }
         }
      }

      // Compute estimation
      error = rox_odometry_nooverlap_make_p7p(estpose, estcams[0], cur1, ref1, estcams[1], cur2, ref2, estcams[2], cur3, ref3); 
      if (error) continue;
      error = rox_array2d_double_svdinverse(invrefpose, estcams[0]); 
      if (error) continue;
      error = rox_array2d_double_mulmatmat(bufpose, estpose, estcams[0]); 
      if (error) continue;
      error = rox_array2d_double_mulmatmat(estpose, invrefpose, bufpose); 
      if (error) continue;

      // score
      error = rox_ransac_nonoverlap_check_consensus3d(&card_consensus, estpose, relativeposes, calibrations, refs, curs); 
      if (error) continue;

      // If score for current pose is better than ever, keep it
      if (card_consensus > max_consensus)
      {
         // Update pose
         max_consensus = card_consensus;

         rox_array2d_double_copy(pose, estpose);

         // Ransac update
         q = pow(((double)max_consensus) / ((double)count_pairs), (int)minsize_mss);
         p_q = 1.0 - q;
         if (p_q < DBL_EPSILON) ransac_max_iterations = 0;
         else
         {
            ransac_max_iterations = (int)(0.5 + (log_probability_of_never_selecting_good_subset / log(p_q)));
         }
      }
   }

function_terminate:

   rox_objset_combination_del(&randpts_set);
   rox_combination_del(&randcam);
   rox_array2d_double_del(&estpose);
   rox_array2d_double_del(&bufpose);
   rox_array2d_double_del(&invrefpose);

   return error;
}
