//==============================================================================
//
//    OPENROX   : File p7p.h
//
//    Contents  : API of p7p module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "p7p.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/decomposition/svd.h>
#include <baseproc/array/decomposition/svdsort.h>
#include <baseproc/array/decomposition/qr.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/transpose/transpose.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/geometry/point/point3d_matse3_transform.h>
#include <baseproc/geometry/line/line_from_points.h>
#include <baseproc/geometry/line/line_transform.h>
#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point3d_struct.h>

#include <core/indirect/multinonoverlap/linear7pts.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_odometry_nooverlap_make_p7l(Rox_Array2D_Double pose, Rox_Point2D_Double  gac, Rox_Point3D_Double gbc, Rox_Point2D_Double gar, Rox_Point3D_Double gbr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double C, subC, subCt;
   Rox_Array2D_Double QRQ;
   Rox_Array2D_Double QRR;
   Rox_Array2D_Double QRP;
   Rox_Array2D_Double M;
   Rox_Array2D_Double MU;
   Rox_Array2D_Double MS;
   Rox_Array2D_Double MV;
   Rox_Array2D_Double Ams;
   Rox_Array2D_Double iAms;
   Rox_Array2D_Double Aps;
   Rox_Array2D_Double R;
   Rox_Array2D_Double vecT;
   Rox_Array2D_Double vec;

   Rox_Double ** dc, **dams, **daps, **dvec, **dmv, **dm, **dq;
   Rox_Point2D_Double_Struct ar[4], ac[4];
   Rox_Point3D_Double_Struct br[4], bc[4];

   Rox_Double zs,ys,xs,z2s,y2s,x2s,xe,ye,ze;
   int r1,r2,r3,r4,pos;

   if (!pose || !gac || !gbc || !gar || !gbr) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_check_size(pose, 4, 4); ROX_ERROR_CHECK_TERMINATE ( error );

   subC = NULL;
   subCt = NULL;
   C = NULL;
   QRQ = NULL;
   QRR = NULL;
   QRP = NULL;
   M = NULL;
   MU = NULL;
   MS = NULL;
   MV = NULL;
   R = NULL;
   vecT = NULL;
   Ams = NULL;
   iAms = NULL;
   Aps = NULL;
   vec = NULL;

   error = rox_array2d_double_new(&C, 7 * 35, 165); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new_subarray2d(&subC, C, 0, 0, 7 * 35, 164); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&subCt, 164, 7 * 35); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&QRQ, 164, 164); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&QRR, 164, 7 * 35); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&QRP, 7 * 35, 7 * 35); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&M, 7, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&MU, 7, 7); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&MS, 4, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&MV, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new_subarray2d(&R, pose, 0, 0, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new_subarray2d(&vecT, pose, 0, 3, 3, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&Ams, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&iAms, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&Aps, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&vec, 3, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dc, C); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer( &dm, M); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer( &dvec, vec); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer( &dmv, MV); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer( &dams, Ams); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer( &daps, Aps); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer( &dq, QRQ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Make 4 from 7 combinations (35)
   pos = 0;
   for (r1 = 0; r1 < 7; r1++)
   {
      for (r2 = r1 + 1; r2 < 7; r2++)
      {
         for (r3 = r2 + 1; r3 < 7; r3++)
         {
            for (r4 = r3 + 1; r4 < 7; r4++)
            {
               ac[0] = gac[r1];
               ar[0] = gar[r1];
               bc[0] = gbc[r1];
               br[0] = gbr[r1];
               ac[1] = gac[r2];
               ar[1] = gar[r2];
               bc[1] = gbc[r2];
               br[1] = gbr[r2];
               ac[2] = gac[r3];
               ar[2] = gar[r3];
               bc[2] = gbc[r3];
               br[2] = gbr[r3];
               ac[3] = gac[r4];
               ar[3] = gar[r4];
               bc[3] = gbc[r4];
               br[3] = gbr[r4];

               //Build 7 equations from this set of four points
               make_equations(&dc[pos], ac, bc, ar, br);
               pos+=7;
            }
         }
      }
   }

   error = rox_array2d_double_transpose(subCt, subC); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_qrp(QRQ, QRR, QRP, subCt); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   zs = dq[163][163];
   ys = dq[162][163];
   xs = dq[161][163];
   z2s = dq[160][163];
   y2s = dq[158][163];
   x2s = dq[155][163];

   xe = xs;
   if (fabs(xs) > sqrt(DBL_EPSILON))
   {
      xe = x2s / xs;
   }

   ye = ys;
   if (fabs(ys) > sqrt(DBL_EPSILON))
   {
      ye = y2s / ys;
   }

   ze = zs;
   if (fabs(zs) > sqrt(DBL_EPSILON))
   {
      ze = z2s / zs;
   }

   error = rox_array2d_double_fillunit(pose); 
   ROX_ERROR_CHECK_TERMINATE(error)

   dams[0][0] = 1; dams[0][1] = ze; dams[0][2] = -ye;
   dams[1][0] = -ze; dams[1][1] = 1; dams[1][2] = xe;
   dams[2][0] = ye; dams[2][1] = -xe; dams[2][2] = 1;
   daps[0][0] = 1; daps[0][1] = -ze; daps[0][2] = ye;
   daps[1][0] = ze; daps[1][1] = 1; daps[1][2] = -xe;
   daps[2][0] = -ye; daps[2][1] = xe; daps[2][2] = 1;

   error = rox_array2d_double_svdinverse(iAms, Ams); 
   ROX_ERROR_CHECK_TERMINATE(error);

   error = rox_array2d_double_mulmatmat(R, iAms, Aps); 
   ROX_ERROR_CHECK_TERMINATE(error);

   makeMt(dm, gac, gbc, gar, gbr, xe, ye, ze);

   error = rox_array2d_double_svd(MU,MS,MV,M); 
   ROX_ERROR_CHECK_TERMINATE(error);

   error = rox_array2d_double_svd_sort(MU,MS,MV); 
   ROX_ERROR_CHECK_TERMINATE(error);

   dvec[0][0] = dmv[0][3] / dmv[3][3];
   dvec[1][0] = dmv[1][3] / dmv[3][3];
   dvec[2][0] = dmv[2][3] / dmv[3][3];

   error = rox_array2d_double_mulmatmat(vecT, iAms, vec); 
   ROX_ERROR_CHECK_TERMINATE(error);

function_terminate:
   rox_array2d_double_del(&subC);
   rox_array2d_double_del(&subCt);
   rox_array2d_double_del(&C);
   rox_array2d_double_del(&QRQ);
   rox_array2d_double_del(&QRR);
   rox_array2d_double_del(&QRP);
   rox_array2d_double_del(&M);
   rox_array2d_double_del(&MU);
   rox_array2d_double_del(&MS);
   rox_array2d_double_del(&MV);
   rox_array2d_double_del(&R);
   rox_array2d_double_del(&vecT);
   rox_array2d_double_del(&Aps);
   rox_array2d_double_del(&Ams);
   rox_array2d_double_del(&iAms);
   rox_array2d_double_del(&vec);

   return error;
}

Rox_ErrorCode rox_odometry_nooverlap_make_p7p(Rox_Array2D_Double pose, Rox_Array2D_Double cam1To, Rox_Point2D_Double  curcam1, Rox_Point2D_Double  refcam1, Rox_Array2D_Double cam2To, Rox_Point2D_Double  curcam2, Rox_Point2D_Double  refcam2, Rox_Array2D_Double cam3To, Rox_Point2D_Double  curcam3, Rox_Point2D_Double  refcam3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double oTcam, c1Tc2, c1Tc3;
   Rox_Point3D_Double_Struct origin, pt;
   Rox_Line3D_Plucker_Struct plucker, transformed_plucker;
   
   Rox_Uint pos;

   Rox_Point2D_Double_Struct ar[7], ac[7];
   Rox_Point3D_Double_Struct br[7], bc[7];

   if (!pose || !cam1To || !cam2To || !cam3To) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }
   if (!curcam1 || !curcam2 || !curcam3) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }
   if (!refcam1 || !refcam2 || !refcam3) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   error = rox_array2d_double_check_size(pose, 4, 4); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_check_size(cam1To, 4, 4); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_check_size(cam2To, 4, 4); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_check_size(cam3To, 4, 4); ROX_ERROR_CHECK_TERMINATE ( error );

   oTcam = NULL;
   c1Tc2 = NULL;
   c1Tc3 = NULL;

   error = rox_array2d_double_new(&oTcam, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE(error);
   error = rox_array2d_double_new(&c1Tc2, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE(error);
   error = rox_array2d_double_new(&c1Tc3, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE(error);

   error = rox_array2d_double_svdinverse(oTcam, cam2To); 
   ROX_ERROR_CHECK_TERMINATE(error);
   error = rox_array2d_double_mulmatmat(c1Tc2, cam1To, oTcam); 
   ROX_ERROR_CHECK_TERMINATE(error);
   error = rox_array2d_double_svdinverse(oTcam, cam3To); 
   ROX_ERROR_CHECK_TERMINATE(error);
   error = rox_array2d_double_mulmatmat(c1Tc3, cam1To, oTcam); 
   ROX_ERROR_CHECK_TERMINATE(error);

   origin.X = 0;
   origin.Y = 0;
   origin.Z = 0;
   pos = 0;

   for ( Rox_Sint k = 0; k < 3; k++)
   {
      pt.X = refcam1[k].u;
      pt.Y = refcam1[k].v;
      pt.Z = 1;
      
      rox_line3d_plucker_from_2_point3d(&plucker, &origin, &pt);

      ar[pos].u = plucker.displacement[0] / plucker.displacement[2];
      ar[pos].v = plucker.displacement[1] / plucker.displacement[2];
      br[pos].X = plucker.moment[0] / plucker.displacement[2];
      br[pos].Y = plucker.moment[1] / plucker.displacement[2];
      br[pos].Z = plucker.moment[2] / plucker.displacement[2];

      pt.X = curcam1[k].u;
      pt.Y = curcam1[k].v;
      pt.Z = 1;
      
      rox_line3d_plucker_from_2_point3d(&plucker, &origin, &pt);

      ac[pos].u = plucker.displacement[0] / plucker.displacement[2];
      ac[pos].v = plucker.displacement[1] / plucker.displacement[2];
      bc[pos].X = plucker.moment[0] / plucker.displacement[2];
      bc[pos].Y = plucker.moment[1] / plucker.displacement[2];
      bc[pos].Z = plucker.moment[2] / plucker.displacement[2];

      pos++;
   }

   for ( Rox_Sint k = 0; k < 2; k++)
   {
      pt.X = refcam2[k].u;
      pt.Y = refcam2[k].v;
      pt.Z = 1;
      
      rox_line3d_plucker_from_2_point3d(&plucker, &origin, &pt);
      rox_line3d_plucker_transform(&transformed_plucker, &plucker, c1Tc2);

      ar[pos].u = transformed_plucker.displacement[0] / transformed_plucker.displacement[2];
      ar[pos].v = transformed_plucker.displacement[1] / transformed_plucker.displacement[2];
      br[pos].X = transformed_plucker.moment[0] / transformed_plucker.displacement[2];
      br[pos].Y = transformed_plucker.moment[1] / transformed_plucker.displacement[2];
      br[pos].Z = transformed_plucker.moment[2] / transformed_plucker.displacement[2];

      pt.X = curcam2[k].u;
      pt.Y = curcam2[k].v;
      pt.Z = 1;
      
      rox_line3d_plucker_from_2_point3d(&plucker, &origin, &pt);
      rox_line3d_plucker_transform(&transformed_plucker, &plucker, c1Tc2);

      ac[pos].u = transformed_plucker.displacement[0] / transformed_plucker.displacement[2];
      ac[pos].v = transformed_plucker.displacement[1] / transformed_plucker.displacement[2];
      bc[pos].X = transformed_plucker.moment[0] / transformed_plucker.displacement[2];
      bc[pos].Y = transformed_plucker.moment[1] / transformed_plucker.displacement[2];
      bc[pos].Z = transformed_plucker.moment[2] / transformed_plucker.displacement[2];

      pos++;
   }

   for ( Rox_Sint k = 0; k < 2; k++)
   {
      pt.X = refcam3[k].u;
      pt.Y = refcam3[k].v;
      pt.Z = 1;
      
      rox_line3d_plucker_from_2_point3d(&plucker, &origin, &pt);
      rox_line3d_plucker_transform(&transformed_plucker, &plucker, c1Tc3);

      ar[pos].u = transformed_plucker.displacement[0] / transformed_plucker.displacement[2];
      ar[pos].v = transformed_plucker.displacement[1] / transformed_plucker.displacement[2];
      br[pos].X = transformed_plucker.moment[0] / transformed_plucker.displacement[2];
      br[pos].Y = transformed_plucker.moment[1] / transformed_plucker.displacement[2];
      br[pos].Z = transformed_plucker.moment[2] / transformed_plucker.displacement[2];

      pt.X = curcam3[k].u;
      pt.Y = curcam3[k].v;
      pt.Z = 1;
      
      rox_line3d_plucker_from_2_point3d(&plucker, &origin, &pt);
      rox_line3d_plucker_transform(&transformed_plucker, &plucker, c1Tc3);

      ac[pos].u = transformed_plucker.displacement[0] / transformed_plucker.displacement[2];
      ac[pos].v = transformed_plucker.displacement[1] / transformed_plucker.displacement[2];
      bc[pos].X = transformed_plucker.moment[0] / transformed_plucker.displacement[2];
      bc[pos].Y = transformed_plucker.moment[1] / transformed_plucker.displacement[2];
      bc[pos].Z = transformed_plucker.moment[2] / transformed_plucker.displacement[2];

      pos++;
   }

   error = rox_odometry_nooverlap_make_p7l(pose, ar, br, ac, bc); 
   ROX_ERROR_CHECK_TERMINATE(error);

function_terminate:

   rox_array2d_double_del(&oTcam);
   rox_array2d_double_del(&c1Tc2);
   rox_array2d_double_del(&c1Tc3);

   return error;
}
