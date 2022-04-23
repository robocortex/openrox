//==============================================================================
//
//    OPENROX   : File linsys_generalized_geometric_weighted_premul.c
//
//    Contents  : Implementation of linsys_generalized_geometric_weighted_premul module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_generalized_geometric_weighted_premul.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point3d_struct.h>

#include <baseproc/array/fill/fillval.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_jacobian_generalized_geometric_weighted_premul (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Array2D_Double error_out, 
   const Rox_Array2D_Double weight, 
   const Rox_Array2D_Double pose, 
   const Rox_Array2D_Double E, 
   const Rox_Point2D_Double  ars, 
   const Rox_Point3D_Double  brs, 
   const Rox_Point2D_Double  acs, 
   const Rox_Point3D_Double  bcs, 
   const Rox_Uint nb_points, 
   const Rox_Uint unused_translation_id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Uint idpt, pos, k, l;

   Rox_Double r1, r2, r3;
   Rox_Double r4, r5, r6;
   Rox_Double r7, r8, r9;
   Rox_Double tx, ty, tz;
   Rox_Double w;
   Rox_Double rx,ry,rz,cx,cy,cz;
   Rox_Point2D_Double_Struct ac, ar;
   Rox_Point3D_Double_Struct br, bc;
   Rox_Double invnorm1, invnorm2;

   Rox_Double **dt, **dLtL, *dLte, *de, *dw;
   Rox_Double Jve[9][6];
   Rox_Double Jvr[9][6];
   Rox_Double Jvet[9][6];
   Rox_Double Jvrt[9][6];
   Rox_Double Jpoint[9];
   //Rox_Double Jscale[9];
   Rox_Double J[6], J5[5];

   if (!pose || !ars || !brs || !acs || !bcs || !LtL || !Lte) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (nb_points == 0) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}
   if (unused_translation_id >= 3) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_check_size(pose, 4, 4); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_check_size(error_out, nb_points * 2, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_check_size(weight, nb_points, 1); ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(LtL, 5, 5);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(Lte, 5, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );


   error = rox_array2d_double_get_data_pointer_to_pointer( &dt, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer ( &de, error_out);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer ( &dw, weight);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dLtL, LtL);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_get_data_pointer ( &dLte, Lte);
   ROX_ERROR_CHECK_TERMINATE ( error );

   rox_array2d_double_fillval(LtL, 0);
   rox_array2d_double_fillval(Lte, 0);

   r1 = dt[0][0];
   r2 = dt[0][1];
   r3 = dt[0][2];
   r4 = dt[1][0];
   r5 = dt[1][1];
   r6 = dt[1][2];
   r7 = dt[2][0];
   r8 = dt[2][1];
   r9 = dt[2][2];
   tx = dt[0][3];
   ty = dt[1][3];
   tz = dt[2][3];

#if 0
    Error function is the geometric bilateral error :
    Let xr be the reference points, xc the current points, and E the essential matrix
    E = [t]x * R
    error = ar'*R'*bc + br*R'*ac + ar*E'*ac=0

    Note that there is a matrix of coefficients C such that error = C * vec(E)

    Objective is to find the left update x for the pose :
    M = [R(x) t(x); 0 1]
    Rn = R(x)*R
    tn = R(x)*t+t(x)
    Mn = [Rn tn; 0 1]

    derror/dx = derror/dvec(E) * dvec(E)/dMn * dMn/dM * dM/dx
    Note that only derror/dvec(E) is dependent on the points so the other jacobians may be computed only once per iteration
#endif

   //dvec(R)/dx
   Jvr[0][0] = 0;
   Jvr[0][1] = 0;
   Jvr[0][2] = 0;
   Jvr[0][3] = 0;
   Jvr[0][4] = r7;
   Jvr[0][5] = -r4;
   Jvr[1][0] = 0;
   Jvr[1][1] = 0;
   Jvr[1][2] = 0;
   Jvr[1][3] = -r7;
   Jvr[1][4] = 0;
   Jvr[1][5] = r1;
   Jvr[2][0] = 0;
   Jvr[2][1] = 0;
   Jvr[2][2] = 0;
   Jvr[2][3] = r4;
   Jvr[2][4] = -r1;
   Jvr[2][5] = 0;
   Jvr[3][0] = 0;
   Jvr[3][1] = 0;
   Jvr[3][2] = 0;
   Jvr[3][3] = 0;
   Jvr[3][4] = r8;
   Jvr[3][5] = -r5;
   Jvr[4][0] = 0;
   Jvr[4][1] = 0;
   Jvr[4][2] = 0;
   Jvr[4][3] = -r8;
   Jvr[4][4] = 0;
   Jvr[4][5] = r2;
   Jvr[5][0] = 0;
   Jvr[5][1] = 0;
   Jvr[5][2] = 0;
   Jvr[5][3] = r5;
   Jvr[5][4] = -r2;
   Jvr[5][5] = 0;
   Jvr[6][0] = 0;
   Jvr[6][1] = 0;
   Jvr[6][2] = 0;
   Jvr[6][3] = 0;
   Jvr[6][4] = r9;
   Jvr[6][5] = -r6;
   Jvr[7][0] = 0;
   Jvr[7][1] = 0;
   Jvr[7][2] = 0;
   Jvr[7][3] = -r9;
   Jvr[7][4] = 0;
   Jvr[7][5] = r3;
   Jvr[8][0] = 0;
   Jvr[8][1] = 0;
   Jvr[8][2] = 0;
   Jvr[8][3] = r6;
   Jvr[8][4] = -r3;
   Jvr[8][5] = 0;

   //dvec(R')/dx
   Jvrt[0][0] = 0;
   Jvrt[0][1] = 0;
   Jvrt[0][2] = 0;
   Jvrt[0][3] = 0;
   Jvrt[0][4] = r7;
   Jvrt[0][5] = -r4;
   Jvrt[1][0] = 0;
   Jvrt[1][1] = 0;
   Jvrt[1][2] = 0;
   Jvrt[1][3] = 0;
   Jvrt[1][4] = r8;
   Jvrt[1][5] = -r5;
   Jvrt[2][0] = 0;
   Jvrt[2][1] = 0;
   Jvrt[2][2] = 0;
   Jvrt[2][3] = 0;
   Jvrt[2][4] = r9;
   Jvrt[2][5] = -r6;
   Jvrt[3][0] = 0;
   Jvrt[3][1] = 0;
   Jvrt[3][2] = 0;
   Jvrt[3][3] = -r7;
   Jvrt[3][4] = 0;
   Jvrt[3][5] = r1;
   Jvrt[4][0] = 0;
   Jvrt[4][1] = 0;
   Jvrt[4][2] = 0;
   Jvrt[4][3] = -r8;
   Jvrt[4][4] = 0;
   Jvrt[4][5] = r2;
   Jvrt[5][0] = 0;
   Jvrt[5][1] = 0;
   Jvrt[5][2] = 0;
   Jvrt[5][3] = -r9;
   Jvrt[5][4] = 0;
   Jvrt[5][5] = r3;
   Jvrt[6][0] = 0;
   Jvrt[6][1] = 0;
   Jvrt[6][2] = 0;
   Jvrt[6][3] = r4;
   Jvrt[6][4] = -r1;
   Jvrt[6][5] = 0;
   Jvrt[7][0] = 0;
   Jvrt[7][1] = 0;
   Jvrt[7][2] = 0;
   Jvrt[7][3] = r5;
   Jvrt[7][4] = -r2;
   Jvrt[7][5] = 0;
   Jvrt[8][0] = 0;
   Jvrt[8][1] = 0;
   Jvrt[8][2] = 0;
   Jvrt[8][3] = r6;
   Jvrt[8][4] = -r3;
   Jvrt[8][5] = 0;

   //dvec(E)/dx
   Jve[0][0] = 0;
   Jve[0][1] = r7;
   Jve[0][2] = -r4;
   Jve[0][3] = 0;
   Jve[0][4] = -ty * r1 + tx * r4;
   Jve[0][5] = -tz * r1 + tx * r7;
   Jve[1][0] = -r7;
   Jve[1][1] = 0;
   Jve[1][2] = r1;
   Jve[1][3] = ty * r1 - tx * r4;
   Jve[1][4] = 0;
   Jve[1][5] = -tz * r4 + ty * r7;
   Jve[2][0] = r4;
   Jve[2][1] = -r1;
   Jve[2][2] = 0;
   Jve[2][3] = tz * r1 - tx * r7;
   Jve[2][4] = tz * r4 - ty * r7;
   Jve[2][5] = 0;
   Jve[3][0] = 0;
   Jve[3][1] = r8;
   Jve[3][2] = -r5;
   Jve[3][3] = 0;
   Jve[3][4] = -ty * r2 + tx * r5;
   Jve[3][5] = -tz * r2 + tx * r8;
   Jve[4][0] = -r8;
   Jve[4][1] = 0;
   Jve[4][2] = r2;
   Jve[4][3] = ty * r2 - tx * r5;
   Jve[4][4] = 0;
   Jve[4][5] = -tz * r5 + ty * r8;
   Jve[5][0] = r5;
   Jve[5][1] = -r2;
   Jve[5][2] = 0;
   Jve[5][3] = tz * r2 - tx * r8;
   Jve[5][4] = tz * r5 - ty * r8;
   Jve[5][5] = 0;
   Jve[6][0] = 0;
   Jve[6][1] = r9;
   Jve[6][2] = -r6;
   Jve[6][3] = 0;
   Jve[6][4] = -ty * r3 + tx * r6;
   Jve[6][5] = -tz * r3 + tx * r9;
   Jve[7][0] = -r9;
   Jve[7][1] = 0;
   Jve[7][2] = r3;
   Jve[7][3] = ty * r3 - tx * r6;
   Jve[7][4] = 0;
   Jve[7][5] = -tz * r6 + ty * r9;
   Jve[8][0] = r6;
   Jve[8][1] = -r3;
   Jve[8][2] = 0;
   Jve[8][3] = tz * r3 - tx * r9;
   Jve[8][4] = tz * r6 - ty * r9;
   Jve[8][5] = 0;

   //dvec(E')/dx
   Jvet[0][0] = 0;
   Jvet[0][1] = r7;
   Jvet[0][2] = -r4;
   Jvet[0][3] = 0;
   Jvet[0][4] = -ty * r1 + tx * r4;
   Jvet[0][5] = -tz * r1 + tx * r7;
   Jvet[1][0] = 0;
   Jvet[1][1] = r8;
   Jvet[1][2] = -r5;
   Jvet[1][3] = 0;
   Jvet[1][4] = -ty * r2 + tx * r5;
   Jvet[1][5] = -tz * r2 + tx * r8;
   Jvet[2][0] = 0;
   Jvet[2][1] = r9;
   Jvet[2][2] = -r6;
   Jvet[2][3] = 0;
   Jvet[2][4] = -ty * r3 + tx * r6;
   Jvet[2][5] = -tz * r3 + tx * r9;
   Jvet[3][0] = -r7;
   Jvet[3][1] = 0;
   Jvet[3][2] = r1;
   Jvet[3][3] = ty * r1 - tx * r4;
   Jvet[3][4] = 0;
   Jvet[3][5] = -tz * r4 + ty * r7;
   Jvet[4][0] = -r8;
   Jvet[4][1] = 0;
   Jvet[4][2] = r2;
   Jvet[4][3] = ty * r2 - tx * r5;
   Jvet[4][4] = 0;
   Jvet[4][5] = -tz * r5 + ty * r8;
   Jvet[5][0] = -r9;
   Jvet[5][1] = 0;
   Jvet[5][2] = r3;
   Jvet[5][3] = ty * r3 - tx * r6;
   Jvet[5][4] = 0;
   Jvet[5][5] = -tz * r6 + ty * r9;
   Jvet[6][0] = r4;
   Jvet[6][1] = -r1;
   Jvet[6][2] = 0;
   Jvet[6][3] = tz * r1 - tx * r7;
   Jvet[6][4] = tz * r4 - ty * r7;
   Jvet[6][5] = 0;
   Jvet[7][0] = r5;
   Jvet[7][1] = -r2;
   Jvet[7][2] = 0;
   Jvet[7][3] = tz * r2 - tx * r8;
   Jvet[7][4] = tz * r5 - ty * r8;
   Jvet[7][5] = 0;
   Jvet[8][0] = r6;
   Jvet[8][1] = -r3;
   Jvet[8][2] = 0;
   Jvet[8][3] = tz * r3 - tx * r9;
   Jvet[8][4] = tz * r6 - ty * r9;
   Jvet[8][5] = 0;

   for (idpt = 0; idpt < nb_points; idpt++)
   {
      w = dw[idpt];
      ac = acs[idpt];
      ar = ars[idpt];
      bc = bcs[idpt];
      br = brs[idpt];

      //norm1 = sqrt(bc.X*bc.X + bc.Y*bc.Y + bc.Z*bc.Z) * sqrt(ar.u*ar.u + ar.v*ar.v + 1.0);
      invnorm1 = 1.0;
      //if (norm1 > DBL_EPSILON) invnorm1 = 1.0 / norm1;
      //norm2 = sqrt(ac.u*ac.u + ac.v*ac.v + 1.0) * sqrt(br.X*br.X + br.Y*br.Y + br.Z*br.Z);
      invnorm2 = 1.0;
      //if (norm2 > DBL_EPSILON) invnorm2 = 1.0 / norm2;

      //ar*R'*bc=0
      rx = bc.X;
      ry = bc.Y;
      rz = bc.Z;
      cx = ar.u;
      cy = ar.v;
      cz = 1.0;

      Jpoint[0] = rx * cx;
      Jpoint[1] = rx * cy;
      Jpoint[2] = rx * cz;
      Jpoint[3] = ry * cx;
      Jpoint[4] = ry * cy;
      Jpoint[5] = ry * cz;
      Jpoint[6] = rz * cx;
      Jpoint[7] = rz * cy;
      Jpoint[8] = rz * cz;

      //Error : scale*x'*T*x where scale = 1 / (||pref|| * ||pcur||)
      //derror/dvec(E) = dalgdist/dE * scale 

      //Computing jacobian row
      pos = 0;
      for (k = 0; k < 6; k++)
      {
         J[k] = 0;

         for (l = 0; l < 9; l++)
         {
            J[k] += invnorm1 * Jpoint[l] * Jvrt[l][k];
         }

         if (k != unused_translation_id)
         {
            J5[pos] = -J[k] * w;
            pos++;
         }
      }

      //+br*R'*ac
      rx = ac.u;
      ry = ac.v;
      rz = 1.0;
      cx = br.X;
      cy = br.Y;
      cz = br.Z;

      Jpoint[0] = rx * cx;
      Jpoint[1] = rx * cy;
      Jpoint[2] = rx * cz;
      Jpoint[3] = ry * cx;
      Jpoint[4] = ry * cy;
      Jpoint[5] = ry * cz;
      Jpoint[6] = rz * cx;
      Jpoint[7] = rz * cy;
      Jpoint[8] = rz * cz;

      //Computing jacobian row
      pos = 0;
      for (k = 0; k < 6; k++)
      {
         J[k] = 0;

         for (l = 0; l < 9; l++)
         {
            J[k] += invnorm2 * Jpoint[l] * Jvrt[l][k];
         }

         if (k != unused_translation_id)
         {
            J5[pos] += -J[k] * w;
            pos++;
         }
      }

      //+ar*E'*ac
      rx = ac.u;
      ry = ac.v;
      rz = 1.0;
      cx = ar.u;
      cy = ar.v;
      cz = 1.0;
      
#if 0
      //Error : scale*x'*E*x where scale = sqrt((E*x)1^2 + (E*x)2^2)
      //derror/dvec(E) = dscale/dE * algdist + dalgdist/dE * scale 
      a = dE[0][0] * rx + dE[1][0] * ry + dE[2][0];
      b = dE[0][1] * rx + dE[1][1] * ry + dE[2][1];
      c = dE[0][2] * rx + dE[1][2] * ry + dE[2][2];
      algdist = a * cx + b * cy + c;
      scale = 1.0 / sqrt(a * a + b * b);

      Jscale[0] = dE[0][0] * rx * rx + rx * dE[1][0] * ry + dE[2][0] * rx;
      Jscale[1] = dE[0][0] * rx * ry + dE[1][0] * ry * ry + ry * dE[2][0];
      Jscale[2] = dE[0][0] * rx + dE[1][0] * ry + dE[2][0];
      Jscale[3] = dE[0][1] * rx * rx + rx * dE[1][1] * ry + rx * dE[2][1];
      Jscale[4] = dE[0][1] * rx * ry + dE[1][1] * ry * ry + dE[2][1] * ry;
      Jscale[5] = dE[0][1] * rx + dE[1][1] * ry + dE[2][1];
      Jscale[6] = 0;
      Jscale[7] = 0;
      Jscale[8] = 0;
#endif

      Jpoint[0] = rx * cx;
      Jpoint[1] = rx * cy;
      Jpoint[2] = rx * cz;
      Jpoint[3] = ry * cx;
      Jpoint[4] = ry * cy;
      Jpoint[5] = ry * cz;
      Jpoint[6] = rz * cx;
      Jpoint[7] = rz * cy;
      Jpoint[8] = rz * cz;

#if 0
      Jpoint[0] = Jpoint[0] * scale + Jscale[0] * scale * algdist;
      Jpoint[1] = Jpoint[1] * scale + Jscale[1] * scale * algdist;
      Jpoint[2] = Jpoint[2] * scale + Jscale[2] * scale * algdist;
      Jpoint[3] = Jpoint[3] * scale + Jscale[3] * scale * algdist;
      Jpoint[4] = Jpoint[4] * scale + Jscale[4] * scale * algdist;
      Jpoint[5] = Jpoint[5] * scale + Jscale[5] * scale * algdist;
      Jpoint[6] = Jpoint[6] * scale + Jscale[6] * scale * algdist;
      Jpoint[7] = Jpoint[7] * scale + Jscale[7] * scale * algdist;
      Jpoint[8] = Jpoint[8] * scale + Jscale[8] * scale * algdist;
#endif

      //Computing jacobian row
      pos = 0;
      for (k = 0; k < 6; k++)
      {
         J[k] = 0;

         for (l = 0; l < 9; l++)
         {
            J[k] += Jpoint[l] * Jvet[l][k];
         }

         if (k != unused_translation_id)
         {
            J5[pos] += -J[k] * w;
            pos++;
         }
      }


      for (k = 0; k < 5; k++)
      {
         for (l = 0; l < 5; l++)
         {
            dLtL[k][l] += J5[k] * J5[l];
         }

         dLte[k] += J5[k] * de[idpt * 2];
      }

      //TRANSPOSE

      //bc*R*ar=0
      cx = bc.X;
      cy = bc.Y;
      cz = bc.Z;
      rx = ar.u;
      ry = ar.v;
      rz = 1.0;

      Jpoint[0] = rx * cx;
      Jpoint[1] = rx * cy;
      Jpoint[2] = rx * cz;
      Jpoint[3] = ry * cx;
      Jpoint[4] = ry * cy;
      Jpoint[5] = ry * cz;
      Jpoint[6] = rz * cx;
      Jpoint[7] = rz * cy;
      Jpoint[8] = rz * cz;

      //Computing jacobian row
      pos = 0;
      for (k = 0; k < 6; k++)
      {
         J[k] = 0;

         for (l = 0; l < 9; l++)
         {
            J[k] += invnorm1 * Jpoint[l] * Jvr[l][k];
         }

         if (k != unused_translation_id)
         {
            J5[pos] = -J[k] * w;
            pos++;
         }
      }

      //+ac*R*br
      cx = ac.u;
      cy = ac.v;
      cz = 1.0;
      rx = br.X;
      ry = br.Y;
      rz = br.Z;

      Jpoint[0] = rx * cx;
      Jpoint[1] = rx * cy;
      Jpoint[2] = rx * cz;
      Jpoint[3] = ry * cx;
      Jpoint[4] = ry * cy;
      Jpoint[5] = ry * cz;
      Jpoint[6] = rz * cx;
      Jpoint[7] = rz * cy;
      Jpoint[8] = rz * cz;

      // Computing jacobian row
      pos = 0;
      for (k = 0; k < 6; k++)
      {
         J[k] = 0;

         for (l = 0; l < 9; l++)
         {
            J[k] += invnorm2 * Jpoint[l] * Jvr[l][k];
         }

         if (k != unused_translation_id)
         {
            J5[pos] += -J[k] * w;
            pos++;
         }
      }

      //+ac*E*ar
      cx = ac.u;
      cy = ac.v;
      cz = 1.0;
      rx = ar.u;
      ry = ar.v;
      rz = 1.0;

#if 0
      a = dE[0][0] * cx + dE[0][1] * cy + dE[0][2];
      b = dE[1][0] * cx + dE[1][1] * cy + dE[1][2];
      c = dE[2][0] * cx + dE[2][1] * cy + dE[2][2];
      algdist = a * rx + b * ry + c;
      scale = 1.0 / sqrt(a * a + b * b);

      Jscale[0] = dE[0][0] * rx * rx + dE[0][1] * rx * ry + dE[0][2] * rx;
      Jscale[1] = dE[1][0] * rx * rx + rx * dE[1][1] * ry + rx * dE[1][2];
      Jscale[2] = 0;
      Jscale[3] = dE[0][0] * rx * ry + dE[0][1] * ry * ry + ry * dE[0][2];
      Jscale[4] = rx * dE[1][0] * ry + dE[1][1] * ry * ry + dE[1][2] * ry;
      Jscale[5] = 0;
      Jscale[6] = dE[0][0] * rx + dE[0][1] * ry + dE[0][2];
      Jscale[7] = dE[1][0] * rx + dE[1][1] * ry + dE[1][2];
      Jscale[8] = 0;
#endif

      Jpoint[0] = rx * cx;
      Jpoint[1] = rx * cy;
      Jpoint[2] = rx * cz;
      Jpoint[3] = ry * cx;
      Jpoint[4] = ry * cy;
      Jpoint[5] = ry * cz;
      Jpoint[6] = rz * cx;
      Jpoint[7] = rz * cy;
      Jpoint[8] = rz * cz;

#if 0
      Jpoint[0] = Jpoint[0] * scale + Jscale[0] * scale * algdist;
      Jpoint[1] = Jpoint[1] * scale + Jscale[1] * scale * algdist;
      Jpoint[2] = Jpoint[2] * scale + Jscale[2] * scale * algdist;
      Jpoint[3] = Jpoint[3] * scale + Jscale[3] * scale * algdist;
      Jpoint[4] = Jpoint[4] * scale + Jscale[4] * scale * algdist;
      Jpoint[5] = Jpoint[5] * scale + Jscale[5] * scale * algdist;
      Jpoint[6] = Jpoint[6] * scale + Jscale[6] * scale * algdist;
      Jpoint[7] = Jpoint[7] * scale + Jscale[7] * scale * algdist;
      Jpoint[8] = Jpoint[8] * scale + Jscale[8] * scale * algdist;
#endif

      //Computing jacobian row
      pos = 0;
      for (k = 0; k < 6; k++)
      {
         J[k] = 0;

         for (l = 0; l < 9; l++)
         {
            J[k] += Jpoint[l] * Jve[l][k];
         }

         if (k != unused_translation_id)
         {
            J5[pos] += -J[k] * w;
            pos++;
         }
      }


      for (k = 0; k < 5; k++)
      {
         for (l = 0; l < 5; l++)
         {
            dLtL[k][l] += J5[k] * J5[l];
         }

         dLte[k] += J5[k] * de[idpt * 2 + 1];
      }
   }

function_terminate:
   return error;
}
