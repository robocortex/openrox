//==============================================================================
//
//    OPENROX   : File linsys_essential_geometric_weighted_premul.c
//
//    Contents  : Implementation of essential_geometric_weighted_premul module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_essential_geometric_weighted_premul.h"

#include <baseproc/maths/maths_macros.h>

#include <baseproc/array/fill/fillval.h>
#include <baseproc/geometry/point/point2d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_jacobian_essential_geometric_weighted_premul (
   Rox_Matrix LtL,
   Rox_Matrix Lte,
   const Rox_Array2D_Double error_out,
   const Rox_Array2D_Double weight,
   const Rox_Array2D_Double pose,
   const Rox_Array2D_Double E,
   const Rox_Point2D_Float  refs,
   const Rox_Point2D_Float  curs,
   const Rox_Uint nb_points,
   const Rox_Uint unused_translation_id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double r1, r2, r3;
   Rox_Double r4, r5, r6;
   Rox_Double r7, r8, r9;
   Rox_Double tx, ty, tz;
   Rox_Double ru, rv, cu, cv;
   Rox_Double a, b, c, algdist, scale, commondenom, w;

   Rox_Double **dt, **dLtL, *dLte, *de, **dE, *dw;
   Rox_Double Jve[9][6];
   Rox_Double Jpoint[9];
   Rox_Double Jscale[9];
   Rox_Double J[6], J5[5];

   if (!pose || !refs || !curs || !LtL || !Lte)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (nb_points == 0)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (unused_translation_id >= 3)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(error_out, nb_points * 2, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(weight, nb_points, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

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

   error = rox_array2d_double_get_data_pointer_to_pointer ( &dE, E);
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

   // Error function is the geometric bilateral error :
   // Let xr be the reference points, xc the current points, and E the essential matrix
   // E = [t]x * R
   // error = xc' * E * xr

   // Note that there is a matrix of coefficients C such that error = C * vec(E)

   // Objective is to find the left update x for the pose :
   // M = [R(x) t(x); 0 1]
   // Rn = R(x)*R
   // tn = R(x)*t+t(x)
   // Mn = [Rn tn; 0 1]

   // derror/dx = derror/dvec(E) * dvec(E)/dMn * dMn/dM * dM/dx
   // Note that only derror/dvec(E) is dependent on the points so the other jacobians may be computed only once per iteration

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

   for (Rox_Uint idpt = 0; idpt < nb_points; idpt++)
   {
      ru = refs[idpt].u;
      rv = refs[idpt].v;
      cu = curs[idpt].u;
      cv = curs[idpt].v;
      w = dw[idpt];

      // Error : scale*x'*E*x where scale = sqrt((E*x)1^2 + (E*x)2^2)
      // derror/dvec(E) = dscale/dE * algdist + dalgdist/dE * scale

      // Direct
      a = dE[0][0] * ru + dE[0][1] * rv + dE[0][2];
      b = dE[1][0] * ru + dE[1][1] * rv + dE[1][2];
      c = dE[2][0] * ru + dE[2][1] * rv + dE[2][2];
      algdist = a * cu + b * cv + c;
      scale = 1.0 / sqrt(a * a + b * b);

      commondenom = 1.0 / (pow(dE[0][0] * dE[0][0] * ru * ru + 0.2e1 * ru * (dE[0][1] * rv + dE[0][2]) * dE[0][0] + dE[0][1] * dE[0][1] * rv * rv + 0.2e1 * dE[0][1] * rv * dE[0][2] + dE[0][2] * dE[0][2] + pow(dE[1][0] * ru + dE[1][1] * rv + dE[1][2], 0.2e1), 0.3e1 / 0.2e1));

      Jscale[0] = (-ru * (dE[0][0] * ru + dE[0][1] * rv + dE[0][2])) * commondenom;
      Jscale[1] = (-ru * (dE[1][0] * ru + dE[1][1] * rv + dE[1][2])) * commondenom;
      Jscale[2] = 0;
      Jscale[3] = (-rv * (dE[0][0] * ru + dE[0][1] * rv + dE[0][2])) * commondenom;
      Jscale[4] = (-rv * (dE[1][0] * ru + dE[1][1] * rv + dE[1][2])) * commondenom;
      Jscale[5] = 0;
      Jscale[6] = (-dE[0][0] * ru - dE[0][1] * rv - dE[0][2]) * commondenom;
      Jscale[7] = (-dE[1][0] * ru - dE[1][1] * rv - dE[1][2]) * commondenom;
      Jscale[8] = 0;

      Jpoint[0] = ru * cu;
      Jpoint[1] = ru * cv;
      Jpoint[2] = ru;
      Jpoint[3] = rv * cu;
      Jpoint[4] = rv * cv;
      Jpoint[5] = rv;
      Jpoint[6] = cu;
      Jpoint[7] = cv;
      Jpoint[8] = 1;

      Jpoint[0] = Jpoint[0] * scale + Jscale[0] * algdist;
      Jpoint[1] = Jpoint[1] * scale + Jscale[1] * algdist;
      Jpoint[2] = Jpoint[2] * scale + Jscale[2] * algdist;
      Jpoint[3] = Jpoint[3] * scale + Jscale[3] * algdist;
      Jpoint[4] = Jpoint[4] * scale + Jscale[4] * algdist;
      Jpoint[5] = Jpoint[5] * scale + Jscale[5] * algdist;
      Jpoint[6] = Jpoint[6] * scale + Jscale[6] * algdist;
      Jpoint[7] = Jpoint[7] * scale + Jscale[7] * algdist;
      Jpoint[8] = Jpoint[8] * scale + Jscale[8] * algdist;

      // Computing jacobian row
      Rox_Sint pos = 0;
      for (Rox_Sint k = 0; k < 6; k++)
      {
         J[k] = 0;

         for (Rox_Sint l = 0; l < 9; l++)
         {
            J[k] += Jpoint[l] * Jve[l][k];
         }

         if (k != unused_translation_id)
         {
            J5[pos] = -J[k] * w;
            pos++;
         }
      }

      for (Rox_Sint k = 0; k < 5; k++)
      {
         for (Rox_Sint l = 0; l < 5; l++)
         {
            dLtL[k][l] += J5[k] * J5[l];
         }

         dLte[k] += J5[k] * de[idpt * 2];
      }

      // Reversed
      a = dE[0][0] * cu + dE[1][0] * cv + dE[2][0];
      b = dE[0][1] * cu + dE[1][1] * cv + dE[2][1];
      c = dE[0][2] * cu + dE[1][2] * cv + dE[2][2];
      algdist = a * ru + b * rv + c;
      scale = 1.0 / sqrt(a * a + b * b);

      commondenom = 1.0 / (pow((double) (dE[0][0] * dE[0][0] * cu * cu + 2 * cu * (dE[1][0] * cv + dE[2][0]) * dE[0][0] + dE[0][1] * dE[0][1] * cu * cu + 2 * cu * (dE[1][1] * cv + dE[2][1]) * dE[0][1] + 2 * dE[1][0] * cv * dE[2][0] + dE[2][0] * dE[2][0] + 2 * dE[1][1] * cv * dE[2][1] + dE[1][0] * dE[1][0] * cv * cv + dE[1][1] * dE[1][1] * cv * cv + dE[2][1] * dE[2][1]), 0.3e1 / 0.2e1));

      Jscale[0] = (-cu * (dE[0][0] * cu + dE[1][0] * cv + dE[2][0])) * commondenom;
      Jscale[1] = (-cv * (dE[0][0] * cu + dE[1][0] * cv + dE[2][0])) * commondenom;
      Jscale[2] = (-dE[0][0] * cu - dE[1][0] * cv - dE[2][0]) * commondenom;
      Jscale[3] = (-cu * (dE[0][1] * cu + dE[1][1] * cv + dE[2][1])) * commondenom;
      Jscale[4] = (-cv * (dE[0][1] * cu + dE[1][1] * cv + dE[2][1])) * commondenom;
      Jscale[5] = (-dE[0][1] * cu - dE[1][1] * cv - dE[2][1]) * commondenom;
      Jscale[6] = 0;
      Jscale[7] = 0;
      Jscale[8] = 0;

      Jpoint[0] = cu * ru;
      Jpoint[1] = cu * rv;
      Jpoint[2] = ru;
      Jpoint[3] = cu * rv;
      Jpoint[4] = cv * rv;
      Jpoint[5] = rv;
      Jpoint[6] = cu;
      Jpoint[7] = cv;
      Jpoint[8] = 1;

      Jpoint[0] = Jpoint[0] * scale + Jscale[0] * algdist;
      Jpoint[1] = Jpoint[1] * scale + Jscale[1] * algdist;
      Jpoint[2] = Jpoint[2] * scale + Jscale[2] * algdist;
      Jpoint[3] = Jpoint[3] * scale + Jscale[3] * algdist;
      Jpoint[4] = Jpoint[4] * scale + Jscale[4] * algdist;
      Jpoint[5] = Jpoint[5] * scale + Jscale[5] * algdist;
      Jpoint[6] = Jpoint[6] * scale + Jscale[6] * algdist;
      Jpoint[7] = Jpoint[7] * scale + Jscale[7] * algdist;
      Jpoint[8] = Jpoint[8] * scale + Jscale[8] * algdist;

      //Computing jacobian row
      pos = 0;
      for (Rox_Sint k = 0; k < 6; k++)
      {
         J[k] = 0;

         for (Rox_Sint l = 0; l < 9; l++)
         {
            J[k] += Jpoint[l] * Jve[l][k];
         }

         if (k != unused_translation_id)
         {
            J5[pos] = -J[k] * w;
            pos++;
         }
      }

      for (Rox_Sint k = 0; k < 5; k++)
      {
         for (Rox_Sint l = 0; l < 5; l++)
         {
            dLtL[k][l] += J5[k] * J5[l];
         }

         dLte[k] += J5[k] * de[idpt * 2 + 1];
      }
   }

function_terminate:
   return error;
}
