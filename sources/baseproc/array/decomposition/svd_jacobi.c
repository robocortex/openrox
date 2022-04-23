//==============================================================================
//
//    OPENROX   : File svdsort.c
//
//    Contents  : Implementation of svdsort module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "svd.h"

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/transpose/transpose.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_svd_jacobi(Rox_Array2D_Double U, Rox_Array2D_Double S, Rox_Array2D_Double V, const Rox_Array2D_Double Q, const Rox_Array2D_Double R, const Rox_Array2D_Double P)
{
   Rox_Sint cols, rows, diagonalsize;
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint end;
   Rox_Array2D_Double workbuffer;
   
   Rox_Double **dwb, **dr, **ds, **du, **dv;

   Rox_Sint p, q, r, c, maxpos;
   Rox_Double threshold, test, norm, z, rotc, rots, i, j, t, d, u, x, y, mc, ms, tau, w, st, n, maxval, swap;
   Rox_Double m[2][2], j_left[2][2], j_right[2][2];

   // Check pointers
   if (!U || !S || !V || !Q || !R || !P) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Check input sizes
   error = rox_array2d_double_get_rows(&rows, Q); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_cols(&cols, P); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   diagonalsize = ROX_MIN(rows, cols);

   error = rox_array2d_double_check_size(U, rows, rows); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_check_size(S, cols, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_check_size(V, cols, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_check_size(Q, rows, rows); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_check_size(R, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_check_size(P, cols, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   workbuffer = 0;

   error = rox_array2d_double_new(&workbuffer, diagonalsize, diagonalsize);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer ( &dwb, workbuffer);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dr, R );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer ( &ds, S );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer ( &du, U );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dv, V );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Copy upper part of R
   for (r = 0; r < diagonalsize; r++)
   {
      for (c = 0; c < diagonalsize; c++)
      {
         dwb[r][c] = dr[r][c];
      }
   }

   // Copy Q & P
   rox_array2d_double_copy(U, Q);
   rox_array2d_double_transpose(V, P);

   // Diagonalize
   end = 0;
   while (!end)
   {
      end = 1;

      // Get all possible (p,q) unique pairs
      for (p = 1; p < diagonalsize; p++)
      {
         for (q = 0; q < p; q++)
         {
            // Check that the 2*2 block is not diagonal
            threshold = ROX_MAX(2.0 * DBL_MIN, 2.0 * DBL_EPSILON * ROX_MAX(fabs(dwb[p][p]), fabs(dwb[q][q])));
            test = ROX_MAX(fabs(dwb[p][q]), fabs(dwb[q][p]));

            if (test > threshold)
            {
               end = 0;

               // Precondition block
               norm = sqrt(dwb[p][p] * dwb[p][p] + dwb[q][p] * dwb[q][p]);

               if (norm == 0.0)
               {
                  z = fabs(dwb[p][q]) / dwb[p][q];
                  for (c = 0; c < diagonalsize; c++) dwb[p][c] *= z;

                  if (U)
                  {
                     for (r = 0; r < rows; r++) du[r][p] *= z;
                  }

                  z = fabs(dwb[q][q]) / dwb[q][q];
                  for (c = 0; c < diagonalsize; c++) dwb[q][c] *= z;

                  if (U)
                  {
                     for (r = 0; r < rows; r++) du[r][q] *= z;
                  }
               }
               else
               {
                  rotc = dwb[p][p] / norm;
                  rots = dwb[q][p] / norm;

                  for (c = 0; c < diagonalsize; c++)
                  {
                     i = dwb[p][c];
                     j = dwb[q][c];
                     dwb[p][c] = rotc * i + rots * j;
                     dwb[q][c] = - rots * i + rotc * j;
                  }

                  if (U)
                  {
                     for (r = 0; r < rows; r++)
                     {
                        i = du[r][p];
                        j = du[r][q];

                        du[r][p] = rotc * i + rots * j;
                        du[r][q] = - rots * i + rotc * j;
                     }
                  }

                  if (dwb[p][q] != 0.0)
                  {
                     z = fabs(dwb[p][q]) / dwb[p][q];
                     for (r = 0; r < diagonalsize; r++) dwb[r][q] *= z;

                     if (V)
                     {
                        for (r = 0; r < cols; r++) dv[r][q] *= z;
                     }
                  }

                  if (dwb[q][q] != 0.0)
                  {
                     z = fabs(dwb[q][q]) / dwb[q][q];
                     for (c = 0; c < diagonalsize; c++) dwb[q][c] *= z;

                     if (U)
                     {
                        for (r = 0; r < rows; r++) du[r][q] *= z;
                     }
                  }
               }

               // 2*2 block svd
               m[0][0] = dwb[p][p]; m[0][1] = dwb[p][q];
               m[1][0] = dwb[q][p]; m[1][1] = dwb[q][q];
               t = m[0][0] + m[1][1];
               d = m[1][0] - m[0][1];

               if (t == 0.0)
               {
                  rotc = 0;

                  if (d > 0.0)
                  {
                     rots = 1;
                  }
                  else
                  {
                     rots = -1;
                  }
               }
               else
               {
                  u = d / t;
                  rotc = 1.0 / sqrt(1.0 + u * u);
                  rots = rotc * u;
               }

               i = m[0][0];
               j = m[1][0];
               m[0][0] = rotc * i + rots * j;
               m[1][0] = -rots * i + rotc * j;

               i = m[0][1];
               j = m[1][1];
               m[0][1] = rotc * i + rots * j;
               m[1][1] = -rots * i + rotc * j;

               x = m[0][0];
               y = m[0][1];
               z = m[1][1];

               if (y == 0.0)
               {
                  mc = 1;
                  ms = 0;
               }
               else
               {
                  tau = (x - z) / (2.0 * fabs(y));
                  w = sqrt(tau * tau + 1.0);

                  if (tau > 0.0)
                  {
                     t = 1.0 / (tau + w);
                  }
                  else
                  {
                     t = 1.0 / (tau - w);
                  }

                  if (t > 0.0) st = 1;
                  else st = -1;

                  n = 1.0 / sqrt(t * t + 1.0);
                  ms = - st * (y / fabs(y)) * fabs(t) * n;
                  mc = n;
               }

               j_right[0][0] = mc;
               j_right[0][1] = ms;
               j_right[1][0] = -ms;
               j_right[1][1] = mc;

               // *j_left  = rot1 * j_right->transpose();
               j_left[0][0] = rotc * j_right[0][0] + rots * j_right[0][1];
               j_left[1][0] = -rots * j_right[0][0] + rotc * j_right[0][1];
               j_left[0][1] = rotc * j_right[1][0] + rots * j_right[1][1];
               j_left[1][1] = -rots * j_right[1][0] + rotc * j_right[1][1];

               // Apply j_left on the left
               for (c = 0; c < diagonalsize; c++)
               {
                  i = dwb[p][c];
                  j = dwb[q][c];

                  dwb[p][c] = j_left[0][0] * i + j_left[0][1] * j;
                  dwb[q][c] = j_left[1][0] * i + j_left[1][1] * j;
               }

               if (U)
               {
                  for (r = 0; r < rows; r++)
                  {
                     i = du[r][p];
                     j = du[r][q];

                     du[r][p] = j_left[0][0] * i + j_left[0][1] * j;
                     du[r][q] = j_left[1][0] * i + j_left[1][1] * j;
                  }
               }

               // Apply j_right on the right
               for (r = 0; r < diagonalsize; r++)
               {
                  i = dwb[r][p];
                  j = dwb[r][q];

                  dwb[r][p] = j_right[0][0] * i + j_right[1][0] * j;
                  dwb[r][q] = j_right[0][1] * i + j_right[1][1] * j;
               }

               if (V)
               {
                  for (r = 0; r < cols; r++)
                  {
                     i = dv[r][p];
                     j = dv[r][q];

                     dv[r][p] = j_right[0][0] * i + j_right[1][0] * j;
                     dv[r][q] = j_right[0][1] * i + j_right[1][1] * j;
                  }
               }
            }
         }
      }
   }

   // Compute singular values
   for (c = 0; c < diagonalsize; c++)
   {
      // Use absolute value
      n = fabs(dwb[c][c]);
      ds[c][0] = n;

      // Change u sign accordingly
      if (U && n != 0.0)
      {
         for (r = 0; r < rows; r++)
         {
            du[r][c] *= dwb[c][c] / n;
         }
      }
   }

   // sort
   for (r = 0; r < diagonalsize; r++)
   {
      maxval = -1.0;
      maxpos = 0;

      for (c = r; c < diagonalsize; c++)
      {
         if (ds[c][0] > maxval)
         {
            maxval = ds[c][0];
            maxpos = c;
         }
      }

      if (maxval <= 0.0) break;


      swap = ds[r][0];
      ds[r][0] = ds[maxpos][0];
      ds[maxpos][0] = swap;

      // Swap U columns
      for (c = 0; c < rows; c++)
      {
         swap = du[c][r];
         du[c][r] = du[c][maxpos];
         du[c][maxpos] = swap;
      }

      // Swap V columns
      for (c = 0; c < cols; c++)
      {
         swap = dv[c][r];
         dv[c][r] = dv[c][maxpos];
         dv[c][maxpos] = swap;
      }
   }

function_terminate:
   rox_array2d_double_del(&workbuffer);

   return error;
}
