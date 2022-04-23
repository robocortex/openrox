//============================================================================
//
//    OPENROX   : File bch.c
//
//    Contents  : Implementation of bch module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#include "bch.h"
#include <system/errors/errors.h>
#include <inout/system/errors_print.h>

const int alpha_to_8_8[] = {1, 2, 4, 8, 16, 32, 64, 3, 6, 12, 24, 48, 96, 67, 5, 10, 20, 40, 80, 35,
                        70, 15, 30, 60, 120, 115, 101, 73, 17, 34, 68, 11, 22, 44, 88, 51, 102, 
                        79, 29, 58, 116, 107, 85, 41, 82, 39, 78, 31, 62, 124, 123, 117, 105, 81, 
                        33, 66, 7, 14, 28, 56, 112, 99, 69, 9, 18, 36, 72, 19, 38, 76, 27, 54, 108, 
                        91, 53, 106, 87, 45, 90, 55, 110, 95, 61, 122, 119, 109, 89, 49, 98, 71, 13, 
                        26, 52, 104, 83, 37, 74, 23, 46, 92, 59, 118, 111, 93, 57, 114, 103, 77, 25, 
                        50, 100, 75, 21, 42, 84, 43, 86, 47, 94, 63, 126, 127, 125, 121, 113, 97, 65, 0
                      };

const int alpha_to_6_2[] = {1, 2, 4, 8, 16, 5, 10, 20, 13, 26, 17, 7, 14, 28, 29, 31, 27, 19, 
                            3, 6, 12, 24, 21, 15, 30, 25, 23, 11, 22, 9, 18, 0};

const int index_of_8_8[] = {-1, 2, 4, 8, 16, 32, 64, 3, 6, 12, 24, 48, 96, 67, 5, 10, 20, 40, 80, 35, 70,
                        15, 30, 60, 120, 115, 101, 73, 17, 34, 68, 11, 22, 44, 88, 51, 102, 79, 29, 
                        58, 116, 107, 85, 41, 82, 39, 78, 31, 62, 124, 123, 117, 105, 81, 33, 66, 7, 
                        14, 28, 56, 112, 99, 69, 9, 18, 36, 72, 19, 38, 76, 27, 54, 108, 91, 53, 106, 
                        87, 45, 90, 55, 110, 95, 61, 122, 119, 109, 89, 49, 98, 71, 13, 26, 52, 104, 83, 
                        37, 74, 23, 46, 92, 59, 118, 111, 93, 57, 114, 103, 77, 25, 50, 100, 75, 21, 42, 
                        84, 43, 86, 47, 94, 63, 126, 127, 125, 121, 113, 97, 65, 0};

const int index_of_6_2[] = {-1, 0, 1, 18, 2, 5, 19, 11, 3, 29, 6, 27, 20, 8, 12, 23, 4, 10, 30, 17, 
                            7, 22, 28, 26, 21, 25, 9, 16, 13, 14, 24, 15};

const int g_8_8[] = {1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 
                  0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1,
                  1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0};

const int g_6_2[] = {1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

Rox_ErrorCode rox_bch_c8_e8_encode(Rox_Ulint * output, Rox_Uchar input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   int i, j, feedback;
   int data[8], bb[64];
   Rox_Ulint res;
   long bit;

   const int length = 64;
   const int k = 8;

   //Actually encode
   for (bit = 0; bit < 8; bit++)
   {
      Rox_Uchar pot = (1 << bit);
      Rox_Uchar val = (input & pot);
	   data[bit] = 0;
      if (val != 0) data[bit] = 1;
   }

   for (i = 0; i < length - k; i++)
   {
      bb[i] = 0;
   }

   for (i = k - 1; i >= 0; i--) 
   {
      feedback = data[i] ^ bb[length - k - 1];
      if (feedback != 0) 
      {
         for (j = length - k - 1; j > 0; j--)
         {
            if (g_8_8[j] != 0)
            {
               bb[j] = bb[j - 1] ^ feedback;
            }
            else
            {
               bb[j] = bb[j - 1];
            }
         }
         bb[0] = g_8_8[0] && feedback;
      } 
      else 
      {
         for (j = length - k - 1; j > 0; j--)
         {
            bb[j] = bb[j - 1];
         }
         bb[0] = 0;
      }
   }

   for (i = 0; i < k; i++)
   {
      bb[i + length - k] = data[i];
   }

   res = 0;
   for (bit = 0; bit < 64; bit++)
   {
      if (bb[bit])
      {
         long pot = (1l << bit);
         res = res | pot;
      }

   }
   *output = res;

   return error;
}

Rox_ErrorCode rox_bch_c6_e2_encode(Rox_Ushort * output, Rox_Uchar input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   int i, j, feedback;
   int data[8], bb[16];
   Rox_Ushort res;
   Rox_Ushort bit;

   const int length = 16;
   const int k = 6;

   //Actually encode
   for (bit = 0; bit < 8; bit++)
   {
      Rox_Uchar pot = (1 << bit);
      Rox_Uchar val = (input & pot);
	        data[bit] = 0;
      if (val != 0) data[bit] = 1;
   }

   for (i = 0; i < length - k; i++)
   {
      bb[i] = 0;
   }

   for (i = k - 1; i >= 0; i--) 
   {
      feedback = data[i] ^ bb[length - k - 1];
      if (feedback != 0) 
      {
         for (j = length - k - 1; j > 0; j--)
         {
            if (g_6_2[j] != 0)
            {
               bb[j] = bb[j - 1] ^ feedback;
            }
            else
            {
               bb[j] = bb[j - 1];
            }
         }
         bb[0] = g_6_2[0] && feedback;
      } 
      else 
      {
         for (j = length - k - 1; j > 0; j--)
         {
            bb[j] = bb[j - 1];
         }
         bb[0] = 0;
      }
   }

   for (i = 0; i < k; i++)
   {
      bb[i + length - k] = data[i];
   }

   res = 0;
   for (bit = 0; bit < 16; bit++)
   {
      if (bb[bit])
      {
         Rox_Ushort pot = (1 << bit);
         res = res | pot;
      }

   }
   *output = res;

   return error;
}

Rox_ErrorCode rox_bch_c8_e8_decode(Rox_Uchar *output, Rox_Ulint input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   int i, j, u, q, t2, count = 0, syn_error = 0;
   int elp[130][128], d[130], l[130], u_lu[130], s[129];
   int loc[200], reg[201];
   int recd[64];
   long bit;
   Rox_Uchar res;

   const int n = 127;
   const int t = 8;
   const int length = 64;
   t2 = 2 * t;

   // From integer to bitstring
   for (bit = 0; bit < 64; bit++)
   {
      long pot = (1l << bit);
      if (input & pot)
      {
         recd[bit] = 1;
      }
      else
      {
         recd[bit] = 0;
      }
   }
   
   for (i = 1; i <= t2; i++) 
   {
      s[i] = 0;
      for (j = 0; j < length; j++)
      {
         if (recd[j] != 0)
         {
            s[i] ^= alpha_to_8_8[(i * j) % n];
         }
      }
      if (s[i] != 0)
      {
         syn_error = 1;
      }

      s[i] = index_of_8_8[s[i]];
   }

   if (syn_error) 
   {
      d[0] = 0;   
      d[1] = s[1];
      elp[0][0] = 0;
      elp[1][0] = 1;
      for (i = 1; i < t2; i++) 
      {
         elp[0][i] = -1; 
         elp[1][i] = 0;
      }

      l[0] = 0;
      l[1] = 0;
      u_lu[0] = -1;
      u_lu[1] = 0;
      u = 0;
 
      do 
      {
         u++;
         if (d[u] == -1) 
         {
            l[u + 1] = l[u];
            for (i = 0; i <= l[u]; i++) 
            {
               elp[u + 1][i] = elp[u][i];
               elp[u][i] = index_of_8_8[elp[u][i]];
            }
         } 
         else
         {
            q = u - 1;
            while ((d[q] == -1) && (q > 0))
            {
               q--;
            }

            
            if (q > 0) 
            {
              j = q;
              do 
              {
                j--;
                if ((d[j] != -1) && (u_lu[q] < u_lu[j]))
                {
                  q = j;
                }
              } 
              while (j > 0);
            }
 
            if (l[u] > l[q] + u - q)
            {
               l[u + 1] = l[u];
            }
            else
            {
               l[u + 1] = l[q] + u - q;
            }
 
            // form new elp(x) 
            for (i = 0; i < t2; i++)
            {
               elp[u + 1][i] = 0;
            }

            for (i = 0; i <= l[q]; i++)
            {
               if (elp[q][i] != -1)
               {
                  elp[u + 1][i + u - q] = alpha_to_8_8[(d[u] + n - d[q] + elp[q][i]) % n];
               }
            }
            for (i = 0; i <= l[u]; i++) 
            {
               elp[u + 1][i] ^= elp[u][i];
               elp[u][i] = index_of_8_8[elp[u][i]];
            }
         }
         u_lu[u + 1] = u - l[u + 1];
 
         // form (u+1)th discrepancy 
         if (u < t2) 
         {  
           if (s[u + 1] != -1)
           {
             d[u + 1] = alpha_to_8_8[s[u + 1]];
          }
           else
           {
             d[u + 1] = 0;
          }
             for (i = 1; i <= l[u + 1]; i++)
             {
               if ((s[u + 1 - i] != -1) && (elp[u + 1][i] != 0))
               {
                 d[u + 1] ^= alpha_to_8_8[(s[u + 1 - i] + index_of_8_8[elp[u + 1][i]]) % n];
              }
           }
           // put d[u+1] into index form 
           d[u + 1] = index_of_8_8[d[u + 1]]; 
         }
      } 
      while ((u < t2) && (l[u + 1] <= t));
 
      u++;
      if (l[u] <= t) 
      {
         for (i = 0; i <= l[u]; i++) 
         {
            elp[u][i] = index_of_8_8[elp[u][i]];
         }

         // Chien search: find roots of the error location polynomial 
         for (i = 1; i <= l[u]; i++)
         {
            reg[i] = elp[u][i];
         }

         count = 0;
         for (i = 1; i <= n; i++) 
         {
            q = 1;
            for (j = 1; j <= l[u]; j++)
            {
               if (reg[j] != -1) 
               {
                  reg[j] = (reg[j] + j) % n;
                  q ^= alpha_to_8_8[reg[j]];
               }
            }

            if (!q) 
            {
               loc[count] = n - i;
               count++;
            }
         }

         if (count == l[u])   
         {
            for (i = 0; i < l[u]; i++)
            {
               recd[loc[i]] ^= 1;
            }
         }
         else
         {
            error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error );
         }
      }
   }

   res = 0;
   for (bit = 0; bit < 8; bit++)
   {
      if (recd[56 + bit]) 
      {
         res = res | (1 << bit);
      }
   }

   *output = res;

function_terminate:
   return error;
}

Rox_ErrorCode rox_bch_c6_e2_decode(Rox_Uchar *output, Rox_Ushort input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   int i, j, u, q, t2, count = 0, syn_error = 0;
   int elp[18][16], d[18]= { 0 }, l[18]= { 0 }, u_lu[18]= { 0 }, s[17]= { 0 };
   int loc[200]= { 0 }, reg[201] = { 0 };
   int recd[64]= { 0 };
   Rox_Ushort bit;
   Rox_Uchar res;

   const int n = 31;
   const int t = 2;
   const int length = 16;
   t2 = 2 * t;

   //From integer to bitstring
   for (bit = 0; bit < 16; bit++)
   {
      Rox_Ushort pot = (1 << bit);
      if (input & pot)
      {
         recd[bit] = 1;
      }
      else
      {
         recd[bit] = 0;
      }
   }
   
   for (i = 1; i <= t2; i++) 
   {
      s[i] = 0;
      for (j = 0; j < length; j++)
      {
         if (recd[j] != 0)
         {
            s[i] ^= alpha_to_6_2[(i * j) % n];
         }
      }
      if (s[i] != 0)
      {
         syn_error = 1;
      }

      s[i] = index_of_6_2[s[i]];
   }

   if (syn_error) 
   {
      d[0] = 0;   
      d[1] = s[1];
      elp[0][0] = 0;
      elp[1][0] = 1;
      for (i = 1; i < t2; i++) 
      {
         elp[0][i] = -1; 
         elp[1][i] = 0;
      }

      l[0] = 0;
      l[1] = 0;
      u_lu[0] = -1;
      u_lu[1] = 0;
      u = 0;
 
      do 
      {
         u++;
         if (d[u] == -1) 
         {
            l[u + 1] = l[u];
            for (i = 0; i <= l[u]; i++) 
            {
               elp[u + 1][i] = elp[u][i];
               elp[u][i] = index_of_6_2[elp[u][i]];
            }
         } 
         else
         {
            q = u - 1;
            while ((d[q] == -1) && (q > 0))
            {
               q--;
            }

            
            if (q > 0) 
            {
              j = q;
              do 
              {
                j--;
                if ((d[j] != -1) && (u_lu[q] < u_lu[j]))
                {
                  q = j;
                }
              } 
              while (j > 0);
            }
 
            if (l[u] > l[q] + u - q)
            {
               l[u + 1] = l[u];
            }
            else
            {
               l[u + 1] = l[q] + u - q;
            }
 
            // form new elp(x) 
            for (i = 0; i < t2; i++)
            {
               elp[u + 1][i] = 0;
            }

            for (i = 0; i <= l[q]; i++)
            {
               if (elp[q][i] != -1)
               {
                  elp[u + 1][i + u - q] = alpha_to_6_2[(d[u] + n - d[q] + elp[q][i]) % n];
               }
            }
            for (i = 0; i <= l[u]; i++) 
            {
               elp[u + 1][i] ^= elp[u][i];
               elp[u][i] = index_of_6_2[elp[u][i]];
            }
         }
         u_lu[u + 1] = u - l[u + 1];
 
         // form (u+1)th discrepancy 
         if (u < t2) 
         {  
           if (s[u + 1] != -1)
           {
             d[u + 1] = alpha_to_6_2[s[u + 1]];
          }
           else
           {
             d[u + 1] = 0;
          }
             for (i = 1; i <= l[u + 1]; i++)
             {
               if ((s[u + 1 - i] != -1) && (elp[u + 1][i] != 0))
               {
                 d[u + 1] ^= alpha_to_6_2[(s[u + 1 - i] + index_of_6_2[elp[u + 1][i]]) % n];
              }
           }
           // put d[u+1] into index form 
           d[u + 1] = index_of_6_2[d[u + 1]]; 
         }
      } 
      while ((u < t2) && (l[u + 1] <= t));
 
      u++;
      if (l[u] <= t) 
      {
         for (i = 0; i <= l[u]; i++) 
         {
            elp[u][i] = index_of_6_2[elp[u][i]];
         }

         // Chien search: find roots of the error location polynomial 
         for (i = 1; i <= l[u]; i++)
         {
            reg[i] = elp[u][i];
         }

         count = 0;
         for (i = 1; i <= n; i++) 
         {
            q = 1;
            for (j = 1; j <= l[u]; j++)
            {
               if (reg[j] != -1) 
               {
                  reg[j] = (reg[j] + j) % n;
                  q ^= alpha_to_6_2[reg[j]];
               }
            }

            if (!q) 
            {
               loc[count] = n - i;
               count++;
            }
         }

         if (count == l[u])   
         {
            for (i = 0; i < l[u]; i++)
            {
               recd[loc[i]] ^= 1;
            }
         }
         else
         {
            
            error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error );
         }
      }
   }

   res = 0;
   for (bit = 0; bit < 6; bit++)
   {
      if (recd[10 + bit]) 
      {
         res = res | (1 << bit);
      }
   }

   *output = res;

function_terminate:
   return error;
}