//==============================================================================
//
//    OPENROX   : File fastst_score.h
//
//    Contents  : API of fastst_score module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "fastst_score.h"

#include <baseproc/maths/maths_macros.h>
#include <generated/dynvec_segment_point_struct.h>
#include <inout/system/errors_print.h>

#define CIRCLE_SIZE 16
#define SEGMENT_SIZE 9
#define TAB_SIZE CIRCLE_SIZE + SEGMENT_SIZE
#define Compare(X, Y) ((X)>=(Y))

Rox_ErrorCode rox_ansi_fastst_detector_score (Rox_DynVec_Segment_Point points, Rox_Image source, Rox_Sint barrier)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uchar * ptr;
   Rox_Uint idpt, iddif, idsegment;
   Rox_Sint u,v;
   Rox_Sint differences[TAB_SIZE];
   Rox_Sint mindif;
   Rox_Sint score;

   if (!points || !source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uchar ** source_data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &source_data, source );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint stride = 0;
   error = rox_array2d_uchar_get_stride(&stride, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // We assume points contains only valid corner coordinates
   {
      Rox_Sint tabs[] = {
          0 + stride *  3,  1 + stride *  3,  2 + stride *  2,  3 + stride *  1,
          3 + stride *  0,  3 + stride * -1,  2 + stride * -2,  1 + stride * -3,
          0 + stride * -3, -1 + stride * -3, -2 + stride * -2, -3 + stride * -1,
         -3 + stride *  0, -3 + stride *  1, -2 + stride *  2, -1 + stride *  3,
          0 + stride *  3,  1 + stride *  3,  2 + stride *  2,  3 + stride *  1,
          3 + stride *  0,  3 + stride * -1,  2 + stride * -2,  1 + stride * -3,
          0 + stride * -3, -1 + stride * -3, -2 + stride * -2, -3 + stride * -1
      };

      for (idpt = 0; idpt < points->used; idpt++)
      {
         u     = points->data[idpt].j;
         v     = points->data[idpt].i;
         ptr   = &source_data[v][u];
         score = barrier;

         // Compute differences on whole circle + segment size
         for (iddif = 0; iddif < TAB_SIZE; iddif++)
         {
            differences[iddif] = ptr[0] - ptr[tabs[iddif]];
         }

         for (idsegment = 0; idsegment < CIRCLE_SIZE; idsegment++)
         {
            // Check upper bound
            mindif = ROX_MIN(differences[idsegment], differences[idsegment + 1]);
            mindif = ROX_MIN(mindif, differences[idsegment + 2]);
            mindif = ROX_MIN(mindif, differences[idsegment + 3]);
            mindif = ROX_MIN(mindif, differences[idsegment + 4]);
            mindif = ROX_MIN(mindif, differences[idsegment + 5]);
            mindif = ROX_MIN(mindif, differences[idsegment + 6]);
            mindif = ROX_MIN(mindif, differences[idsegment + 7]);
            mindif = ROX_MIN(mindif, differences[idsegment + 8]);
            score = ROX_MAX(score, mindif);

            // Check lower bound
            mindif = ROX_MAX(differences[idsegment], differences[idsegment + 1]);
            mindif = ROX_MAX(mindif, differences[idsegment + 2]);
            mindif = ROX_MAX(mindif, differences[idsegment + 3]);
            mindif = ROX_MAX(mindif, differences[idsegment + 4]);
            mindif = ROX_MAX(mindif, differences[idsegment + 5]);
            mindif = ROX_MAX(mindif, differences[idsegment + 6]);
            mindif = ROX_MAX(mindif, differences[idsegment + 7]);
            mindif = ROX_MAX(mindif, differences[idsegment + 8]);
            score = -ROX_MIN(-score, mindif);
         }

         points->data[idpt].score = score;
      }
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_fastst_detector_score(Rox_DynVec_Segment_Point points, Rox_Image source, Rox_Sint barrier)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uchar * ptr;
   Rox_Uint idpt, iddif, idsegment;
   Rox_Sint u,v;
   Rox_Sint differences[TAB_SIZE];
   Rox_Sint mindif;
   Rox_Sint score;

   if (!points || !source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uchar ** source_data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &source_data, source );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint stride = 0;
   error = rox_array2d_uchar_get_stride(&stride, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // We assume points contains only valid corner coordinates
   {
      Rox_Sint tabs[] = {
          0 + stride *  3,  1 + stride *  3,  2 + stride *  2,  3 + stride *  1,
          3 + stride *  0,  3 + stride * -1,  2 + stride * -2,  1 + stride * -3,
          0 + stride * -3, -1 + stride * -3, -2 + stride * -2, -3 + stride * -1,
         -3 + stride *  0, -3 + stride *  1, -2 + stride *  2, -1 + stride *  3,
          0 + stride *  3,  1 + stride *  3,  2 + stride *  2,  3 + stride *  1,
          3 + stride *  0,  3 + stride * -1,  2 + stride * -2,  1 + stride * -3,
          0 + stride * -3, -1 + stride * -3, -2 + stride * -2, -3 + stride * -1
      };

      for (idpt = 0; idpt < points->used; idpt++)
      {
         u     = points->data[idpt].j;
         v     = points->data[idpt].i;
         ptr   = &source_data[v][u];
         score = barrier;

         // Compute differences on whole circle + segment size
         for (iddif = 0; iddif < TAB_SIZE; iddif++)
         {
            differences[iddif] = ptr[0] - ptr[tabs[iddif]];
         }

         for (idsegment = 0; idsegment < CIRCLE_SIZE; idsegment++)
         {
            // Check upper bound
            mindif = ROX_MIN(differences[idsegment], differences[idsegment + 1]);
            mindif = ROX_MIN(mindif, differences[idsegment + 2]);
            mindif = ROX_MIN(mindif, differences[idsegment + 3]);
            mindif = ROX_MIN(mindif, differences[idsegment + 4]);
            mindif = ROX_MIN(mindif, differences[idsegment + 5]);
            mindif = ROX_MIN(mindif, differences[idsegment + 6]);
            mindif = ROX_MIN(mindif, differences[idsegment + 7]);
            mindif = ROX_MIN(mindif, differences[idsegment + 8]);
            score = ROX_MAX(score, mindif);

            // Check lower bound
            mindif = ROX_MAX(differences[idsegment], differences[idsegment + 1]);
            mindif = ROX_MAX(mindif, differences[idsegment + 2]);
            mindif = ROX_MAX(mindif, differences[idsegment + 3]);
            mindif = ROX_MAX(mindif, differences[idsegment + 4]);
            mindif = ROX_MAX(mindif, differences[idsegment + 5]);
            mindif = ROX_MAX(mindif, differences[idsegment + 6]);
            mindif = ROX_MAX(mindif, differences[idsegment + 7]);
            mindif = ROX_MAX(mindif, differences[idsegment + 8]);
            score = -ROX_MIN(-score, mindif);
         }

         points->data[idpt].score = score;
      }
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_fastst_nonmax_suppression(Rox_DynVec_Segment_Point ptr, Rox_DynVec_Segment_Point corners)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   const Rox_Uint sz = corners->used;
   Rox_Uint point_above = 0;
   Rox_Uint point_below = 0;
   int prev_row = -1;

   if (!ptr || !corners) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (corners->used < 1) 
   { error = ROX_ERROR_NONE; goto function_terminate; }

   rox_dynvec_segment_point_reset(ptr);

   Rox_Uint last_row = corners->data[sz-1].i;
   Rox_Sint * row_start = (int*)malloc((last_row+1)*sizeof(int));
   if (!row_start) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Build Row look up table
   for ( Rox_Sint i=0; i < last_row+1; i++) row_start[i] = -1;
   for ( Rox_Sint i=0; i < sz; i++)
   {
      if(((int)corners->data[i].i) != prev_row)
      {
         row_start[corners->data[i].i] = i;
         prev_row = corners->data[i].i;
      }
   }

   for ( Rox_Sint i=0; i < sz; i++)
   {
      Rox_Segment_Point_Struct pos = corners->data[i];
      Rox_Float score = pos.score;

      // Check left
      if(i > 0)
      {
         if(corners->data[i-1].j == pos.j-1 && corners->data[i-1].i == pos.i && Compare(corners->data[i-1].score, score)) continue;
      }

      // Check right
      if(i < (sz - 1))
      {
         if(corners->data[i+1].j == pos.j+1 && corners->data[i+1].i == pos.i && Compare(corners->data[i+1].score, score)) continue;
      }

      // Check above (if there is a valid row above)
      if (pos.i != 0 && row_start[pos.i - 1] != -1)
      {
         // Make sure that current point_above is one row above.
         if (corners->data[point_above].i < pos.i - 1) point_above = row_start[pos.i-1];

         // Make point_above point to the first of the pixels above the current point, if it exists.
         for (; corners->data[point_above].i < pos.i && corners->data[point_above].j < pos.j - 1; point_above++)
         {
         }

         // Check if a point above is close
         for ( Rox_Sint j=point_above; corners->data[j].i < pos.i && corners->data[j].j <= pos.j + 1; j++)
         {
            int x = corners->data[j].j;
            if (((x == (int) pos.j - 1) || (x == (int) pos.j) || (x == (int) pos.j+1)) && Compare(corners->data[j].score, score))
            {
               goto cont;
            }
         }
      }

      // Check below (if there is anything below)
      if(pos.i != last_row && row_start[pos.i + 1] != -1 && point_below < sz)// Nothing below
      {
         if(corners->data[point_below].i < pos.i + 1) point_below = row_start[pos.i+1];

         //  Make point below point to one of the pixels belowthe current point, if it exists.
         for (; point_below < sz && corners->data[point_below].i == pos.i+1 && corners->data[point_below].j < pos.j - 1; point_below++)
         {
         }

         // Check if a point below is close
         for ( Rox_Sint j = point_below; j < sz && corners->data[j].i == pos.i+1 && corners->data[j].j <= pos.j + 1; j++)
         {
            int x = corners->data[j].j;
            if (((x == (int)pos.j - 1) || (x == (int) pos.j) || (x == (int)pos.j+1)) && Compare(corners->data[j].score,score))
            {
               goto cont;
            }
         }
      }

      rox_dynvec_segment_point_append(ptr, &corners->data[i]);

   cont:
      ;
   }

   free(row_start);

function_terminate:
   return error;
}


int comparePoints(const Rox_Segment_Point c1, const Rox_Segment_Point  c2)
{
   if (c1->score > c2->score) return -1;
   if (c1->score < c2->score) return 1;
   return 0;
}


Rox_ErrorCode rox_fastst_detector_sort(Rox_DynVec_Segment_Point ptr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if (!ptr) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   if (ptr->used == 0) goto function_terminate;

   qsort(ptr->data, ptr->used, sizeof(Rox_Segment_Point_Struct), (int (*)(const void *, const void *))comparePoints);

function_terminate:
   return error;
}
