//==============================================================================
//
//    OPENROX   : File fpsm.c
//
//    Contents  : Implementation of fpsm module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "fpsm.h"
#include "fpsm_struct.h"

#include <float.h>

#include <generated/dynvec_edgel_struct.h>
#include <generated/objset_dynvec_edgel_struct.h>

#include <system/errors/errors.h>
#include <system/memory/memory.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/random/random.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/image/transform/distance.h>

#include <core/features/detectors/edges/canny.h>

#include <inout/image/pgm/pgmfile.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

//#define FPSM_DEBUG 1

Rox_ErrorCode rox_fpsm_new(Rox_Fpsm *obj, Rox_Sint width, Rox_Sint height, Rox_Uint nbr_ref_points, Rox_Uint min_gradient)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Fpsm ret = NULL;


   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   *obj = NULL;

   ret = (Rox_Fpsm)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->width = width;
   ret->height = height;
   ret->nbr_ref_points = nbr_ref_points;
   ret->preproc = NULL;
   ret->drawing = NULL;
   ret->postproc = NULL;
   ret->edges = NULL;
   ret->angles = NULL;
   ret->distancemap = NULL;
   ret->distancemap_points = NULL;
   ret->anglemap = NULL;
   ret->count_edgels = 0;

   CHECK_ERROR_TERMINATE(rox_edgepreproc_gray_new(&ret->preproc, width, height));
   CHECK_ERROR_TERMINATE(rox_edgedraw_new(&ret->drawing, width, height, min_gradient, 1));
   CHECK_ERROR_TERMINATE(rox_edgepostproc_ac_new(&ret->postproc, width, height));
   CHECK_ERROR_TERMINATE(rox_array2d_uchar_new(&ret->edges, height, width));
   CHECK_ERROR_TERMINATE(rox_array2d_sshort_new(&ret->angles, height, width));
   CHECK_ERROR_TERMINATE(rox_array2d_sshort_new(&ret->anglemap, height, width));
   CHECK_ERROR_TERMINATE(rox_array2d_sshort_new(&ret->distancemap, height, width));
   CHECK_ERROR_TERMINATE(rox_array2d_point2d_sshort_new(&ret->distancemap_points, height, width));

   *obj = ret;

function_terminate:
   if (error) rox_fpsm_del(&ret);

   return error;
}

Rox_ErrorCode rox_fpsm_del(Rox_Fpsm *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Fpsm todel = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *obj;
   *obj = NULL;

   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_edgepreproc_gray_del(&todel->preproc);
   rox_edgedraw_del(&todel->drawing);
   rox_edgepostproc_ac_del(&todel->postproc);
   rox_array2d_uchar_del(&todel->edges);
   rox_array2d_sshort_del(&todel->angles);
   rox_array2d_sshort_del(&todel->anglemap);
   rox_array2d_sshort_del(&todel->distancemap);
   rox_array2d_point2d_sshort_del(&todel->distancemap_points);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_fpsm_extract_edges(Rox_Fpsm obj, Rox_Image source, Rox_Double min_NFA, Rox_Uint nbr_blur_passes, Rox_Uint min_segment_size, Rox_Bool straight_edge_only)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_ObjSet_DynVec_Edgel segments;
   Rox_DynVec_Edgel segment;
   Rox_Uchar cur_color;
   Rox_Point2D_Sshort_Struct g;
   Rox_Uint idsegment, idedgel;
   Rox_Sshort u,v;


   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uchar ** de = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &de, obj->edges);

   Rox_Sshort ** da = NULL;
   error = rox_array2d_sshort_get_data_pointer_to_pointer( &da, obj->angles);

   Rox_Point2D_Sshort_Struct ** dg = NULL;
   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer( &dg, obj->preproc->gradients);

   error = rox_array2d_uchar_fillval(obj->edges, 0);

   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_sshort_fillval(obj->angles, -1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_edgepreproc_gray_process(obj->preproc, source, nbr_blur_passes);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_edgedraw_process(obj->drawing, obj->preproc->gradients, min_segment_size, straight_edge_only);
   ROX_ERROR_CHECK_TERMINATE ( error );

#ifdef FPSM_DEBUG
   static int pic = 0;
   rox_log("Found %d segments in %08d.pgm\n", obj->drawing->resultsegments->used, pic);

   //Write result mask to a pgm file
   char filename[256];
   sprintf(filename, "orig_%08d.pgm", pic);
   rox_array2d_uchar_save_pgm(filename, obj->drawing->resultmask);
   pic++;
#endif //FPSM_DEBUG

   error = rox_edgepostproc_ac_process(obj->postproc, obj->drawing->resultsegments, min_NFA);
   ROX_ERROR_CHECK_TERMINATE ( error );

   segments = obj->postproc->resultsegments;

   obj->count_edgels = 0;
   for (idsegment = 0; idsegment < segments->used; idsegment++)
   {
      segment = segments->data[idsegment];

      cur_color = 64 + rox_rand() % (255-64);
      for (idedgel = 0; idedgel < segment->used; idedgel++)
      {
         u = segment->data[idedgel].u;
         v = segment->data[idedgel].v;
         g = dg[v][u];

         de[v][u] = cur_color;//255;
         da[v][u] = (Rox_Sshort)(10000.0*atan2((Rox_Double)g.v, (Rox_Double)g.u));//x1000 to avoid storing floats when float precision is not needed
      }

      obj->count_edgels += segment->used;
   }

#ifdef FPSM_DEBUG
   static int pic2 = 0;
   rox_log("Found %d count_edgels\n", obj->count_edgels);

   //Write result mask to a pgm file
   sprintf(filename, "seg_%08d.pgm", pic2);
   rox_array2d_uchar_save_pgm(filename, obj->edges);
   pic2++;
#endif	//FPSM_DEBUG

function_terminate:
   return error;
}

Rox_ErrorCode rox_fpsm_preprocess(Rox_Fpsm obj, Rox_Image source, Rox_Double min_NFA, Rox_Uint nbr_blur_passes, Rox_Uint min_segment_size, Rox_Bool straight_edge_only)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Sshort_Struct closest;


   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Point2D_Sshort_Struct ** dp = NULL;
   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer( &dp, obj->distancemap_points);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sshort ** da = NULL;
   error = rox_array2d_sshort_get_data_pointer_to_pointer( &da, obj->angles);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sshort ** dm = NULL;
   error = rox_array2d_sshort_get_data_pointer_to_pointer( &dm, obj->anglemap);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_fpsm_extract_edges(obj, source, min_NFA, nbr_blur_passes, min_segment_size, straight_edge_only);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_distancetransform(obj->distancemap, obj->distancemap_points, obj->edges);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Fill angle map with per point closest edge angle
   for ( Rox_Sint i = 0; i < obj->height; i++)
   {
      for ( Rox_Sint j = 0; j < obj->width; j++)
      {
         closest = dp[i][j];
         // temporary test to solve a bug. TODO : check why obj->distancemap_points is not in bounds
         if((closest.u >= 0) && (closest.v >= 0) && (closest.u < (Rox_Sshort)obj->width) && (closest.v < (Rox_Sshort)obj->height))
         {
            dm[i][j] = da[closest.v][closest.u];
         }
         else
         {error = ROX_ERROR_PROCESS_FAILED; ROX_ERROR_CHECK_TERMINATE ( error );}
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_fpsm_compute(Rox_Fpsm_Feature_Struct * feat, Rox_Fpsm obj, Rox_Uint x, Rox_Uint y, Rox_Sint width, Rox_Sint height)
{
   Rox_ErrorCode error = 0;
   Rox_Sshort ** dd = NULL;
   Rox_Point2D_Sshort_Struct ** dp = NULL;
   Rox_Sshort ** da = NULL;
   Rox_Sint pos = 0;
   Rox_Sint px, py, u, v;
   Rox_Sint sq_nb_refp;
   Rox_Sint half_x, half_y, delta_x, delta_y;


   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!feat)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //if (feat->angles == NULL)
   //{ error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //if (!feat->distances)
   //{ error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }


   error = rox_array2d_sshort_get_data_pointer_to_pointer( &dd, obj->distancemap);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer(&dp, obj->distancemap_points);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_sshort_get_data_pointer_to_pointer(&da, obj->angles);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //determine how the reference points will be spread throughout the object
   //TODO uniform distribution
   sq_nb_refp = (Rox_Uint)sqrt((Rox_Double)obj->nbr_ref_points);

   delta_x = width/sq_nb_refp;
   delta_y = height/sq_nb_refp;

   half_x = delta_x / 2;
   half_y = delta_y / 2;

   feat->top = y;
   feat->left = x;

   for (px = (Rox_Sint)(x+half_x); px < (Rox_Sint)(x+width); px+=delta_x)
   {
      for (py = (Rox_Sint)(y+half_y); py < (Rox_Sint)(y+height); py+=delta_y)
      {
         //out of the test image, just skip it (needed for non-centered templates)
         if(py>=(Rox_Sint)(obj->height) || px>=(Rox_Sint)(obj->width) || py<0|| px<0)
            continue;

         u = dp[py][px].u;
         v = dp[py][px].v;

         feat->distances[pos] = sqrt((Rox_Double)dd[py][px]);

         //temporary test to solve a bug. TODO : check why obj->distancemap_points is not in bounds
         if((u >= 0) && (v >= 0) && (u < (Rox_Sint)(obj->width)) && (v < (Rox_Sint)(obj->height)))
         {
            feat->angles[pos] = da[v][u]/10000.0;//x1000 to avoid storing float when precision is not needed
         }
         else
            feat->angles[pos] = 0;

         pos++;
      }
   }

function_terminate:
   return error;
}
