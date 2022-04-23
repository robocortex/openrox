//==============================================================================
//
//    OPENROX   : File sdwm.h
//
//    Contents  : Implementation of sdwm module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "sdwm.h"

#include <stdio.h>
#include <baseproc/maths/maths_macros.h>

#include <generated/dynvec_rect_sint.h>
#include <generated/dynvec_sint_struct.h>
#include <generated/dynvec_point2d_sshort_struct.h>

#include <system/errors/errors.h>
#include <system/memory/memory.h>
#include <system/time/timer.h>

#include <core/templatesearch/ocm.h>
#include <core/features/descriptors/fpsm/fpsm_struct.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

ROX_API Rox_ErrorCode rox_sdwm_new(Rox_Sdwm * obj, Rox_Sdwm_Create_Params params)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sdwm ret = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Sdwm)rox_memory_allocate(sizeof(*ret), 1);

   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->pyramid = NULL;
   ret->objects = NULL;
   ret->fpsm_model = NULL;
   ret->fpsm_currents = NULL;
   ret->results = NULL;
   ret->results_indices = NULL;
   ret->results_scores = NULL;
   ret->index = NULL;

   ret->nb_levels = params->nbr_levels;
   ret->scale_per_level = params->scale_per_Level;
   ret->width_current = params->width_current;
   ret->height_current = params->height_current;

   ret->width_model = params->width_model;
   ret->height_model = params->height_model;
   ret->sum_edges = 0;
   ret->count_templates = 0;

   //  Create a npot (not power of two) pyramid object
   error = rox_pyramid_npot_uchar_new(&ret->pyramid, ret->width_current, ret->height_current, ret->nb_levels, ret->scale_per_level);

   ROX_ERROR_CHECK_TERMINATE ( error );
   
   //  Create a set of sdwm objects 
   error = rox_objset_sdwm_object_new(&ret->objects, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   //  Create a set of rectangles for results 
   error = rox_objset_dynvec_rect_sint_new(&ret->results, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_dynvec_sint_new(&ret->results_indices, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_dynvec_sint_new(&ret->results_scores, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_fpsm_new(&ret->fpsm_model, ret->width_model, ret->height_model, params->nbr_ref_points, params->min_gradient);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_fpsm_index_new(&ret->index, params->nbr_distances, params->nbr_angles, params->nbr_ref_points, params->min_votes, ret->width_model, ret->height_model);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->fpsm_currents = (Rox_Fpsm*)rox_memory_allocate(sizeof(Rox_Fpsm), ret->nb_levels);
   if (!ret->fpsm_currents)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   //  Init all pointers to NULL
   for (Rox_Uint idlevel = 0; idlevel < ret->nb_levels; idlevel++)
   {
      ret->fpsm_currents[idlevel] = NULL;
   }

   //  Init each level of the pyramid
   for (Rox_Uint idlevel = 0; idlevel < ret->nb_levels; idlevel++)
   {
      Rox_Sint cols = 0, rows = 0;
      error = rox_array2d_uchar_get_size(&rows, &cols, ret->pyramid->levels[idlevel]);
      ROX_ERROR_CHECK_TERMINATE ( error );
     error = rox_fpsm_new(&ret->fpsm_currents[idlevel], cols, rows, params->nbr_ref_points, params->min_gradient);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = rox_fpsm_index_init(ret->index);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = ROX_ERROR_NONE;
   *obj = ret;

function_terminate:
   if (error) rox_sdwm_del(&ret);

   return error;
}

Rox_ErrorCode rox_sdwm_del(Rox_Sdwm *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sdwm todel = NULL;

   if (!obj)
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *obj;
   *obj = NULL;


   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_objset_dynvec_rect_sint_del(&todel->results);
   rox_dynvec_sint_del(&todel->results_indices);
   rox_dynvec_sint_del(&todel->results_scores);
   rox_objset_sdwm_object_del(&todel->objects);
   rox_pyramid_npot_uchar_del(&todel->pyramid);
   rox_fpsm_del(&todel->fpsm_model);
   rox_fpsm_index_del(&todel->index);

   if (todel->fpsm_currents)
   {
      for (Rox_Uint idlevel = 0; idlevel < todel->nb_levels; idlevel++)
      {
         rox_fpsm_del(&todel->fpsm_currents[idlevel]);
      }
      rox_memory_delete(todel->fpsm_currents);
   }

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_sdwm_process_level(Rox_Sdwm obj, Rox_Uint level, Rox_Sdwm_Process_Params params)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Image source = NULL;
   Rox_Fpsm fpsm = NULL;
   Rox_Sint width = 0, height = 0;
   Rox_Uint rescnt, idres;
   Rox_Double scale, score, mean;
   Rox_Sint score_int = 0;
   Rox_Fpsm_Feature_Struct lfeat;
   Rox_Fpsm_Template_Struct tmp;
   Rox_Sdwm_Object curobj = NULL;
   Rox_Rect_Sint_Struct detectedpt;
   Rox_Sshort i, j, max_i, max_j, min_i, min_j;
   //Rox_Timer timer = NULL;
   //Rox_Double timems = 0.0;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   fpsm = obj->fpsm_currents[level];
   source = obj->pyramid->levels[level];

   error = rox_array2d_uchar_get_size(&height, &width, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   scale = pow((double)obj->scale_per_level, (int)level);

   mean = (double)obj->sum_edges / (double)obj->count_templates;

   error = rox_fpsm_preprocess(fpsm, source, params->min_NFA, params->nbr_blur_passes, (Rox_Uint)(params->min_segment_size/scale), params->straight_edge_only);
   //error = rox_timer_new(&timer);
   //error = rox_timer_start(timer);

   // if non centered template, decal tests up to half of template size outside tested image (neg and pos)
   max_i = height - obj->height_model + (Rox_Sint)(obj->height_model*params->template_ratio);
   max_j = width - obj->width_model + (Rox_Sint)(obj->width_model*params->template_ratio);
   min_i = -(Rox_Sint)(obj->height_model*params->template_ratio);
   min_j = -(Rox_Sint)(obj->width_model*params->template_ratio);

   if(max_i<0)max_i = 0;
   if(max_j<0)max_j = 0;
   //<= test instead of strict <, otherwise skip the exact size template test
   for (i = min_i; i <= max_i; i+=params->step_search)
   {
      for (j = min_j; j <= max_j; j+=params->step_search)
      {

#ifdef SDWM_DEBUG
   rox_log("\npos : %d, %d\n", i , j);
#endif   //SDWM_DEBUG
         error = rox_fpsm_compute(&lfeat, fpsm, j, i, obj->width_model, obj->height_model);
         error = rox_fpsm_index_search(obj->index, &lfeat);
         rescnt = obj->index->results->used;
         if (rescnt == 0) continue;

         for (idres = 0; idres < rescnt; idres++)
         {
            tmp = obj->index->results->data[idres];
            curobj = obj->objects->data[tmp.object_id];


            error = rox_ocm_cardinal_process(&score, j, i
               , curobj->pointsset->data[tmp.view_id], curobj->anglemaps->data[tmp.view_id]
            , fpsm->anglemap, fpsm->distancemap, params->ocm_lambda, mean, params->ocm_max_dist, params->ocm_max_angle);

#ifdef SDWM_DEBUG
   rox_log("obj %d score %.2f\n", tmp.object_id, score);
#endif   //SDWM_DEBUG
            if (score < params->ocm_score_min) continue;

            score_int = (Rox_Sint)(score*10000);
            detectedpt.x = (Rox_Sint)(j * scale);
            detectedpt.y = (Rox_Sint)(i * scale);
            detectedpt.width = (Rox_Sint)(obj->width_model * scale);
            detectedpt.height = (Rox_Sint)(obj->height_model * scale);
            error = rox_dynvec_rect_sint_append(obj->results->data[tmp.object_id], &detectedpt);
            error = rox_dynvec_sint_append(obj->results_indices, &tmp.view_id);
            error = rox_dynvec_sint_append(obj->results_scores, &score_int);
         }
      }
   }

   // error = rox_timer_stop(timer);
   // error = rox_timer_get_elapsed_ms(&timems, timer);
   // error = rox_timer_del(&timer);

function_terminate:
   return error;
}

Rox_ErrorCode rox_sdwm_process(Rox_Sdwm obj, Rox_Sdwm_Process_Params params )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint level = 0, idobject;

   if (!obj || !params->image) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   //  Create the pyramid from the source image
   error = rox_pyramid_npot_uchar_assign(obj->pyramid, params->image);

   //  Reset the resulting rectangles
   for (idobject = 0; idobject < obj->results->used; idobject++)
   {
      rox_dynvec_rect_sint_reset(obj->results->data[idobject]);
   }
   rox_dynvec_sint_reset(obj->results_indices);
   rox_dynvec_sint_reset(obj->results_scores);

   //  Process each level of the pyramid
   if(params->only_process_last_level)
      level = obj->pyramid->nb_levels-1;

   for (; level < obj->pyramid->nb_levels; level++)
   {
      error = rox_sdwm_process_level(obj, level, params);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_sdwm_add_object(Rox_Sdwm obj, Rox_Sdwm_Object object_toadd, Rox_Double min_NFA, Rox_Uint nbr_blur_passes, Rox_Uint min_segment_size, Rox_Bool straight_edge_only)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint idobject, idtemplate;
   Rox_Image source = NULL;
   Rox_Uchar ** de;
   Rox_Point2D_Sshort_Struct npt;
   Rox_DynVec_Point2D_Sshort listpt;
   Rox_DynVec_Rect_Sint resultvec = NULL;
   Rox_DynVec_Sint intvec;
   Rox_Ushort i,j;

   if (!obj || !object_toadd)
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   idobject = obj->objects->used;
   for (idtemplate = 0; idtemplate < object_toadd->templates->used; idtemplate++)
   {
      source = object_toadd->templates->data[idtemplate];
      error = rox_fpsm_preprocess(obj->fpsm_model, source, min_NFA, nbr_blur_passes, min_segment_size, straight_edge_only);
      error = rox_fpsm_compute(&object_toadd->features->data[idtemplate], obj->fpsm_model, 0, 0, obj->width_model, obj->height_model);
      error = rox_array2d_uchar_copy(object_toadd->edgemaps->data[idtemplate], obj->fpsm_model->edges);
      error = rox_array2d_sshort_copy(object_toadd->anglemaps->data[idtemplate], obj->fpsm_model->anglemap);

      // Create list of edges points 2D from edge map
      listpt = object_toadd->pointsset->data[idtemplate];
      error = rox_array2d_uchar_get_data_pointer_to_pointer(&de, obj->fpsm_model->edges);

      for (i = 0; i < obj->height_model; i++)
      {
         npt.v = i;
         for (j = 0; j < obj->width_model; j++)
         {
            if (de[i][j])
            {
               npt.u = j;
               error = rox_dynvec_point2d_sshort_append(listpt, &npt);
            }
         }
      }

      obj->sum_edges += listpt->used;
   }

   obj->count_templates += object_toadd->templates->used;

   error = rox_fpsm_index_append_object(obj->index, object_toadd->features, idobject);
   error = rox_objset_sdwm_object_append(obj->objects, object_toadd);
   error = rox_dynvec_rect_sint_new(&resultvec, 10);
   error = rox_dynvec_sint_new(&intvec, 10);
   error = rox_objset_dynvec_rect_sint_append(obj->results, resultvec);
   error = rox_dynvec_sint_append(obj->results_indices, intvec->data);
   error = rox_dynvec_sint_append(obj->results_scores, intvec->data);

   resultvec = NULL;

   error = ROX_ERROR_NONE;

function_terminate:
   rox_dynvec_rect_sint_del(&resultvec);

   return error;
}



