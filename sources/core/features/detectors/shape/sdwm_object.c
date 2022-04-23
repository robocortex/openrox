//==============================================================================
//
//    OPENROX   : File sdwm_object.h
//
//    Contents  : Implementation of sdwm_object module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "sdwm_object.h"

#include <baseproc/maths/maths_macros.h>

#include <system/errors/errors.h>
#include <system/memory/memory.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_sdwm_object_new(Rox_Sdwm_Object * obj, Rox_Sint width_model, Rox_Sint height_model)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sdwm_Object ret = NULL;

   if ((width_model<1) || (height_model<1)) { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   *obj = NULL;

   ret = (Rox_Sdwm_Object)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->templates = NULL;
   ret->edgemaps = NULL;
   ret->anglemaps = NULL;
   ret->features = NULL;
   ret->pointsset = NULL;
   ret->width = width_model;
   ret->height = height_model;

   CHECK_ERROR_TERMINATE(rox_objset_array2d_uchar_new(&ret->templates, 10));
   CHECK_ERROR_TERMINATE(rox_objset_array2d_uchar_new(&ret->edgemaps, 10));
   CHECK_ERROR_TERMINATE(rox_objset_array2d_sshort_new(&ret->anglemaps, 10));
   CHECK_ERROR_TERMINATE(rox_dynvec_fpsm_feature_new(&ret->features, 10));
   CHECK_ERROR_TERMINATE(rox_objset_dynvec_point2d_sshort_new(&ret->pointsset, 10));

   *obj = ret;

function_terminate:
   if (error) rox_sdwm_object_del(&ret);

   return error;
}

Rox_ErrorCode rox_sdwm_object_del(Rox_Sdwm_Object *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sdwm_Object todel = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *obj;
   *obj = NULL;

   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_objset_array2d_uchar_del(&todel->templates);
   rox_objset_array2d_uchar_del(&todel->edgemaps);
   rox_objset_array2d_sshort_del(&todel->anglemaps);
   rox_dynvec_fpsm_feature_del(&todel->features);
   rox_objset_dynvec_point2d_sshort_del(&todel->pointsset);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_sdwm_object_add_template(Rox_Sdwm_Object obj, Rox_Image image_template)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Image ntemp = NULL, nedge = NULL;
   Rox_Array2D_Sshort nangle = NULL;
   
   Rox_Fpsm_Feature_Struct nfeature;

   Rox_DynVec_Point2D_Sshort points;

   if (!obj || !image_template) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ntemp = NULL;
   nedge = NULL;
   nangle = NULL;
   points = NULL;

   CHECK_ERROR_TERMINATE(rox_array2d_uchar_check_size(image_template, obj->height, obj->width));

   {
      CHECK_ERROR_TERMINATE(rox_array2d_uchar_new(&ntemp, obj->height, obj->width));
      CHECK_ERROR_TERMINATE(rox_array2d_uchar_copy(ntemp, image_template));
      CHECK_ERROR_TERMINATE(rox_objset_array2d_uchar_append(obj->templates, ntemp));
      ntemp = NULL;
   }

   {
      CHECK_ERROR_TERMINATE(rox_array2d_uchar_new(&nedge, obj->height, obj->width));
      CHECK_ERROR_TERMINATE(rox_objset_array2d_uchar_append(obj->edgemaps, nedge));
      nedge = NULL;
   }

   {
      CHECK_ERROR_TERMINATE(rox_array2d_sshort_new(&nangle, obj->height, obj->width));
      CHECK_ERROR_TERMINATE(rox_objset_array2d_sshort_append(obj->anglemaps, nangle));
      nangle = NULL;
   }

   {
      CHECK_ERROR_TERMINATE(rox_dynvec_fpsm_feature_append(obj->features, &nfeature));
      nangle = NULL;
   }

   {
      CHECK_ERROR_TERMINATE(rox_dynvec_point2d_sshort_new(&points, 10));
      CHECK_ERROR_TERMINATE(rox_objset_dynvec_point2d_sshort_append(obj->pointsset, points));
      points = NULL;
   }

function_terminate:
   rox_array2d_uchar_del(&ntemp);
   rox_array2d_uchar_del(&nedge);
   rox_array2d_sshort_del(&nangle);
   rox_dynvec_point2d_sshort_del(&points);

   return error;
}


