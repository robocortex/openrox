//==============================================================================
//
//    OPENROX   : File multiident.c
//
//    Contents  : Implementation of multiident module
//                This module allows the matching of several reference images
//                with a current image using SRAID features
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "multiident.h"
#include "multiident_struct.h"

#include <stdio.h>
#include <generated/dynvec_uint_struct.h>
#include <generated/dynvec_point2d_float_struct.h>
#include <generated/dynvec_point3d_float_struct.h>
#include <generated/dynvec_point3d_double_struct.h>

#include <baseproc/image/remap/remap_bilinear_nomask_float_to_float/remap_bilinear_nomask_float_to_float_doubled.h>

#include <core/features/descriptors/sraid/sraid.h>
#include <core/features/descriptors/sraid/sraid_matchset.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_multi_ident_new(Rox_Multi_Ident *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Multi_Ident ret = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret = (Rox_Multi_Ident) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->idents = NULL;
   ret->matcheslist = NULL;
   ret->current_features = NULL;

   error = rox_objset_template_ident_new(&ret->idents, 10);

   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_dynvec_sint_new(&ret->matcheslist, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_dynvec_sraiddesc_new(&ret->current_features, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *obj = ret;

function_terminate:
   if (error) rox_multi_ident_del(&ret);
   return error;
}

Rox_ErrorCode rox_multi_ident_del(Rox_Multi_Ident *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Multi_Ident todel = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_sraiddesc_del(&todel->current_features);
   rox_objset_template_ident_del(&todel->idents);
   rox_dynvec_sint_del(&todel->matcheslist);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_multi_ident_add_template(Rox_Multi_Ident obj, Rox_Array2D_Float reference, Rox_Array2D_Double calib_template, Rox_Uint use_affine, Rox_Uint use_double_image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Template_Ident toadd = NULL;


   if (!obj || !reference) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_template_ident_new(&toadd);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (use_affine)
   {
      error = rox_template_ident_set_model_affine(toadd, reference, calib_template, use_double_image);
      ROX_ERROR_CHECK_TERMINATE ( error );
  }
   else
   {
      error = rox_template_ident_set_model(toadd, reference, calib_template, use_double_image);
      ROX_ERROR_CHECK_TERMINATE ( error );
  }

   error = rox_objset_template_ident_append(obj->idents, toadd);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if (error) rox_template_ident_del(&toadd);

   return error;
}

Rox_ErrorCode rox_multi_ident_reset(Rox_Multi_Ident obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   
   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   rox_objset_template_ident_reset(obj->idents);

function_terminate:
   return error;
}

Rox_ErrorCode rox_multi_ident_compile(Rox_Multi_Ident obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   
   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

function_terminate:
   return error;
}

Rox_ErrorCode rox_multi_ident_make(Rox_Multi_Ident obj, Rox_Array2D_Float current_image, Rox_Uint dbl_image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_SRAID_Feature ref = NULL;
   Rox_Point2D_Float_Struct toadd;
   Rox_Array2D_Float source = NULL;


   if (!obj || !current_image) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   Rox_Sint w = 0, h = 0;

   error = rox_array2d_float_get_size(&h, &w, current_image); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (dbl_image)
   {
      error = rox_array2d_float_new(&source, h * 2, w * 2);

      ROX_ERROR_CHECK_TERMINATE ( error );

      error = remap_bilinear_nomask_float_to_float_doubled(source, current_image);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      source = current_image;
   }

   // reset dynvec
   rox_dynvec_sraiddesc_reset(obj->current_features);

   // sraid detection
   error = rox_sraidpipeline_process(obj->current_features, source, -1, 1.6f, 3.0f, 0.5f, 3, 0.04f, 10, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Rescale
   if (dbl_image)
   {
      for (Rox_Uint idpt = 0; idpt < obj->current_features->used; idpt++)
      {
         obj->current_features->data[idpt].x /= 2.0;
         obj->current_features->data[idpt].y /= 2.0;
      }
   }

   // For all reference views
   for (Rox_Uint idtemplate = 0; idtemplate < obj->idents->used; idtemplate++)
   {
      Rox_Template_Ident tmp = obj->idents->data[idtemplate];

      rox_dynvec_point2d_float_reset(tmp->current_points_matched);
      rox_dynvec_point2d_float_reset(tmp->reference_points_matched);

      tmp->count_matched = 0;

      for (Rox_Uint idset = 0; idset < 12 + 1; idset++)
      {
         ref = tmp->reference_features_subsets->data[idset];
         if (ref->used == 0) continue;

         // Retrieve matches between current and reference points
         error = rox_sraid_matchset(obj->matcheslist, obj->current_features, ref);
         ROX_ERROR_CHECK_TERMINATE ( error );

         for (Rox_Uint idmatch = 0; idmatch < obj->matcheslist->used; idmatch++)
         {
            Rox_Sint idx = obj->matcheslist->data[idmatch];

            if (idx < 0) continue;
            if (idx >= (Rox_Sint)ref->used) continue;

            toadd.u = ref->data[idx].x;
            toadd.v = ref->data[idx].y;
            rox_dynvec_point2d_float_append(tmp->reference_points_matched, &toadd);

            toadd.u = obj->current_features->data[idmatch].x;
            toadd.v = obj->current_features->data[idmatch].y;
            rox_dynvec_point2d_float_append(tmp->current_points_matched, &toadd);

            tmp->count_matched++;
         }
      }
   }

function_terminate:
   if (dbl_image) rox_array2d_float_del(&source);
   return error;
}

Rox_ErrorCode rox_multi_ident_find_next_best(Rox_Uint * id, Rox_Multi_Ident multi_ident)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint idmax = -1, maxval = -1;

   if (!id || !multi_ident) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   for (Rox_Uint i = 0; i < multi_ident->idents->used; i++)
   {
      // rox_log("multi_ident->idents->data[i]->count_matched = %d\n", multi_ident->idents->data[i]->count_matched);
      if (multi_ident->idents->data[i]->count_matched > maxval)
      {
         // Number of matched points
         maxval = multi_ident->idents->data[i]->count_matched;
         // Identifier
         idmax = i;
      }
   }

   if (idmax < 0) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   // Test if enought matched points (maxval) are available
   if (maxval <= 4) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   multi_ident->idents->data[idmax]->count_matched = 0;
   *id = idmax;

function_terminate:
   return error;
}

Rox_ErrorCode rox_multi_ident_make_features ( Rox_Multi_Ident obj, Rox_Array2D_Float current_image, Rox_Uint dbl_image, Rox_DynVec_Point3D_Float reference_points_meters_matched, Rox_DynVec_Point3D_Double reference_points_meters_extracted)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint idx;
   Rox_Uint idtemplate = 0;
   Rox_Uint idset = 0;
   Rox_Uint idmatch = 0;
   Rox_Uint idpt = 0;

   Rox_Point3D_Float_Struct point3D;
   Rox_DynVec_SRAID_Feature ref;
   Rox_Point2D_Float_Struct toadd;

   Rox_Array2D_Float source = NULL;


   if (!obj || !current_image) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint w = 0, h = 0; 
   error = rox_array2d_float_get_size(&h, &w, current_image); 
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   if (dbl_image)
   {
      error = rox_array2d_float_new(&source, h * 2, w * 2);

      ROX_ERROR_CHECK_TERMINATE ( error );

      error = remap_bilinear_nomask_float_to_float_doubled(source, current_image);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      source = current_image;
   }

   // reset dynvec
   rox_dynvec_sraiddesc_reset(obj->current_features);

   // sraid detection
   error = rox_sraidpipeline_process(obj->current_features, source, -1, 1.6f, 3.0f, 0.5f, 3, 0.04f, 10, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Rescale
   if (dbl_image)
   {
      for (idpt = 0; idpt < obj->current_features->used; idpt++)
      {
         obj->current_features->data[idpt].x /= 2.0;
         obj->current_features->data[idpt].y /= 2.0;
      }
   }

   // Reset the 3D matched points
   rox_dynvec_point3d_float_reset(reference_points_meters_matched);

   // For all reference views
   for (idtemplate = 0; idtemplate < obj->idents->used; idtemplate++)
   {
      Rox_Template_Ident tmp = obj->idents->data[idtemplate];

      rox_dynvec_point2d_float_reset(tmp->current_points_matched);
      rox_dynvec_point2d_float_reset(tmp->reference_points_matched);

      tmp->count_matched = 0;

      for (idset = 0; idset < 12 + 1; idset++)
      {
         ref = tmp->reference_features_subsets->data[idset];
         if (ref->used == 0) continue;

         // Retrieve matches between current and reference points
         error = rox_sraid_matchset(obj->matcheslist, obj->current_features, ref);
         ROX_ERROR_CHECK_TERMINATE ( error );

         for (idmatch = 0; idmatch < obj->matcheslist->used; idmatch++)
         {
            idx = obj->matcheslist->data[idmatch];

            if (idx < 0) continue;
            if (idx >= (Rox_Sint)ref->used) continue;

            toadd.u = ref->data[idx].x;
            toadd.v = ref->data[idx].y;
            rox_dynvec_point2d_float_append(tmp->reference_points_matched, &toadd);

            toadd.u = obj->current_features->data[idmatch].x;
            toadd.v = obj->current_features->data[idmatch].y;
            rox_dynvec_point2d_float_append(tmp->current_points_matched, &toadd);

            // add the 3D point to the list
            point3D.X = (Rox_Float)reference_points_meters_extracted->data[idx].X;
            point3D.Y = (Rox_Float)reference_points_meters_extracted->data[idx].Y;
            point3D.Z = (Rox_Float)reference_points_meters_extracted->data[idx].Z;

            rox_dynvec_point3d_float_append(reference_points_meters_matched, &point3D);

            tmp->count_matched++;
         }
      }
   }

function_terminate:
   if (dbl_image) rox_array2d_float_del(&source);
   return error;
}

Rox_ErrorCode rox_multi_ident_get_selected_features(Rox_DynVec_SRAID_Feature sraid_ref, Rox_DynVec_Uint list_match_flag, Rox_Multi_Ident multi_ident)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_SRAID_Feature ref = NULL;

      
   if (!sraid_ref || !list_match_flag || !multi_ident) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // For all reference views
   for (Rox_Uint idtemplate = 0; idtemplate < multi_ident->idents->used; idtemplate++)
   {
      Rox_Template_Ident tmp = multi_ident->idents->data[idtemplate];

      rox_dynvec_point2d_float_reset(tmp->current_points_matched);
      rox_dynvec_point2d_float_reset(tmp->reference_points_matched);

      tmp->count_matched = 0;

      for (Rox_Uint idset = 0; idset < 12 + 1; idset++)
      {
         ref = tmp->reference_features_subsets->data[idset];
         if (ref->used == 0) continue;

         // Retrieve matches between current and reference points
         // error = rox_sraid_matchset_constrained(obj->matcheslist, obj->current_features, ref);
         // ROX_ERROR_CHECK_TERMINATE ( error );

         for (Rox_Uint idmatch = 0; idmatch < multi_ident->matcheslist->used; idmatch++)
         {
            Rox_Sint idx = multi_ident->matcheslist->data[idmatch];

            if (idx < 0) continue;
            if (idx >= (Rox_Sint)ref->used) continue;

            //toadd.u = ref->data[idx].x;
            //toadd.v = ref->data[idx].y;
            //rox_dynvec_point2d_float_append(tmp->reference_points_matched, &toadd);

            //toadd.u = obj->current_features->data[idmatch].x;
            //toadd.v = obj->current_features->data[idmatch].y;
            //rox_dynvec_point2d_float_append(tmp->current_points_matched, &toadd);

            if(list_match_flag->data[tmp->count_matched])
            {
               // add the sraid_feature to the list
               rox_dynvec_sraiddesc_append(sraid_ref, &ref->data[idx]);
            }
            tmp->count_matched++;
         }
      }
   }

function_terminate:
   return error;
}

