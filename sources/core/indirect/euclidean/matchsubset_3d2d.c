//==============================================================================
//
//    OPENROX   : File matchsubset_3d2d.c
//
//    Contents  : Implementation of matchsubset_3d2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "matchsubset_3d2d.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_match_float_extract_subset3d2d(Rox_DynVec_Point3D_Float subset3d, Rox_DynVec_Point2D_Float subset2d, Rox_DynVec_Sint matches, Rox_DynVec_Point3D_Float iniset3d, Rox_DynVec_Point2D_Float iniset2d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!subset3d || !subset2d || !matches || !iniset3d || !iniset2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (matches->used != iniset2d->used)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   subset3d->used = 0;
   subset2d->used = 0;
   
   for ( Rox_Sint i = 0; i < matches->used; i++)
   {
      idx = matches->data[i];
      if (idx < 0) continue;
      if (idx >= iniset3d->used) continue;
      
      subset3d->data[subset3d->used] = iniset3d->data[idx];
      subset2d->data[subset2d->used] = iniset2d->data[i];
      
      rox_dynvec_point3d_float_usecells(subset3d, 1);
      rox_dynvec_point2d_float_usecells(subset2d, 1);
   }
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_match_float_extract_subset2d2d(Rox_DynVec_Point2D_Float subsetref, Rox_DynVec_Point2D_Float subsetcur, Rox_DynVec_Sint matches, Rox_DynVec_Point2D_Float inisetref, Rox_DynVec_Point2D_Float inisetcur)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!subsetref || !subsetcur || !matches || !inisetref || !inisetcur) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (matches->used != inisetcur->used) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   subsetref->used = 0;
   subsetcur->used = 0;
   
   for ( Rox_Sint i = 0; i < matches->used; i++)
   {
      Rox_Sint idx = matches->data[i];
      if (idx < 0) continue;
      if (idx >= inisetref->used) continue;
      
      subsetref->data[subsetref->used] = inisetref->data[idx];
      subsetcur->data[subsetref->used] = inisetcur->data[i];
      
      rox_dynvec_point2d_float_usecells(subsetref, 1);
      rox_dynvec_point2d_float_usecells(subsetcur, 1);
   }
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_match_float_extract_subset2d2d_with_indices(Rox_DynVec_Point2D_Float subsetref, Rox_DynVec_Point2D_Float subsetcur, Rox_DynVec_Uint subsetidx, Rox_DynVec_Sint matches, Rox_DynVec_Point2D_Float inisetref, Rox_DynVec_Point2D_Float inisetcur)
{ 
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!subsetref || !subsetcur || !subsetidx || !matches || !inisetref || !inisetcur) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (matches->used != inisetcur->used) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   subsetref->used = 0;
   subsetcur->used = 0;
   subsetidx->used = 0;
   
   for ( Rox_Sint i = 0; i < matches->used; i++)
   {
      Rox_Sint idx = matches->data[i];
      if (idx < 0) continue;
      if (idx >= inisetref->used) continue;
      
      subsetref->data[subsetref->used] = inisetref->data[idx];
      subsetcur->data[subsetref->used] = inisetcur->data[i];
      subsetidx->data[subsetref->used] = i;
      
      rox_dynvec_point2d_float_usecells(subsetref, 1);
      rox_dynvec_point2d_float_usecells(subsetcur, 1);
      rox_dynvec_uint_usecells(subsetidx, 1);
   }
   
function_terminate:
   return error;
}