//==============================================================================
//
//    OPENROX   : File ident_database_se3.c
//
//    Contents  : Implementation of ident_database_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ident_database_se3.h"

#include <baseproc/maths/maths_macros.h>
#include <core/identification/dbident_se3_struct.h>
#include <inout/system/errors_print.h>
#include <user/sensor/camera/camera_struct.h>
#include <user/identification/database/database_struct.h>

Rox_ErrorCode rox_ident_database_se3_new (
  Rox_Ident_Database_SE3 *ident, 
  const Rox_Uint max_templates_simultaneous)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_db_ident_se3_new ( ident, max_templates_simultaneous );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_database_se3_del ( 
  Rox_Ident_Database_SE3 * ident 
)
{
   return rox_db_ident_se3_del(ident);
}

Rox_ErrorCode rox_ident_database_se3_set_database (
  Rox_Ident_Database_SE3 ident, 
  const Rox_Database db
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!db || !ident)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_db_ident_se3_set_database(ident, db->database);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_database_se3_make (
  Rox_Ident_Database_SE3 ident, 
  const Rox_Camera camera 
)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    if (!ident || !camera)
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    error = rox_db_ident_se3_extract ( ident, camera->image );
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_db_ident_se3_estimate_poses ( ident, camera->calib_camera );
    ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
    return error;
}

Rox_ErrorCode rox_ident_database_se3_make_features (
  Rox_Ident_Database_SE3 ident, 
  const Rox_DynVec_Ehid_Point features, 
  const Rox_Matrix calib_camera
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ident || !features || !calib_camera)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_db_ident_se3_set_extracted_features(ident, features);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_db_ident_se3_estimate_poses(ident, calib_camera);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_database_se3_extract_features (
  Rox_DynVec_Ehid_Point features, 
  const Rox_Ident_Database_SE3 ident, 
  const Rox_Camera camera
)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    if (!features || !ident || !camera)
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    rox_dynvec_ehid_point_reset(features);

    error = rox_db_ident_se3_extract(ident, camera->image);
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_dynvec_ehid_point_stack(features, ident->_curfeats_pyr);
    ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
    return error;
}

Rox_ErrorCode rox_ident_database_se3_getcountframes (
  Rox_Uint * count, 
  const Rox_Ident_Database_SE3 ident
)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    if(!count || !ident)
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    *count = ident->_database->_targets->used;

function_terminate:
    return error;
}

Rox_ErrorCode rox_ident_database_se3_getresult (
  Rox_Sint * is_identified, 
  Rox_MatSE3 pose, 
  const Rox_Ident_Database_SE3 ident, 
  const Rox_Uint id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!is_identified || !pose || !ident)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if(id > ident->_database->_targets->used)
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if(ident->_database->_targets->data[id]->posefound)
   {
      *is_identified = 1;
      error = rox_matse3_copy(pose, ident->_database->_targets->data[id]->pose);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      *is_identified = 0;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_database_se3_get_result_force_sizex (
  Rox_Sint * is_identified, 
  Rox_MatSE3 pose, 
  const Rox_Ident_Database_SE3 ident, 
  const Rox_Double sizex, 
  const Rox_Uint id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!is_identified || !pose || !ident)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if(id > ident->_database->_targets->used)
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if(ident->_database->_targets->data[id]->posefound)
   {
      *is_identified = 1;

      // Copy pose
      error = rox_matse3_copy(pose, ident->_database->_targets->data[id]->pose);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Scale pose fixing sizex
      Rox_Double cols = (Rox_Double) ident->_database->_targets->data[id]->width_meters;
      error = rox_matse3_scale_translation(pose, sizex/cols);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      *is_identified = 0;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_database_se3_get_result_force_sizey (
  Rox_Sint * is_identified, 
  Rox_MatSE3 pose, 
  const Rox_Ident_Database_SE3 ident, 
  const Rox_Double sizey, 
  const Rox_Uint id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!is_identified || !pose || !ident)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if(id > ident->_database->_targets->used)
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if(ident->_database->_targets->data[id]->posefound)
   {
      *is_identified = 1;

      // Copy pose
      error = rox_matse3_copy(pose, ident->_database->_targets->data[id]->pose);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Scale pose fixing sizex
      Rox_Double rows = (Rox_Double) ident->_database->_targets->data[id]->height_meters;
      error = rox_matse3_scale_translation(pose, sizey/rows);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      *is_identified = 0;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_database_se3_getscore (
  Rox_Double * score, 
  const Rox_Ident_Database_SE3 ident, 
  const Rox_Uint id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ident || !score)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (id > ident->_database->_targets->used)
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if(ident->_database->_targets->data[id]->posefound)
   {
      double val = 1;

      if (ident->_database->_targets->data[id]->best_score_minimization > 0)
      {
         val = (double)ident->_database->_targets->data[id]->best_score_minimization;
      }

      *score = 1.0 - exp(-val/10.0);
   }
   else
   {
      *score = 0.0;
   }

function_terminate:
   return error;
}
