//==============================================================================
//
//    OPENROX   : File ident_database_sl3.c
//
//    Contents  : Implementation of ident_database_sl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ident_database_sl3.h"

#include <math.h>
#include <core/identification/dbident_sl3.h>
#include <inout/system/errors_print.h>
#include <user/identification/database/database_struct.h>

Rox_ErrorCode rox_ident_database_sl3_new(Rox_Ident_Database_SL3 * ident, Rox_Uint max_templates_simultaneous)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_db_ident_sl3_new(ident, max_templates_simultaneous);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_database_sl3_del(Rox_Ident_Database_SL3 * ident)
{
   return rox_db_ident_sl3_del(ident);
}

Rox_ErrorCode rox_ident_database_sl3_set_database(Rox_Ident_Database_SL3 ident, Rox_Database db)
{
   return rox_db_ident_sl3_set_database(ident, db->database);
}

Rox_ErrorCode rox_ident_database_sl3_make(Rox_Ident_Database_SL3 ident, Rox_Image image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!ident || !image) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_db_ident_sl3_extract(ident, image); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_db_ident_sl3_estimate_homographies(ident); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
    return error;
}

Rox_ErrorCode rox_ident_database_sl3_extract_features(Rox_DynVec_Ehid_Point features, Rox_Ident_Database_SL3 ident, Rox_Image image)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    if (!features || !ident ||!image) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    rox_dynvec_ehid_point_reset(features);

    error = rox_db_ident_sl3_extract(ident, image); 
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_dynvec_ehid_point_stack(features, ident->_curfeats_pyr);  
    ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
    return error;
}

Rox_ErrorCode rox_ident_database_sl3_make_features(Rox_Ident_Database_SL3 ident, Rox_DynVec_Ehid_Point features)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    if(!ident || !features) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    error = rox_db_ident_sl3_set_extracted_features(ident, features); 
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_db_ident_sl3_estimate_homographies(ident); 
    ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
    return error;
}

Rox_ErrorCode rox_ident_database_sl3_getcountframes(Rox_Uint * count, Rox_Ident_Database_SL3 ident)
{	 
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!count || !ident)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *count = ident->_database->_targets->used;

function_terminate:
    return error;
}

Rox_ErrorCode rox_ident_database_sl3_getresult(Rox_Sint * is_identified, Rox_MatSL3 homography, Rox_Ident_Database_SL3 ident, Rox_Uint id)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!is_identified || !homography || !ident) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
    
   if(id > ident->_database->_targets->used) 
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if(ident->_database->_targets->data[id]->posefound)
   {
      *is_identified = 1;
      
      error = rox_matsl3_copy(homography, ident->_database->_targets->data[id]->homography);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      *is_identified = 0;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_database_sl3_get_result_force_sizeu(Rox_Sint * is_identified, Rox_MatSL3 homography, Rox_Ident_Database_SL3 ident, Rox_Double sizeu, Rox_Uint id)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
      
   Rox_MatSL3 temp1 = NULL;
   Rox_MatSL3 temp2 = NULL;
   Rox_Double ** data = NULL;

   if (!is_identified || !homography || !ident) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
    
   if (id > ident->_database->_targets->used) 
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (ident->_database->_targets->data[id]->posefound)
   {
      *is_identified = 1;
      
      error = rox_matsl3_new(&temp1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_matsl3_new(&temp2);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Copy pose
      error = rox_matsl3_copy(temp1, ident->_database->_targets->data[id]->homography);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      // Scale pose fixing sizex
      Rox_Double cols = (Rox_Double) ident->_database->_targets->data[id]->width_pixels;

      error = rox_matsl3_get_data_pointer_to_pointer(&data, temp2);
      ROX_ERROR_CHECK_TERMINATE ( error );
     
      Rox_Double scale = pow((cols*cols)/(sizeu*sizeu), 1.0/3.0);
      data[0][0] = (cols/sizeu)/scale;
      data[1][1] = (cols/sizeu)/scale;
      data[2][2] = 1.0/scale;

      error = rox_matsl3_mulmatmat(homography, temp1, temp2);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      *is_identified = 0;
   }

function_terminate:

   rox_matsl3_del(&temp1);
   rox_matsl3_del(&temp2);

   return error;
}

Rox_ErrorCode rox_ident_database_sl3_get_result_force_sizev(Rox_Sint * is_identified, Rox_MatSL3 homography, Rox_Ident_Database_SL3 ident, Rox_Double sizev, Rox_Uint id)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_MatSL3 temp1 = NULL;
   Rox_MatSL3 temp2 = NULL;
   Rox_Double ** data = NULL;

   if(!is_identified || !homography || !ident) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
    
   if(id > ident->_database->_targets->used) 
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if(ident->_database->_targets->data[id]->posefound)
   {
      *is_identified = 1;
      
      error = rox_matsl3_new(&temp1);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_matsl3_new(&temp2);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Copy pose
      error = rox_matsl3_copy(temp1, ident->_database->_targets->data[id]->homography);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Scale pose fixing sizex
      Rox_Double rows = (Rox_Double) ident->_database->_targets->data[id]->height_pixels;

      error = rox_matsl3_get_data_pointer_to_pointer(&data, temp2);
      ROX_ERROR_CHECK_TERMINATE ( error );
     
      Rox_Double scale = pow((rows*rows)/(sizev*sizev), 1.0/3.0);
      data[0][0] = (rows/sizev)/scale;
      data[1][1] = (rows/sizev)/scale;
      data[2][2] = 1.0/scale;

      error = rox_matsl3_mulmatmat(homography, temp1, temp2);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      *is_identified = 0;
   }

function_terminate:

   rox_matsl3_del(&temp1);
   rox_matsl3_del(&temp2);

   return error;
}

Rox_ErrorCode rox_ident_database_sl3_getscore(Rox_Double * score, Rox_Ident_Database_SL3 ident, Rox_Uint id)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ident || !score) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (id > ident->_database->_targets->used) 
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if(ident->_database->_targets->data[id]->posefound)
   {
      Rox_Double val = 1;

      if (ident->_database->_targets->data[id]->best_score_minimization > 0)
      {
         val = (Rox_Double) ident->_database->_targets->data[id]->best_score_minimization;
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
