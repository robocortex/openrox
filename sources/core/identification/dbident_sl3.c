//==============================================================================
//
//    OPENROX   : File dbident_sl3.c
//
//    Contents  : Implementation of dbident_sl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "dbident_sl3.h"

#include <generated/dllist_quadtree_item_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/rectangle/rectangle_struct.h>
#include <baseproc/image/pyramid/pyramid_uchar_struct.h>

#include <core/features/detectors/segment/fastst.h>
#include <core/features/detectors/segment/fastst_score.h>
#include <core/features/descriptors/ehid/ehid.h>

#include <inout/system/errors_print.h>
#include <inout/serialization/dynvec_ehid_point_serialization.h>

Rox_ErrorCode rox_db_ident_sl3_new(Rox_DB_Ident_SL3 * database_ident_sl3, Rox_Uint max_templates_simultaneous)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DB_Ident_SL3 ret = NULL;

   if (!database_ident_sl3) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret = (Rox_DB_Ident_SL3)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->_pyramid = NULL;
   ret->_fast_points = NULL;
   ret->_fast_points_nonmax = NULL;
   ret->_curfeats = NULL;
   ret->_curfeats_pyr = NULL;
   ret->_quad = NULL;
   ret->_list_results = NULL;
   ret->_database = NULL;
   ret->_matcher = NULL;

   error = rox_dynvec_segment_point_new(&ret->_fast_points, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_segment_point_new(&ret->_fast_points_nonmax, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_ehid_point_new(&ret->_curfeats, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_ehid_point_new(&ret->_curfeats_pyr, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dllist_quadtree_item_new(&ret->_list_results);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ehid_matcher_new(&ret->_matcher, max_templates_simultaneous);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *database_ident_sl3 = ret;

function_terminate:
   if (error) rox_db_ident_sl3_del(&ret);
   return error;
}

Rox_ErrorCode rox_db_ident_sl3_del(Rox_DB_Ident_SL3 * database_ident_sl3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DB_Ident_SL3 todel = NULL;


   if (!database_ident_sl3) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *database_ident_sl3;
   *database_ident_sl3 = NULL;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dllist_quadtree_item_del(&todel->_list_results);
   rox_dynvec_ehid_point_del(&todel->_curfeats);
   rox_dynvec_ehid_point_del(&todel->_curfeats_pyr);
   rox_dynvec_segment_point_del(&todel->_fast_points_nonmax);
   rox_dynvec_segment_point_del(&todel->_fast_points);
   rox_pyramid_uchar_del(&todel->_pyramid);
   rox_quadtree_ref_del(&todel->_quad);
   rox_ehid_matcher_del(&todel->_matcher);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_db_ident_sl3_extract(Rox_DB_Ident_SL3 database_ident_sl3, Rox_Image source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint lvlwidth, lvlheight;
   Rox_Uint idcur, idlvl;
   Rox_Double scale;
   Rox_Rect_Sint_Struct rect_search;

   int radius_deadzone = 5;
   int diameter_deadzone = radius_deadzone * 2 + 1;

   if (!database_ident_sl3) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_ehid_point_reset(database_ident_sl3->_curfeats_pyr);

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //  Check if pyramid exists
   if(!database_ident_sl3->_pyramid)
   {

      error = rox_pyramid_uchar_new(&database_ident_sl3->_pyramid, cols, rows, 3, 2); 
		ROX_ERROR_CHECK_TERMINATE ( error );
   }

   if(!database_ident_sl3->_quad)
   {
      Rox_Rect_Sint_Struct box;
      box.x = 0; box.y = 0;
      box.width = cols; box.height = rows;

      error = rox_quadtree_ref_new(&database_ident_sl3->_quad, &box); 
		ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = rox_pyramid_uchar_assign(database_ident_sl3->_pyramid, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (idlvl = 0; idlvl < database_ident_sl3->_pyramid->nb_levels; idlvl++)
   {
      scale = pow(2.0, (double)idlvl);

      error = rox_array2d_uchar_get_size(&lvlheight, &lvlwidth, database_ident_sl3->_pyramid->levels[idlvl]);
      ROX_ERROR_CHECK_TERMINATE ( error );


      error = rox_fastst_detector(database_ident_sl3->_fast_points, database_ident_sl3->_pyramid->levels[idlvl], 20, 0); 
		ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_fastst_detector_score(database_ident_sl3->_fast_points, database_ident_sl3->_pyramid->levels[idlvl], 20); 
		ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_fastst_nonmax_suppression(database_ident_sl3->_fast_points_nonmax, database_ident_sl3->_fast_points); 
		ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_fastst_detector_sort(database_ident_sl3->_fast_points_nonmax); 
		ROX_ERROR_CHECK_TERMINATE ( error );

      rox_dynvec_ehid_point_reset(database_ident_sl3->_curfeats);
      rox_quadtree_ref_reset(database_ident_sl3->_quad);

      for (idcur = 0; idcur < database_ident_sl3->_fast_points_nonmax->used; idcur++)
      {
         Rox_Ehid_Point_Struct curpt;

         curpt.pos.u = database_ident_sl3->_fast_points_nonmax->data[idcur].j;
         curpt.pos.v = database_ident_sl3->_fast_points_nonmax->data[idcur].i;

         rect_search.x = (Rox_Sint) (curpt.pos.u - radius_deadzone);
         rect_search.y = (Rox_Sint) (curpt.pos.v - radius_deadzone);
         rect_search.width = diameter_deadzone;
         rect_search.height = diameter_deadzone;

         error = rox_quadtree_ref_search(database_ident_sl3->_list_results, database_ident_sl3->_quad, &rect_search);
         if (error) continue;

         if (database_ident_sl3->_list_results->used != 0) continue;


         error = rox_quadtree_ref_add(database_ident_sl3->_quad, curpt.pos.u, curpt.pos.v, idcur); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Remove points on border
         if (curpt.pos.u < 10) continue;
         if (curpt.pos.v < 10) continue;
         if (curpt.pos.u >= lvlwidth-10) continue;
         if (curpt.pos.v >= lvlheight-10) continue;

         error = rox_dynvec_ehid_point_append(database_ident_sl3->_curfeats, &curpt); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         if (idlvl == 0)
         {
            if (database_ident_sl3->_curfeats->used >= 300) break;
         }
         else
         {
            if (database_ident_sl3->_curfeats->used >= 100) break;
         }
      }

      error = rox_ehid_points_compute(database_ident_sl3->_curfeats, database_ident_sl3->_pyramid->levels[idlvl]);
      if (error) continue;

      for (idcur = 0; idcur < database_ident_sl3->_curfeats->used; idcur++)
      {
         Rox_Ehid_Point_Struct * curpt;
         curpt = &database_ident_sl3->_curfeats->data[idcur];
         curpt->pos.u *= scale;
         curpt->pos.v *= scale;
      }

      error = rox_dynvec_ehid_point_stack(database_ident_sl3->_curfeats_pyr, database_ident_sl3->_curfeats); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // rox_dynvec_ehid_point_print(database_ident_sl3->_curfeats_pyr);

function_terminate:
   return error;
}

Rox_ErrorCode rox_db_ident_sl3_set_database(Rox_DB_Ident_SL3 database_ident_sl3, Rox_Ehid_Database db)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!database_ident_sl3 || !db) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   database_ident_sl3->_database = db;

function_terminate:
   return error;
}

Rox_ErrorCode rox_db_ident_sl3_estimate_homographies(Rox_DB_Ident_SL3 database_ident_sl3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!database_ident_sl3->_database) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_ehid_matcher_match_sl3(database_ident_sl3->_matcher, database_ident_sl3->_database, database_ident_sl3->_curfeats_pyr);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_db_ident_sl3_set_extracted_features(Rox_DB_Ident_SL3 database_ident_sl3, Rox_DynVec_Ehid_Point curfeats)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!database_ident_sl3) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if(!curfeats) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_ehid_point_reset(database_ident_sl3->_curfeats_pyr);

   error = rox_dynvec_ehid_point_stack(database_ident_sl3->_curfeats_pyr, curfeats);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_db_ident_sl3_serialize_extracted_features(char * buffer, Rox_DB_Ident_SL3 database_ident_sl3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if(!database_ident_sl3) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if(!buffer) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_ehid_points_serialize(buffer, database_ident_sl3->_curfeats_pyr);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_db_ident_sl3_get_extracted_features_size(Rox_Uint * size, Rox_DB_Ident_SL3 database_ident_sl3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!size) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if(!database_ident_sl3) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_ehid_points_get_octet_size(size, database_ident_sl3->_curfeats_pyr);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
