//==============================================================================
//
//    OPENROX   : File dbident_se3.c
//
//    Contents  : Implementation of dbident_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "dbident_se3.h"
#include "dbident_se3_struct.h"

#include <generated/dllist_quadtree_item_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/image/pyramid/pyramid_uchar_struct.h>
#include <baseproc/geometry/rectangle/rectangle_struct.h>

#include <core/features/detectors/segment/fastst.h>
#include <core/features/detectors/segment/fastst_score.h>
#include <core/features/descriptors/ehid/ehid.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_db_ident_se3_new(Rox_DB_Ident_SE3 * obj, Rox_Uint max_templates_simultaneous)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DB_Ident_SE3 ret = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret = (Rox_DB_Ident_SE3)rox_memory_allocate(sizeof(*ret), 1);
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

   *obj = ret;
   error = ROX_ERROR_NONE;

function_terminate:
   if (error) rox_db_ident_se3_del(&ret);

   return error;
}

Rox_ErrorCode rox_db_ident_se3_del(Rox_DB_Ident_SE3 *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DB_Ident_SE3 todel = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   todel = *obj;
   *obj = NULL;

   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

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

Rox_ErrorCode rox_db_ident_se3_extract(Rox_DB_Ident_SE3 obj, Rox_Image source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint lvlwidth, lvlheight;
   Rox_Uint idcur, idlvl;
   Rox_Double scale;
   Rox_Rect_Sint_Struct rect_search;
   int radius_deadzone = 5;
   int diameter_deadzone = radius_deadzone * 2 + 1;

   if (!obj || !source) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_ehid_point_reset(obj->_curfeats_pyr);

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Check if pyramid exists
   if(!obj->_pyramid)
   {

      error = rox_pyramid_uchar_new(&obj->_pyramid, cols, rows, 3, 2); 
      ROX_ERROR_CHECK_TERMINATE ( error ); 
   }

   if(!obj->_quad)
   {
      Rox_Rect_Sint_Struct box;
      box.x = 0; box.y = 0;
      box.width = cols; box.height = rows;


      error = rox_quadtree_ref_new(&obj->_quad, &box); 
      ROX_ERROR_CHECK_TERMINATE ( error ); 
   }

   error = rox_pyramid_uchar_assign(obj->_pyramid, source);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   for (idlvl = 0; idlvl < obj->_pyramid->nb_levels; idlvl++)
   {
      scale = pow(2.0, (double)idlvl);

      error = rox_array2d_uchar_get_size(&lvlheight, &lvlwidth, obj->_pyramid->levels[idlvl]);

      ROX_ERROR_CHECK_TERMINATE ( error ); 

      error = rox_fastst_detector(obj->_fast_points, obj->_pyramid->levels[idlvl], 20, 0); 
      ROX_ERROR_CHECK_TERMINATE ( error ); 
      
      error = rox_fastst_detector_score(obj->_fast_points, obj->_pyramid->levels[idlvl], 20);
      ROX_ERROR_CHECK_TERMINATE ( error ); 
      
      error = rox_fastst_nonmax_suppression(obj->_fast_points_nonmax, obj->_fast_points); 
      ROX_ERROR_CHECK_TERMINATE ( error ); 
      
      error = rox_fastst_detector_sort(obj->_fast_points_nonmax); 
      ROX_ERROR_CHECK_TERMINATE ( error ); 

      rox_dynvec_ehid_point_reset(obj->_curfeats);
      rox_quadtree_ref_reset(obj->_quad);

      for (idcur = 0; idcur < obj->_fast_points_nonmax->used; idcur++)
      {
         Rox_Ehid_Point_Struct curpt;

         curpt.pos.u = obj->_fast_points_nonmax->data[idcur].j;
         curpt.pos.v = obj->_fast_points_nonmax->data[idcur].i;

         rect_search.x =  (Rox_Sint) (curpt.pos.u - radius_deadzone);
         rect_search.y =  (Rox_Sint) (curpt.pos.v - radius_deadzone);
         rect_search.width = diameter_deadzone;
         rect_search.height = diameter_deadzone;

         error = rox_quadtree_ref_search(obj->_list_results, obj->_quad, &rect_search);
         if (error) continue;

         if (obj->_list_results->used != 0) continue;


         error = rox_quadtree_ref_add(obj->_quad, curpt.pos.u, curpt.pos.v, idcur);  
         ROX_ERROR_CHECK_TERMINATE ( error ); 

         // Remove points on border
         if (curpt.pos.u < 10) continue;
         if (curpt.pos.v < 10) continue;
         if (curpt.pos.u >= lvlwidth-10) continue;
         if (curpt.pos.v >= lvlheight-10) continue;


         error = rox_dynvec_ehid_point_append(obj->_curfeats, &curpt);  
         ROX_ERROR_CHECK_TERMINATE ( error ); 

         if (idlvl == 0)
         {
            if (obj->_curfeats->used >= 300) break;
         }
         else
         {
            if (obj->_curfeats->used >= 100) break;
         }
      }

      error = rox_ehid_points_compute(obj->_curfeats, obj->_pyramid->levels[idlvl]);
      if (error) continue;

      for (idcur = 0; idcur < obj->_curfeats->used; idcur++)
      {
         Rox_Ehid_Point_Struct * curpt;
         curpt = &obj->_curfeats->data[idcur];
         curpt->pos.u *= scale;
         curpt->pos.v *= scale;
      }


      error = rox_dynvec_ehid_point_stack(obj->_curfeats_pyr, obj->_curfeats);  
      ROX_ERROR_CHECK_TERMINATE ( error ); 
   }

   error = ROX_ERROR_NONE;

function_terminate:
   return error;
}

Rox_ErrorCode rox_db_ident_se3_set_database(Rox_DB_Ident_SE3 obj, Rox_Ehid_Database db)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !db) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   obj->_database = db;

function_terminate:
   return error;
}

Rox_ErrorCode rox_db_ident_se3_estimate_poses (
   Rox_DB_Ident_SE3 obj,
   const Rox_MatUT3 calib_camera )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

  
   if (!obj || !calib_camera) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!obj->_database) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_ehid_matcher_match_se3 ( obj->_matcher, obj->_database, obj->_curfeats_pyr, calib_camera );
   ROX_ERROR_CHECK_TERMINATE ( error ); 

function_terminate:
   return error;
}

Rox_ErrorCode rox_db_ident_se3_set_extracted_features(Rox_DB_Ident_SE3 obj, Rox_DynVec_Ehid_Point curfeats)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

  
   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!curfeats) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_ehid_point_reset(obj->_curfeats_pyr);

   error = rox_dynvec_ehid_point_stack(obj->_curfeats_pyr, curfeats);

   ROX_ERROR_CHECK_TERMINATE ( error ); 

function_terminate:
   return error;
}

Rox_ErrorCode rox_db_ident_se3_serialize_extracted_features(char *buffer, Rox_DB_Ident_SE3 obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

  
   if(!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if(!buffer) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_ehid_points_serialize(buffer, obj->_curfeats_pyr);   
   ROX_ERROR_CHECK_TERMINATE ( error ); 

function_terminate:
   return error;

}

Rox_ErrorCode rox_db_ident_se3_get_extracted_features_size(Rox_Uint *size, Rox_DB_Ident_SE3 obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   
   if(!size) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if(!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_ehid_points_get_octet_size(size, obj->_curfeats_pyr);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

function_terminate:
   return error;
}
