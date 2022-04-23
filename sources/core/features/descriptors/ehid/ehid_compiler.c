//==============================================================================
//
//    OPENROX   : File ehid_compiler.c
//
//    Contents  : Implementation of ehid_compiler module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ehid_compiler.h"
#include "ehid_compiler_struct.h"
#include "ehid_database_struct.h"
#include "ehid_target_struct.h"

#include <generated/dynvec_ehid_point_struct.h>
#include <generated/dynvec_ehid_dbindex_struct.h>
#include <generated/dynvec_ehid_dbindex.h>

#include <system/errors/errors.h>

#include <baseproc/geometry/transforms/transform_tools.h>

#include <core/features/descriptors/ehid/ehid_target.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_ehid_compiler_new(Rox_Ehid_Compiler * ehid_compiler)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ehid_Compiler ret = NULL;


   if (!ehid_compiler) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *ehid_compiler = NULL;

   ret = (Rox_Ehid_Compiler) rox_memory_allocate(sizeof(struct Rox_Ehid_Compiler_Struct), 1);

   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->_pwidths = NULL;
   ret->_mwidths = NULL;
   ret->_pheights = NULL;
   ret->_mheights = NULL;
   ret->flatlist = NULL;
   ret->count_targets = 0;

   for ( Rox_Sint ididx = 0; ididx < INDEX_MAX_VAL; ididx++)
   {
      ret->indexed_lists[ididx] = NULL;
      ret->indexed_dbs[ididx] = NULL;
   }

   error = rox_dynvec_double_new(&ret->_pwidths, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_double_new(&ret->_mwidths, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_double_new(&ret->_pheights, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_double_new(&ret->_mheights, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_ehid_point_new(&ret->flatlist, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint ididx = 0; ididx < INDEX_MAX_VAL; ididx++)
   {
      error = rox_dynvec_ehid_point_new(&ret->indexed_lists[ididx], 100);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_ehid_searchtree_new(&ret->indexed_dbs[ididx]);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   *ehid_compiler = ret;

function_terminate:
   if (error) rox_ehid_compiler_del(&ret);
   return error;
}

Rox_ErrorCode rox_ehid_compiler_del(Rox_Ehid_Compiler * ehid_compiler)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ehid_Compiler todel = NULL;


   if (!ehid_compiler) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *ehid_compiler;
   *ehid_compiler = NULL;


   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Sint ididx = 0; ididx < INDEX_MAX_VAL; ididx++)
   {
      rox_dynvec_ehid_point_del(&todel->indexed_lists[ididx]);
      rox_ehid_searchtree_del(&todel->indexed_dbs[ididx]);
   }

   rox_dynvec_double_del(&todel->_pwidths);
   rox_dynvec_double_del(&todel->_mwidths);
   rox_dynvec_double_del(&todel->_pheights);
   rox_dynvec_double_del(&todel->_mheights);

   rox_dynvec_ehid_point_del(&todel->flatlist);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_compiler_removealldbs(Rox_Ehid_Compiler ehid_compiler)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!ehid_compiler) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ehid_compiler->count_targets = 0;
   rox_dynvec_ehid_point_reset(ehid_compiler->flatlist);

   for ( Rox_Sint iddb = 0; iddb < INDEX_MAX_VAL; iddb++)
   {
      rox_dynvec_ehid_point_reset(ehid_compiler->indexed_lists[iddb]);
      rox_ehid_searchtree_reset(ehid_compiler->indexed_dbs[iddb]);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_compiler_add_db(Rox_Ehid_Compiler ehid_compiler, Rox_DynVec_Ehid_Point pointslist, Rox_DynVec_Ehid_DbIndex indices, Rox_Double pixel_width, Rox_Double pixel_height, Rox_Double meter_width, Rox_Double meter_height)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double calib = NULL;

   if (!ehid_compiler || !pointslist | !indices)
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   if (pointslist->used != indices->used)
   {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_new(&calib, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_transformtools_build_calibration_matrix_for_template(calib, pixel_width, pixel_height, meter_width, meter_height);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dk = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dk, calib);

   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ipx = 1.0 / dk[0][0];
   Rox_Double ipy = 1.0 / dk[1][1];
   Rox_Double iu0 = - dk[0][2] * ipx;
   Rox_Double iv0 = - dk[1][2] * ipy;

   rox_ehid_points_set_target(pointslist, ehid_compiler->count_targets);

   for ( Rox_Uint idpt = 0; idpt < pointslist->used; idpt++)
   {
      // Set global id for this point in the global db
      pointslist->data[idpt].uid = ehid_compiler->flatlist->used + idpt;

      // Compute meter coordinate
      pointslist->data[idpt].pos_meters.u = ipx * pointslist->data[idpt].pos.u + iu0;
      pointslist->data[idpt].pos_meters.v = ipy * pointslist->data[idpt].pos.v + iv0;

      // Append feature to indexed db
      for ( Rox_Sint iddb = 0; iddb < INDEX_MAX_VAL; iddb++)
      {
         if (indices->data[idpt].flag_indices[iddb] == 0) continue;
         rox_dynvec_ehid_point_append(ehid_compiler->indexed_lists[iddb], &pointslist->data[idpt]);
      }
   }

   // Appent point list to the global list
   rox_dynvec_ehid_point_stack(ehid_compiler->flatlist, pointslist);
   rox_dynvec_double_append(ehid_compiler->_pwidths, &pixel_width);
   rox_dynvec_double_append(ehid_compiler->_mwidths, &meter_width);
   rox_dynvec_double_append(ehid_compiler->_pheights, &pixel_height);
   rox_dynvec_double_append(ehid_compiler->_mheights, &meter_height);

   ehid_compiler->count_targets++;

function_terminate:
   rox_array2d_double_del(&calib);
   return error;
}

Rox_ErrorCode rox_ehid_compiler_compile(Rox_Ehid_Database db, Rox_Ehid_Compiler ehid_compiler)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ehid_Target target = NULL;


   if (!ehid_compiler || !db) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_ehid_database_reset(db);

   rox_dynvec_ehid_point_clone(db->_fulllist, ehid_compiler->flatlist);

   for ( Rox_Sint iddb = 0; iddb < INDEX_MAX_VAL; iddb++)
   {
      error = rox_ehid_searchtree_compile(db->_trees[iddb], ehid_compiler->indexed_lists[iddb]);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   for (Rox_Uint idtarget = 0; idtarget < ehid_compiler->count_targets; idtarget++)
   {
      error = rox_ehid_target_new(&target);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_transformtools_build_calibration_matrix_for_template(target->calib_input, ehid_compiler->_pwidths->data[idtarget], ehid_compiler->_pheights->data[idtarget], ehid_compiler->_mwidths->data[idtarget], ehid_compiler->_mheights->data[idtarget]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Store sizes
      target->width_meters   = ehid_compiler->_mwidths->data[idtarget];
      target->height_meters  = ehid_compiler->_mheights->data[idtarget];
      target->width_pixels   = ehid_compiler->_pwidths->data[idtarget];
      target->height_pixels  = ehid_compiler->_pheights->data[idtarget];

      rox_objset_ehid_target_append(db->_targets, target);
   }

function_terminate:
   return error;
}
