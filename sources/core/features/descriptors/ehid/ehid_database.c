//==============================================================================
//
//    OPENROX   : File ehid_database.c
//
//    Contents  : Implementation of ehid_database module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ehid_database.h"
#include "ehid_database_struct.h"

#include <stdio.h>
#include <string.h>

#include <generated/dynvec_segment_point_struct.h>
#include <generated/dynvec_ehid_point_struct.h>
#include <generated/dynvec_point3d_double_struct.h>
#include <generated/dllist_quadtree_item_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/rectangle/rectangle_struct.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
#include <baseproc/image/remap/remap_bilinear_omo_uchar_to_uchar/remap_bilinear_omo_uchar_to_uchar.h>
#include <baseproc/array/crosscor/zncrosscor.h>
#include <baseproc/geometry/point/point3d_sphere.h>
#include <baseproc/image/pyramid/pyramid_uchar_struct.h>
#include <baseproc/image/pyramid/pyramid_uchar.h>

#include <core/features/descriptors/ehid/ehid.h>
#include <core/features/descriptors/ehid/ehid_target.h>
#include <core/features/detectors/segment/fastst.h>
#include <core/features/detectors/segment/fastst_score.h>
#include <core/occupancy/quadtree_ref.h>
#include <core/features/descriptors/ehid/ehid_matcher.h>
#include <core/virtualview/planar_view_generator.h>
#include <core/features/descriptors/ehid/ehid_viewpointbin.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>
#include <inout/image/pgm/pgmfile.h>
#include <inout/serialization/dynvec_ehid_point_serialization.h>

Rox_ErrorCode rox_ehid_database_new(Rox_Ehid_Database * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ehid_Database ret = NULL;

   if (!obj)

   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Ehid_Database)rox_memory_allocate(sizeof(struct Rox_Ehid_Database_Struct), 1);
   if (!ret)

   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Sint ididx = 0; ididx < INDEX_MAX_VAL; ididx++)
   {
      ret->_trees[ididx] = 0;
   }

   ret->_fulllist = NULL;
   ret->_targets = NULL;

   for (Rox_Sint ididx = 0; ididx < INDEX_MAX_VAL; ididx++)
   {
      error = rox_ehid_searchtree_new(&ret->_trees[ididx]);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = rox_objset_ehid_target_new(&ret->_targets, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_ehid_point_new(&ret->_fulllist, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Assign ret to obj
   *obj = ret;

function_terminate:
   if (error) rox_ehid_database_del(&ret);
   return error;
}

Rox_ErrorCode rox_ehid_database_del(Rox_Ehid_Database * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ehid_Database todel = NULL;

   if (!obj) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error) }

   todel = *obj;
   *obj = NULL;

   if (!todel) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error) }

   for (Rox_Sint ididx = 0; ididx < INDEX_MAX_VAL; ididx++)
   {
      rox_ehid_searchtree_del(&todel->_trees[ididx]);
   }

   rox_objset_ehid_target_del(&todel->_targets);
   rox_dynvec_ehid_point_del(&todel->_fulllist);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_database_reset(Rox_Ehid_Database obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error) }

   for (Rox_Sint ididx = 0; ididx < INDEX_MAX_VAL; ididx++)
   {
      rox_ehid_searchtree_reset(obj->_trees[ididx]);
   }

   rox_dynvec_ehid_point_reset(obj->_fulllist);
   rox_objset_ehid_target_reset(obj->_targets);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_database_save(Rox_Ehid_Database db, const Rox_Char * filename)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * file = NULL;

   file = fopen(filename, "wb");

   if (!file)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   fwrite(&db->_targets->used, sizeof(Rox_Uint), 1, file);

   for (Rox_Uint idtgt = 0; idtgt < db->_targets->used; idtgt++)
   {
      Rox_Double ** dt = NULL;
      error = rox_array2d_double_get_data_pointer_to_pointer(&dt, db->_targets->data[idtgt]->calib_input);
      ROX_ERROR_CHECK_TERMINATE ( error );

      fwrite(&db->_targets->data[idtgt]->width_pixels, sizeof(Rox_Double), 1, file);
      fwrite(&db->_targets->data[idtgt]->height_pixels, sizeof(Rox_Double), 1, file);
      fwrite(&db->_targets->data[idtgt]->width_meters, sizeof(Rox_Double), 1, file);
      fwrite(&db->_targets->data[idtgt]->height_meters, sizeof(Rox_Double), 1, file);

      fwrite(dt[0], sizeof(Rox_Double), 3, file);
      fwrite(dt[1], sizeof(Rox_Double), 3, file);
      fwrite(dt[2], sizeof(Rox_Double), 3, file);
   }

   error = rox_ehid_points_save_stream(file, db->_fulllist);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint ididx = 0; ididx < INDEX_MAX_VAL; ididx++)
   {
      error = rox_ehid_searchtree_save_tree(db->_trees[ididx], file);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   if (file) fclose(file);
   return error;
}

Rox_ErrorCode rox_ehid_database_load(Rox_Ehid_Database db, const Rox_Char * filename)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * file = NULL;
   Rox_Uint nbtargets = 0;
   Rox_Ehid_Target target = NULL;

   if (!db || !filename)

   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_ehid_database_reset(db);

   file = fopen(filename, "rb");
   if (!file)

   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint read = (Rox_Uint)fread(&nbtargets, sizeof(Rox_Uint), 1, file);
   if (read != 1)

   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint idtgt = 0; idtgt < nbtargets; idtgt++)
   {
      error = rox_ehid_target_new(&target);
      ROX_ERROR_CHECK_TERMINATE ( error );

      Rox_Double ** dt = NULL;
      error = rox_array2d_double_get_data_pointer_to_pointer(&dt, target->calib_input);


      read = (Rox_Uint) fread(&target->width_pixels, sizeof(Rox_Double), 1, file);
      if (read != 1) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error );}

      read = (Rox_Uint) fread(&target->height_pixels, sizeof(Rox_Double), 1, file);
      if (read != 1) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error );}

      read = (Rox_Uint) fread(&target->width_meters, sizeof(Rox_Double), 1, file);
      if (read != 1) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error );}

      read = (Rox_Uint) fread(&target->height_meters, sizeof(Rox_Double), 1, file);
      if (read != 1) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error );}

      read = (Rox_Uint) fread(dt[0], sizeof(Rox_Double), 3, file);
      if (read != 3) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error );}

      read = (Rox_Uint) fread(dt[1], sizeof(Rox_Double), 3, file);
      if (read != 3) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error );}

      read = (Rox_Uint) fread(dt[2], sizeof(Rox_Double), 3, file);
      if (read != 3) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error );}

      rox_objset_ehid_target_append(db->_targets, target);
   }

   error = rox_ehid_points_load_stream(db->_fulllist, file);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint ididx = 0; ididx < INDEX_MAX_VAL; ididx++)
   {
      error = rox_ehid_searchtree_load_tree(db->_trees[ididx], file);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   if (file) fclose(file);
   return error;
}

Rox_ErrorCode rox_ehid_database_test_current_view(Rox_Image view, Rox_Image original, Rox_Ehid_Database db)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint cols, rows, ocols, orows, lvlcols, lvlrows, found;
   Rox_Uint idcur, idlvl;
   Rox_Double scale = 0.0, zncc = 0.0;

   Rox_Pyramid_Uchar pyr = NULL;
   Rox_DynVec_Segment_Point fast_points = NULL, fast_points_nonmax = NULL;
   Rox_DynVec_Ehid_Point curfeats = NULL, curfeats_pyr = NULL;
   Rox_QuadTree_Ref quad = NULL;
   Rox_Dllist_QuadTree_Item list_result = NULL;
   Rox_Array2D_Double calib_output = NULL;
   Rox_Ehid_Matcher matcher = NULL;
   Rox_Image imcompare = NULL;
   Rox_Imask imcomparemask = NULL;
   Rox_MeshGrid2D_Float imcomparemap = NULL;
   Rox_Rect_Sint_Struct box, rect_search;

   int radius_deadzone = 5;
   int diameter_deadzone = radius_deadzone * 2 + 1;

   error = rox_array2d_uchar_get_size(&rows, &cols, view);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_get_size(&orows, &ocols, original);
   ROX_ERROR_CHECK_TERMINATE ( error );

   box.x = 0;
   box.y = 0;
   box.width = cols;
   box.height = rows;

   error = rox_array2d_uchar_new(&imcompare, orows, ocols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new(&imcomparemask, orows, ocols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_new(&imcomparemap, orows, ocols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&calib_output, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_pyramid_uchar_new(&pyr, cols, rows, 3, 2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_pyramid_uchar_assign(pyr, view);
   //error = rox_pyramid_uchar_assign(pyr, original);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_quadtree_ref_new(&quad, &box);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Init variables : equivalent of function rox_db_ident_sl3_new in core/identificartion/dbident_sl3.c

   error = rox_dynvec_segment_point_new(&fast_points, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_segment_point_new(&fast_points_nonmax, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_ehid_point_new(&curfeats, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_ehid_point_new(&curfeats_pyr, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dllist_quadtree_item_new(&list_result);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ehid_matcher_new(&matcher, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Extract features : equivalent of function rox_db_ident_sl3_extract in core/identificartion/dbident_sl3.c

   for (idlvl = 0; idlvl < pyr->nb_levels; idlvl++) // idlvl < pyr->nb_levels
   {
      scale = pow(2.0, (double)idlvl);


      error  = rox_array2d_uchar_get_size(&lvlrows, &lvlcols, pyr->levels[idlvl]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_fastst_detector(fast_points, pyr->levels[idlvl], 20, 0);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_fastst_detector_score(fast_points, pyr->levels[idlvl], 20);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_fastst_nonmax_suppression(fast_points_nonmax, fast_points);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_fastst_detector_sort(fast_points_nonmax);
      ROX_ERROR_CHECK_TERMINATE ( error );

      rox_dynvec_ehid_point_reset(curfeats);
      rox_quadtree_ref_reset(quad);

      for (idcur = 0; idcur < fast_points_nonmax->used; idcur++)
      {
         Rox_Ehid_Point_Struct curpt;

         curpt.pos.u = fast_points_nonmax->data[idcur].j;
         curpt.pos.v = fast_points_nonmax->data[idcur].i;

         rect_search.x = (Rox_Sint)(curpt.pos.u - radius_deadzone);
         rect_search.y = (Rox_Sint)(curpt.pos.v - radius_deadzone);
         rect_search.width = diameter_deadzone;
         rect_search.height = diameter_deadzone;

         error = rox_quadtree_ref_search(list_result, quad, &rect_search);
         if (error) continue;

         if (list_result->used != 0) continue;

         error = rox_quadtree_ref_add(quad, curpt.pos.u, curpt.pos.v, idcur);
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Remove points on border
         if (curpt.pos.u < 10) continue;
         if (curpt.pos.v < 10) continue;
         if (curpt.pos.u >= lvlcols - 10) continue;
         if (curpt.pos.v >= lvlrows - 10) continue;

         error = rox_dynvec_ehid_point_append(curfeats, &curpt);
         ROX_ERROR_CHECK_TERMINATE ( error );

         if (idlvl == 0)
         {
            if (curfeats->used >= 300) break;
         }
         else
         {
            if (curfeats->used >= 100) break;
         }
      }

      error = rox_ehid_points_compute(curfeats, pyr->levels[idlvl]);
      if (error) continue;

      for (idcur = 0; idcur < curfeats->used; idcur++)
      {
         Rox_Ehid_Point  curpt;
         curpt = &curfeats->data[idcur];
         curpt->pos.u *= scale;
         curpt->pos.v *= scale;
      }

      error = rox_dynvec_ehid_point_stack(curfeats_pyr, curfeats);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = rox_ehid_matcher_match_sl3(matcher, db, curfeats_pyr);
   ROX_ERROR_CHECK_TERMINATE ( error );

   found = 0;
   for (Rox_Uint idtgt = 0; idtgt < db->_targets->used; idtgt++)
   {
      if (db->_targets->data[idtgt]->posefound)
      {
         // found = 1;
         Rox_Array2D_Double H = db->_targets->data[idtgt]->best_homography;

         error = rox_warp_grid_sl3_float(imcomparemap, H);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_remap_bilinear_omo_uchar_to_uchar(imcompare, imcomparemask, view, imcomparemap);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_uchar_zncc(&zncc, imcompare, original, imcomparemask);
         ROX_ERROR_CHECK_TERMINATE ( error );

         if (zncc > 0.9)
         {
            found = 1;
            break;
         }
      }
   }

   if (!found)

   { error = ROX_ERROR_TEMPLATE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

function_terminate:
   rox_array2d_double_del(&calib_output);
   rox_pyramid_uchar_del(&pyr);
   rox_dynvec_segment_point_del(&fast_points);
   rox_dynvec_segment_point_del(&fast_points_nonmax);
   rox_quadtree_ref_del(&quad);
   rox_dynvec_ehid_point_del(&curfeats);
   rox_dynvec_ehid_point_del(&curfeats_pyr);
   rox_dllist_quadtree_item_del(&list_result);
   rox_ehid_matcher_del(&matcher);
   rox_array2d_uchar_del(&imcompare);
   rox_array2d_uint_del(&imcomparemask);
   rox_meshgrid2d_float_del(&imcomparemap);

   return error;
}

Rox_ErrorCode rox_ehid_database_test(Rox_Double * score, Rox_Ehid_Database db, Rox_Image source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_Point3D_Double vector_origins = NULL;
   Rox_ViewGenerator_Planar generator = NULL;

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_new_from_sphere(&vector_origins, 4, 40);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_viewgenerator_planar_new(&generator);
   ROX_ERROR_CHECK_TERMINATE ( error );

   rox_viewgenerator_planar_set_source_size(generator, cols, rows);

   Rox_Uint countfound = 0;
   for (Rox_Uint idview = 0; idview < vector_origins->used; idview++)
      // for ( Rox_Sint idview = 0; idview < 1; idview++)
   {
      Rox_Point3D_Double_Struct vec;
      Rox_Double inplane_rot, scale;

      vec = vector_origins->data[idview];
      vec.X = 0;
      vec.Y = 0;
      vec.Z = 1.0;
      inplane_rot = 360.0 * (double)rox_rand() / (double)ROX_RAND_MAX;
      scale = 0.8 + 1.0 * (double)rox_rand() / (double)ROX_RAND_MAX;

      error = rox_viewgenerator_planar_generate(generator, source, scale, inplane_rot, &vec);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_ehid_database_test_current_view(generator->generated, source, db);
      if (error)
      {
         continue;
      }

      countfound++;
   }

   *score = (Rox_Double)countfound / (Rox_Double)vector_origins->used;

   // Ignore previous non fatal errors
   error = ROX_ERROR_NONE;

function_terminate:
   rox_viewgenerator_planar_del(&generator);
   rox_dynvec_point3d_double_del(&vector_origins);

   return error;
}

Rox_ErrorCode rox_ehid_database_testfast(Rox_Double * pscore, Rox_Image source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint cols = 0, rows = 0, scols = 0, srows = 0;
   Rox_Sint count_full = 0;
   Rox_Sint nbblocs = 0;
   Rox_Double score = 0.0;

   Rox_Ehid_ViewpointBin vpbin = NULL;
   Rox_Pyramid_Uchar pyr = NULL;

   if (!pscore || !source)

   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_get_size(&rows, &cols, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_pyramid_uchar_new(&pyr, cols, rows, 3, 2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_pyramid_uchar_assign(pyr, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   score = 0.0;
   for (Rox_Sint idlvl = pyr->nb_levels - 1; idlvl >= 0; idlvl--)
   {
      Rox_Image lsrc = pyr->levels[idlvl];

      error = rox_array2d_uchar_get_size(&srows, &scols, lsrc);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_ehid_viewpointbin_new(&vpbin, scols, srows, 1.0, 1.0, 0, 40.0, 4.0);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_ehid_viewpointbin_test(&count_full, &nbblocs, vpbin, lsrc);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      rox_ehid_viewpointbin_del(&vpbin);

      if (idlvl == pyr->nb_levels - 1)
      {
         if (count_full < nbblocs
             - 1)
         {
            score = 0.0;
            break;
         }
      }
      else
      {
         score += 0.5 * ((double)count_full) / ((double)nbblocs);
      }
   }

   *pscore = score;

   error = ROX_ERROR_NONE;

function_terminate:
   rox_pyramid_uchar_del(&pyr);

   return error;
}

Rox_ErrorCode rox_ehid_database_serialize(char * ser, const Rox_Ehid_Database db)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint offset = 0;
   Rox_Uint size = 0;

   if (!ser || !db) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error) }

   memcpy(ser + offset, &db->_targets->used, sizeof(db->_targets->used));
   offset += sizeof(db->_targets->used);

   for (Rox_Uint idtgt = 0; idtgt < db->_targets->used; idtgt++)
   {
      Rox_Double ** dt = NULL;
      error = rox_array2d_double_get_data_pointer_to_pointer(&dt, db->_targets->data[idtgt]->calib_input);

      memcpy(ser + offset, &db->_targets->data[idtgt]->width_pixels, sizeof(db->_targets->data[idtgt]->width_pixels));
      offset += sizeof(db->_targets->data[idtgt]->width_pixels);

      memcpy(ser + offset, &db->_targets->data[idtgt]->height_pixels, sizeof(db->_targets->data[idtgt]->height_pixels));
      offset += sizeof(db->_targets->data[idtgt]->height_pixels);

      memcpy(ser + offset, &db->_targets->data[idtgt]->width_meters, sizeof(db->_targets->data[idtgt]->width_meters));
      offset += sizeof(db->_targets->data[idtgt]->width_meters);

      memcpy(ser + offset, &db->_targets->data[idtgt]->height_meters, sizeof(db->_targets->data[idtgt]->height_meters));
      offset += sizeof(db->_targets->data[idtgt]->height_meters);

      memcpy(ser + offset, dt[0], 3 * sizeof(*dt[0]));
      offset += 3 * sizeof(*dt[0]);

      memcpy(ser + offset, dt[1], 3 * sizeof(*dt[1]));
      offset += 3 * sizeof(*dt[1]);

      memcpy(ser + offset, dt[2], 3 * sizeof(*dt[2]));
      offset += 3 * sizeof(*dt[2]);
   }

   error = rox_ehid_points_serialize(ser + offset, db->_fulllist);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get points size
   error = rox_ehid_points_get_octet_size(&size, db->_fulllist);
   ROX_ERROR_CHECK_TERMINATE ( error );
   offset += size;

   // Save trees
   for (Rox_Sint ididx = 0; ididx < INDEX_MAX_VAL; ididx++)
   {
      error = rox_ehid_searchtree_serialize(ser + offset, db->_trees[ididx]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_ehid_searchtree_get_octet_size(&size, db->_trees[ididx]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      offset += size;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_database_deserialize(Rox_Ehid_Database db, const char* ser)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint offset = 0, size = 0;
   Rox_Uint nbtargets;
   Rox_Ehid_Target target;
   Rox_Uint idtgt, ididx;


   if(db == 0 || ser == 0)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   memcpy(&nbtargets, ser + offset, sizeof(nbtargets));
   offset += sizeof(nbtargets);

   for (idtgt = 0; idtgt < nbtargets; idtgt++)
   {
      Rox_Double **dt;
      error = rox_ehid_target_new(&target);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_data_pointer_to_pointer(&dt, target->calib_input);
      ROX_ERROR_CHECK_TERMINATE ( error );

      memcpy(&target->width_pixels, ser + offset, sizeof(target->width_pixels));
      offset += sizeof(target->width_pixels);
      memcpy(&target->height_pixels, ser + offset, sizeof(target->height_pixels));
      offset += sizeof(target->height_pixels);
      memcpy(&target->width_meters, ser + offset, sizeof(target->width_meters));
      offset += sizeof(target->width_meters);
      memcpy(&target->height_meters, ser + offset, sizeof(target->height_meters));
      offset += sizeof(target->height_meters);

      memcpy(dt[0], ser + offset, 3 * sizeof(*dt[0]));
      offset += 3 * sizeof(*dt[0]);
      memcpy(dt[1], ser + offset, 3 * sizeof(*dt[1]));
      offset += 3 * sizeof(*dt[1]);
      memcpy(dt[2], ser + offset, 3 * sizeof(*dt[2]));
      offset += 3 * sizeof(*dt[2]);


      rox_objset_ehid_target_append(db->_targets, target);
   }

   error = rox_ehid_points_deserialize(db->_fulllist, ser + offset);
   ROX_ERROR_CHECK_TERMINATE(error)

      // Get points size
      error = rox_ehid_points_get_octet_size(&size, db->_fulllist);
   ROX_ERROR_CHECK_TERMINATE(error)
      offset += size;

   // Load trees
   for (ididx = 0; ididx < INDEX_MAX_VAL; ididx++)
   {
      error = rox_ehid_searchtree_deserialize(db->_trees[ididx], ser + offset);
      ROX_ERROR_CHECK_TERMINATE(error)

         error = rox_ehid_searchtree_get_octet_size(&size, db->_trees[ididx]);
      ROX_ERROR_CHECK_TERMINATE(error)

         offset += size;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_database_get_octet_size(Rox_Uint* size, const Rox_Ehid_Database db)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint ret = 0, struct_size = 0;

   if (!size || !db) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error) }

   ret += sizeof(db->_targets->used);
   ret += db->_targets->used * 9 * sizeof(Rox_Double); // calib_input
   ret += db->_targets->used * 4 * sizeof(Rox_Double); // Target size (pixel + meters)

   // Get points size
   error = rox_ehid_points_get_octet_size(&struct_size, db->_fulllist);
   ROX_ERROR_CHECK_TERMINATE ( error );
   ret += struct_size;

   // get tree size
   for (Rox_Sint ididx = 0; ididx < INDEX_MAX_VAL; ididx++)
   {
      error = rox_ehid_searchtree_get_octet_size(&struct_size, db->_trees[ididx]);
      ROX_ERROR_CHECK_TERMINATE ( error );
      ret += struct_size;
   }

   *size = ret;

function_terminate:
   return error;
}
