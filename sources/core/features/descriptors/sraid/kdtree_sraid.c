//==============================================================================
//
//    OPENROX   : File kdtree_sraid.c
//
//    Contents  : Implementation of kdtree_sraid module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "kdtree_sraid.h"

#include <string.h>
#include <stdio.h>

#include <generated/dynvec_sraiddesc_struct.h>
#include <generated/dynvec_uint_struct.h>

#include <baseproc/maths/random/random.h>

#include <core/features/descriptors/sraid/sraid_match.h>
#include <core/features/descriptors/sraid/sraiddesc_struct.h>

#include <inout/system/errors_print.h>

void shuffle_array_uint(Rox_Uint * array, Rox_Uint size)
{
   Rox_Uint buf;

   int pos = size - 1;
   while (pos > 0)
   {
      int id = rox_rand() % pos;
      buf = array[id];
      array[id] = array[pos];
      array[pos] = buf;

      pos--;
   }
}

Rox_ErrorCode rox_kdtree_sraid_new(Rox_Kdtree_Sraid * obj, Rox_Uint count_trees)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Kdtree_Sraid ret = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (count_trees < 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   *obj = NULL;

   ret = (Rox_Kdtree_Sraid) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->_count_leaves = 0;
   ret->_roots = NULL;
   ret->_checked = NULL;
   ret->_heap = NULL;
   ret->_count_trees = count_trees;

   error = rox_dynvec_uint_new(&ret->_checked, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->_roots = (Rox_Kdtree_Sraid_Node*) rox_memory_allocate(sizeof(Rox_Kdtree_Sraid_Node), count_trees);

   if (!ret->_roots) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   // Set all pointers to NULL
   for (Rox_Uint idtree = 0; idtree < count_trees; idtree++)
   {
      ret->_roots[idtree] = NULL;
   }

   *obj = ret;

function_terminate:
   if (error) rox_kdtree_sraid_del(&ret);
   return error;
}

Rox_ErrorCode rox_kdtree_sraid_del(Rox_Kdtree_Sraid * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Kdtree_Sraid todel = NULL;

   if (!obj) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *obj;
   if (!todel) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   *obj = NULL;

   rox_heap_branch_del(&todel->_heap);
   rox_dynvec_uint_del(&todel->_checked);

   if (todel->_roots)
   {
      rox_kdtree_sraid_clean(todel);
      rox_memory_delete(todel->_roots);
   }

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_kdtree_sraid_node_new(Rox_Kdtree_Sraid_Node * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Kdtree_Sraid_Node ret = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   *obj = NULL;

   ret = (Rox_Kdtree_Sraid_Node) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->_child_left = NULL;
   ret->_child_right = NULL;

   *obj = ret;

function_terminate:
   if (error) rox_memory_delete(ret);
   return error;
}

Rox_ErrorCode rox_kdtree_sraid_node_del(Rox_Kdtree_Sraid_Node * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Kdtree_Sraid_Node todel = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *obj;
   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   *obj = NULL;

   rox_kdtree_sraid_node_del(&todel->_child_left);
   rox_kdtree_sraid_node_del(&todel->_child_right);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_kdtree_sraid_node_split(Rox_Kdtree_Sraid_Node obj, Rox_DynVec_SRAID_Feature features, Rox_Uint * indices, Rox_Uint indices_count)
{
   Rox_Sint count_features;
   Rox_Uint id_idx, index, elem;
   Rox_Sint feat_sum[128];
   Rox_Float feat_mean[128];
   Rox_Float feat_var[128];
   Rox_Float denom, sigma;
   Rox_Sint topidx[TOP_RAND];
   Rox_Sint pos, cut_index, cut_val;
   Rox_Sint lim1, lim2, left, right, swap, halfcount;
   Rox_Sint j;

   memset(feat_sum, 0, sizeof(Rox_Sint) * 128);

   count_features = indices_count;
   if (count_features > MEAN_SAMPLES) count_features = MEAN_SAMPLES;

   // Compute sum of set of features
   for (id_idx = 0; id_idx < (Rox_Uint) count_features; id_idx++)
   {
      Rox_Ushort * data = 0;

      index = indices[id_idx];
      data = features->data[index].descriptor;

      for (elem = 0; elem < 128; elem++)
      {
         feat_sum[elem] += data[elem];
      }
   }

   // Compute mean of set of features
   denom = 1.0f / count_features;
   for (elem = 0; elem < 128; elem++)
   {
      feat_mean[elem] = denom * ((Rox_Float) feat_sum[elem]);
      feat_var[elem] = 0;
   }

   // Compute variance*/
   for (id_idx = 0; id_idx < (Rox_Uint) count_features; id_idx++)
   {
      Rox_Ushort * data;
      index = indices[id_idx];
      data = features->data[index].descriptor;

      for (elem = 0; elem < 128; elem++)
      {
         sigma = ((Rox_Float) data[elem]) - feat_mean[elem];
         feat_var[elem] += sigma * sigma;
      }
   }

   pos = 0;
   for (elem = 0; elem < 128; elem++)
   {
      if ((pos < TOP_RAND) || (feat_var[elem] > feat_var[topidx[pos - 1]]))
      {
         if (pos < TOP_RAND)
         {
            topidx[pos++] = elem;
         }
         else
         {
            topidx[pos - 1] = elem;
         }
         j = pos - 1;
         while (j > 0 && feat_var[topidx[j]] > feat_var[topidx[j - 1]])
         {
            swap = topidx[j];
            topidx[j] = topidx[j - 1];
            topidx[j - 1] = swap;
            --j;
         }
      }
   }

   // Select one random cutting dimension among the top_rand dimensions
   cut_index = topidx[rox_rand() % TOP_RAND];
   // Separation point on this dimension is the mean
   cut_val = (Rox_Sint) feat_mean[cut_index];

   // Sort indices so that they respect the subdivision cut
   left = 0;
   right = indices_count - 1;
   while (1)
   {
      while (left <= right && features->data[indices[left]].descriptor[cut_index] < cut_val)
         ++left;
      while (left <= right && features->data[indices[right]].descriptor[cut_index] >= cut_val)
         --right;

      if (left > right) break;

      swap = indices[left];
      indices[left] = indices[right];
      indices[right] = swap;
      left++;
      right--;
   }
   lim1 = left;
   right = indices_count - 1;

   while (1)
   {
      while (left <= right && features->data[indices[left]].descriptor[cut_index] <= cut_val)
         ++left;
      while (left <= right && features->data[indices[right]].descriptor[cut_index] > cut_val)
         --right;

      if (left > right) break;

      swap = indices[left];
      indices[left] = indices[right];
      indices[right] = swap;
      left++;
      right--;
   }
   lim2 = left;

   halfcount = indices_count / 2;
   if (lim1 > halfcount) index = lim1;
   else if (lim2 < halfcount) index = lim2;
   else index = halfcount;

   if (lim1 == indices_count) index = halfcount;
   else if (lim2 == 0) index = halfcount;

   obj->_index = index;
   obj->_cut_index = cut_index;
   obj->_cut_val = cut_val;

   return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_kdtree_sraid_node_new_from_list(Rox_Kdtree_Sraid_Node * obj, Rox_DynVec_SRAID_Feature features, Rox_Uint * indices, Rox_Uint indices_count)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Kdtree_Sraid_Node node;

   error = rox_kdtree_sraid_node_new(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );

   node = *obj;

   if (indices_count == 0)
   {
      {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
   }
   else if (indices_count == 1)
   {
      node->_cut_index = indices[0];
      node->_child_left = NULL;
      node->_child_right = NULL;
   }
   else
   {
      error = rox_kdtree_sraid_node_split(node, features, indices, indices_count);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_kdtree_sraid_node_new_from_list(&node->_child_left, features, indices, node->_index);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_kdtree_sraid_node_new_from_list(&node->_child_right, features, indices + node->_index, indices_count - node->_index);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_kdtree_sraid_clean(Rox_Kdtree_Sraid obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_dynvec_uint_reset(obj->_checked);
   rox_heap_branch_del(&obj->_heap);

   for (Rox_Uint idtree = 0; idtree < obj->_count_trees; idtree++)
   {
      rox_kdtree_sraid_node_del(&obj->_roots[idtree]);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_kdtree_sraid_build(Rox_Kdtree_Sraid obj, Rox_DynVec_SRAID_Feature features)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_Uint indices;

   rox_kdtree_sraid_clean(obj);

   obj->_count_leaves = features->used;

   rox_dynvec_uint_reset(obj->_checked);
   error = rox_dynvec_uint_usecells(obj->_checked, obj->_count_leaves);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_heap_branch_new(&obj->_heap, obj->_count_leaves);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_uint_new(&indices, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Uint id_feat = 0; id_feat < obj->_count_leaves; id_feat++)
   {
      rox_dynvec_uint_append(indices, &id_feat);
   }

   for (Rox_Uint id_tree = 0; id_tree < obj->_count_trees; id_tree++)
   {
      // Randomize indices of features
      shuffle_array_uint(indices->data, indices->used);

      // Build a new kd-tree
      error = rox_kdtree_sraid_node_new_from_list(&obj->_roots[id_tree], features, indices->data, indices->used);
      if (error) break;
   }

function_terminate:
   rox_dynvec_uint_del(&indices);
   return error;
}

Rox_ErrorCode rox_kdtree_sraid_search_node(Rox_SRAID_MatchResultSet results, Rox_Kdtree_Sraid_Node obj, Rox_DynVec_SRAID_Feature features, Rox_SRAID_Feature_Struct * feat, Rox_DynVec_Uint checked, Rox_Uint * checks, Rox_Uint maxchecks, Rox_Uint mindist, Rox_Heap_Branch heap)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint diff;
   Rox_Kdtree_Sraid_Node best, other;
   Rox_Uint dist;
   Rox_SRAID_Feature_Struct * featdb;

   if (obj == NULL) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Did we reach a leaf node ?*/
   if (obj->_child_left == NULL || obj->_child_right == NULL)
   {
      if (*checks >= maxchecks && rox_sraid_matchresultset_isfull(results)) {error = ROX_ERROR_NONE; goto function_terminate;}
      if (checked->data[obj->_cut_index]) {error = ROX_ERROR_NONE; goto function_terminate;}

      checked->data[obj->_cut_index] = 1;
      *checks = (*checks) + 1;

      // Compute full distance between searched feature and indexed feature*/
      featdb = &features->data[obj->_cut_index];
      dist = rox_sraid_match(featdb->descriptor, feat->descriptor);
      rox_sraid_matchresultset_addresult(results, obj->_cut_index, dist);

      error = ROX_ERROR_NONE; goto function_terminate;
   }

   // Space was divided in two : In which half should our feature lie considering only this dimension ?
   // By implementation choice, left child is the lower half.

   diff = ((Rox_Sint) feat->descriptor[obj->_cut_index]) - obj->_cut_val;
   if (diff < 0)
   {
      best = obj->_child_left;
      other = obj->_child_right;
   }
   else
   {
      best = obj->_child_right;
      other = obj->_child_left;
   }

   // Maybe considering only this dimensions guide us to a wrong subspace, keep the branch if needed in memory for more results
   dist = mindist + diff * diff;

   error = rox_heap_branch_push(heap, other, dist);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Recurse using the best children*/
   error = rox_kdtree_sraid_search_node(results, best, features, feat, checked, checks, maxchecks, mindist, heap);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_kdtree_sraid_search(Rox_SRAID_MatchResultSet results, Rox_Kdtree_Sraid obj, Rox_DynVec_SRAID_Feature features, Rox_SRAID_Feature_Struct * feat)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   const Rox_Uint max_checks = 32;
   Rox_Uint score;
   Rox_Uint checks;
   Rox_Kdtree_Sraid_Node node;

   if (!obj || !features || !feat) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_sraid_matchresultset_clear(results);
   rox_heap_branch_reset(obj->_heap);

   checks = 0;

   // Reset checked flags
   memset(obj->_checked->data, 0, sizeof(Rox_Uint) * features->used);

   // Try to find closest features for all trees
   for (Rox_Uint id_tree = 0; id_tree < obj->_count_trees; id_tree++)
   {
      error = rox_kdtree_sraid_search_node(results, obj->_roots[id_tree], features, feat, obj->_checked, &checks, max_checks, 0, obj->_heap);
      if (error) break;
   }

   // Make some new trials based on potentially erroneous branching
   error = rox_heap_branch_pop(obj->_heap, &node, &score);
   while (error == ROX_ERROR_NONE && (rox_sraid_matchresultset_isfull(results) == 0 || checks < max_checks))
   {
      error = rox_kdtree_sraid_search_node(results, node, features, feat, obj->_checked, &checks, max_checks, score, obj->_heap);
      if (error) break;

      error = rox_heap_branch_pop(obj->_heap, &node, &score);
   }

   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_kdtree_sraid_node_serialize(Rox_Kdtree_Sraid_Node node, FILE * out)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint valterm = ~0;

   if (node == NULL)
   {
      fwrite(&valterm, sizeof(int), 1, out);
      fwrite(&valterm, sizeof(int), 1, out);
      error = ROX_ERROR_NONE; goto function_terminate;
   }

   fwrite(&node->_cut_val, sizeof(int), 1, out);
   fwrite(&node->_cut_index, sizeof(int), 1, out);

   error = rox_kdtree_sraid_node_serialize(node->_child_left, out);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_kdtree_sraid_node_serialize(node->_child_right, out);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_kdtree_sraid_node_deserialize(Rox_Kdtree_Sraid_Node * node, FILE * input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint cutval, cutidx;
   size_t read;
   Rox_Kdtree_Sraid_Node creatednode;

   read = fread(&cutval, sizeof(int), 1, input);
   if (read != 1) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   read = fread(&cutidx, sizeof(int), 1, input);
   if (read != 1) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   if (cutval == ~0)
   {
      *node = NULL;
      error = ROX_ERROR_NONE; goto function_terminate;
   }

   error = rox_kdtree_sraid_node_new(&creatednode);
   ROX_ERROR_CHECK_TERMINATE ( error );

   creatednode->_cut_index = cutidx;
   creatednode->_cut_val = cutval;
   creatednode->_index = 0;

   error = rox_kdtree_sraid_node_deserialize(&creatednode->_child_left, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_kdtree_sraid_node_deserialize(&creatednode->_child_right, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *node = creatednode;

function_terminate:
   return error;
}

Rox_ErrorCode rox_kdtree_sraid_save(Rox_Kdtree_Sraid obj, char *filename)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * out = NULL;

   if (!filename || !obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   out = fopen(filename, "wb");
   if (!out) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   fwrite(&obj->_count_trees, sizeof(Rox_Uint), 1, out);
   fwrite(&obj->_count_leaves, sizeof(Rox_Uint), 1, out);

   for (Rox_Uint idtree = 0; idtree < obj->_count_trees; idtree++)
   {
      error = rox_kdtree_sraid_node_serialize(obj->_roots[idtree], out);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   if (out) fclose(out);
   return error;
}

Rox_ErrorCode rox_kdtree_sraid_load(Rox_Kdtree_Sraid obj, char *filename)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * input = NULL;
   size_t read;
   Rox_Uint nbtree;

   if (!filename || !obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_kdtree_sraid_clean(obj);

   input = fopen(filename, "rb");
   if (!input) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   // Check trees count*/
   read = fread(&nbtree, sizeof(Rox_Uint), 1, input);
   if (read != 1) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}
   if (nbtree != obj->_count_trees) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   // Count final leaves*/
   read = fread(&obj->_count_leaves, sizeof(Rox_Uint), 1, input);
   if (read != 1) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_dynvec_uint_reset(obj->_checked);
   error = rox_dynvec_uint_usecells(obj->_checked, obj->_count_leaves);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_heap_branch_new(&obj->_heap, obj->_count_leaves);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint idtree = 0; idtree < obj->_count_trees; idtree++)
   {
      error = rox_kdtree_sraid_node_deserialize(&obj->_roots[idtree], input);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   if (input) fclose(input);
   return error;
}
