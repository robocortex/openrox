//==============================================================================
//
//    OPENROX   : File sraiddesc_kmean.c
//
//    Contents  : Implementation of sraiddesc_kmean module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "sraiddesc_kmean.h"
#include "sraiddesc_kmean_struct.h"

#include <time.h>
#include <limits.h>
#include <string.h>

#include <generated/dynvec_sraiddesc_struct.h>
#include <generated/dynvec_sint_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/random/random.h>

#include <core/features/descriptors/sraid/sraid_match.h>

#include <inout/system/errors_print.h>

//! To be commented
struct ResultDual_Struct
{
   //! To be commented
   Rox_Uint id1;
   //! To be commented
   Rox_Uint dist1;
   //! To be commented
   Rox_Uint id2;
   //! To be commented
   Rox_Uint dist2;
    //! To be commented
   Rox_Uint count;
    //! To be commented
   Rox_Uint worst;
};

   //! To be commented
typedef struct ResultDual_Struct ResultDual_Struct;

   //! To be commented
struct NodeDist_Struct
{
   //! To be commented
   Rox_Uint id;
   //! To be commented
   Rox_Uint dist;
};

  //! To be commented
typedef struct NodeDist_Struct NodeDist_Struct;

int compareIndices(const int * a,const int * b)
{
   // Check order of indices by increasing order
   if (*a==*b) return 0;
   else
   {
      if (*a < *b) return -1;
      else return 1;
   }
}

Rox_ErrorCode rox_sraid_tree_node_new(Rox_Sraid_Tree_Node * obj, Rox_Uint nodeid)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sraid_Tree_Node ret = NULL;
   Rox_Sraid_Tree_Node aligned = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   // Create a new node (Either a branch or a leaf
   ret = (Rox_Sraid_Tree_Node)rox_memory_allocate_aligned((void**)&aligned, sizeof(struct Rox_Sraid_Tree_Node_Struct), 1, ROX_DEFAULT_ALIGNMENT);

   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Assign true poitner for deletion*
	aligned->base_pointer = ret;

	// Set no children by default
   aligned->children = NULL;

   // Set no indices pool by default
   aligned->indices_pool = NULL;

   aligned->idnode = nodeid;

   *obj = aligned;

function_terminate:
   return error;
}

Rox_ErrorCode rox_sraid_tree_node_del(Rox_Sraid_Tree_Node * obj, Rox_Sraid_Tree tree)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sraid_Tree_Node todel = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   if (todel->children)
   {
      // Delete all children
      for (Rox_Uint id = 0; id < tree->branch_factor; id++)
      {
         rox_sraid_tree_node_del(&todel->children[id], tree);
      }

      // Delele children array
      rox_memory_delete(todel->children);
   }

   // Delete object
	rox_memory_delete(todel->base_pointer);

function_terminate:
   return error;
}

Rox_ErrorCode rox_sraid_tree_new(Rox_Sraid_Tree * obj, Rox_DynVec_SRAID_Feature features, Rox_Uint branching_factor, Rox_Sint max_level)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sraid_Tree ret = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!features) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (features->used < 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Sraid_Tree)rox_memory_allocate(sizeof(struct Rox_Sraid_Tree_Struct), 1);

   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->border_factor = 0.4f;

   // *If the tree try to create more than max_level leves, block it
   ret->max_level = max_level;

   // How many clusters per level are created
   ret->branch_factor = branching_factor;

   // Iterations for clustering
   ret->max_iters = 100;

   // features list
   ret->database = features;

   // Root node
   ret->root = NULL;
   ret->indices = NULL;
   ret->centers = NULL;
   ret->random_pool = NULL;
   ret->node_count = 0;

   // Create buffer for centers
   ret->centers = (Rox_Uint*)rox_memory_allocate(sizeof(Rox_Uint), ret->branch_factor);
   if (!ret->centers)

   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Create randomization pool (to avoid repetitions)
   ret->random_pool = (Rox_Uint*)rox_memory_allocate(sizeof(Rox_Uint), ret->database->used);
   if (!ret->centers)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = ret;

function_terminate:
   return error;
}

Rox_ErrorCode rox_sraid_tree_del(Rox_Sraid_Tree * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sraid_Tree todel = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;


   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Delete all nodes in a recursive way
   if (todel->root)
   {
      rox_sraid_tree_node_del(&todel->root, todel);
   }

   // Delete buffers
   rox_memory_delete(todel->indices);
   rox_memory_delete(todel->centers);
   rox_memory_delete(todel->random_pool);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_sraid_tree_node_build_statistics(Rox_Sraid_Tree obj, Rox_Sraid_Tree_Node node, Rox_Uint * indices, Rox_Uint nb_indices)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint mean[ROX_SRAID_DESCRIPTOR_SIZE], summean;
   Rox_Sint dist, bestdist, delta;


   if (!obj || !node || !indices) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Sint cell = 0; cell < ROX_SRAID_DESCRIPTOR_SIZE; cell++) mean[cell] = 0;

   // Compute the mean of cluster children descriptors

   for (Rox_Uint id = 0; id < obj->database->used; id++)
   {
      Rox_Uint cid = indices[id];
      Rox_Ushort * desc = obj->database->data[cid].descriptor;

      for ( Rox_Sint cell = 0; cell < ROX_SRAID_DESCRIPTOR_SIZE; cell++)
      {
         mean[cell] += (int)desc[cell];
      }
   }

   summean = 0;
   for ( Rox_Sint cell = 0; cell < ROX_SRAID_DESCRIPTOR_SIZE; cell++)
   {
      mean[cell] = mean[cell] / ROX_SRAID_DESCRIPTOR_SIZE;
      node->mean[cell] = mean[cell];
      summean += mean[cell] * mean[cell];
   }

   // Compute greatest distance from cluster to children (radius of cluster)
   bestdist = 0;
   for (Rox_Uint id = 0; id < nb_indices; id++)
   {
      Rox_Uint cid = indices[id];
      Rox_Ushort * desc = obj->database->data[cid].descriptor;

      dist = 0;
      for ( Rox_Sint cell = 0; cell < ROX_SRAID_DESCRIPTOR_SIZE; cell++)
      {
         delta = desc[cell] - mean[cell];
         dist += delta * delta;
      }

      if (dist > bestdist)
      {
         bestdist = dist;
      }
   }

   node->radius = bestdist;

function_terminate:
   return error;
}

Rox_Uint rox_sraid_tree_node_norm(Rox_Ushort * d1)
{
   // Compute the norm of a descriptor
   Rox_Sint norm = 0;
   for ( Rox_Sint i = 0; i < ROX_SRAID_DESCRIPTOR_SIZE; i++)
   {
      Rox_Sint v1 = d1[i];
      norm += v1*v1;
   }

   return norm;
}

Rox_ErrorCode rox_sraid_tree_node_randomseeds(Rox_Sraid_Tree obj, Rox_Uint * indices, Rox_Uint nb_indices)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!obj || !indices) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (nb_indices < obj->branch_factor) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Fill in random pool
   for (Rox_Uint i = 0; i < nb_indices; i++) obj->random_pool[i] = i;
   Rox_Uint maxval = nb_indices;

   // Try to find branch_factor random nodes for cluster centers
   for (Rox_Uint i = 0; i < obj->branch_factor; i++)
   {
      Rox_Uint unique = 0;

      while (!unique)
      {
         unique = 1;


         if (maxval == 0) 
         { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

         // Retrieve an index
         Rox_Uint rnd_idx = rox_rand() % maxval;
         Rox_Uint index = indices[obj->random_pool[rnd_idx]];

         // "Remove" index from list, reduce list size
         obj->random_pool[rnd_idx] = obj->random_pool[maxval - 1];
         maxval--;

         // Add center
         obj->centers[i] = index;

         // Check that no other clusters are exactly similar (in case multiple features have same descriptors)
         for (Rox_Uint k = 0; k < i; k++)
         {
            Rox_Uint dist = rox_sraid_match(obj->database->data[obj->centers[k]].descriptor, obj->database->data[obj->centers[i]].descriptor);
            if (dist < 1) unique = 0;
         }
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_sraid_tree_node_cluster(Rox_Sraid_Tree obj, Rox_Sraid_Tree_Node node, Rox_Uint * indices, Rox_Uint nb_indices, Rox_Uint level)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint idx, bestdist, dist, iter, done, modified, exc, start, end;
   Rox_Ushort * vec = NULL;
   Rox_Uint * veccenter = NULL;
   Rox_Uint * assignments = NULL;


   if (!obj || !node || !indices) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   node->size = nb_indices;
   node->level = level;

   // Is it a terminal node ? If yes, we just want to keep the indices
   if (nb_indices <= obj->branch_factor || node->level >= (Rox_Uint) obj->max_level)
   {
      node->indices_pool = indices;
      node->children = NULL;
      node->idnode = obj->node_count;
      obj->node_count++;

      // Sort indices by ascending order
      qsort(node->indices_pool, nb_indices, sizeof(Rox_Uint), (int (*)(const void *, const void *)) compareIndices);

      return ROX_ERROR_NONE;
   }

   // Create seeds for centers of this node k-mean
   error = rox_sraid_tree_node_randomseeds(obj, indices, nb_indices);
   if (error)
   {
      node->indices_pool = indices;
      node->children = NULL;

      // Sort indices
      qsort(node->indices_pool, nb_indices, sizeof(Rox_Uint), (int (*)(const void *, const void *)) compareIndices);

      return ROX_ERROR_NONE;
   }

   // Create array of children
   node->children = (Rox_Sraid_Tree_Node*)rox_memory_allocate(sizeof(Rox_Sraid_Tree_Node), obj->branch_factor);

   if (node->children == NULL) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint i = 0; i < obj->branch_factor; i++)
   {
      // Initialize children node
      error = rox_sraid_tree_node_new(&node->children[i], obj->node_count);

      if (error) 
      { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

      obj->node_count++;

      // Assign choosen center
      idx = obj->centers[i];
      memcpy(node->children[i]->mean, obj->database->data[idx].descriptor, sizeof(Rox_Ushort) * ROX_SRAID_DESCRIPTOR_SIZE);

      node->children[i]->radius = 0;
      node->children[i]->size = 0;
   }

   assignments = (Rox_Uint*)rox_memory_allocate(sizeof(Rox_Uint), nb_indices);

   if (!assignments) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Compute assignments for seeds
   for (Rox_Uint i = 0; i < nb_indices; i++)
   {
      idx = indices[i];
      bestdist = rox_sraid_match(obj->database->data[idx].descriptor, node->children[0]->mean);
      assignments[i] = 0;

      // Estimate the closest center
      for (Rox_Uint k = 0; k < obj->branch_factor; k++)
      {
         dist = rox_sraid_match(obj->database->data[idx].descriptor, node->children[k]->mean);
         if (dist < bestdist)
         {
            bestdist = dist;
            assignments[i] = k;
         }
      }

      // Assign node to closest center
      node->children[assignments[i]]->size++;
      if ((Rox_Sint) bestdist > node->children[assignments[i]]->radius)
      {
         node->children[assignments[i]]->radius = bestdist;
      }
   }

   // Minimization of k-means
   iter = 0;
   done = 0;
   while (iter < obj->max_iters && !done)
   {
      done = 1;
      iter++;

      // Compute cluster centers given assignments

      // Reset stats
      for (Rox_Uint k = 0; k < obj->branch_factor; k++)
      {
         node->children[k]->radius = 0;

         for ( Rox_Sint l = 0; l < ROX_SRAID_DESCRIPTOR_SIZE; l++)
         {
            node->children[k]->acc[l] = 0;
         }
      }

      // Compute sum of cells
      for (Rox_Uint i = 0; i < nb_indices; i++)
      {
         vec = obj->database->data[indices[i]].descriptor;
         veccenter = node->children[assignments[i]]->acc;

         for ( Rox_Sint l = 0; l < ROX_SRAID_DESCRIPTOR_SIZE; l++)
         {
            veccenter[l] += vec[l];
         }
      }

      // Compute mean of cells
      for (Rox_Uint k = 0; k < obj->branch_factor; k++)
      {
         for ( Rox_Sint l = 0; l < ROX_SRAID_DESCRIPTOR_SIZE; l++)
         {
            node->children[k]->acc[l] /= node->children[k]->size;
            node->children[k]->mean[l] = node->children[k]->acc[l];
         }
      }

      // Compute assignments for new clusters
      for (Rox_Uint i = 0; i < nb_indices; i++)
      {
         idx = indices[i];
         bestdist = rox_sraid_match(obj->database->data[idx].descriptor, node->children[0]->mean);
         modified = 0;

         for (Rox_Uint k = 1; k < obj->branch_factor; k++)
         {
            dist = rox_sraid_match(obj->database->data[idx].descriptor, node->children[k]->mean);
            if (dist < bestdist)
            {
               bestdist = dist;
               modified = k;
            }
         }

         // Update radius, which is the maximal distance to the center
         if ((Rox_Sint) bestdist > node->children[modified]->radius)
         {
            node->children[modified]->radius = bestdist;
         }

         // A change occured ?
         if (modified != assignments[i])
         {
            node->children[modified]->size++;
            node->children[assignments[i]]->size--;
            assignments[i] = modified;
            done = 0;
         }
      }

      // If a cluster get empty, there is a problem
      for (Rox_Uint k = 0; k < obj->branch_factor; k++)
      {

         if (node->children[k]->size <= 0) 
         { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
      }
   }

   // Build up children nodes
   start = 0;
   end = 0;
   for (Rox_Uint k = 0; k < obj->branch_factor; k++)
   {
      for (Rox_Uint i = 0; i < nb_indices; ++i)
      {
         if (assignments[i] == k)
         {
            // Reorganize data to partition
            exc = indices[i];
            indices[i] = indices[end];
            indices[end] = exc;
            exc = assignments[i];
            assignments[i] = assignments[end];
            assignments[end] = exc;

            end++;
         }
      }

      // Recursive build
      error = rox_sraid_tree_node_cluster(obj, node->children[k], indices + start, end - start, level + 1);
      ROX_ERROR_CHECK_TERMINATE ( error );
      start = end;
   }

function_terminate:
   rox_memory_delete(assignments);
   return error;
}

Rox_ErrorCode rox_sraid_tree_build(Rox_Sraid_Tree obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint id;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Set seed
   rox_srand(4);

   obj->indices = (Rox_Uint*) rox_memory_allocate(sizeof(Rox_Uint), obj->database->used);

   if (!obj->indices) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (id = 0; id < obj->database->used; id++) { obj->indices[id] = id; }

   // Create root node
   error = rox_sraid_tree_node_new(&obj->root, obj->node_count);
   ROX_ERROR_CHECK_TERMINATE ( error );
   obj->node_count++;

   // Compute initial statistics
   error = rox_sraid_tree_node_build_statistics(obj, obj->root, obj->indices, obj->database->used);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Cluster nodes hierarchically

   error = rox_sraid_tree_node_cluster(obj, obj->root, obj->indices, obj->database->used, 0); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

int compareNodeDist(NodeDist_Struct * one, NodeDist_Struct * two)
{
   if (one->dist < two->dist) return -1;
   else if (one->dist > two->dist) return 1;
   return 0;
}

Rox_ErrorCode rox_sraid_tree_match_node(ResultDual_Struct *res, Rox_Sraid_Tree obj, Rox_Sraid_Tree_Node node, Rox_Ushort * feat)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ushort * vec;
   Rox_Sint dist;
   NodeDist_Struct * listdist;
   NodeDist_Struct exc;
   Rox_Uint i, j;
   int id, check1, bestid, bestdist;

   if (!node->children)
   {
      if (!node->indices_pool)
      { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

      // Leaf
      for (i = 0; i < node->size; i++)
      {
         vec = obj->database->data[node->indices_pool[i]].descriptor;
         dist = rox_sraid_match(vec, feat);
         id = node->indices_pool[i];

         if (res->count == 0)
         {
            res->count++;
            res->dist1 = dist;
            res->id1 = id;
         }
         else if (res->count == 1)
         {
            res->count++;
            if (dist < (Rox_Sint) res->dist1)
            {
               res->dist2 = res->dist1;
               res->id2 = res->id1;
               res->dist1 = dist;
               res->id1 = id;
            }
            else
            {
               res->dist2 = dist;
               res->id2 = id;
            }
         }
         else
         {
            if (dist < (Rox_Sint) res->dist2)
            {
               if (dist < (Rox_Sint) res->dist1)
               {
                  res->dist2 = res->dist1;
                  res->id2 = res->id1;
                  res->dist1 = dist;
                  res->id1 = id;
               }
               else
               {
                  res->dist2 = dist;
                  res->id2 = id;
               }
            }

            res->worst = res->dist2;
         }
      }
   }
   else
   {
      listdist = (NodeDist_Struct *)rox_memory_allocate(sizeof(NodeDist_Struct), obj->branch_factor);

      for (i = 0; i < obj->branch_factor; i++)
      {
         vec = node->children[i]->mean;
         listdist[i].id = i;
         listdist[i].dist = rox_sraid_match(vec, feat);
      }

      // Sort children by distance to point
      for (i = 0; i < obj->branch_factor; i++)
      {
         bestdist = listdist[i].dist;
         bestid = i;

         for (j = i + 1; j < obj->branch_factor; j++)
         {
            if (listdist[j].dist < (Rox_Uint) bestdist)
            {
               bestdist = listdist[j].dist;
               bestid = j;
            }
         }

         exc = listdist[i];
         listdist[i] = listdist[bestid];
         listdist[bestid] = exc;
      }

      // Poll children
      for (i = 0; i < obj->branch_factor; i++)
      {
         id = listdist[i].id;

         // Try to avoid comparisons
         if (res->count == 2)
         {
            check1 = listdist[i].dist - node->children[id]->radius - res->worst;
            if (check1 > 0) break;
         }

         error = rox_sraid_tree_match_node(res, obj, node->children[id], feat);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      rox_memory_delete(listdist);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_sraid_tree_match(ResultDual_Struct * res, Rox_Sraid_Tree obj, Rox_Ushort * feat)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!obj || !feat || !res) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   res->count = 0;
   res->worst = INT_MAX;

   error = rox_sraid_tree_match_node(res, obj, obj->root, feat);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_sraid_tree_matchset(Rox_DynVec_Sint matchassocs, Rox_Sraid_Tree obj, Rox_DynVec_SRAID_Feature features)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!obj || !matchassocs || !features) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   matchassocs->used = 0;
   rox_dynvec_sint_usecells(matchassocs, features->used);

   for (Rox_Uint id = 0; id < features->used; id++)
   {
      ResultDual_Struct res;

      error = rox_sraid_tree_match(&res, obj, features->data[id].descriptor);
      if (error) continue;
      if (res.count == 0) continue;

      matchassocs->data[id] = -1;

      if (res.count == 2)
      {
         if (res.dist1 > res.dist2 * 0.7) continue;
      }

      matchassocs->data[id] = res.id1;
   }

function_terminate:
   return error;
}
