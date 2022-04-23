//==============================================================================
//
//    OPENROX   : File ehid_searchtree.c
//
//    Contents  : Implementation of ehid_searchtree module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ehid_searchtree.h"
#include "ehid_searchtree_struct.h"

#include <stdio.h>
#include <string.h>

#include <generated/dynvec_ehid_point_struct.h>

#include <baseproc/maths/maths_macros.h>

#include <core/features/descriptors/ehid/ehid.h>
#include <core/features/descriptors/ehid/ehid_match_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_ehid_searchtree_new(Rox_Ehid_SearchTree * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ehid_SearchTree ret = NULL;


   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Ehid_SearchTree) rox_memory_allocate(sizeof(*ret), 1);

   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->is_compiled = 0;
   ret->max_height = 1;
   ret->roots = NULL;
   ret->stack = NULL;

   error = rox_dynvec_ehid_dbnode_new(&ret->roots, 100);
   if (error)
   { rox_ehid_searchtree_del(&ret); goto function_terminate; }

   *obj = ret;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_node_del(Rox_Ehid_DbNode_Struct * node)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!node)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }// ROX_ERROR_CHECK_TERMINATE(error)}

   rox_ehid_node_del(node->left);
   rox_ehid_node_del(node->right);

   rox_memory_delete(node);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_searchtree_del(Rox_Ehid_SearchTree * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ehid_SearchTree todel = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *obj;
   *obj = NULL;

   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Delete children node of each roots recursively
   rox_ehid_searchtree_reset(todel);

   rox_dynvec_ehid_dbnode_del(&todel->roots);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_searchtree_reset(Rox_Ehid_SearchTree obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   obj->is_compiled = 0;

   // Delete children node of each roots recursively
   for (Rox_Uint idpt = 0; idpt < obj->roots->used; idpt++)
   {
      rox_ehid_node_del(obj->roots->data[idpt].left);
      rox_ehid_node_del(obj->roots->data[idpt].right);
   }

   rox_memory_delete(obj->stack);
   obj->stack = NULL;

   rox_dynvec_ehid_dbnode_reset(obj->roots);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_searchtree_compile(Rox_Ehid_SearchTree obj, Rox_DynVec_Ehid_Point db)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint score, maxcommon, globalmaxcommon;
   Rox_Uint maxid;
   Rox_Ehid_DbNode_Struct node;
   Rox_Ehid_DbNode_Struct * cleft = NULL, * cright = NULL;
   Rox_Ehid_DbNode_Struct * refnode = NULL, * assocnode = NULL, * delnode = NULL, * curnode = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Delete previous compilation
   rox_ehid_searchtree_reset(obj);

   // Add all points as roots
   node.left = NULL;
   node.right = NULL;
   node.closest = NULL;
   node.closestdist = 0;
   node.level = 0;
   for (Rox_Uint idpt = 0; idpt < db->used; idpt++)
   {
      node.dbid = db->data[idpt].uid;
      node.desc[0] = db->data[idpt].Description[0];
      node.desc[1] = db->data[idpt].Description[1];
      node.desc[2] = db->data[idpt].Description[2];
      node.desc[3] = db->data[idpt].Description[3];
      node.desc[4] = db->data[idpt].Description[4];
      rox_dynvec_ehid_dbnode_append(obj->roots, &node);
   }

   // Initialize closest indices
   globalmaxcommon = 0;
   for (Rox_Uint idpt = 0; idpt < obj->roots->used; idpt++)
   {
      maxid = 0;
      maxcommon = 0;

      // Search for greater distance
      for (Rox_Uint idcmp = 0; idcmp < obj->roots->used; idcmp++)
      {
         if (idpt == idcmp) continue;

         // Closest point is the one with biggest matching score
         rox_ehid_point_match(&score, obj->roots->data[idpt].desc, obj->roots->data[idcmp].desc);
         if (score >= maxcommon)
         {
            maxcommon = score;
            maxid = idcmp;
         }
      }

      // Store the closest node
      obj->roots->data[idpt].closestdist = maxcommon;
      obj->roots->data[idpt].closest = &obj->roots->data[maxid];

      // Retrieve the best pair
      if (obj->roots->data[idpt].closestdist > globalmaxcommon)
      {
         refnode = &obj->roots->data[idpt];
         globalmaxcommon = obj->roots->data[idpt].closestdist;
      }
   }

   // Begin tree building
   if (obj->roots->used > 1)
   {
      while (refnode)
      {
         if (refnode->closestdist < 1) break;

         assocnode = refnode->closest;

         // Create two new nodes from best association
         cleft = (Rox_Ehid_DbNode_Struct *) rox_memory_allocate(sizeof(struct Rox_Ehid_DbNode_Struct), 1);
         cright = (Rox_Ehid_DbNode_Struct *) rox_memory_allocate(sizeof(struct Rox_Ehid_DbNode_Struct), 1);

         if (!cleft || !cright)

         { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

         // Copy nodes to merge
         cleft->dbid = refnode->dbid;
         cright->dbid = assocnode->dbid;
         cleft->level = refnode->level;
         cright->level = assocnode->level;
         cleft->left = refnode->left;
         cright->left = assocnode->left;
         cright->right = assocnode->right;
         cleft->right = refnode->right;

         // Copy descriptions to merge
         for ( Rox_Sint iddesc = 0; iddesc < 5; iddesc++)
         {
            cleft->desc[iddesc] = refnode->desc[iddesc];
            cright->desc[iddesc] = assocnode->desc[iddesc];
         }

         // Merge these two nodes
         refnode->dbid = -1; // A merged point has no associated db index obviously
         refnode->desc[0] = cleft->desc[0] & cright->desc[0];
         refnode->desc[1] = cleft->desc[1] & cright->desc[1];
         refnode->desc[2] = cleft->desc[2] & cright->desc[2];
         refnode->desc[3] = cleft->desc[3] & cright->desc[3];
         refnode->desc[4] = cleft->desc[4] & cright->desc[4];
         refnode->left = cleft;
         refnode->right = cright;
         refnode->level = ROX_MAX(cleft->level, cright->level) + 1;

         // Remove second node from roots
         delnode = &obj->roots->data[obj->roots->used-1];
         obj->roots->used--;
         *assocnode = *delnode;

         // Update distances for old points
         globalmaxcommon = 0;
         refnode = 0;
         for (Rox_Uint idpt = 0; idpt < obj->roots->used; idpt++)
         {
            Rox_Uint istorefresh = 0;
            curnode = &obj->roots->data[idpt];

            if (curnode == refnode) istorefresh = 1;
            if (curnode->closest == refnode) istorefresh = 1;
            if (curnode->closest == assocnode) istorefresh = 1;
            if (curnode->closest == delnode) istorefresh = 1;

            if (istorefresh)
            {
               maxcommon = 0;
               maxid = 0;

               for (Rox_Uint idcmp = 0; idcmp < obj->roots->used; idcmp++)
               {
                  if (idpt == idcmp) continue;

                  rox_ehid_point_match(&score, obj->roots->data[idpt].desc, obj->roots->data[idcmp].desc);
                  if (score >= maxcommon)
                  {
                     maxcommon = score;
                     maxid = idcmp;
                  }
               }

               // Store the closest node
               obj->roots->data[idpt].closestdist = maxcommon;
               obj->roots->data[idpt].closest = &obj->roots->data[maxid];
            }

            // Retrieve the best pair
            if (obj->roots->data[idpt].closestdist > globalmaxcommon)
            {
               refnode = &obj->roots->data[idpt];
               globalmaxcommon = obj->roots->data[idpt].closestdist;
            }
         }
      }
   }

   // Retrieve graph depth
   obj->max_height = 0;
   for (Rox_Uint idpt = 0; idpt < obj->roots->used; idpt++)
   {
      obj->max_height = ROX_MAX(obj->roots->data[idpt].level, obj->max_height);
   }

   // Mark as compiled
   obj->is_compiled = 1;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_searchtree_lookup(Rox_DynVec_Ehid_Match result, Rox_Ehid_SearchTree obj, Rox_Ehid_Description base)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint idroot, stacksize, score, count;
   Rox_Ehid_DbNode_Struct * cur;
   Rox_Ehid_Match_Struct match;


   if (!result || !obj || !base)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_ehid_match_reset(result);

   if (obj->stack == NULL)
   {
      obj->stack = (Rox_Ehid_DbNode_Struct **) rox_memory_allocate(sizeof(Rox_Ehid_DbNode_Struct *), obj->max_height+1);

      if (!obj->stack)
      { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   }

   count=0;
   for (idroot = 0; idroot < obj->roots->used; idroot++)
   {
      obj->stack[0] = &obj->roots->data[idroot];
      stacksize = 1;

      while (stacksize)
      {
         cur = obj->stack[stacksize - 1];
         stacksize--;

         count++;
         rox_ehid_point_match(&score, cur->desc, base);
         if (score > 4) continue;

         if (cur->level == 0)
         {
            if (cur->dbid >= 0)
            {
               match.dbid = cur->dbid;
               match.score = score;
               rox_dynvec_ehid_match_append(result, &match);
            }
            continue;
         }

         if (cur->right)
         {
            obj->stack[stacksize] = cur->right;
            stacksize++;
         }

         if (cur->left)
         {
            obj->stack[stacksize] = cur->left;
            stacksize++;
         }
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_searchtree_save_tree_node(Rox_Ehid_DbNode_Struct * obj, FILE * output)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (obj == NULL)
   {
      Rox_Sint invalid = -2;
      fwrite(&invalid, sizeof(Rox_Sint), 1, output);

      error = ROX_ERROR_NONE;
      goto function_terminate;
   }

   fwrite(&obj->dbid, sizeof(Rox_Sint), 1, output);
   fwrite(&obj->level, sizeof(Rox_Uint), 1, output);
   fwrite(&obj->desc, sizeof(Rox_Ehid_Description), 1, output);

   error = rox_ehid_searchtree_save_tree_node(obj->left, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ehid_searchtree_save_tree_node(obj->right, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

int total = 0;

Rox_ErrorCode rox_ehid_searchtree_load_tree_node(Rox_Ehid_DbNode_Struct ** obj, FILE * input, Rox_Uint first)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint dbid;
   Rox_Uint read;
   Rox_Ehid_DbNode_Struct * retnode;

   read = (Rox_Uint) fread(&dbid, sizeof(Rox_Sint), 1, input); if (read != 1) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}
   if (dbid == -2)
   {
      error = ROX_ERROR_NONE;
      goto function_terminate;
   }

   // Except for root we have to allocate a new node
   if (first == 0)
   {
      retnode = (Rox_Ehid_DbNode_Struct *)rox_memory_allocate(sizeof(struct Rox_Ehid_DbNode_Struct), 1);
      if (!retnode) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
      *obj = retnode;
   }
   else
   {
      retnode = *obj;
   }

   retnode->dbid = dbid;
   retnode->left = NULL;
   retnode->right = NULL;

   read = (Rox_Uint) fread(&retnode->level, sizeof(Rox_Uint), 1, input); if (read != 1) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}
   read = (Rox_Uint) fread(&retnode->desc, sizeof(Rox_Ehid_Description), 1, input); if (read != 1) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_ehid_searchtree_load_tree_node(&retnode->left, input, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ehid_searchtree_load_tree_node(&retnode->right, input, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_searchtree_save_tree(Rox_Ehid_SearchTree obj, FILE * output)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   fwrite(&obj->max_height, sizeof(Rox_Uint), 1, output);
   fwrite(&obj->roots->used, sizeof(Rox_Uint), 1, output);

   for (Rox_Uint idroot = 0; idroot < obj->roots->used; idroot++)
   {
      error = rox_ehid_searchtree_save_tree_node(&obj->roots->data[idroot], output);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_searchtree_load_tree(Rox_Ehid_SearchTree obj, FILE * input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint nbroots = 0, read = 0;


   if (!obj || !input)
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Delete previous compilation
   rox_ehid_searchtree_reset(obj);

   // Read tree depth

   read = (Rox_Uint) fread(&obj->max_height, sizeof(Rox_Uint), 1, input);
   if (read != 1)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Read tree counter
   read = (Rox_Uint) fread(&nbroots, sizeof(Rox_Uint), 1, input);
   if (read != 1)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Load each trees
   for (Rox_Uint idroot = 0; idroot < nbroots; idroot++)
   {
      Rox_Ehid_DbNode_Struct node;
      Rox_Ehid_DbNode_Struct * ptrnode = NULL;

      rox_dynvec_ehid_dbnode_append(obj->roots, &node);

      ptrnode = &obj->roots->data[idroot];

      error = rox_ehid_searchtree_load_tree_node(&ptrnode, input, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   obj->is_compiled = 1;

function_terminate:
   return error;
}

static unsigned int serialization_offset = 0;

Rox_ErrorCode rox_ehid_searchtree_serialize(char* ser, const Rox_Ehid_SearchTree tree)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint offset = 0, size = 0;

   if (!ser || !tree) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   memcpy(ser + offset, &tree->max_height, sizeof(tree->max_height));
   offset += sizeof(tree->max_height);

   memcpy(ser + offset, &tree->roots->used, sizeof(tree->roots->used));
   offset += sizeof(tree->roots->used);

   for (Rox_Uint idroot = 0; idroot < tree->roots->used; idroot++)
   {
      //  Reset offset
      serialization_offset = 0;

      error = rox_ehid_searchtree_serialize_tree_node(ser + offset, &tree->roots->data[idroot]); ROX_ERROR_CHECK_TERMINATE(error)
      error = rox_ehid_searchtree_get_octet_size_tree_node(&size, &tree->roots->data[idroot]); ROX_ERROR_CHECK_TERMINATE(error)
      offset += size;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_searchtree_deserialize(Rox_Ehid_SearchTree tree, const char* ser)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint nbroots = 0, offset = 0, size = 0;

   if (!ser || !tree) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Delete previous compilation
   rox_ehid_searchtree_reset(tree);

   // Read tree depth
   memcpy(&tree->max_height, ser + offset, sizeof(tree->max_height));
   offset += sizeof(tree->max_height);

   // Read tree counter
   memcpy(&nbroots, ser + offset, sizeof(nbroots));
   offset += sizeof(nbroots);

   for (Rox_Uint idroot = 0; idroot < nbroots; idroot++)
   {
      Rox_Ehid_DbNode_Struct node;
      Rox_Ehid_DbNode_Struct * ptrnode;

      //  Reset offset
      serialization_offset = 0;

      rox_dynvec_ehid_dbnode_append(tree->roots, &node);
      ptrnode = &tree->roots->data[idroot];
      error = rox_ehid_searchtree_deserialize_tree_node(&ptrnode, ser + offset, 1);
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_ehid_searchtree_get_octet_size_tree_node(&size, ptrnode);
      ROX_ERROR_CHECK_TERMINATE(error)
      offset += size;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_searchtree_get_octet_size(Rox_Uint *size, const Rox_Ehid_SearchTree tree)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint ret = 0, node_size = 0;

   if(!tree || !size) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   *size = 0;

   ret += sizeof(tree->max_height);
   ret += sizeof(tree->roots->used);

   for (Rox_Uint id = 0; id < tree->roots->used; id ++)
   {
      error = rox_ehid_searchtree_get_octet_size_tree_node(&node_size, &tree->roots->data[id]); ROX_ERROR_CHECK_TERMINATE(error)
      ret += node_size;
   }

   *size = ret;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_searchtree_serialize_tree_node(char* ser, const Rox_Ehid_DbNode_Struct * node)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(ser == 0) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if(node == 0)
   {
      Rox_Sint invalid = -2;
      memcpy(ser + serialization_offset, &invalid, sizeof(invalid));
      serialization_offset += sizeof(invalid);
      error = ROX_ERROR_NONE;
      goto function_terminate;
   }

   memcpy(ser + serialization_offset, &node->dbid, sizeof(node->dbid));
   serialization_offset += sizeof(node->dbid);

   memcpy(ser + serialization_offset, &node->level, sizeof(node->level));
   serialization_offset += sizeof(node->level);

   memcpy(ser + serialization_offset, &node->desc, sizeof(node->desc));
   serialization_offset += sizeof(node->desc);

   error = rox_ehid_searchtree_serialize_tree_node(ser, node->left);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_ehid_searchtree_serialize_tree_node(ser, node->right);
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_searchtree_deserialize_tree_node(Rox_Ehid_DbNode_Struct ** node, const char* ser, const Rox_Uint first)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint dbid ;
   Rox_Ehid_DbNode_Struct * retnode = 0;

   if(ser == 0 || node == 0) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   memcpy(&dbid, ser + serialization_offset, sizeof(dbid));
   serialization_offset += sizeof(dbid);

   if (dbid == -2)
   {
      *node = 0;
      error = ROX_ERROR_NONE;
      goto function_terminate;
   }

   // Except for root we have to allocate a new node
   if (first == 0)
   {
      retnode = (Rox_Ehid_DbNode_Struct *) rox_memory_allocate(sizeof(struct Rox_Ehid_DbNode_Struct), 1);
      if (!retnode) {error = ROX_ERROR_NULL_POINTER; *node = retnode; ROX_ERROR_CHECK_TERMINATE(error)}
   }
   else
   {
      retnode = *node;
   }

   retnode->dbid = dbid;
   retnode->left = NULL;
   retnode->right = NULL;

   memcpy(&retnode->level, ser + serialization_offset, sizeof(retnode->level));
   serialization_offset += sizeof(retnode->level);
   memcpy(&retnode->desc, ser + serialization_offset, sizeof(retnode->desc));
   serialization_offset += sizeof(retnode->desc);

   error = rox_ehid_searchtree_deserialize_tree_node(&retnode->left, ser, 0);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_ehid_searchtree_deserialize_tree_node(&retnode->right, ser, 0);
   ROX_ERROR_CHECK_TERMINATE(error)

   *node = retnode;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_searchtree_get_octet_size_tree_node(Rox_Uint *size, const Rox_Ehid_DbNode_Struct * node)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint ret = 0;
   Rox_Uint node_size = 0;

   if(size == 0) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if(node == 0)
   {
      *size = sizeof(Rox_Sint); //  if node is NULL, -2 will be written !
      error = ROX_ERROR_NONE;
      goto function_terminate;
   }
   *size = 0;

   ret += sizeof(node->dbid);
   ret += sizeof(node->level);
   ret += sizeof(node->desc);

   error = rox_ehid_searchtree_get_octet_size_tree_node(&node_size, node->left);
   ROX_ERROR_CHECK_TERMINATE(error)
   ret += node_size;

   error = rox_ehid_searchtree_get_octet_size_tree_node(&node_size, node->right);
   ROX_ERROR_CHECK_TERMINATE(error)
   ret += node_size;

   *size = ret;

function_terminate:
   return error;
}
