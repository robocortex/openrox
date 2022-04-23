//==============================================================================
//
//    OPENROX   : File quad_gradientclusterer.c
//
//    Contents  : Implementation of quad_gradientclusterer module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "quad_gradientclusterer.h"
#include "quad_gradientclusterer_struct.h"

#include <string.h>
#include <generated/dynvec_edgel_struct.h>

#include <system/memory/memory.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/geometry/point/points_struct.h>
#include <baseproc/geometry/transforms/matsl3/sl3from4points.h>
#include <baseproc/maths/maths_macros.h>
#include <core/features/detectors/quad/quad_detection_struct.h>
#include <system/time/timer.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

#define THETATHRESH 150.0 // 

extern Rox_Double mod2pi_pos(Rox_Double vin);
extern Rox_Double mod2pi(Rox_Double vin);
extern Rox_Double mod2pi2(Rox_Double ref, Rox_Double v);

Rox_ErrorCode rox_gradientclusterer_new(Rox_GradientClusterer *ptr, Rox_Uint iwidth, Rox_Uint iheight)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_GradientClusterer ret = NULL;

   if (!ptr)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)
   }

   if ((iwidth == 0) || (iheight == 0))
   {
      error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)
   }

   ret = (Rox_GradientClusterer)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)
   }

   ret->nbgroups = 0;
   ret->width = iwidth;
   ret->height = iheight;

   ret->gmag = NULL; // Rox_Uint
   ret->gmagmax = NULL; // Rox_Uint
   ret->gmagmin = NULL; // Rox_Uint
   ret->gtheta = NULL; // Rox_Float
   ret->gthetamin = NULL; // Rox_Float
   ret->gthetamax = NULL; // Rox_Float
   ret->gmagval = NULL;


   ret->nodes = NULL;
   ret->edges = NULL;
   ret->sortededges = NULL;
   ret->edgecounters = NULL;
   ret->groups = NULL;
   ret->edgedraw = NULL;
   ret->preproc = NULL;
   ret->postproc = NULL;
   ret->pointset = NULL;

   ret->gmag = (Rox_Uint *) rox_memory_allocate(sizeof(*ret->gmag), iwidth*iheight);
   if ( !ret->gmag )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }
   
   ret->gmagmax = (Rox_Uint *) rox_memory_allocate(sizeof(*ret->gmagmax), iwidth*iheight);
   if ( !ret->gmagmax )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   ret->gmagmin = (Rox_Uint *) rox_memory_allocate(sizeof(*ret->gmagmin), iwidth*iheight);
   if ( !ret->gmagmin )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   ret->gtheta = (Rox_Float *) rox_memory_allocate(sizeof(*ret->gtheta), iwidth*iheight);
   if ( !ret->gtheta )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   ret->gthetamin = (Rox_Float *) rox_memory_allocate(sizeof(*ret->gthetamin), iwidth*iheight);
   if ( !ret->gthetamin )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }
   
   ret->gthetamax = (Rox_Float *) rox_memory_allocate(sizeof(*ret->gthetamax), iwidth*iheight);
   if ( !ret->gthetamax )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   ret->nodes = (LabelNode) rox_memory_allocate(sizeof(struct LabelNode_Struct), iwidth*iheight);
   if ( !ret->nodes )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   ret->edges = (LabelEdge) rox_memory_allocate(sizeof(struct LabelEdge_Struct), iwidth*iheight * 4);
   if ( !ret->edges )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   ret->sortededges = (LabelEdge) rox_memory_allocate(sizeof(struct LabelEdge_Struct), iwidth*iheight);
   if ( !ret->sortededges )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   ret->edgecounters = (Rox_Uint *) rox_memory_allocate(sizeof(*ret->edgecounters), iwidth*iheight);
   if ( !ret->edgecounters )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   ret->groups = (Rox_DynVec_OrientedImagePoint *) rox_memory_allocate(sizeof(Rox_DynVec_OrientedImagePoint), iwidth*iheight);
   if ( !ret->groups )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   error = rox_array2d_uint_new(&ret->gmagval, iheight, iwidth);
   ROX_ERROR_CHECK_TERMINATE(error);

   error = rox_edgepreproc_gray_new(&ret->preproc, iwidth, iheight);
   ROX_ERROR_CHECK_TERMINATE(error);

   error = rox_edgedraw_new(&ret->edgedraw, iwidth, iheight, 12, 1);
   ROX_ERROR_CHECK_TERMINATE(error);

   error = rox_edgepostproc_ac_new(&ret->postproc, iwidth, iheight);
   ROX_ERROR_CHECK_TERMINATE(error);

   error = rox_edgepostproc_normal_new(&ret->normals, iwidth, iheight);
   ROX_ERROR_CHECK_TERMINATE(error);

   error = rox_objset_dynvec_orientedimagepoint_new(&ret->pointset, 10);
   ROX_ERROR_CHECK_TERMINATE(error);

   // Important : push 0 to all pointers
   memset(ret->groups, 0, sizeof(Rox_DynVec_OrientedImagePoint) * iwidth * iheight);
   memset(ret->gmag, 0, sizeof(*ret->gmag) * iwidth * iheight);
   memset(ret->gmagmax, 0, sizeof(*ret->gmagmax) * iwidth * iheight);
   memset(ret->gmagmin, 0, sizeof(*ret->gmagmin) * iwidth * iheight);
   memset(ret->gtheta, 0, sizeof(*ret->gtheta) * iwidth * iheight);
   memset(ret->gthetamin, 0, sizeof(*ret->gthetamin) * iwidth * iheight);
   memset(ret->gthetamax, 0, sizeof(*ret->gthetamax) * iwidth * iheight);

   *ptr = ret;

function_terminate:
   if (error) rox_gradientclusterer_del(&ret);
   return error;
}

Rox_ErrorCode rox_gradientclusterer_reset(Rox_GradientClusterer ptr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!ptr)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error);
   }

   Rox_Uint basesize = ptr->width * ptr->height;

   // Delete groups of pixels
   for (Rox_Uint i = 0; i < ptr->nbgroups; i++)
   {
      // Free groups if needed
      if (ptr->groups[i])
      {
         rox_dynvec_orientedimagepoint_del(&ptr->groups[i]);
         ptr->groups[i] = 0;
      }
   }

   // Prepare edges, nodes and groups
   ptr->nbedges = 0;
   ptr->nbgroups = 0;

   if (ptr->nodes)
   {
      for (Rox_Uint i = 0; i < basesize; i++)
      {
         ptr->nodes[i].countref = 1;
         ptr->nodes[i].parent = i;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_gradientclusterer_del(Rox_GradientClusterer *ptr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_GradientClusterer todel = NULL;


   if (!ptr)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error);
   }

   todel = *ptr;
   *ptr = NULL;


   if (!todel)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error);
   }

   rox_gradientclusterer_reset(todel);

   ROX_ERROR_CHECK(rox_array2d_uint_del(&todel->gmagval));

   rox_memory_delete(todel->gmag);
   rox_memory_delete(todel->gmagmin);
   rox_memory_delete(todel->gmagmax);
   rox_memory_delete(todel->gtheta);
   rox_memory_delete(todel->gthetamin);
   rox_memory_delete(todel->gthetamax);
   rox_memory_delete(todel->nodes);
   rox_memory_delete(todel->edges);
   rox_memory_delete(todel->sortededges);
   rox_memory_delete(todel->edgecounters);
   rox_memory_delete(todel->groups);

   ROX_ERROR_CHECK(rox_edgepostproc_normal_del(&todel->normals));
   ROX_ERROR_CHECK(rox_edgepostproc_ac_del(&todel->postproc));
   ROX_ERROR_CHECK(rox_edgepreproc_gray_del(&todel->preproc));
   ROX_ERROR_CHECK(rox_edgedraw_del(&todel->edgedraw));
   ROX_ERROR_CHECK(rox_objset_dynvec_orientedimagepoint_del(&todel->pointset));

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_Sint rox_gradientclusterer_edgecost(Rox_Double theta0, Rox_Uint mag0, Rox_Double theta1, Rox_Uint mag1)
{
   Rox_Double thetaErr = 0.0;
   Rox_Double diff = 0.0;

   // Both magnitudes must be significant enough !
   if (mag0 < MINMAG || mag1 < MINMAG) return -1;

   // Compute difference between gradient angles
   diff = theta1 - theta0;
   thetaErr = fabs(mod2pi(diff));

   // Check if difference is small enough
   if (thetaErr > MAXEDGECOST) return -1;

   // Normalize this cost between [0;1]
   thetaErr = thetaErr / MAXEDGECOST;

   // TODO: should be ??? if uncommented we will detect less quads !!!
   // thetaErr = thetaErr / (2*ROX_PI);

   // We want integers as return type for fast sorting so we quantize between [0;WEIGHT_SCALE]
   return (int)(thetaErr * WEIGHT_SCALE);
}

// Fast method to sort edges by their cost
Rox_ErrorCode rox_gradientclusterer_edgesort(Rox_GradientClusterer ptr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!ptr)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error);
   }

   // Compute maximum edge cost
   Rox_Sint maxv = -1;
   for (Rox_Uint i = 0; i < ptr->nbedges; i++)
   {
      maxv = ROX_MAX(maxv, (int)ptr->edges[i].cost);
   }

   // Initialize edge counters
   Rox_Uint clength = maxv + 2;
   for (Rox_Uint i = 0; i < clength; i++)
   {
      ptr->edgecounters[i] = 0;
   }

   // For each edge cost value, count the number of edges with this value
   for (Rox_Uint i = 0; i < ptr->nbedges; i++)
   {
      int w = (int)(ptr->edges[i].cost);
      ptr->edgecounters[w + 1]++;
   }

   // Each edge bin counter is accumulated with the previous one
   for (Rox_Uint i = 1; i < clength; i++)
   {
      ptr->edgecounters[i] += ptr->edgecounters[i - 1];
   }

   // Initialize output
   for (Rox_Uint i = 0; i < ptr->nbedges; i++)
   {
      ptr->sortededges[i].cur = 0;
      ptr->sortededges[i].assoc = 0;
      ptr->sortededges[i].cost = 0;
   }

   // Using this accumulated counter, we are able to quickly place edges sorted by their cost
   for (Rox_Uint i = 0; i < ptr->nbedges; i++)
   {
      int w = (int)(ptr->edges[i].cost);
      ptr->sortededges[ptr->edgecounters[w]] = ptr->edges[i];
      ptr->edgecounters[w]++;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_gradientclusterer_computeedges(Rox_GradientClusterer ptr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!ptr)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error);
   }

   Rox_Uint * dgm = ptr->gmag;
   Rox_Float * dgt = ptr->gtheta;
   LabelEdge edges = ptr->edges;

   // Loop over all pixels where a gradient is defined
   for (Rox_Sint i = 1; i < ptr->height - 1; i++)
   {
      for (Rox_Sint j = 1; j < ptr->width - 1; j++)
      {
         double theta0 = 0.0;
         Rox_Sint cost = 0;

         Rox_Sint idr = i * ptr->width + j;

         // Check if current pixel gradient is strong enough
         Rox_Uint mag0 = dgm[idr];
         if (mag0 < MINMAG)
         {
            continue;
         }

         // Initialize min and max
         ptr->gmagmin[idr] = mag0;
         ptr->gmagmax[idr] = mag0;

         theta0 = dgt[idr];
         ptr->gthetamin[idr] = (Rox_Float)theta0;
         ptr->gthetamax[idr] = (Rox_Float)theta0;

         // For the 4 bottom neighboors of the pixel, compute the cost of the edge
         // This cost is the distance between angles of the gradients
         // If the cost is small enough, we store a new edge as a connection between 2 pixels and add it to a set.
         // It is easy to understand that there is max 4*nbpixels connections in the set.

         Rox_Sint id1 = i    * ptr->width + j + 1; // pixel [i  ][j+1]
         Rox_Sint id2 = (i + 1) * ptr->width + j; // pixel [i+1][j  ]
         Rox_Sint id3 = (i + 1) * ptr->width + j + 1; // pixel [i+1][j+1]
         Rox_Sint id4 = (i + 1) * ptr->width + j - 1; // pixel [i+1][j-1]

         // Right pixel
         cost = rox_gradientclusterer_edgecost(theta0, mag0, dgt[id1], dgm[id1]);
         if (cost >= 0)
         {
            edges->cur = idr;
            edges->assoc = id1;
            edges->cost = cost;

            edges++;
            ptr->nbedges++;
         }

         // Bottom pixel
         cost = rox_gradientclusterer_edgecost(theta0, mag0, dgt[id2], dgm[id2]);
         if (cost >= 0)
         {
            edges->cur = idr;
            edges->assoc = id2;
            edges->cost = cost;

            edges++;
            ptr->nbedges++;
         }

         // Bottom Right pixel
         cost = rox_gradientclusterer_edgecost(theta0, mag0, dgt[id3], dgm[id3]);
         if (cost >= 0)
         {
            edges->cur = idr;
            edges->assoc = id3;
            edges->cost = cost;

            edges++;
            ptr->nbedges++;
         }

         // Bottom left pixel
         cost = (j == 0) ? -1 : rox_gradientclusterer_edgecost(theta0, mag0, dgt[id4], dgm[id4]);
         if (cost >= 0)
         {
            edges->cur = idr;
            edges->assoc = id4;
            edges->cost = cost;

            edges++;
            ptr->nbedges++;
         }
      }
   }

function_terminate:
   return error;
}

// Get root of partition for current pixel
Rox_Uint rox_gradientclusterer_getrepresentative(LabelNode data, Rox_Uint id)
{
   Rox_Uint ret, curid, previd;

   // If root is the pixel (it's his own root), return
   if (data[id].parent == id) return id;

   // Loop through parents until the root is found
   ret = id;
   do
   {
      curid = ret;
      ret = data[curid].parent;
   } while (curid != ret);

   // To avoid unecessary loop in the future, set the real parent to all the path
   curid = id;
   do
   {
      previd = curid;
      curid = data[curid].parent;
      data[previd].parent = ret;
   } while (curid != ret);

   return ret;
}

// Connect two nodes
Rox_Uint rox_gradientclusterer_connectnodes(LabelNode data, Rox_Uint aid, Rox_Uint bid)
{
   Rox_Uint asz, bsz;

   // Get roots of two pixels, it is direct because we already set it in getrepresentative
   Rox_Uint aroot = data[aid].parent;
   Rox_Uint broot = data[bid].parent;

   // If same root, they are already connected
   if (aroot == broot) return aroot;

   // New node is the node with the more children
   asz = data[aroot].countref;
   bsz = data[broot].countref;
   if (asz > bsz)
   {
      data[broot].parent = aroot;
      data[aroot].countref += bsz;
      return aroot;
   }
   else
   {
      data[aroot].parent = broot;
      data[broot].countref += asz;
      return broot;
   }
}

// Cluster edges in multiple partitions
Rox_ErrorCode rox_gradientclusterer_cluster(Rox_GradientClusterer ptr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint sza = 0, szb = 0;
   Rox_Double tmina = 0.0, tminb = 0.0;
   Rox_Double tmaxa = 0.0, tmaxb = 0.0;
   Rox_Double costa = 0.0, costb = 0.0;
   Rox_Double bshift = 0.0;
   Rox_Double tminab = 0.0;
   Rox_Double tmaxab = 0.0;
   Rox_Sint mminab = 0;
   Rox_Sint mmaxab = 0;
   Rox_Double costab = 0.0;


   if (!ptr)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error);
   }

   // Loop over edges
   for (Rox_Uint i = 0; i < ptr->nbedges; i++)
   {
      // Get id of pixels
      int idb = ptr->sortededges[i].cur;
      int ida = ptr->sortededges[i].assoc;

      // Get current root of graph
      ida = rox_gradientclusterer_getrepresentative(ptr->nodes, ida);
      idb = rox_gradientclusterer_getrepresentative(ptr->nodes, idb);

      // If already the same root, ignore
      if (ida == idb) continue;

      // Is the countref
      sza = ptr->nodes[ida].countref;
      szb = ptr->nodes[idb].countref;

      tmina = ptr->gthetamin[ida];
      tmaxa = ptr->gthetamax[ida];
      tminb = ptr->gthetamin[idb];
      tmaxb = ptr->gthetamax[idb];
      costa = (tmaxa - tmina);
      costb = (tmaxb - tminb);

      // Compute the best shift (k * 2PI) to align the set of angles
      bshift = mod2pi2((tmina + tmaxa) / 2.0, (tminb + tmaxb) / 2.0) - (tminb + tmaxb) / 2.0;

      // Align both sets and retrieve min/max theta/magnitude
      tminab = ROX_MIN(tmina, tminb + bshift);
      tmaxab = ROX_MAX(tmaxa, tmaxb + bshift);
      if (tmaxab - tminab > ROX_PI * 2.0) tmaxab = tminab + 2.0*ROX_PI;
      mmaxab = ROX_MAX(ptr->gmagmax[ida], ptr->gmagmax[idb]);
      mminab = ROX_MIN(ptr->gmagmin[ida], ptr->gmagmin[idb]);
      costab = (tmaxab - tminab);

      // If the union is better than the two separate sets, merge them
      if (costab <= (ROX_MIN(costa, costb) + THETATHRESH / (double)(sza + szb)) \
          && (mmaxab - mminab) <= ROX_MIN(ptr->gmagmax[ida] - ptr->gmagmin[ida], ptr->gmagmax[idb] - ptr->gmagmin[idb]) + MAGTHRESH / (double)(sza + szb))
      {
         // Merge both nodes
         Rox_Sint idab = rox_gradientclusterer_connectnodes(ptr->nodes, ida, idb);

         // Set node information
         ptr->gthetamin[idab] = (Rox_Float)tminab;
         ptr->gthetamax[idab] = (Rox_Float)tmaxab;
         ptr->gmagmin[idab] = (Rox_Uint)mminab;
         ptr->gmagmax[idab] = (Rox_Uint)mmaxab;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_gradientclusterer_groups(Rox_GradientClusterer ptr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Uint idx = 0;
   Rox_OrientedImagePoint_Struct pt;


   if (!ptr)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error);
   }

   //  Loop through pixels
   for (Rox_Sint i = 2; i < ptr->height - 2; i++)
   {
      for (Rox_Sint j = 2; j < ptr->width - 2; j++)
      {
         Rox_Uint size, rep;

         idx = i*ptr->width + j;

         // Get node data associated with pixel
         rep = rox_gradientclusterer_getrepresentative(ptr->nodes, idx);
         size = ptr->nodes[rep].countref;

         //  size is the length (almost) of the "chain of pixels"
         if (size < MINSEGSIZE)
         {
            idx++;
            continue;
         }

         // Allocate group if necessary (first pixel seen for this group)
         if (ptr->groups[rep] == 0)
         {
            error = rox_dynvec_orientedimagepoint_new(&ptr->groups[rep], 100);
            if (error) return error;
         }

         //  fill group element with pixel information
         pt.i = i;
         pt.j = j;
         pt.theta = ptr->gtheta[rep];
         pt.mag = ptr->gmag[rep];
         rox_dynvec_orientedimagepoint_append(ptr->groups[rep], &pt);
      }
   }

   //  compress the group arrays to avoid holes and enable indexed access
   ptr->nbgroups = 0;
   for (Rox_Sint i = 2; i < ptr->height - 2; i++)
   {
      for (Rox_Sint j = 2; j < ptr->width - 2; j++)
      {
         idx = i*ptr->width + j;
         if (ptr->groups[idx] == 0)
         {
            continue;
         }
         // swapping
         ptr->groups[ptr->nbgroups] = ptr->groups[idx];
         ptr->groups[idx] = 0;
         ptr->nbgroups++;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_gradientclusterer_make(
   Rox_GradientClusterer ptr,
   const Rox_Image source,
   const Rox_Imask mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!ptr || !source || !mask)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error);
   }

   // First, reset stored information from previous frame
   error = rox_gradientclusterer_reset(ptr);
   ROX_ERROR_CHECK_TERMINATE(error);

   // // Define timer object
   // Rox_Timer timer = 0;
   // Rox_Double time = 0.0;
   // error = rox_timer_new(&timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_timer_start(timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute gradients
   error = rox_gradientclusterer_buildgradients(ptr->gmagval, ptr->gmag, ptr->gtheta, source, mask);
   ROX_ERROR_CHECK_TERMINATE(error);

   // error = rox_timer_stop(timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_timer_get_elapsed_ms(&time, timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // rox_log("time to buildgradients = %f\n", time);

   // Make pairs of pixels given their neighboorhood proximity in gradient angle space
   error = rox_gradientclusterer_computeedges(ptr);
   ROX_ERROR_CHECK_TERMINATE(error);

   // Sort edges by cost
   error = rox_gradientclusterer_edgesort(ptr);
   ROX_ERROR_CHECK_TERMINATE(error);

   // Merge edges in a graph
   error = rox_gradientclusterer_cluster(ptr);
   ROX_ERROR_CHECK_TERMINATE(error);

   // Store groups which are nodes with good properties
   error = rox_gradientclusterer_groups(ptr);
   ROX_ERROR_CHECK_TERMINATE(error);

function_terminate:
   return error;
}

Rox_ErrorCode rox_gradientclusterer_make_ac(
   Rox_GradientClusterer ptr,
   const Rox_Image image,
   const Rox_Imask mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // TODO : expose hard coded parameter
   Rox_Uint nbr_blur_passes = 3; // The more blur passes the more detected ? was 1 on feb 21 2016

   Rox_DynVec_OrientedImagePoint ptvec = NULL;


   if (!ptr || !image || !mask)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error);
   }

   Rox_Point2D_Sshort * dg = NULL;
   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer(&dg, ptr->preproc->gradients);
   ROX_ERROR_CHECK_TERMINATE(error);

   // First, reset stored information from previous frame
   error = rox_gradientclusterer_reset(ptr);

   ROX_ERROR_CHECK_TERMINATE(error);

   // Process image with parameter nbr_blur_passes 
   error = rox_edgepreproc_gray_process(ptr->preproc, image, nbr_blur_passes);
   ROX_ERROR_CHECK_TERMINATE(error);

   // Draw edges with min_segment_size = 1 and straight_edge_only = 0 
   error = rox_edgedraw_process(ptr->edgedraw, ptr->preproc->gradients, 1, 0);
   ROX_ERROR_CHECK_TERMINATE(error);

   // Postprocess gradients with min_NFA = 1.0
   error = rox_edgepostproc_ac_process(ptr->postproc, ptr->edgedraw->resultsegments, 1.0);
   ROX_ERROR_CHECK_TERMINATE(error);

   // Postprocess gradients
   error = rox_edgepostproc_normal_process(ptr->normals, ptr->postproc->resultsegments, ptr->preproc->gradients);

   ROX_ERROR_CHECK_TERMINATE(error);

   rox_objset_dynvec_orientedimagepoint_reset(ptr->pointset);

   for (Rox_Uint i = 0; i < ptr->normals->resultsegments->used; i++)
   {
      Rox_DynVec_Edgel pts = ptr->normals->resultsegments->data[i];

      error = rox_dynvec_orientedimagepoint_new(&ptvec, 10);
      ROX_ERROR_CHECK_TERMINATE(error);

      // Compute magnitude and angle of image points
      for (Rox_Uint j = 0; j < pts->used; j++)
      {
         Rox_OrientedImagePoint_Struct toadd;
         Rox_Double du, dv;

         toadd.j = pts->data[j].u;
         toadd.i = pts->data[j].v;

         du = dg[toadd.i][toadd.j].u;
         dv = dg[toadd.i][toadd.j].v;

         toadd.mag = du*du + dv*dv;
         toadd.theta = atan2(dv, du);

         error = rox_dynvec_orientedimagepoint_append(ptvec, &toadd);
         ROX_ERROR_CHECK_TERMINATE(error);
      }

      error = rox_objset_dynvec_orientedimagepoint_append(ptr->pointset, ptvec);
      ROX_ERROR_CHECK_TERMINATE(error);
   }

function_terminate:
   return error;
}
