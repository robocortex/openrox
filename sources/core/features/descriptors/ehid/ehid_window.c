//==============================================================================
//
//    OPENROX   : File ehid_window.c
//
//    Contents  : Implementation of ehid_window module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ehid_window.h"
#include "ehid_window_struct.h"

#include <generated/dynvec_uint.h>
#include <generated/dynvec_uint_struct.h>
#include <generated/dynvec_ehid_point_struct.h>

#include <baseproc/maths/maths_macros.h>

#include <core/features/descriptors/ehid/ehid.h>

#include <inout/system/errors_print.h>

//! Structure Neigboorhood
struct Rox_Ehid_Neighboorhood_Struct
{
   //! To be commented
   Rox_Uint idref;
   //! To be commented
   Rox_Uint used;
   //! To be commented
   Rox_DynVec_Uint idneighboors;
};

//! ehid_windowect
typedef struct Rox_Ehid_Neighboorhood_Struct Rox_Ehid_Neighboorhood_Struct;

//!
typedef struct Rox_Ehid_Neighboorhood_Struct * Rox_Ehid_Neighboorhood;

Rox_Sint comparNeighboorlist(Rox_Ehid_Neighboorhood one, Rox_Ehid_Neighboorhood two)
{
   Rox_Sint result = 0;

   if (one->idneighboors->used > two->idneighboors->used)
   { result = -1; goto function_terminate;}
   if (one->idneighboors->used < two->idneighboors->used)
   { result = +1; goto function_terminate;}

function_terminate:
   return result;
}

//! Structure Indices
struct Rox_Ehid_Index_Struct
{
     //! To be commented
   Rox_Uint index;
     //! To be commented
   Rox_Uint count;
};

//! ehid_windowect
typedef struct Rox_Ehid_Index_Struct Rox_Ehid_Index_Struct;

//!
typedef struct Rox_Ehid_Index_Struct * Rox_Ehid_Index;

int comparIndexlist(Rox_Ehid_Index one, Rox_Ehid_Index two)
{
   Rox_Sint result = 0;
   if (one->count > two->count)
   { result = -1; goto function_terminate; }

   if (one->count < two->count)
   {result = +1; goto function_terminate;}

function_terminate:
   return result;
}

Rox_ErrorCode rox_ehid_window_new(Rox_Ehid_Window * ehid_window, Rox_Sint top, Rox_Sint left, Rox_Sint height, Rox_Sint width, Rox_Uint maxpts)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ehid_Window ret = NULL;


   if (!ehid_window) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }


   *ehid_window = NULL;

   ret = (Rox_Ehid_Window)rox_memory_allocate(sizeof(struct Rox_Ehid_Window_Struct), 1);

   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }


   ret->top = top;
   ret->left = left;
   ret->height = height;
   ret->width = width;
   ret->bottom = top + height - 1;
   ret->right = left + width - 1;
   ret->maxlocalpts = maxpts;
   ret->countlocalviews = 0;


   ret->localpoints = NULL;

   error = rox_dynvec_ehid_point_new(&ret->localpoints, 10); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   ret->globalpoints = NULL;
   error = rox_dynvec_ehid_point_new(&ret->globalpoints, 10); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   ret->clusteredpoints = NULL;
   error = rox_dynvec_ehid_point_new(&ret->clusteredpoints, 10); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   ret->dbindices = NULL;
   error = rox_dynvec_ehid_dbindex_new(&ret->dbindices, 10); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   *ehid_window = ret;

function_terminate:
   if (error) rox_ehid_window_del(&ret);
   return error;
}

Rox_ErrorCode rox_ehid_window_del(Rox_Ehid_Window * ehid_window)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ehid_Window todel = NULL;


   if (!ehid_window) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *ehid_window;
   *ehid_window = NULL;


   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_ehid_point_del(&todel->localpoints);
   rox_dynvec_ehid_point_del(&todel->globalpoints);
   rox_dynvec_ehid_point_del(&todel->clusteredpoints);
   rox_dynvec_ehid_dbindex_del(&todel->dbindices);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_window_begin_view(Rox_Ehid_Window ehid_window)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   
   if (!ehid_window) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_ehid_point_reset(ehid_window->localpoints);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_window_terminate_view(Rox_Ehid_Window ehid_window)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

  
   if (!ehid_window) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_ehid_point_stack(ehid_window->globalpoints, ehid_window->localpoints);
   ehid_window->countlocalviews++;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_window_append_point (
   Rox_Ehid_Window ehid_window,
   Rox_Point2D_Double refpt,
   Rox_Point2D_Double viewpt
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ehid_Point_Struct pt;


   if (!ehid_window) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (ehid_window->localpoints->used >= ehid_window->maxlocalpts) 
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   // { error = ROX_ERROR_TOO_LARGE_VALUE; goto function_terminate; }

   if (floor(refpt->u) < ehid_window->left || ceil(refpt->u) > ehid_window->right || floor(refpt->v) < ehid_window->top || ceil(refpt->v) > ehid_window->bottom) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //  { error = ROX_ERROR_INVALID_VALUE; goto function_terminate; }

   pt.pos.u = viewpt->u;
   pt.pos.v = viewpt->v;
   rox_dynvec_ehid_point_append(ehid_window->localpoints, &pt);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_window_cluster(Rox_Ehid_Window ehid_window)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint countused, countref;
   Rox_Uint idlist, idref, idcur, idr, idn, idi;
   Rox_Double refx, refy, refdx, refdy;
   Rox_Double curx, cury, curdx, curdy;
   Rox_Double dx,dy,distpos, distangle;
   Rox_Uint k, l, histsum, sumidx, maxidx;

   Rox_Uint description_ref[64][5];
   Rox_Uint description_cur[64][5];
   Rox_Ehid_Index_Struct indices[32];
   Rox_Ehid_Point_Struct curpt;
   Rox_Ehid_DbIndex_Struct dbidx;

   Rox_Ehid_Neighboorhood_Struct * listn = NULL;

   if (!ehid_window) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Check that we have points
   Rox_Uint nbpts = ehid_window->globalpoints->used;

   // TODO: should be an error if there are no points to cluster ?

   if (nbpts == 0) 
   { error = ROX_ERROR_INSUFFICIENT_DATA; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   rox_dynvec_ehid_point_reset(ehid_window->clusteredpoints);
   rox_dynvec_ehid_dbindex_reset(ehid_window->dbindices);

   Rox_Uint mincard = (Rox_Uint) floor (0.2 * (Rox_Double)ehid_window->countlocalviews);
   Rox_Uint maxused = (Rox_Uint) ceil (0.5 * (Rox_Double)nbpts);

   // Create list of list of neighboors
   listn = (Rox_Ehid_Neighboorhood_Struct*) rox_memory_allocate(sizeof(Rox_Ehid_Neighboorhood_Struct), nbpts);

   if (!listn) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Create lists inside
   for (idlist = 0; idlist < nbpts; idlist++)
   {
      error = rox_dynvec_uint_new(&listn[idlist].idneighboors, 10);
      ROX_ERROR_CHECK_TERMINATE ( error );
      listn[idlist].used = 0;
      listn[idlist].idref = idlist;
   }

   // Compute neigboors
   for (idref = 0; idref < nbpts; idref++)
   {
      refx = ehid_window->globalpoints->data[idref].pos.u;
      refy = ehid_window->globalpoints->data[idref].pos.v;
      refdx = ehid_window->globalpoints->data[idref].dir.u;
      refdy = ehid_window->globalpoints->data[idref].dir.v;
      listn[idref].idref = idref;

      for (idcur = 0; idcur < nbpts; idcur++)
      {
         if (idref == idcur) continue;

         curx = ehid_window->globalpoints->data[idcur].pos.u;
         cury = ehid_window->globalpoints->data[idcur].pos.v;
         curdx = ehid_window->globalpoints->data[idcur].dir.u;
         curdy = ehid_window->globalpoints->data[idcur].dir.v;

         // Check squared distance
         dx = refx - curx;
         dy = refy - cury;
         distpos = dx*dx+dy*dy;
         if (distpos > 4.0) continue;

         // Check angular distance (cos(angle) comparison)
         distangle = refdx * curdx + refdy * curdy;
         if (distangle < 0.9848) continue; // == dist(angleref, anglecur) > 10 degrees [ cos(10*pi/180) = 0.9848 ]

         rox_dynvec_uint_append(listn[idref].idneighboors, &idcur);
      }
   }

   // Sort neighboor list by cardinal, beware that it obviously change list internal indices
   qsort(listn, nbpts, sizeof(struct Rox_Ehid_Neighboorhood_Struct), (int (*)(const void *, const void *))comparNeighboorlist);

   // Perform clustering
   countused = 0;
   countref = 0;
   for (idr = 0; idr < nbpts; idr++)
   {
      Rox_Uint nbneighboors, isused;
      Rox_Ehid_Neighboorhood_Struct * reflist;
      reflist = &listn[idr];
      idref = reflist->idref;
      nbneighboors = reflist->idneighboors->used;

      // If we go below minimal neighboordhood size, as it is sorted, we can safely exit
      if (nbneighboors < mincard)
      {
         break;
      }

      // If we clustered enough points, exit
      if (countused > maxused)
      {
         break;
      }

      // If the reference has already been used as a neighboor, loop
      if (reflist->used)
      {
         continue;
      }

      // Check that none of the neighboors have already been selected
      isused = 0;
      for (idn = 0; idn < nbneighboors; idn++)
      {
         idcur = reflist->idneighboors->data[idn];

         if (listn[idcur].used)
         {
            isused = 1;
         }
      }
      if (isused)
      {
         continue;
      }

      // Reset indices counters
      for (idi = 0; idi < 32; idi++)
      {
         indices[idi].index = idi;
         indices[idi].count = 0;
      }

      // Build description
      rox_ehid_description_from_bits_to_int(description_ref, ehid_window->globalpoints->data[idref].Description);

      indices[ehid_window->globalpoints->data[idref].index].count++;

      for (idn = 0; idn < nbneighboors; idn++)
      {
         idcur = reflist->idneighboors->data[idn];
         indices[ehid_window->globalpoints->data[idcur].index].count++;

         rox_ehid_description_from_bits_to_int(description_cur, ehid_window->globalpoints->data[idcur].Description);

         for (k = 0; k < 64; k++)
         {
            for (l = 0; l < 5; l++)
            {
               description_ref[k][l] += description_cur[k][l];
            }
         }
      }

      // Sort indices by use
      qsort(indices, 32, sizeof(Rox_Ehid_Index_Struct), (int (*)(const void *, const void *))comparIndexlist);

      // Compute statistics
      for (k = 0; k < 64; k++)
      {
         histsum = 0;

         for (l = 0; l < 5; l++)
         {
            histsum += description_ref[k][l];
         }

         for (l = 0; l < 5; l++)
         {
            if (description_ref[k][l] < ceil(0.05 * (Rox_Double)histsum))
            {
               description_ref[k][l] = 1;
            }
            else
            {
               description_ref[k][l] = 0;
            }
         }
      }

      // Creating new feature
      curpt.pos = ehid_window->globalpoints->data[idref].pos;
      curpt.dir = ehid_window->globalpoints->data[idref].dir;
      curpt.scale = ehid_window->globalpoints->data[idref].scale;
      rox_ehid_description_from_int_to_bits(curpt.Description, description_ref);

      // Append feature to db
      rox_dynvec_ehid_point_append(ehid_window->clusteredpoints, &curpt);

      // Store possible indices for features
      sumidx = 0;
      maxidx = (Rox_Uint) (nbneighboors * 0.8);

      for (idi = 0; idi < 32; idi++) dbidx.flag_indices[idi] = 0;

      for (idi = 0; idi < 32; idi++)
      {
         if (indices[idi].count == 0) break;
         dbidx.flag_indices[indices[idi].index] = 1;

         sumidx += indices[idi].count;
         if (sumidx >= maxidx) break;
      }

      rox_dynvec_ehid_dbindex_append(ehid_window->dbindices, &dbidx);

      // Set neighboorhood as used
      for (idn = 0; idn < nbneighboors; idn++)
      {
         idcur = reflist->idneighboors->data[idn];
         listn[idcur].used = 1;
         countused++;
      }

      // Set the reference as used
      reflist->used = 1;
      countused++;
      countref++;
   }

   // Delete lists inside
   for (idlist = 0; idlist < nbpts; idlist++)
   {
      rox_dynvec_uint_del(&listn[idlist].idneighboors);
   }
   rox_memory_delete(listn);

function_terminate:
   return error;
}
