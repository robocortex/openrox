//==============================================================================
// 
//    OPENROX   : File cluster.c
//
//    Contents  : Implementation of cluster module 
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "cluster.h"
#include "cluster_struct.h"

#include <string.h>

#include <generated/array2d_sint.h>

#include <system/memory/memory.h>

#include <baseproc/array/fill/fillval.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_cluster_new ( Rox_Cluster * cluster )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Cluster ret = NULL;
   
   if ( cluster == NULL ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   ret = (Rox_Cluster) rox_memory_allocate(sizeof(*ret), 1);
   if(ret == NULL) 
   {
      error = ROX_ERROR_NULL_POINTER; 
      ROX_ERROR_CHECK_TERMINATE(error)
   }
   
   ret->count = 0;
   ret->bounds = (Rox_Sint *) rox_memory_allocate(4*ROX_MAX_CLUSTER*sizeof(Rox_Sint), 1);
   if(ret->bounds == 0)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *cluster = ret;
   
function_terminate:
   if(error != ROX_ERROR_NONE) rox_cluster_del(&ret);
   return error;
}

Rox_Void rox_cluster_del ( Rox_Cluster * cluster )
{
   Rox_Cluster todel = NULL;
   
   if (cluster != 0)
   {
      todel = *cluster;
      *cluster = 0;
      
      if(todel != 0)
      {
         rox_memory_delete(todel->bounds);
         rox_memory_delete(todel);
      }
   }
}

Rox_ErrorCode rox_cluster_get_count ( Rox_Sint * count, Rox_Cluster cluster )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!count || !cluster)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *count = cluster->count;
   
function_terminate:
   return error;
}

Rox_Sint * rox_cluster_get_bounds ( Rox_Cluster cluster )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;   
   Rox_Sint * ret = NULL;
   
   if (!cluster || !cluster->bounds) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret = cluster->bounds;
   
function_terminate:
   return ret;
}

Rox_ErrorCode rox_cluster_binary ( Rox_Cluster cluster, Rox_Image source, Rox_Sint bandwidth[2] )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Sint count = 0;
   Rox_Sint rows = 0;
   Rox_Sint cols = 0;
   Rox_Float query[2];
   Rox_Bool neighbor_labled = 0;
   Rox_Sint mode = 0;
   
   Rox_Array2D_Sint modes = NULL;
   Rox_Sint **mode_map = NULL;
   Rox_Uchar **src_data = NULL;
   
   if (!cluster || !cluster->bounds || !source || !bandwidth)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_array2d_uchar_get_size(&rows, &cols, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_sint_new(&modes, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_sint_fillval(modes, 0); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_sint_get_data_pointer_to_pointer ( &mode_map, modes);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &src_data, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if(mode_map == 0 || src_data == 0) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // rox_array2d_uchar_save_pgm("debug_cluser.pgm", source);
   
   // Rasterize data in a grid with bucket size = bandwidth 
   // For each point iterate over the neighboring buckets to compute the mode 
   for ( Rox_Sint r = 0; r < rows; r++)
   {
      for ( Rox_Sint c = 0; c < cols; c++)
      {
         // Reset variables 
         mode = 0;
         neighbor_labled = 0;
         
         // No data point 
         if(src_data[r][c]==0) continue;
         
         // Is this already a mode? 
         if(mode_map[r][c]>0) continue;
         
         query[0] = (Rox_Float)r;
         query[1] = (Rox_Float)c;
         
         for(Rox_Sint rr = -1; rr <= 1; rr++)
         {
            for(Rox_Sint cc=-1;cc<=1; cc++)
            {
               if(r+rr<0 || r+rr>=rows) continue;
               if(c+cc<0 || c+cc>=cols) continue;
               
               mode = mode_map[r+rr][c+cc];
               if(mode != 0)
               {
                  query[0] = (Rox_Float)(r+rr);
                  query[1] = (Rox_Float)(c+cc);
                  neighbor_labled = 1;
                  break;
               }
            }
            if(mode != 0)break;
         }
         if(neighbor_labled == 0)
         {
            Rox_Bool iterate = 1;
            int loop = 0;
            while(iterate)
            {
               Rox_Float weights = 0.0f;
               Rox_Float massR = 0.0f;
               Rox_Float massC = 0.0f;
               
               Rox_Sint r_kernel_min = (Rox_Sint)(query[0]) - (Rox_Sint)bandwidth[0];
               Rox_Sint r_kernel_max = (Rox_Sint)(query[0]) + (Rox_Sint)bandwidth[0];
               Rox_Sint c_kernel_min = (Rox_Sint)(query[1]) - (Rox_Sint)bandwidth[1];
               Rox_Sint c_kernel_max = (Rox_Sint)(query[1]) + (Rox_Sint)bandwidth[1];
               
               r_kernel_min = (r_kernel_min<0?0:r_kernel_min);
               r_kernel_max = (r_kernel_max>=rows?rows-1:r_kernel_max);
               c_kernel_min = (c_kernel_min<0?0:c_kernel_min);
               c_kernel_max = (c_kernel_max>=cols?cols-1:c_kernel_max);
               
               // Calculate center of mass 
               for ( Rox_Sint rr=r_kernel_min; rr<r_kernel_max; rr++)
               {
                  for ( Rox_Sint cc=c_kernel_min; cc<c_kernel_max; cc++)
                  {
                     if(src_data[rr][cc] == 0) continue;
                     // Don't consider the query point itself 
                     if(rr==r && cc==c) continue;
                     
                     // Integrate weighted center of mass 
                     {
                        Rox_Float distSqr = ((Rox_Float)rr-query[0])*((Rox_Float)rr-query[0])/((Rox_Float)(bandwidth[0]*bandwidth[0])) +
                        ((Rox_Float)cc-query[1])*((Rox_Float)cc-query[1])/((Rox_Float)(bandwidth[1]*bandwidth[1]));
                        Rox_Float weight = ROX_MS_EPANECHNIKOV_KERNEL(distSqr);
                        weights+=weight;
                        
                        massR += (Rox_Float)rr*weight;
                        massC += (Rox_Float)cc*weight;
                     }
                  }
               }
               if(weights<1.0)
               {
                  iterate = 0;
                  continue;
               }
               
               massR /= weights;
               massC /= weights;
               
               // Shift query point 
               {
                  Rox_Float shift[2];
                  shift[0] = massR - query[0];
                  shift[1] = massC - query[1];
                  query[0] += shift[0];
                  query[1] += shift[1];
                  // Stop if shift movement is below a certain threshold 
                  {
                     Rox_Float lenSq = shift[0]*shift[0]+shift[1]*shift[1];
                     Rox_Float thresh = 1e-10f;
                     if(lenSq<thresh) iterate = 0;
                     if(loop++>250) iterate = 0;
                  }
               }
            } // while(iterate) 
            }
            
            // Write mode 
            {
               int modeR = (int)(query[0]+0.5f);
               int modeC = (int)(query[1]+0.5f);
               mode = mode_map[modeR][modeC];
               
               if(mode == 0)
               {
                  for(Rox_Sint rr = -1; rr <= 1; rr++)
                  {
                     for(Rox_Sint cc = -1; cc <=1; cc++)
                     {
                        if(modeR + rr < 0 || modeR + rr >= rows) continue;
                        if(modeC + cc < 0 || modeC + cc >= cols) continue;
                        mode = mode_map[modeR + rr][modeC + cc];
                        if(mode != 0)break;
                     }
                     if(mode != 0)break;
                  }
               }
               
               if(mode == 0)
               {
                  count++;
                  mode = count;
               }
               
               mode_map[r][c] = mode;
               mode_map[modeR][modeC] = mode;
            }
         }
      }
      
      if(count > ROX_MAX_CLUSTER) count = ROX_MAX_CLUSTER;
      
      // set upper bounds to zero 
      // and lower bounds to a high value 
      memset(cluster->bounds, 0, (Rox_Sint)(4*count)*sizeof(Rox_Sint));
      
      for ( Rox_Sint i=0; i<count; i++)
      {
         cluster->bounds[i*4] = 65535;
         cluster->bounds[i*4+1] = 65535;
      }
      
      // Determine bounds (rectangular) of cluster 
      cluster->count = (Rox_Sint) count;
      
      for ( Rox_Sint r = 0; r < rows; r++)
      {
         for ( Rox_Sint c = 0; c < cols; c++)
         {
            int cur_mode = mode_map[r][c];
            if ( cur_mode > 0 && (cluster->bounds != NULL) )
            {
               Rox_Sint* bounds = &cluster->bounds[4*(cur_mode-1)];               
               if (bounds != NULL)
               {
                  if (c < bounds[0]) bounds[0] = c;
                  if (r < bounds[1]) bounds[1] = r;
                  if (c > bounds[2]) bounds[2] = c;
                  if (r > bounds[3]) bounds[3] = r;
               }
               else
               {
                  error = ROX_ERROR_NULL_POINTER; 
                  ROX_ERROR_CHECK_TERMINATE ( error );
               }
            }
         }
      }
      
function_terminate:
      rox_array2d_sint_del(&modes);
      return error;
}