//==============================================================================
//
//    OPENROX   : File checkerboard_detect.c
//
//    Contents  : Implementation of checkerboard detect module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "checkerboard_detect.h"

#include <baseproc/maths/maths_macros.h>
#include <float.h>
#include <inout/system/errors_print.h>

#ifdef _MSC_VER
   #define isnan _isnan
#endif

#define MIN_NUMBER_CORNERS 10

Rox_ErrorCode rox_checkerboard_detector_new ( Rox_CheckerBoard_Detector * obj )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_CheckerBoard_Detector ret = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *obj = NULL;

   ret = (Rox_CheckerBoard_Detector) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->usedcorners    = NULL;
   ret->buffer_indices = NULL;
   ret->checkerboards  = NULL;

   error = rox_dynvec_uint_new(&ret->usedcorners, 100);

   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_dynvec_uint_new(&ret->buffer_indices, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_checkerboard_new(&ret->checkerboards, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *obj = ret;

function_terminate:
   if(error) rox_checkerboard_detector_del(&ret);
   return error;
}

Rox_ErrorCode rox_checkerboard_detector_del(Rox_CheckerBoard_Detector * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_CheckerBoard_Detector todel = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;


   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_objset_checkerboard_del(&todel->checkerboards);
   rox_dynvec_uint_del(&todel->buffer_indices);
   rox_dynvec_uint_del(&todel->usedcorners);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_checkerboard_detector_findclosest (
   Rox_Double * ret_dist,
   Rox_Uint * ret_id,
   Rox_CheckerBoard_Detector obj,
   Rox_CheckerCorner_Detector cdetect,
   Rox_Point2D_Double pt,
   Rox_Point2D_Double curv
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint idmin = -1;
   Rox_Double minval = DBL_MAX;

   for (Rox_Uint idpt = 0; idpt < cdetect->corners->used; idpt++)
   {
      if (obj->usedcorners->data[idpt]) continue;

      // Direction between current corner and references
      Rox_Double dirx = cdetect->corners->data[idpt].coords.u - pt->u;
      Rox_Double diry = cdetect->corners->data[idpt].coords.v - pt->v;
      Rox_Double dist = dirx * curv->u + diry * curv->v;

      if (dist < 0) continue;

      Rox_Double dx = dirx - curv->u * dist;
      Rox_Double dy = diry - curv->v * dist;
      Rox_Double distedge = sqrt(dx*dx+dy*dy);
      Rox_Double val = dist + 5.0 * distedge;

      if (val < minval)
      {
         minval = val;
         idmin = idpt;
      }
   }

   // Closest not found
   if (idmin < 0)
   { error = ROX_ERROR_ALGORITHM_FAILURE; if(error) goto function_terminate; } // ROX_ERROR_CHECK_TERMINATE(error)}

   obj->usedcorners->data[idmin] = 1;
   *ret_id = idmin;
   *ret_dist = minval;

function_terminate:
   return error;
}

Rox_ErrorCode rox_checkerboard_detector_findclosest_dist (
   Rox_Uint * ret_id,
   Rox_CheckerBoard_Detector obj,
   Rox_CheckerCorner_Detector cdetect,
   Rox_Point2D_Double pt
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint idmin, found;
   Rox_Double minval, dx, dy, dist;

   idmin = -1;
   minval = DBL_MAX;
   for (Rox_Uint idpt = 0; idpt < cdetect->corners->used; idpt++)
   {
      if (obj->usedcorners->data[idpt]) continue;

      found = 0;
      for (Rox_Uint idid = 0; idid < obj->buffer_indices->used; idid++)
      {
         if (idpt == obj->buffer_indices->data[idid])
         {
            found = 1;
         }
      }

      if (found) continue;

      // Direction between current corner and references
      dx = cdetect->corners->data[idpt].coords.u - pt->u;
      dy = cdetect->corners->data[idpt].coords.v - pt->v;
      dist = sqrt(dx*dx+dy*dy);
      //if (dist < 1.0) continue;

      if (dist < minval)
      {
         minval = dist;
         idmin = idpt;
      }
   }

   if (idmin < 0)
   { error = ROX_ERROR_ALGORITHM_FAILURE; if(error) goto function_terminate; } // ROX_ERROR_CHECK_TERMINATE(error)}

   *ret_id = idmin;

function_terminate:
   return error;
}

Rox_ErrorCode rox_checkerboard_detector_findseed (
   Rox_CheckerBoard checker,
   Rox_CheckerBoard_Detector obj,
   Rox_CheckerCorner_Detector cdetect,
   Rox_Uint idseed
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Point2D_Double_Struct pt, v1, v2, curv;
   Rox_Double distances[8];
   Rox_Double meandist, std;


   if (!obj || !cdetect || !checker) 
   {  error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (idseed >= cdetect->corners->used) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint ** seed = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer(&seed, checker->indices);

   Rox_Point2D_Double * pseed = NULL;
   error = rox_array2d_point2d_double_get_data_pointer_to_pointer ( &pseed, checker->points);

   // Reset flags
   for (Rox_Uint idpt = 0; idpt < cdetect->corners->used; idpt++)
   {
      obj->usedcorners->data[idpt] = 0;
   }

   // Set current point to used flag
   obj->usedcorners->data[idseed] = 1;
   seed[1][1] = idseed;

   pt = cdetect->corners->data[idseed].coords;
   v1 = cdetect->corners->data[idseed].edge1;
   v2 = cdetect->corners->data[idseed].edge2;

   curv.u = v1.u;
   curv.v = v1.v;
   error = rox_checkerboard_detector_findclosest(&distances[0], &seed[1][2], obj, cdetect, &pt, &curv);
   if(error) goto function_terminate; // ROX_ERROR_CHECK_TERMINATE ( error );

   curv.u = -v1.u;
   curv.v = -v1.v;
   error = rox_checkerboard_detector_findclosest(&distances[1], &seed[1][0], obj, cdetect, &pt, &curv);
   if(error) goto function_terminate; // ROX_ERROR_CHECK_TERMINATE ( error );

   curv.u = v2.u;
   curv.v = v2.v;
   error = rox_checkerboard_detector_findclosest(&distances[2], &seed[2][1], obj, cdetect, &pt, &curv);
   if(error) goto function_terminate; // ROX_ERROR_CHECK_TERMINATE ( error );

   curv.u = -v2.u;
   curv.v = -v2.v;
   error = rox_checkerboard_detector_findclosest(&distances[3], &seed[0][1], obj, cdetect, &pt, &curv);
   if(error) goto function_terminate; // ROX_ERROR_CHECK_TERMINATE ( error );

   curv.u = -v2.u;
   curv.v = -v2.v;
   pt = cdetect->corners->data[seed[1][0]].coords;
   error = rox_checkerboard_detector_findclosest(&distances[4], &seed[0][0], obj, cdetect, &pt, &curv);
   if(error) goto function_terminate; // ROX_ERROR_CHECK_TERMINATE ( error );

   curv.u = v2.u;
   curv.v = v2.v;
   pt = cdetect->corners->data[seed[1][0]].coords;
   error = rox_checkerboard_detector_findclosest(&distances[5], &seed[2][0], obj, cdetect, &pt, &curv);
   if(error) goto function_terminate; // ROX_ERROR_CHECK_TERMINATE ( error );

   curv.u = -v2.u;
   curv.v = -v2.v;
   pt = cdetect->corners->data[seed[1][2]].coords;
   error = rox_checkerboard_detector_findclosest(&distances[6], &seed[0][2], obj, cdetect, &pt, &curv);
   if(error) goto function_terminate; // ROX_ERROR_CHECK_TERMINATE ( error );

   curv.u = v2.u;
   curv.v = v2.v;
   pt = cdetect->corners->data[seed[1][2]].coords;
   error = rox_checkerboard_detector_findclosest(&distances[7], &seed[2][2], obj, cdetect, &pt, &curv);
   if(error) goto function_terminate; // ROX_ERROR_CHECK_TERMINATE ( error );

   meandist = (distances[0] + distances[2]) / 2.0;
   std = (distances[0] - meandist) * (distances[0] - meandist);
   std += (distances[2] - meandist) * (distances[2] - meandist);
   std = sqrt(std);
   std /= 1;
   if (std / meandist > 0.3) {error = ROX_ERROR_ALGORITHM_FAILURE; if(error) goto function_terminate;} // ROX_ERROR_CHECK_TERMINATE(error)}

   meandist = (distances[1] + distances[3]  + distances[4] + distances[5] + distances[6] + distances[7]) / 6.0;
   std = (distances[1] - meandist) * (distances[1] - meandist);
   std += (distances[3] - meandist) * (distances[3] - meandist);
   std += (distances[4] - meandist) * (distances[4] - meandist);
   std += (distances[5] - meandist) * (distances[5] - meandist);
   std += (distances[6] - meandist) * (distances[6] - meandist);
   std += (distances[7] - meandist) * (distances[7] - meandist);
   std /= 5;
   std = sqrt(std);
   if (std / meandist > 0.3) {error = ROX_ERROR_ALGORITHM_FAILURE; if(error) goto function_terminate;} // ROX_ERROR_CHECK_TERMINATE(error)}

   pseed[0][0] = cdetect->corners->data[seed[0][0]].coords;
   pseed[0][1] = cdetect->corners->data[seed[0][1]].coords;
   pseed[0][2] = cdetect->corners->data[seed[0][2]].coords;
   pseed[1][0] = cdetect->corners->data[seed[1][0]].coords;
   pseed[1][1] = cdetect->corners->data[seed[1][1]].coords;
   pseed[1][2] = cdetect->corners->data[seed[1][2]].coords;
   pseed[2][0] = cdetect->corners->data[seed[2][0]].coords;
   pseed[2][1] = cdetect->corners->data[seed[2][1]].coords;
   pseed[2][2] = cdetect->corners->data[seed[2][2]].coords;

function_terminate:
   return error;
}

Rox_ErrorCode rox_checkerboard_detector_predict (
   Rox_Point2D_Double res,
   Rox_Point2D_Double p1,
   Rox_Point2D_Double p2,
   Rox_Point2D_Double p3
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Double_Struct v1, v2;


   if (!res) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   v1.u = p2->u - p1->u;
   v1.v = p2->v - p1->v;
   v2.u = p3->u - p2->u;
   v2.v = p3->v - p2->v;

   Rox_Double a1 = atan2(v1.v, v1.u);
   Rox_Double a2 = atan2(v2.v, v2.u);
   Rox_Double a3 = 2.0 * a2 - a1;

   Rox_Double s1 = sqrt(v1.u * v1.u + v1.v * v1.v);
   Rox_Double s2 = sqrt(v2.u * v2.u + v2.v * v2.v);
   Rox_Double s3 = 2.0 * s2 - s1;

   res->u = p3->u + 0.75 * s3 * cos(a3);
   res->v = p3->v + 0.75 * s3 * sin(a3);

function_terminate:
   return error;
}

Rox_ErrorCode rox_checkerboard_detector_grow (
   Rox_CheckerBoard_Detector obj, 
   Rox_CheckerCorner_Detector cdetect, 
   Rox_CheckerBoard checker
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Uint id, idmin;
   Rox_Point2D_Double_Struct p1, p2, p3, pred;
   Rox_Point2D_Double *ddst = NULL;
   Rox_Uint ** dis = NULL, ** did = NULL;
   Rox_CheckerBoard tempchecker[4];
   Rox_Double min;

   if (!obj || !cdetect || !checker) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!checker->points) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0; 
   error = rox_array2d_point2d_double_get_size ( &rows, &cols, checker->points ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point2D_Double * dp = NULL;
   error = rox_array2d_point2d_double_get_data_pointer_to_pointer ( &dp, checker->points );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_get_data_pointer_to_pointer(&dis, checker->indices);
   ROX_ERROR_CHECK_TERMINATE ( error );

   tempchecker[0] = NULL;
   tempchecker[1] = NULL;
   tempchecker[2] = NULL;
   tempchecker[3] = NULL;

   // Create 4 putative checkerboards with 4 sides growing
   error = rox_checkerboard_new(&tempchecker[0]);
   if (tempchecker[0] == NULL) error = ROX_ERROR_NULL_POINTER;
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_checkerboard_new(&tempchecker[1]);
   if (tempchecker[1] == NULL) error = ROX_ERROR_NULL_POINTER;
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_checkerboard_new(&tempchecker[2]);
   if (tempchecker[2] == NULL) error = ROX_ERROR_NULL_POINTER;
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_checkerboard_new(&tempchecker[3]);
   if (tempchecker[3] == NULL) error = ROX_ERROR_NULL_POINTER;
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_checkerboard_set_size(tempchecker[0], rows, cols + 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_checkerboard_set_size(tempchecker[1], rows, cols + 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_checkerboard_set_size(tempchecker[2], rows + 1, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_checkerboard_set_size(tempchecker[3], rows + 1, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   tempchecker[0]->energy = DBL_MAX;
   tempchecker[1]->energy = DBL_MAX;
   tempchecker[2]->energy = DBL_MAX;
   tempchecker[3]->energy = DBL_MAX;

   for (Rox_Uint i = 0; i < obj->usedcorners->used; i++)
   {
      obj->usedcorners->data[i] = 0;
   }

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         obj->usedcorners->data[dis[i][j]] = 1;
      }
   }

   // Extand on the right border
   rox_dynvec_uint_reset(obj->buffer_indices);
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      p1 = dp[i][cols - 3];
      p2 = dp[i][cols - 2];
      p3 = dp[i][cols - 1];

      error = rox_checkerboard_detector_predict(&pred, &p1, &p2, &p3);
      if (error) break;

      error = rox_checkerboard_detector_findclosest_dist(&id, obj, cdetect, &pred);
      if (error) break;

      rox_dynvec_uint_append(obj->buffer_indices, &id);
   }

   if (obj->buffer_indices->used == rows)
   {
      error = rox_array2d_uint_get_data_pointer_to_pointer ( &did, tempchecker[0]->indices);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_point2d_double_get_data_pointer_to_pointer ( &ddst, tempchecker[0]->points);
      ROX_ERROR_CHECK_TERMINATE ( error );

      for ( Rox_Sint i = 0; i < rows; i++)
      {
         for ( Rox_Sint j = 0; j < cols; j++)
         {
            ddst[i][j] = dp[i][j];
            did[i][j] = dis[i][j];
         }

         did[i][cols] = obj->buffer_indices->data[i];
         ddst[i][cols] = cdetect->corners->data[obj->buffer_indices->data[i]].coords;
      }

      error = rox_checkerboard_compute_energy(tempchecker[0]);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Extand on the left border
   rox_dynvec_uint_reset(obj->buffer_indices);
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      p1 = dp[i][2];
      p2 = dp[i][1];
      p3 = dp[i][0];

      error = rox_checkerboard_detector_predict(&pred, &p1, &p2, &p3);
      if (error) break;

      error = rox_checkerboard_detector_findclosest_dist(&id, obj, cdetect, &pred);
      if (error) break;

      rox_dynvec_uint_append(obj->buffer_indices, &id);
   }

   if (obj->buffer_indices->used == rows)
   {
      error = rox_array2d_uint_get_data_pointer_to_pointer ( &did, tempchecker[1]->indices );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_point2d_double_get_data_pointer_to_pointer ( &ddst, tempchecker[1]->points );
      ROX_ERROR_CHECK_TERMINATE ( error );

      for ( Rox_Sint i = 0; i < rows; i++)
      {
         for ( Rox_Sint j = 0; j < cols; j++)
         {
            ddst[i][j + 1] = dp[i][j];
            did[i][j + 1] = dis[i][j];
         }

         did[i][0] = obj->buffer_indices->data[i];
         ddst[i][0] = cdetect->corners->data[obj->buffer_indices->data[i]].coords;
      }

      error = rox_checkerboard_compute_energy(tempchecker[1]);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Extand on the top border
   rox_dynvec_uint_reset(obj->buffer_indices);
   for ( Rox_Sint j = 0; j < cols; j++)
   {
      p1 = dp[2][j];
      p2 = dp[1][j];
      p3 = dp[0][j];

      error = rox_checkerboard_detector_predict ( &pred, &p1, &p2, &p3 );
      if (error) break;

      error = rox_checkerboard_detector_findclosest_dist ( &id, obj, cdetect, &pred );
      if (error) break;

      rox_dynvec_uint_append(obj->buffer_indices, &id);
   }

   if (obj->buffer_indices->used == cols)
   {
      error = rox_array2d_uint_get_data_pointer_to_pointer ( &did, tempchecker[2]->indices);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_point2d_double_get_data_pointer_to_pointer ( &ddst, tempchecker[2]->points);
      ROX_ERROR_CHECK_TERMINATE ( error );

      for ( Rox_Sint j = 0; j < cols; j++)
      {
         for ( Rox_Sint i = 0; i < rows; i++)
         {
            ddst[i + 1][j] = dp[i][j];
            did[i+1][j] = dis[i][j];
         }

         did[0][j] = obj->buffer_indices->data[j];
         ddst[0][j] = cdetect->corners->data[obj->buffer_indices->data[j]].coords;
      }

      error = rox_checkerboard_compute_energy(tempchecker[2]);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Extend on the bottom border
   rox_dynvec_uint_reset(obj->buffer_indices);
   for ( Rox_Sint j = 0; j < cols; j++)
   {
      p1 = dp[rows - 3][j];
      p2 = dp[rows - 2][j];
      p3 = dp[rows - 1][j];

      error = rox_checkerboard_detector_predict(&pred, &p1, &p2, &p3);
      if (error) break;

      error = rox_checkerboard_detector_findclosest_dist(&id, obj, cdetect, &pred);
      if (error) break;

      rox_dynvec_uint_append(obj->buffer_indices, &id);
   }

   if (obj->buffer_indices->used == cols)
   {
      error = rox_array2d_uint_get_data_pointer_to_pointer ( &did, tempchecker[3]->indices);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_point2d_double_get_data_pointer_to_pointer ( &ddst, tempchecker[3]->points);
      ROX_ERROR_CHECK_TERMINATE ( error );

      for ( Rox_Sint j = 0; j < cols; j++)
      {
         for ( Rox_Sint i = 0; i < rows; i++)
         {
            ddst[i][j] = dp[i][j];
            did[i][j] = dis[i][j];
         }

         did[rows][j] = obj->buffer_indices->data[j];
         ddst[rows][j] = cdetect->corners->data[obj->buffer_indices->data[j]].coords;
      }

      error = rox_checkerboard_compute_energy(tempchecker[3]);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   if (isnan(tempchecker[0]->energy)) tempchecker[0]->energy = DBL_MAX;
   if (isnan(tempchecker[1]->energy)) tempchecker[1]->energy = DBL_MAX;
   if (isnan(tempchecker[2]->energy)) tempchecker[2]->energy = DBL_MAX;
   if (isnan(tempchecker[3]->energy)) tempchecker[3]->energy = DBL_MAX;

   idmin = 0;
   min = tempchecker[0]->energy;
   if (tempchecker[1]->energy < min)
   {
      min = tempchecker[1]->energy;
      idmin = 1;
   }
   if (tempchecker[2]->energy < min)
   {
      min = tempchecker[2]->energy;
      idmin = 2;
   }
   if (tempchecker[3]->energy < min)
   {
      min = tempchecker[3]->energy;
      idmin = 3;
   }

   if (min > checker->energy)
   {
      error = ROX_ERROR_TOO_LARGE_VALUE;
      // ROX_ERROR_CHECK_TERMINATE(error)
      goto function_terminate;
   }

   error = rox_array2d_point2d_double_get_size (&rows, &cols, tempchecker[idmin]->points); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_checkerboard_set_size ( checker, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_point2d_double_get_data_pointer_to_pointer ( &ddst, checker->points);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_point2d_double_get_data_pointer_to_pointer ( &dp, tempchecker[idmin]->points);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_get_data_pointer_to_pointer (&did, tempchecker[idmin]->indices);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_get_data_pointer_to_pointer (&dis, checker->indices);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         ddst[i][j] = dp[i][j];
         dis[i][j] = did[i][j];
      }
   }

   checker->energy = tempchecker[idmin]->energy;

   error = ROX_ERROR_NONE;




function_terminate:

   rox_checkerboard_del ( &tempchecker[0] );
   rox_checkerboard_del ( &tempchecker[1] );
   rox_checkerboard_del ( &tempchecker[2] );
   rox_checkerboard_del ( &tempchecker[3] );

   return error;
}

Rox_ErrorCode rox_checkerboard_detector_process (
   Rox_CheckerBoard_Detector obj,
   Rox_CheckerCorner_Detector cdetect
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_CheckerBoard checker = NULL;
   Rox_Uint overlap = 0, toremove = 0;

   if (!obj || !cdetect) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

	//  Threshold on number of detected corners 
   if (cdetect->corners->used < MIN_NUMBER_CORNERS) 
   { error = ROX_ERROR_INSUFFICIENT_DATA; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_objset_checkerboard_reset(obj->checkerboards);

   error = rox_dynvec_uint_usecells ( obj->usedcorners, cdetect->corners->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_checkerboard_new ( &checker );

   ROX_ERROR_CHECK_TERMINATE ( error );

   //  Loop over points
   for (Rox_Uint idseed = 0; idseed < cdetect->corners->used; idseed++)
   {
      error = rox_checkerboard_set_size(checker, 3, 3);
      ROX_ERROR_CHECK_TERMINATE ( error );

      //  Find seed for this point
      error = rox_checkerboard_detector_findseed (checker, obj, cdetect, idseed);
      if (error) continue;

      //  Check energy
      error = rox_checkerboard_compute_energy(checker);
      if (error) continue;

      if (checker->energy > 0) continue;

      //  Expansion phase
      while (1)
      {
         error = rox_checkerboard_compute_energy ( checker );
         if (error) continue;

         error = rox_checkerboard_detector_grow ( obj, cdetect, checker );
         if (error) break;
      }

      if (checker->energy > -10.0) continue;

      Rox_Uint ** d1 = NULL;
      error = rox_array2d_uint_get_data_pointer_to_pointer(&d1, checker->indices);
      ROX_ERROR_CHECK_TERMINATE ( error );

      Rox_Sint w1 = 0, h1 = 0;
      error = rox_array2d_uint_get_size ( &h1, &w1, checker->indices );
      ROX_ERROR_CHECK_TERMINATE ( error );

      toremove = 0;
      for (Rox_Uint idchecker = 0; idchecker < obj->checkerboards->used; idchecker++)
      {
         Rox_Uint ** d2 = NULL;
         error = rox_array2d_uint_get_data_pointer_to_pointer(&d2, obj->checkerboards->data[idchecker]->indices);
         ROX_ERROR_CHECK_TERMINATE ( error );

         Rox_Sint w2 = 0, h2 = 0;
         error = rox_array2d_uint_get_size(&h2, &w2, obj->checkerboards->data[idchecker]->indices);
         ROX_ERROR_CHECK_TERMINATE ( error );

         overlap = 0;
         for (Rox_Sint i1 = 0; i1 < h1 && !overlap; i1++)
         {
            for (Rox_Sint j1 = 0; j1 < w1 && !overlap; j1++)
            {
               for (Rox_Sint i2 = 0; i2 < h2 && !overlap; i2++)
               {
                  for (Rox_Sint j2 = 0; j2 < w2 && !overlap; j2++)
                  {
                     if (d1[i1][j1] == d2[i2][j2])
                     {
                        overlap = 1;
                     }
                  }
               }
            }
         }

         if (overlap)
         {
            if (checker->energy < obj->checkerboards->data[idchecker]->energy)
            {
               obj->checkerboards->data[idchecker]->energy = DBL_MAX;
            }
            else
            {
               toremove = 1;
            }
         }
      }

      if ( !toremove )
      {
         error = rox_objset_checkerboard_append(obj->checkerboards, checker);
         ROX_ERROR_CHECK_TERMINATE ( error );
         checker = NULL;

         error = rox_checkerboard_new ( &checker );
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
  }

   // Do not delete the following error set unless you know what you are doing
   error = ROX_ERROR_NONE;

function_terminate:
   rox_checkerboard_del(&checker);
   return error;
}
