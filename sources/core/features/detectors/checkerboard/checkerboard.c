//==============================================================================
//
//    OPENROX   : File checkerboard.c
//
//    Contents  : Implementation of checkerboard module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "checkerboard.h"

#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/transforms/matsl3/sl3from4points.h>
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/array/median/median.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_checkerboard_new(Rox_CheckerBoard * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_CheckerBoard ret = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_CheckerBoard) rox_memory_allocate(sizeof(*ret), 1);

   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->points = NULL;
   ret->indices = NULL;
   ret->energy = 0;

   *obj = ret;

function_terminate:
   return error;
}

Rox_ErrorCode rox_checkerboard_del(Rox_CheckerBoard * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_CheckerBoard todel = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;


   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_array2d_uint_del(&todel->indices);
   rox_array2d_point2d_double_del(&todel->points);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_checkerboard_set_size (
   Rox_CheckerBoard obj, 
   const Rox_Sint rows, 
   const Rox_Sint cols
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (obj->points)
   {
      // Check if the size is already set correctly
      error = rox_array2d_point2d_double_check_size ( obj->points, rows, cols );
      if (!error) { error = ROX_ERROR_NONE; goto function_terminate; }
   }

   // Delete points
   if(obj->points) rox_array2d_point2d_double_del(&obj->points);

   error = rox_array2d_point2d_double_new ( &obj->points, rows, cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if(obj->indices) rox_array2d_uint_del(&obj->indices);

   error = rox_array2d_uint_new ( &obj->indices, rows, cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_checkerboard_compute_energy(Rox_CheckerBoard obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!obj->points) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Point2D_Double * pts = NULL;
   error = rox_array2d_point2d_double_get_data_pointer_to_pointer ( &pts, obj->points );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_point2d_double_get_size(&rows, &cols, obj->points); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ec = -((double) cols * (double) rows);
   Rox_Double es = 0.0;

   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols - 2; j++)
      {
         Rox_Double dx = pts[i][j].u + pts[i][j+2].u - 2.0 * pts[i][j+1].u;
         Rox_Double dy = pts[i][j].v + pts[i][j+2].v - 2.0 * pts[i][j+1].v;
         Rox_Double dist = sqrt(dx*dx+dy*dy);

         dx = pts[i][j].u - pts[i][j+2].u;
         dy = pts[i][j].v - pts[i][j+2].v;
         dist /= sqrt(dx*dx+dy*dy);

         es = ROX_MAX(es, dist);
      }
   }

   for (Rox_Sint j = 0; j < cols; j++)
   {
      for (Rox_Sint i = 0; i < rows - 2; i++)
      {
         Rox_Double dx = pts[i][j].u + pts[i+2][j].u - 2.0 * pts[i+1][j].u;
         Rox_Double dy = pts[i][j].v + pts[i+2][j].v - 2.0 * pts[i+1][j].v;
         Rox_Double dist = sqrt(dx*dx+dy*dy);

         dx = pts[i][j].u - pts[i+2][j].u;
         dy = pts[i][j].v - pts[i+2][j].v;
         dist /= sqrt(dx*dx+dy*dy);

         es = ROX_MAX(es, dist);
      }
   }

   obj->energy = ec + cols * rows * es;

function_terminate:
   return error;
}

Rox_ErrorCode rox_checkerboard_check_order(Rox_CheckerBoard obj, Rox_Image source)
{
   Rox_ErrorCode                error = ROX_ERROR_NONE;

   Rox_Uint                     pos;
   Rox_Sint                     ix, iy;
   Rox_Array2D_Double           H=NULL;
   Rox_Array2D_Double           centers=NULL;
   Rox_Array2D_Double           buffer9=NULL;
   Rox_Array2D_Point2D_Double   newpts=NULL, swappts=NULL;
   Rox_Point2D_Double_Struct    src[4];
   Rox_Point2D_Double_Struct    dst[4];
   Rox_Double                   median, c1, c2, c3, c4;

   
   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint iw = 0, ih = 0;
   error = rox_array2d_uchar_get_size(&ih, &iw, source); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_point2d_double_get_size(&rows, &cols, obj->points);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point2D_Double * pt = NULL;
   error = rox_array2d_point2d_double_get_data_pointer_to_pointer ( &pt, obj->points );
   ROX_ERROR_CHECK_TERMINATE ( error );


   error = rox_array2d_double_new(&buffer9, 9, 1);                  
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&centers, rows - 1, cols - 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&H, 3, 3);                        
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double  ** dh  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dh, H);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double  ** dc  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dc, centers);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double  ** db9 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&db9, buffer9);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** di  = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer(&di, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute centers
   for ( Rox_Sint i = 0; i < rows - 1; i++)
   {
      for ( Rox_Sint j = 0; j < cols - 1; j++)
      {
         src[0].u = 0;     // 0=(0,0)     1=(16,0)
         src[0].v = 0;     //    *--------*
         src[1].u = 16;    //    |        |
         src[1].v = 0;     //    |        |
         src[2].u = 16;    //    |        |
         src[2].v = 16;    //    |        |
         src[3].u = 0;     //    *--------*
         src[3].v = 16;    // 3=(0,16)    2=(16,16)

         dst[0].u = pt[i][j].u;         // (i,j)        (i,j+1)
         dst[0].v = pt[i][j].v;         //    *--------*
         dst[1].u = pt[i][j + 1].u;     //    |        |
         dst[1].v = pt[i][j + 1].v;     //    |        |
         dst[2].u = pt[i + 1][j + 1].u; //    |        |
         dst[2].v = pt[i + 1][j + 1].v; //    |        |
         dst[3].u = pt[i + 1][j].u;     //    *--------*
         dst[3].v = pt[i + 1][j].v;     // (i+1,j)      (i+1,j+1)

         error = rox_matsl3_from_4_points_double(H, src, dst);
         ROX_ERROR_CHECK_TERMINATE(error)

         // Evaluate median value for each center
         pos = 0;
         for (Rox_Sint k = -1; k<= 1; k++)
         {
            for (Rox_Sint l = -1; l <= 1; l++)
            {
               Rox_Double x = dh[0][0]*( 8.0 + (double)k ) + dh[0][1]*( 8.0 + (double)l ) + dh[0][2];
               Rox_Double y = dh[1][0]*( 8.0 + (double)k ) + dh[1][1]*( 8.0 + (double)l ) + dh[1][2];
               Rox_Double w = dh[2][0]*( 8.0 + (double)k ) + dh[2][1]*( 8.0 + (double)l ) + dh[2][2];

               if (fabs(w) > DBL_EPSILON)
               {
                  x = x/w;
                  y = y/w;
               }

               ix = (int) x;
               iy = (int) y;

               if (ix < 0 || ix >= iw) goto function_terminate;
               if (iy < 0 || iy >= ih) goto function_terminate;

               db9[pos][0] = di[iy][ix];
               pos++;
            }
         }

         error = rox_array2d_double_median(&median, buffer9);
         ROX_ERROR_CHECK_TERMINATE(error)

         dc[i][j] = median;
      }
   }

#if 0
   c1 = 0.25 * ( dc[0][0]                  + dc[0][1]                  + dc[1][0]                  + dc[1][1]                  );
   c2 = 0.25 * ( dc[0][cols - 3]          + dc[0][cols - 2]          + dc[1][cols - 2]          + dc[1][cols - 3]          );
   c3 = 0.25 * ( dc[rows - 3][cols - 3] + dc[rows - 3][cols - 2] + dc[rows - 2][cols - 2] + dc[rows - 2][cols - 3] );
   c4 = 0.25 * ( dc[rows - 3][0]         + dc[rows - 3][1]         + dc[rows - 2][0]         + dc[rows - 2][1]         );
#endif

   // Find the darker corner of the grid
   // this way should prove to be more robust to non-uniform lighting
   c1 = fabs( dc[0][0] - dc[0][1] )
      + fabs( dc[1][0] - dc[1][1] );
   c2 = fabs( dc[0][cols - 3] - dc[0][cols - 2] )
      + fabs( dc[1][cols - 2] - dc[1][cols - 3] );
   c3 = fabs( dc[rows - 3][cols - 3] - dc[rows - 3][cols - 2] )
      + fabs( dc[rows - 2][cols - 2] - dc[rows - 2][cols - 3] );
   c4 = fabs( dc[rows - 3][0] - dc[rows - 3][1] )
      + fabs( dc[rows - 2][0] - dc[rows - 2][1] );

   if (c1 < c2 && c1 < c3 && c1 < c4)
   {
      // good order
      error = rox_array2d_point2d_double_new(&newpts, rows, cols);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_point2d_double_copy(newpts, obj->points);

      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else if (c2 < c1 && c2 < c3 && c2 < c4)
   {
      // Top right
      error = rox_array2d_point2d_double_new(&newpts, cols, rows);
      ROX_ERROR_CHECK_TERMINATE ( error );

      Rox_Point2D_Double * npts = NULL;
      error = rox_array2d_point2d_double_get_data_pointer_to_pointer ( &npts, newpts );
      ROX_ERROR_CHECK_TERMINATE ( error );

      for ( Rox_Sint i = 0; i < rows; i++)
      {
         for ( Rox_Sint j = 0; j < cols; j++)
         {
            npts[cols - j - 1][i] = pt[i][j];
         }
      }
   }
   else if (c3 < c1 && c3 < c2 && c3 < c4)
   {
      // bottom right
      error = rox_array2d_point2d_double_new(&newpts, rows, cols);
      ROX_ERROR_CHECK_TERMINATE ( error );

      Rox_Point2D_Double * npts = NULL;
      error = rox_array2d_point2d_double_get_data_pointer_to_pointer ( &npts, newpts );
      ROX_ERROR_CHECK_TERMINATE ( error );

      for ( Rox_Sint i = 0; i < rows; i++)
      {
         for ( Rox_Sint j = 0; j < cols; j++)
         {
            npts[rows - i - 1][cols - j - 1] = pt[i][j];
         }
      }
   }
   else if (c4 < c1 && c4 < c2 && c4 < c3)
   {
      // bottom left

      error = rox_array2d_point2d_double_new(&newpts, cols, rows);

      ROX_ERROR_CHECK_TERMINATE ( error );

      Rox_Point2D_Double * npts = NULL;
      error = rox_array2d_point2d_double_get_data_pointer_to_pointer ( &npts, newpts );
      ROX_ERROR_CHECK_TERMINATE ( error );

      for ( Rox_Sint i = 0; i < rows; i++)
      {
         for ( Rox_Sint j = 0; j < cols; j++)
         {
            npts[j][rows - i - 1] = pt[i][j];
         }
      }
   }
   else
   {
      error = ROX_ERROR_ALGORITHM_FAILURE;
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   swappts     = newpts;
   newpts      = obj->points;
   obj->points = swappts;

function_terminate:
   rox_array2d_point2d_double_del(&newpts);
   rox_array2d_double_del(&H);
   rox_array2d_double_del(&centers);
   rox_array2d_double_del(&buffer9);

   return error;
}
