//==============================================================================
//
//    OPENROX   : File tracking_point_9x9.c
//
//    Contents  : Implementation of tracking_point_9x9 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "tracking_point_9x9.h"

#include <float.h>

#include <baseproc/array/add/add.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/scale/scaleshift.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/solve/symm3x3solve.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/crosscor/zncrosscor.h>
#include <baseproc/array/conversion/array2d_float_from_uchar.h>
#include <baseproc/array/conversion/array2d_uchar_from_float.h>
#include <baseproc/calculus/linsys/linsys_texture_rxry_light_affine.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>

#include <baseproc/image/gradient/basegradient.h>
#include <baseproc/image/remap/remap_bilinear_nomask_uchar_to_uchar/remap_bilinear_nomask_uchar_to_uchar.h>
#include <baseproc/image/remap/remap_bilinear_trans/remap_bilinear_trans.h>
#include <baseproc/maths/maths_macros.h>

#include <inout/system/errors_print.h>

//! To be commented
struct Rox_Tracking_Point_9x9_Struct
{
   //! To be commented
   Rox_Double pos_x;
   //! To be commented
   Rox_Double pos_y;
   //! To be commented
   Rox_Double current_pos_x;
    //! To be commented
   Rox_Double current_pos_y;
   //! To be commented
   Rox_Sint size;

   //! To be commented
   Rox_Float luminosity_beta;

   //! To be commented
   Rox_Array2D_Double refH;
    //! To be commented
   Rox_Array2D_Double invrefH;

   //! The meshgrid to use for image warping
   Rox_MeshGrid2D_Float grid;

    //! To be commented
   Rox_Imask curmask;
   //! To be commented
   Rox_Imask gradientmask;

   //! To be commented
   Rox_Image reference_uchar;
    //! To be commented
   Rox_Image reference_uchar_noborder;
    //! To be commented
   Rox_Array2D_Float reference;
   //! To be commented
   Rox_Array2D_Float current;
   //! To be commented
   Rox_Array2D_Float currentlum;
    //! To be commented
   Rox_Array2D_Float meanlum;
    //! To be commented
   Rox_Array2D_Float diff;
    //! To be commented
   Rox_Array2D_Float gradientx;
    //! To be commented
   Rox_Array2D_Float gradienty;

    //! To be commented
   Rox_Array2D_Double JtJ;
    //! To be commented
   Rox_Array2D_Double iJtJ;
    //! To be commented
   Rox_Array2D_Double Jtf;
   //! To be commented
   Rox_Array2D_Double solsys;
};

Rox_ErrorCode rox_tracking_point_9x9_new(Rox_Tracking_Point_9x9 * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tracking_Point_9x9 ret = NULL;
   Rox_Uint bigsize, size = 9;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   if (size % 2 != 1)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret = (Rox_Tracking_Point_9x9) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->luminosity_beta = 0.0f;
   ret->pos_x = 0.0;
   ret->pos_y = 0.0;
   ret->size = size;
   ret->refH = 0;
   ret->invrefH = 0;
   ret->grid = 0;
   ret->reference_uchar = 0;
   ret->reference_uchar_noborder = 0;
   ret->reference = 0;
   ret->current = 0;
   ret->currentlum = 0;
   ret->curmask = 0;
   ret->diff = 0;
   ret->meanlum = 0;
   ret->gradientx = 0;
   ret->gradientmask = 0;
   ret->JtJ = 0;
   ret->iJtJ = 0;
   ret->Jtf = 0;
   ret->solsys = 0;

   bigsize = size + 2;

   error = rox_array2d_float_new(&ret->reference, bigsize, bigsize);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_uchar_new(&ret->reference_uchar, bigsize, bigsize);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_meshgrid2d_float_new(&ret->grid, bigsize, bigsize);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_float_new(&ret->current, bigsize, bigsize);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_float_new(&ret->currentlum, bigsize, bigsize);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_uint_new(&ret->curmask, bigsize, bigsize);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_float_new(&ret->diff, bigsize, bigsize);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_float_new(&ret->meanlum, bigsize, bigsize);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_float_new(&ret->gradientx, bigsize, bigsize);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_float_new(&ret->gradienty, bigsize, bigsize);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_uint_new(&ret->gradientmask, bigsize, bigsize);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&ret->JtJ, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&ret->iJtJ, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&ret->Jtf, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&ret->solsys, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&ret->refH, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&ret->invrefH, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_uchar_new_subarray2d(&ret->reference_uchar_noborder, ret->reference_uchar, 1, 1, size, size);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (error)
   {
      rox_tracking_point_9x9_del(&ret);
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   }

   *obj = ret;

function_terminate:
   if (error) rox_tracking_point_9x9_del(&ret);
   return error;
}

Rox_ErrorCode rox_tracking_point_9x9_del(Rox_Tracking_Point_9x9 * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tracking_Point_9x9 todel;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   todel = *obj;
   *obj = 0;

   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   rox_array2d_double_del(&todel->JtJ);
   rox_array2d_double_del(&todel->iJtJ);
   rox_array2d_double_del(&todel->Jtf);
   rox_array2d_double_del(&todel->solsys);
   rox_array2d_float_del(&todel->reference);
   rox_array2d_uchar_del(&todel->reference_uchar);
   rox_array2d_uchar_del(&todel->reference_uchar_noborder);
   rox_array2d_float_del(&todel->current);
   rox_array2d_float_del(&todel->currentlum);
   rox_array2d_float_del(&todel->diff);
   rox_array2d_float_del(&todel->meanlum);
   rox_array2d_float_del(&todel->gradientx);
   rox_array2d_float_del(&todel->gradienty);
   rox_array2d_uint_del(&todel->curmask);
   rox_array2d_uint_del(&todel->gradientmask);
   rox_array2d_double_del(&todel->refH);
   rox_array2d_double_del(&todel->invrefH);
   rox_meshgrid2d_float_del(&todel->grid);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_point_9x9_set_reference(Rox_Tracking_Point_9x9 obj, Rox_Image source, Rox_Double pos_x, Rox_Double pos_y)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint bigsize = obj->size + 2;

   //A point patch must be fully inside source bounds
   if (pos_x < 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (pos_y < 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0, cols = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (pos_y + bigsize - 1 > rows) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (pos_x + bigsize - 1 > cols) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_remap_bilinear_trans_uchar_to_float(obj->reference, obj->curmask, source, (Rox_Float) (pos_x - 1), (Rox_Float) (pos_y - 1));
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_from_float(obj->reference_uchar, obj->reference);
   ROX_ERROR_CHECK_TERMINATE ( error );

   obj->luminosity_beta = 0.0f;
   obj->pos_x = pos_x;
   obj->pos_y = pos_y;
   obj->current_pos_x = pos_x;
   obj->current_pos_y = pos_y;

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_point_9x9_set_warped_reference(Rox_Tracking_Point_9x9 obj, Rox_Image source, Rox_Double pos_u, Rox_Double pos_v, Rox_Double Z, Rox_Array2D_Double pose, Rox_Array2D_Double calibration_ref, Rox_Array2D_Double calibration_cur)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_transformtools_build_homography_pointpatch_front(obj->invrefH, pose, calibration_ref, calibration_cur, pos_u, pos_v, Z, obj->size + 2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Create patch
   rox_warp_grid_sl3_float(obj->grid, obj->invrefH);
   rox_remap_bilinear_nomask_uchar_to_uchar(obj->reference_uchar, source, obj->grid);
   rox_array2d_float_from_uchar(obj->reference, obj->reference_uchar);

   obj->luminosity_beta = 0.0f;
   obj->pos_x = 1;
   obj->pos_y = 1;
   obj->current_pos_x = 1;
   obj->current_pos_y = 1;

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_point_9x9_set_referencepatch(Rox_Tracking_Point_9x9 obj, Rox_Array2D_Float source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_float_copy(obj->reference, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_from_float(obj->reference_uchar, obj->reference);
   ROX_ERROR_CHECK_TERMINATE ( error );

   obj->luminosity_beta = 0.0f;
   obj->pos_x = 1;
   obj->pos_y = 1;
   obj->current_pos_x = 1;
   obj->current_pos_y = 1;

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_point_9x9_set_predicted_position(Rox_Tracking_Point_9x9 obj, Rox_Double pos_x, Rox_Double pos_y)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

  if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   obj->current_pos_x = pos_x;
   obj->current_pos_y = pos_y;

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_point_9x9_get_current(Rox_Double * current_x, Rox_Double * current_y, Rox_Tracking_Point_9x9 obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!current_x || !current_y || !obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   *current_x = obj->current_pos_x;
   *current_y = obj->current_pos_y;

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_point_9x9_get_reference_patch(Rox_Image ref, Rox_Tracking_Point_9x9 obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_uchar_check_size(ref, obj->size, obj->size);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_copy(ref, obj->reference_uchar_noborder);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_point_9x9_track(Rox_Tracking_Point_9x9 obj, Rox_Image current, Rox_Double minscore)
{
   Rox_Uint iter;
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double ** dsol, **djtj, *djtf, zncc;
   Rox_Uchar ** ds;
   Rox_Float  **dr;
   Rox_Float **dsum, **ddiff;
   Rox_Float **dc;
   Rox_Sint sheight, swidth;
   Rox_Sint ni, nj;
   Rox_Sint itx, ity;
   Rox_Double tx,ty,update;
   Rox_Float dx, dy, val;
   Rox_Float v1,v2,v3,v4;
   Rox_Float b1,b2,b3,b4;
   Rox_Float gx,gy,d;

   if (!obj || !current)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_get_data_pointer_to_pointer( &ds, current);
   error = rox_array2d_float_get_data_pointer_to_pointer( &dc, obj->current);
   error = rox_array2d_float_get_data_pointer_to_pointer( &dr, obj->reference);
   error = rox_array2d_double_get_data_pointer_to_pointer( &dsol, obj->solsys);
   error = rox_array2d_float_get_data_pointer_to_pointer( &dsum, obj->meanlum);
   error = rox_array2d_float_get_data_pointer_to_pointer( &ddiff, obj->diff);
   error = rox_array2d_double_get_data_pointer_to_pointer( &djtj, obj->JtJ);
   error = rox_array2d_double_get_data_pointer ( &djtf, obj->Jtf);

   error = rox_array2d_uchar_get_size(&sheight, &swidth, current);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (iter = 0; iter < 8; iter++)
   {
      tx = obj->current_pos_x - 1;
      ty = obj->current_pos_y - 1;
      itx = (Rox_Sint)tx;
      ity = (Rox_Sint)ty;
      dx = (Rox_Float) (tx - (Rox_Float)itx);
      dy = (Rox_Float) (ty - (Rox_Float)ity);
      b1 = (Rox_Float) ((1.0 - dx)*(1.0 - dy));
      b2 = (Rox_Float) (dx * (1.0 - dy));
      b3 = (Rox_Float) ((1.0 - dx) * dy);
      b4 = dx * dy;

      if (tx < 0 || ty < 0)
      { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

      if (itx + 11 >= swidth - 1 || ity + 11 >= sheight - 1)
      { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

      djtj[0][0] = 0;
      djtj[0][1] = 0;
      djtj[0][2] = 0;
      djtj[1][1] = 0;
      djtj[1][2] = 0;
      djtj[2][2] = 0;
      djtf[0] = 0;
      djtf[1] = 0;
      djtf[2] = 0;

      for (Rox_Sint i = 0; i < 11; i++)
      {
         ni = i + ity;

         for (Rox_Sint j = 0; j < 11; j++)
         {
            nj = j + itx;
            v1 = (Rox_Float)ds[ni][nj];
            v2 = (Rox_Float)ds[ni][nj+1];
            v3 = (Rox_Float)ds[ni+1][nj];
            v4 = (Rox_Float)ds[ni+1][nj+1];
            val = b1 * v1 + b2 * v2 + b3 * v3 + b4 * v4;
            ddiff[i][j] = dr[i][j] - val - obj->luminosity_beta;
            dsum[i][j] = dr[i][j] + val;
            dc[i][j] = val;
         }
      }

      update = 0;
      for (Rox_Sint i = 1; i < 10; i++)
      {
         for (Rox_Sint j = 1; j < 10; j++)
         {
            gx = (Rox_Float) (0.25 * (dsum[i][j+1] - dsum[i][j-1]));
            gy = (Rox_Float) (0.25 * (dsum[i+1][j] - dsum[i-1][j]));
            d = ddiff[i][j];

            djtj[0][0] += gx*gx;
            djtj[0][1] += gx*gy;
            djtj[0][2] += gx;
            djtj[1][1] += gy*gy;
            djtj[1][2] += gy;
            djtj[2][2] += 1.0;
            djtf[0] += gx*d;
            djtf[1] += gy*d;
            djtf[2] += d;
         }
      }

      djtj[1][0] = djtj[0][1];
      djtj[2][0] = djtj[0][2];
      djtj[2][1] = djtj[1][2];

      error = rox_array2d_double_symm3x3_solve(obj->solsys, obj->JtJ, obj->Jtf);
      ROX_ERROR_CHECK_TERMINATE ( error );

      obj->current_pos_x += dsol[0][0];
      obj->current_pos_y += dsol[1][0];
      obj->luminosity_beta += (Rox_Float) dsol[2][0];

      update = dsol[0][0]*dsol[0][0] + dsol[1][0]*dsol[1][0];
      if (update < 0.001) break;
   }


   error = rox_array2d_float_zncc_nomask_normalizedscore(&zncc, obj->current, obj->reference);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (zncc < minscore)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE(error)}

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_point_9x9_track_x(Rox_Tracking_Point_9x9 obj, Rox_Image current, Rox_Double minscore)
{
   Rox_Uint iter;
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double ** dsol, **djtj, *djtf, zncc;
   Rox_Uchar ** ds;
   Rox_Float  **dr;
   Rox_Float **dsum, **ddiff;
   Rox_Float **dc;
   Rox_Sint i,j,sheight, swidth;
   Rox_Sint ni, nj;
   Rox_Sint itx, ity;
   Rox_Double tx,ty,update;
   Rox_Float dx, dy, val;
   Rox_Float v1,v2,v3,v4;
   Rox_Float b1,b2,b3,b4;
   Rox_Float gx,d;


   if (!obj || !current)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_get_data_pointer_to_pointer( &ds, current);
   error = rox_array2d_float_get_data_pointer_to_pointer( &dc, obj->current);
   error = rox_array2d_float_get_data_pointer_to_pointer( &dr, obj->reference);
   error = rox_array2d_double_get_data_pointer_to_pointer( &dsol, obj->solsys);
   error = rox_array2d_float_get_data_pointer_to_pointer( &dsum, obj->meanlum);
   error = rox_array2d_float_get_data_pointer_to_pointer( &ddiff, obj->diff);
   error = rox_array2d_double_get_data_pointer_to_pointer( &djtj, obj->JtJ);
   error = rox_array2d_double_get_data_pointer ( &djtf, obj->Jtf);


   error = rox_array2d_uchar_get_size(&sheight, &swidth, current);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (iter = 0; iter < 8; iter++)
   {
      tx = obj->current_pos_x - 1;
      ty = obj->current_pos_y - 1;
      itx = (Rox_Sint)tx;
      ity = (Rox_Sint)ty;
      dx = (Rox_Float) (tx - (Rox_Float)itx);
      dy = (Rox_Float) (ty - (Rox_Float)ity);
      b1 = (Rox_Float) ((1.0 - dx)*(1.0 - dy));
      b2 = (Rox_Float) (dx * (1.0 - dy));
      b3 = (Rox_Float) ((1.0 - dx) * dy);
      b4 = dx * dy;


      if (tx < 0 || ty < 0)
      { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

      if (itx + 11 >= swidth - 1 || ity + 11 >= sheight - 1)
      { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

      djtj[0][0] = 0;
      djtj[0][1] = 0;
      djtj[0][2] = 0;
      djtj[1][1] = 0;
      djtj[1][2] = 0;
      djtj[2][2] = 0;
      djtf[0] = 0;
      djtf[1] = 0;
      djtf[2] = 0;

      for (i = 0; i < 11; i++)
      {
         ni = i + ity;

         for (j = 0; j < 11; j++)
         {
            nj = j + itx;
            v1 = (Rox_Float)ds[ni][nj];
            v2 = (Rox_Float)ds[ni][nj+1];
            v3 = (Rox_Float)ds[ni+1][nj];
            v4 = (Rox_Float)ds[ni+1][nj+1];
            val = b1 * v1 + b2 * v2 + b3 * v3 + b4 * v4;
            ddiff[i][j] = dr[i][j] - val - obj->luminosity_beta;
            dsum[i][j] = dr[i][j] + val;
            dc[i][j] = val;
         }
      }

      update = 0;
      for (i = 1; i < 10; i++)
      {
         for (j = 1; j < 10; j++)
         {
            gx = (Rox_Float) (0.25 * (dsum[i][j+1] - dsum[i][j-1]));
            d = ddiff[i][j];

            djtj[0][0] += gx*gx;
            djtj[0][1] += gx;
            djtj[1][1] += 1;
            djtf[0] += gx*d;
            djtf[1] += d;
         }
      }

      djtj[1][0] = djtj[0][1];


      dsol[0][0] = djtj[1][1] * djtf[0] - djtj[0][1] * djtf[1];
      dsol[1][0] = - djtj[0][1] * djtf[0] + djtj[0][0] * djtf[1];
      dsol[0][0] /= djtj[0][0] * djtj[1][1] - djtj[0][1] * djtj[0][1];
      dsol[1][0] /= djtj[0][0] * djtj[1][1] - djtj[0][1] * djtj[0][1];

      obj->current_pos_x += dsol[0][0];
      obj->luminosity_beta += (Rox_Float)dsol[1][0];

      update = dsol[0][0]*dsol[0][0];
      if (update < 0.001) break;
   }

   error = rox_array2d_float_zncc_nomask_normalizedscore(&zncc, obj->current, obj->reference);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (zncc < minscore)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

function_terminate:
   return error;
}
