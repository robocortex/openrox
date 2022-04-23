//==============================================================================
//
//    OPENROX   : File plane_search_uchar.c
//
//    Contents  : Implementation of plane_search module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "plane_search_uchar.h"

#include <baseproc/geometry/point/point3d_tools.h>
#include <baseproc/geometry/point/dynvec_point3d_tools.h>
#include <baseproc/geometry/point/point2d_matsl3_transform.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
#include <baseproc/image/remap/remap_bilinear_omo_uchar_to_uchar/remap_bilinear_omo_uchar_to_uchar.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/inverse/svdinverse.h>

#include <core/indirect/euclidean/matso3_from_vectors.h>
#include <core/templatesearch/region_zncc_search_mask_template_mask.h>

#include <inout/system/errors_print.h>
// temporary include
#include <baseproc/image/warp/image_warp_matsl3.h>

Rox_ErrorCode rox_plane_search_uchar_new (
   Rox_Plane_Search_Uchar * plane_search, 
   const Rox_Sint model_rows, 
   const Rox_Sint model_cols, 
   const Rox_Sint search_radius
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Plane_Search_Uchar ret = NULL;
   Rox_Sint search_cols, search_rows;

   if (!plane_search)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *plane_search = NULL;

   ret = (Rox_Plane_Search_Uchar) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->model = NULL;
   ret->model_mask = NULL;
   ret->search = NULL;
   ret->search_mask = NULL;
   ret->grid = NULL;
   ret->r_G_t = NULL;
   ret->c_G_t = NULL;
   ret->c_G_s = NULL;
   ret->s_G_t = NULL;
   ret->search_radius = search_radius;

   search_rows = model_rows + 2 * search_radius;
   search_cols = model_cols + 2 * search_radius;

   // Allocate memory for model image
   error = rox_array2d_uchar_new(&ret->model, model_rows, model_cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate memory for model imask
   error = rox_array2d_uint_new(&ret->model_mask, model_rows, model_cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate memory for search image
   error = rox_array2d_uchar_new(&ret->search, search_rows, search_cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate memory for search imask
   error = rox_array2d_uint_new(&ret->search_mask, search_rows, search_cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate memory for the warped points
   error = rox_meshgrid2d_float_new(&ret->grid, search_rows, search_cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matsl3_new(&ret->r_G_t);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matsl3_new(&ret->c_G_t);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matsl3_new(&ret->c_G_s);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matsl3_new(&ret->s_G_t);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *plane_search = ret;

function_terminate:
   if (error) rox_plane_search_uchar_del(&ret);
   return error;
}

Rox_ErrorCode rox_plane_search_uchar_del(Rox_Plane_Search_Uchar * plane_search)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Plane_Search_Uchar todel = NULL;

   if (!plane_search)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *plane_search;
   *plane_search = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_array2d_uchar_del(&todel->model);
   rox_array2d_uint_del(&todel->model_mask);
   rox_array2d_uchar_del(&todel->search);
   rox_array2d_uint_del(&todel->search_mask);
   rox_meshgrid2d_float_del(&todel->grid);
   rox_matsl3_del(&todel->r_G_t);
   rox_matsl3_del(&todel->c_G_t);
   rox_matsl3_del(&todel->c_G_s);
   rox_matsl3_del(&todel->s_G_t);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_plane_search_uchar_set_model (
   Rox_Plane_Search_Uchar plane_search, 
   const Rox_Image model, 
   const Rox_Imask model_mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!plane_search || !model || !model_mask)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_copy(plane_search->model, model);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_copy(plane_search->model_mask, model_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_plane_search_uchar_set_model_warp (
   Rox_Plane_Search_Uchar plane_search, 
   const Rox_Image reference_image, 
   const Rox_MatSL3 r_G_t
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !plane_search || !reference_image || !r_G_t )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matsl3_copy ( plane_search->r_G_t, r_G_t );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_imask_warp_matsl3 ( plane_search->model, plane_search->model_mask, reference_image, r_G_t );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_plane_search_uchar_make(Rox_Plane_Search_Uchar plane_search, const Rox_Image current_image, const Rox_MatSL3 c_G_t)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint shift_u, shift_v;

   if (!plane_search || !current_image || !c_G_t)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Reset shift to zero
   plane_search->shift_u = 0.0;
   plane_search->shift_v = 0.0;

   // Copy the input homography
   error = rox_matsl3_copy ( plane_search->c_G_s, c_G_t );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Update homography c_G_s in order to warp also the radius area : c_G_s = c_G_t * [1, 0, search_radius; 0, 1, search_radius; 0, 0, 1]
   error = rox_transformtools_homography_shift ( plane_search->c_G_s, - (Rox_Double) plane_search->search_radius, - (Rox_Double) plane_search->search_radius);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if(0)
   {
      // Warp current image onto the search template
      error = rox_warp_grid_sl3_float(plane_search->grid, plane_search->c_G_s);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_remap_bilinear_omo_uchar_to_uchar(plane_search->search, plane_search->search_mask, current_image, plane_search->grid);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      error = rox_image_imask_warp_matsl3 ( plane_search->search, plane_search->search_mask, current_image, plane_search->c_G_s);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Find the best region of the search template for the given model
   error = rox_array2d_uchar_region_zncc_search_mask_template_mask ( &plane_search->score, &shift_u, &shift_v, plane_search->search, plane_search->search_mask, plane_search->model, plane_search->model_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Update the "found" transformation
   plane_search->shift_u = (Rox_Double) shift_u - (Rox_Double) plane_search->search_radius;
   plane_search->shift_v = (Rox_Double) shift_v - (Rox_Double) plane_search->search_radius;

   // Set the estimated homography s_G_t
   error = rox_matsl3_set_unit ( plane_search->s_G_t );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_transformtools_homography_shift ( plane_search->s_G_t, shift_u, shift_v );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set the estimated homography c_G_t = c_G_s * s_G_t
   rox_matsl3_mulmatmat ( plane_search->c_G_t, plane_search->c_G_s, plane_search->s_G_t );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_plane_search_uchar_get_results(Rox_Double * score, Rox_Double * shift_u, Rox_Double * shift_v, const Rox_Plane_Search_Uchar plane_search)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_plane_search_uchar_get_shift(shift_u, shift_v, plane_search);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_plane_search_uchar_get_score(score, plane_search);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_plane_search_uchar_get_shift (
   Rox_Double * shift_u, 
   Rox_Double * shift_v, 
   const Rox_Plane_Search_Uchar plane_search)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!shift_u || !shift_v || !plane_search)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *shift_u = plane_search->shift_u;
   *shift_v = plane_search->shift_v;

function_terminate:
   return error;
}

Rox_ErrorCode rox_plane_search_uchar_get_score(Rox_Double * score, const Rox_Plane_Search_Uchar plane_search)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!score || !plane_search)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *score = plane_search->score;

function_terminate:
   return error;
}

Rox_ErrorCode rox_plane_search_uchar_update_pose_translation (
   Rox_MatSE3 c_T_r, 
   const Rox_Array2D_Double calib, 
   const Rox_Plane_Search_Uchar plane_search
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!plane_search || !c_T_r || !calib)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( c_T_r );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_check_size ( calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dk = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dk, calib);
   Rox_Double ** dt = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dt, c_T_r);

   // Simply update the pose considering the pixel to meters calibration : c_T_r = c_T_r * [I, [plane_search->shift_u/px; plane_search->shift_v/py; 0]; 0 0 0 1]
   // c_R_r = c_R_r * I
   // c_t_r = c_R_r * [plane_search->shift_u/px; plane_search->shift_v/py; 0] + c_t_r
   Rox_Double px = dk[0][0];
   Rox_Double py = dk[1][1];
   dt[0][3] = (dt[0][0] / px) * plane_search->shift_u + (dt[0][1] / py) * plane_search->shift_v + dt[0][3];
   dt[1][3] = (dt[1][0] / px) * plane_search->shift_u + (dt[1][1] / py) * plane_search->shift_v + dt[1][3];
   dt[2][3] = (dt[2][0] / px) * plane_search->shift_u + (dt[2][1] / py) * plane_search->shift_v + dt[2][3];

function_terminate:
   return error;
}

Rox_ErrorCode rox_plane_search_uchar_update_pose_rotation (
   Rox_MatSE3 c_R_r, 
   const Rox_MatUT3 K, 
   const Rox_Plane_Search_Uchar plane_search
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_Point3D_Double vref = NULL;
   Rox_DynVec_Point3D_Double vcur = NULL;
   Rox_Point2D_Double_Struct qr[4], qc[4], pt[4];
   Rox_MatSL3 c_H_t = NULL, r_H_t = NULL;
   Rox_MatUT3 K_inv = NULL;
   Rox_Sint rows = 0, cols = 0;

   error = rox_dynvec_point3d_double_new(&vref, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_new(&vcur, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_get_size(&rows, &cols, plane_search->model);

   // Build corner points of the template
   pt[0].u =      -0.5;   pt[0].v =      -0.5;
   pt[1].u = cols -0.5;   pt[1].v =      -0.5;
   pt[2].u =      -0.5;   pt[2].v = rows -0.5;
   pt[3].u = cols -0.5;   pt[3].v = rows -0.5;

   error = rox_array2d_double_new(&K_inv, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // homography = K * plane_search->homography
   error = rox_array2d_double_svdinverse(K_inv, K);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matsl3_new ( &r_H_t );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matsl3_new ( &c_H_t );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // cpmpute r_H_t = inv(K) * r_G_t
   error = rox_array2d_double_mulmatmat(r_H_t, K_inv, plane_search->r_G_t);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // c_H_t = inv(K) * c_G_t
   error = rox_array2d_double_mulmatmat(c_H_t, K_inv, plane_search->c_G_t);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // From pixel to normalized coordinates qr = r_H_t * pt
   error = rox_point2d_double_homography(qr, pt, r_H_t, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // From pixel to normalized coordinates qc = c_H_t * pt
   error = rox_point2d_double_homography(qc, pt, c_H_t, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < 4; i++)
   {
      Rox_Point3D_Double_Struct vr, vc;

      vr.X = qr[i].u; vr.Y = qr[i].v; vr.Z = 1.0;

      error = rox_dynvec_point3d_double_append(vref, &vr);
      ROX_ERROR_CHECK_TERMINATE ( error );

      vc.X = qc[i].u; vc.Y = qc[i].v; vc.Z = 1.0;

      error = rox_dynvec_point3d_double_append(vcur, &vc);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Normalize vectors to unit
   error = rox_dynvec_point3d_double_normalize_unit(vref);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_normalize_unit(vcur);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute rotation from normalized vectors min { || vcur - c_R_r * vref ||^2 }
   error = rox_matso3_from_vectors(c_R_r, vref, vcur);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_dynvec_point3d_double_del(&vref);
   rox_dynvec_point3d_double_del(&vcur);
   rox_matut3_del(&K_inv);
   rox_matsl3_del(&r_H_t);
   rox_matsl3_del(&c_H_t);

   return error;
}

Rox_ErrorCode rox_plane_search_uchar_update_homography(Rox_MatSL3 c_G_r, const Rox_Plane_Search_Uchar plane_search)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!c_G_r || !plane_search)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // c_G_r = c_G_t * inv(r_G_t)
   rox_matsl3_mulmatinv( c_G_r, plane_search->c_G_t, plane_search->r_G_t );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

