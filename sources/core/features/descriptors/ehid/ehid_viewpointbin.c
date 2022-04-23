//==============================================================================
//
//    OPENROX   : File ehid_viewpointbin.c
//
//    Contents  : Implementation of ehid_viewpointbin module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ehid_viewpointbin.h"
#include "ehid_viewpointbin_struct.h"

#include <float.h>
#include <stdio.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/random/random.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/inverse/mat3x3inv.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/geometry/point/point3d_sphere.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
#include <baseproc/image/remap/remap_ewa_omo/remap_ewa_omo.h>
#include <baseproc/image/remap/remap_bilinear_omo_uchar_to_uchar/remap_bilinear_omo_uchar_to_uchar.h>
#include <baseproc/image/noise/gaussian_noise.h>

#include <core/features/detectors/segment/fastst.h>
#include <core/features/detectors/segment/fastst_score.h>
#include <core/features/descriptors/ehid/ehid.h>
#include <core/features/descriptors/ehid/ehid_window.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_ehid_viewpointbin_createwindows(Rox_Ehid_ViewpointBin obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   const Rox_Double theoric_maxpts = 50;
   const Rox_Double s = 2;
   const Rox_Double s2 = 3.5 * s;
   const Rox_Double b = ceil(sqrt(2.0 * (s2 * s2))) + 1;
   const Rox_Sint rectsize = 200;

   // IMPORTANT : We assume the reference viewpoint is only a scale of the source image for the moment !!
   Rox_Sint refwidth  = (Rox_Sint) floor(((Rox_Double)obj->image_size.width ) / obj->mean_scale);
   Rox_Sint refheight = (Rox_Sint) floor(((Rox_Double)obj->image_size.height) / obj->mean_scale);

   Rox_Sint usedwidth  = refwidth  - 2 * (Rox_Sint) b;
   Rox_Sint usedheight = refheight - 2 * (Rox_Sint) b;

   Rox_Sint nbfullcols = usedwidth / rectsize;
   Rox_Sint nbfullrows = usedheight / rectsize;

   Rox_Sint widthlast = usedwidth % rectsize;
   Rox_Sint heightlast = usedheight % rectsize;

   Rox_Uint countrow = nbfullrows;
   if (heightlast) countrow++;

   Rox_Uint countcol = nbfullcols;
   if (widthlast) countcol++;

   for (Rox_Uint i = 0; i < countrow; i++)
   {
      for (Rox_Uint j = 0; j < countcol; j++)
      {
         Rox_Sint width = 0, height = 0;

         Rox_Uint posi = (Rox_Uint) (b + i * rectsize);
         Rox_Uint posj = (Rox_Uint) (b + j * rectsize);

         if (i < (Rox_Uint) nbfullrows) height = rectsize;
         else height = heightlast;

         if (j < (Rox_Uint) nbfullcols) width = rectsize;
         else width = widthlast;

         Rox_Double ratiowidth = (Rox_Double)width / (Rox_Double)rectsize;
         Rox_Double ratioheight = (Rox_Double)height / (Rox_Double)rectsize;

         if (countrow == 1 && countcol == 1)
         {
            ratioheight = 1;
            ratiowidth = 1;
         }

         Rox_Double rmaxpts = ceil(ratioheight * ratiowidth * theoric_maxpts);
         Rox_Uint maxpts = (Rox_Uint) rmaxpts;

         Rox_Ehid_Window curwin = NULL;

         error = rox_ehid_window_new(&curwin, posi, posj, height, width, maxpts);
         ROX_ERROR_CHECK_TERMINATE ( error );

         rox_objset_ehid_window_append(obj->windows, curwin);
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_viewpointbin_new(Rox_Ehid_ViewpointBin * obj, const Rox_Sint image_width, const Rox_Sint image_height, const Rox_Double minscale, const Rox_Double maxscale, Rox_Double const minaffine, Rox_Double const maxaffine, const Rox_Double sigma)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ehid_ViewpointBin ret = NULL;

   // Intenal parameters that may be exposed
   Rox_Uint max_points = 150;
   Rox_Uint nb_subdivs = 4;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Ehid_ViewpointBin) rox_memory_allocate(sizeof(*ret), 1);

   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->image_size.width  = image_width;
   ret->image_size.height = image_height;
   ret->min_scale = minscale;
   ret->max_scale = maxscale;
   ret->mean_scale = ret->min_scale + ((ret->max_scale - ret->min_scale) / 2.0);
   ret->max_points = max_points;
   ret->sigma = (Rox_Float) sigma;

   // Generate viewpoints, note that "maxaffine" is "maxangle" and 4 = nb_subdivs
   ret->vpvec = NULL;
   error = rox_dynvec_point3d_double_new_from_sphere(&ret->vpvec, nb_subdivs, maxaffine);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Generate buffers
   ret->homography = NULL;
   error = rox_array2d_double_new(&ret->homography, 3, 3);   
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->invhomography = NULL;
   error = rox_array2d_double_new(&ret->invhomography, 3, 3);   
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->refhomography = NULL;
   error = rox_array2d_double_new(&ret->refhomography, 3, 3);   
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->warphomography = NULL;
   error = rox_array2d_double_new(&ret->warphomography, 3, 3);   
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->calib_input = NULL;
   error = rox_array2d_double_new(&ret->calib_input, 3, 3);   
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->calib_output = NULL;
   error = rox_array2d_double_new(&ret->calib_output, 3, 3);   
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->fast_points = NULL;
   error = rox_dynvec_segment_point_new(&ret->fast_points, 100);   
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->fast_points_nonmax = NULL;
   error = rox_dynvec_segment_point_new(&ret->fast_points_nonmax, 100);   
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->windows = NULL;
   error = rox_objset_ehid_window_new(&ret->windows, 1);   
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->clustered = NULL;
   error = rox_dynvec_ehid_point_new(&ret->clustered, 10);   
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->clustered_index = NULL;
   error = rox_dynvec_ehid_dbindex_new(&ret->clustered_index, 10);   
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->pose = NULL;
   error = rox_array2d_double_new(&ret->pose, 4, 4);   
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Generate reference viewpoint
   error = rox_array2d_double_fillunit(ret->pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value(ret->pose, 2, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Build calibration matrix for input
   error = rox_transformtools_build_calibration_matrix_for_template(ret->calib_input, image_width, image_height, 1.0, 1.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Build calibration matrix for output
   error = rox_transformtools_build_calibration_matrix_for_template(ret->calib_output, ((Rox_Double)image_width) / ret->mean_scale, ((Rox_Double)image_height) / ret->mean_scale, 1.0, 1.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_transformtools_build_homography_intermodel(ret->refhomography, ret->pose, ret->calib_output, ret->calib_input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Generate subwindows for this viewbin
   error = rox_ehid_viewpointbin_createwindows(ret);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *obj = ret;

function_terminate:
   if (error) rox_ehid_viewpointbin_del(&ret);
   return error;
}

Rox_ErrorCode rox_ehid_viewpointbin_del(Rox_Ehid_ViewpointBin * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ehid_ViewpointBin todel = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = * obj;
   *obj = NULL;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_point3d_double_del(&todel->vpvec);
   rox_array2d_double_del(&todel->homography);
   rox_array2d_double_del(&todel->invhomography);
   rox_array2d_double_del(&todel->refhomography);
   rox_array2d_double_del(&todel->warphomography);
   rox_array2d_double_del(&todel->calib_input);
   rox_array2d_double_del(&todel->calib_output);
   rox_array2d_double_del(&todel->pose);
   rox_dynvec_segment_point_del(&todel->fast_points);
   rox_dynvec_segment_point_del(&todel->fast_points_nonmax);
   rox_objset_ehid_window_del(&todel->windows);
   rox_dynvec_ehid_point_del(&todel->clustered);
   rox_dynvec_ehid_dbindex_del(&todel->clustered_index);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_ehid_viewpointbin_generate_windows(Rox_ObjSet_Ehid_Window windows, Rox_DynVec_Segment_Point fast_points_nonmax, Rox_Array2D_Double warphomography, Rox_Image dest)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Point2D_Double_Struct refpt, viewpt;

   Rox_Double ** dh = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dh, warphomography);

   Rox_Sint out_width = 0, out_height = 0;
   error = rox_array2d_uchar_get_size(&out_height, &out_width, dest);

   // Initialize windows
   for (Rox_Uint idwindow = 0; idwindow < windows->used; idwindow++)
   {
      error = rox_ehid_window_begin_view(windows->data[idwindow]);
   }

   // Initialize list of local features
   for (Rox_Uint idpt = 0; idpt < fast_points_nonmax->used; idpt++)
   {
      Rox_Double u = fast_points_nonmax->data[idpt].j;
      Rox_Double v = fast_points_nonmax->data[idpt].i;

      // Fast do not have large enough borders for rotated patch secure gathering
      if (u < 10 || v < 10 || u >= out_width - 10 || v >= out_height - 10) continue;

      // Warp coordinates in reference frame
      Rox_Double x = dh[0][0] * u + dh[0][1] * v + dh[0][2];
      Rox_Double y = dh[1][0] * u + dh[1][1] * v + dh[1][2];
      Rox_Double z = dh[2][0] * u + dh[2][1] * v + dh[2][2];

      if (fabs(z) < DBL_EPSILON) continue;
      x = x / z;
      y = y / z;

      refpt.u = x;
      refpt.v = y;

      viewpt.u = u;
      viewpt.v = v;

      // Distribute feature to windows using sorted vector (by score to keep only best points)
      for (Rox_Uint idwindow = 0; idwindow < windows->used; idwindow++)
      {
         rox_ehid_window_append_point(windows->data[idwindow], &refpt, &viewpt);
      }
   }

   // Compute features
   for (Rox_Uint idwindow = 0; idwindow < windows->used; idwindow++)
   {
      error = rox_ehid_points_compute(windows->data[idwindow]->localpoints, dest); 
      ROX_ERROR_CHECK_TERMINATE ( error );
         
      error = rox_ehid_points_warp(windows->data[idwindow]->localpoints, warphomography); 
      ROX_ERROR_CHECK_TERMINATE ( error );
         
      error = rox_ehid_window_terminate_view(windows->data[idwindow]); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_viewpointbin_process (
   Rox_Ehid_ViewpointBin obj, 
   const Rox_Image image
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Rox_Point2D_Double_Struct refpt, viewpt;

   // Intenal parameters that may be exposed
   Rox_Double sigma = 4.0; // gaussian noise variance

   if (!obj || !image) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Check that the input image matches the stored image size in obj
   error = rox_array2d_uchar_check_size(image, obj->image_size.height, obj->image_size.width); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double difscale = obj->max_scale - obj->min_scale;

   for (Rox_Uint idview = 0; idview < obj->vpvec->used; idview++)
   {
      Rox_MeshGrid2D_Float grid = NULL;

      Rox_Image dest = NULL; // image
      Rox_Imask destm = NULL; // imask

      // Generate homography parameters
      Rox_Point3D_Double_Struct vec = obj->vpvec->data[idview];

      //! Keypoints
      Rox_DynVec_Segment_Point fast_points = obj->fast_points;

      // Keypoints with non maximum suppression
      Rox_DynVec_Segment_Point fast_points_nonmax = obj->fast_points_nonmax;

      // Generate random rotation angle between 0 and 360 degrees
      Rox_Double iprot = 360.0 * ((Rox_Double) rox_rand()) / ((Rox_Double) ROX_RAND_MAX);

      // Generate random scale between min_scale and max_scale
      Rox_Double scale = obj->min_scale + (((Rox_Double) rox_rand()) / ((Rox_Double) ROX_RAND_MAX) * difscale);

      // Get the images size
      Rox_Sint inp_width = obj->image_size.width, inp_height = obj->image_size.height;

      // Get pointers
      Rox_Array2D_Double K_inp       = obj->calib_input;
      Rox_Array2D_Double K_out       = obj->calib_output;
      Rox_Array2D_Double T           = obj->pose;
      Rox_Array2D_Double H           = obj->homography;
      Rox_Array2D_Double H_inv       = obj->invhomography;
      Rox_Array2D_Double H_warp      = obj->warphomography;
      Rox_ObjSet_Ehid_Window windows = obj->windows;

      // Generate homography for this view
      Rox_Double wp = ((Rox_Double) inp_width ) / scale;
      Rox_Double hp = ((Rox_Double) inp_height) / scale;

      if (wp < hp)
      {
         error = rox_transformtools_build_calibration_matrix_for_template ( K_inp, inp_width, inp_height, wp/hp, 1.0);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_transformtools_build_calibration_matrix_for_template ( K_out, wp, hp, wp/hp, 1.0);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      else
      {
         error = rox_transformtools_build_calibration_matrix_for_template ( K_inp, inp_width, inp_height, 1.0, hp/wp);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_transformtools_build_calibration_matrix_for_template ( K_out, wp, hp, 1.0, hp/wp);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      // Compute the pose such that the z-axis point to the center of the image
      error = rox_transformtools_pose_from_sphere ( T, &vec, iprot);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute the intermodel homography to project the image
      error = rox_transformtools_build_homography_intermodel(H, T, K_out, K_inp);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute homography and bounding box size
      Rox_Sint out_width = 0, out_height = 0;
      error = rox_transformtools_homography_optimalforward(&out_width, &out_height, H, inp_width, inp_height);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mat3x3_inverse(H_inv, H);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // From current homography to reference viewpoint
      error = rox_array2d_double_mulmatmat(H_warp, obj->refhomography, H_inv);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Create image buffers
      error = rox_meshgrid2d_float_new(&grid, out_height, out_width); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_uchar_new(&dest, out_height, out_width); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_uint_new(&destm, out_height, out_width); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Warp image to synthetic view
      error = rox_warp_grid_sl3_float ( grid, H_inv ); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_remap_ewa_omo_uchar (dest, destm, image, grid ); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Add synthetic noise
      error = rox_array2d_uchar_gaussian_noise ( dest, dest, sigma ); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Detect keypoints
      error = rox_fastst_detector(fast_points, dest, 10, 0); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_fastst_detector_score(fast_points, dest, 10); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_fastst_nonmax_suppression(fast_points_nonmax, fast_points); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_fastst_detector_sort(fast_points_nonmax); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_objset_ehid_viewpointbin_generate_windows(windows, fast_points_nonmax, H_warp, dest);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Delete buffers
      rox_array2d_uint_del ( &destm );
      rox_meshgrid2d_float_del ( &grid );
      rox_array2d_uchar_del ( &dest );
   }

   // Clustering
   rox_dynvec_ehid_point_reset(obj->clustered);
   rox_dynvec_ehid_dbindex_reset(obj->clustered_index);

   for (Rox_Uint idwindow = 0; idwindow < obj->windows->used; idwindow++)
   {
      error = rox_ehid_window_cluster(obj->windows->data[idwindow]);
      if(error == ROX_ERROR_INSUFFICIENT_DATA) continue; // If we does not have data don't exit but continue
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Stack local result to final db
      rox_dynvec_ehid_point_stack(obj->clustered, obj->windows->data[idwindow]->clusteredpoints);
      rox_dynvec_ehid_dbindex_stack(obj->clustered_index, obj->windows->data[idwindow]->dbindices);
   }

   // If we arrived here, it means that previous errors where indeed warnings
   // Therefore set error to ROX_ERROR_NONE
   error = ROX_ERROR_NONE;

   // scale up features from reference view to source view
   for (Rox_Uint idpt = 0; idpt < obj->clustered->used; idpt++)
   {
      obj->clustered->data[idpt].scale  = obj->mean_scale;
      obj->clustered->data[idpt].pos.u *= obj->mean_scale;
      obj->clustered->data[idpt].pos.v *= obj->mean_scale;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_viewpointbin_test(Rox_Sint * pcount, Rox_Sint * pcount_total, Rox_Ehid_ViewpointBin obj, Rox_Image source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint idwindow = 0;
   Rox_Point3D_Double_Struct vec;
   Rox_Sint out_width, out_height;

   Rox_Double scale, iprot;
   Rox_Uint bcount, tcount;

   Rox_MeshGrid2D_Float grid = NULL;
   Rox_Array2D_Uint destm = NULL;
   Rox_Image dest = NULL;


   if (!obj || !source || !pcount || !pcount_total) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_check_size(source, obj->image_size.height, obj->image_size.width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get pointer to obj-windows
   Rox_ObjSet_Ehid_Window windows = obj->windows;

   //! Keypoints
   Rox_DynVec_Segment_Point fast_points = obj->fast_points;

   // Keypoints with non maximum suppression
   Rox_DynVec_Segment_Point fast_points_nonmax = obj->fast_points_nonmax;

   Rox_Array2D_Double H_warp = obj->warphomography;

   // Generate homography parameters
   vec.X = 0;
   vec.Y = 0;
   vec.Z = 1;
   iprot = 0.0;
   scale = obj->mean_scale;

   // Generate homography for this view
   error = rox_transformtools_build_calibration_matrix_for_template(obj->calib_output, ((Rox_Double)obj->image_size.width) / scale, ((Rox_Double)obj->image_size.height) / scale, 1.0, 1.0);

   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_transformtools_pose_from_sphere(obj->pose, &vec, iprot);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_transformtools_build_homography_intermodel(obj->homography, obj->pose, obj->calib_output, obj->calib_input);

   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_transformtools_homography_optimalforward(&out_width, &out_height, obj->homography, obj->image_size.width, obj->image_size.height);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mat3x3_inverse(obj->invhomography, obj->homography);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // From current homography to reference viewpoint
   error = rox_array2d_double_mulmatmat(obj->warphomography, obj->refhomography, obj->invhomography);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create image buffers

   error = rox_meshgrid2d_float_new(&grid, out_height, out_width); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_uchar_new(&dest, out_height, out_width); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_uint_new(&destm, out_height, out_width); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Warp image to synthetic view
   error = rox_warp_grid_sl3_float(grid, obj->invhomography); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Remap
   // error = rox_remap_ewa_omo_uchar(dest, destm, source, map); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_remap_bilinear_omo_uchar_to_uchar(dest, destm, source, grid); 
   ROX_ERROR_CHECK_TERMINATE ( error );
 
   // Detect keypoints
   error = rox_fastst_detector(fast_points, dest, 20, 0); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_fastst_detector_score(fast_points, dest, 20); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_fastst_nonmax_suppression(fast_points_nonmax, fast_points); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_fastst_detector_sort(fast_points_nonmax); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_ehid_viewpointbin_generate_windows(windows, fast_points_nonmax, H_warp, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   bcount = 0;
   tcount = 0;
   for (idwindow = 0; idwindow < windows->used; idwindow++)
   {
      Rox_Uint min;

      if (windows->data[idwindow]->maxlocalpts == 0) continue;

      tcount++;


      min = (Rox_Uint) (0.8 * windows->data[idwindow]->maxlocalpts);
      if (min < 1) min = 1;

      if (windows->data[idwindow]->localpoints->used > min)
      {
         bcount++;
      }
   }

   *pcount = bcount;
   *pcount_total = tcount;

function_terminate:
   // Delete buffers
   rox_array2d_uint_del(&destm);
   rox_meshgrid2d_float_del(&grid);
   rox_array2d_uchar_del(&dest);

   return error;
}
