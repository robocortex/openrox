//==============================================================================
//
//    OPENROX   : File sraid.c
//
//    Contents  : Implementation of sraid module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "sraid.h"

#include <baseproc/maths/maths_macros.h>
#include <stdio.h>

#include <generated/dynvec_sraiddesc_struct.h>
#include <generated/dynvec_point2d_float_struct.h>

#include <baseproc/maths/kernels/gaussian2d.h>
#include <baseproc/image/convolve/basic_convolve.h>
#include <baseproc/image/convolve/array2d_float_symmetric_separable_convolve.h>
#include <baseproc/image/pyramid/pyramid_tools.h>
#include <baseproc/image/remap/remap_box_halved/remap_box_halved.h>
#include <baseproc/geometry/rectangle/rectangle_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_sraidpipeline_process (
  Rox_DynVec_SRAID_Feature sraid_output,
  Rox_Array2D_Float               input,
  Rox_Sint                        sraid_max_octaves,
  Rox_Float                       sraid_sigma,
  Rox_Float                       cutoff,
  Rox_Float                       sraid_initial_sigma,
  Rox_Sint                        sraid_invls,
  Rox_Float                       sraid_contr_thresh,
  Rox_Sint                        sraid_curv_thresh,
  Rox_Uint                        object_id )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float buffer = NULL;
   Rox_Array2D_Float filtered = NULL;
   Rox_Float sigdiff;
   Rox_Array2D_Float hfilter = NULL, vfilter = NULL, toresize = NULL;
   Rox_Uint pyramidsize = 0;

   Rox_Uint detectcount;
   Rox_Float prelim_contrast;
   Rox_Rect_Sint_Struct bounds;

   Rox_Dog_Feature features = NULL;
   Rox_Array2D_Float_Collection dogspace = NULL;
   Rox_Array2D_Float_Collection scalespace = NULL;

   if (!sraid_output)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!input)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (cols < 32 || rows < 32)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   bounds.x = 0;
   bounds.y = 0;
   bounds.height = rows;
   bounds.width = cols;

   error = rox_array2d_float_new(&buffer, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&filtered, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   sraid_output->used = 0;
   prelim_contrast = (Rox_Float) (0.5 * sraid_contr_thresh / sraid_invls);

   // Initial image filtering
   sigdiff = (Rox_Float) sqrt(sraid_sigma * sraid_sigma - sraid_initial_sigma * sraid_initial_sigma);

   error = rox_kernelgen_gaussian2d_separable_float_new(&hfilter, &vfilter, sigdiff, cutoff);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_symmetric_seperable_convolve(filtered, input, hfilter);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //  What is the optimal pyramid size
   error = rox_pyramid_compute_optimal_level_count(&pyramidsize, cols, rows, 8);
   if (error) pyramidsize = 0;
   pyramidsize += 1; // original level = 1 more

   if (pyramidsize > ((Rox_Uint) (sraid_max_octaves + 1)) && sraid_max_octaves >= 1)
   {
      pyramidsize = sraid_max_octaves;
   }

   // Loop over pyramid levels
   for (Rox_Uint octave = 0; octave < pyramidsize; octave++)
   {
      // Create scale space for current image
      error = rox_array2d_float_build_scale_space_new(&scalespace, filtered, sraid_invls, sraid_sigma, cutoff);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Create dog space
      error = rox_dogspace_create(&dogspace, scalespace);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Detect features
      error = rox_dogdetector_process(&features, &detectcount, dogspace, &bounds, sraid_contr_thresh, (Rox_Float) sraid_curv_thresh, prelim_contrast, sraid_invls, octave, sraid_sigma);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // describe features
      error = rox_sraiddescriptor_process(sraid_output, features, detectcount, scalespace, object_id);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Update size
      cols = cols / 2;
      rows = rows / 2;

      // Create new "reference" image for next octave
      rox_array2d_float_del(&filtered);
      error = rox_array2d_float_new(&filtered, rows, cols);
      ROX_ERROR_CHECK_TERMINATE(error)

      toresize = rox_array2d_float_collection_get(scalespace, sraid_invls);
      error = rox_remap_box_nomask_float_to_float_halved(filtered, toresize);
      ROX_ERROR_CHECK_TERMINATE(error)

      // Delete buffers
      rox_memory_delete(features); features = NULL;
      rox_array2d_float_collection_del(&dogspace);
      rox_array2d_float_collection_del(&scalespace);
   }

function_terminate:
   rox_array2d_float_del(&vfilter);
   rox_array2d_float_del(&hfilter);

   // DEBUG: problem when deleting NULL ?
   rox_memory_delete(features);
   rox_array2d_float_collection_del(&dogspace);
   rox_array2d_float_collection_del(&scalespace);
   rox_array2d_float_del(&buffer);
   rox_array2d_float_del(&filtered);

   return error;
}


Rox_ErrorCode rox_sraid_populate_pointlist(Rox_DynVec_Point2D_Float list, Rox_DynVec_SRAID_Feature features)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!features)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!list)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   list->used = 0;
   if (features->used == 0)
   { error = ROX_ERROR_NONE; goto function_terminate; }

   rox_dynvec_point2d_float_usecells(list, features->used);

   for (Rox_Uint i = 0; i < features->used; i++)
   {
      list->data[i].u = features->data[i].x;
      list->data[i].v = features->data[i].y;
   }
function_terminate:
   return error;
}


Rox_ErrorCode rox_sraid_dump_to_file(const Rox_Char *filename, Rox_DynVec_SRAID_Feature features)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * outfile = NULL;

   if (!filename || !features)
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   if (features->used < 1)
   {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error );}

   outfile = fopen(filename, "w");
   if (!outfile) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   fprintf(outfile, "%d\n", features->used);

   for (Rox_Uint idfeat = 0; idfeat < features->used; idfeat++)
   {
      Rox_SRAID_Feature_Struct * feat = &features->data[idfeat];

      fprintf(outfile, "%d %d ", feat->is_minimal, feat->object_id);
      fprintf(outfile, "%.16f %.16f %.16f ", feat->ori, feat->x, feat->y);

      for ( Rox_Sint idcell = 0; idcell < ROX_SRAID_DESCRIPTOR_SIZE; idcell++)
      {
         fprintf(outfile, "%d ", feat->descriptor[idcell]);
      }

      fprintf(outfile, "\n");
   }

function_terminate:
   if (outfile) fclose(outfile);
   return error;
}


Rox_ErrorCode rox_sraid_load_from_file(Rox_DynVec_SRAID_Feature features, const Rox_Char * filename)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * infile = NULL;
   Rox_Sint cnt, len;
   Rox_Sint cellval;
   Rox_SRAID_Feature_Struct feat;

   if (!filename || !features)
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_dynvec_sraiddesc_reset(features);

   infile = fopen(filename, "r");
   if (!infile) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   len = fscanf(infile, "%d", &cnt);
   if (len == 0)
   { error = ROX_ERROR_BAD_IOSTREAM;  ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (cnt <= 0)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Sint idfeat = 0; idfeat < cnt; idfeat++)
   {
      len = fscanf(infile, "%d %d", &feat.is_minimal, &feat.object_id);
      if (len == 0)
      { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

      len = fscanf(infile, "%f %f %f", &feat.ori, &feat.x, &feat.y);
      if (len == 0)
      { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

      for ( Rox_Sint idcell = 0; idcell < ROX_SRAID_DESCRIPTOR_SIZE; idcell++)
      {
         len = fscanf(infile, "%d ", &cellval);
         if (len == 0)
         { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error) }

         feat.descriptor[idcell] = cellval;
      }

      rox_dynvec_sraiddesc_append(features, &feat);
   }

function_terminate:
   if (infile) fclose(infile);
   return error;
}
