//============================================================================
//
//    OPENROX   : File pyramid_npot_serialize.c
//
//    Contents  : Implementation of pyramid npot serialize module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "pyramid_npot_serialize.h"

#include <inout/numeric/array2d_serialize.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_pyramid_npot_uchar_serialize(FILE* out, Rox_Pyramid_Npot_Uchar input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint write_res = 0;

   //unit tests
   if (!out) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }
   
   if (!input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   //write numeric data
   write_res = (Rox_Sint) fwrite(&(input->nb_levels), sizeof(Rox_Uint), 1, out);
   if (!write_res)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error); }

   write_res = (Rox_Sint) fwrite(&(input->base_width), sizeof(Rox_Uint), 1, out);
   if (!write_res)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error); }

   write_res = (Rox_Sint) fwrite(&(input->base_height), sizeof(Rox_Uint), 1, out);
   if (!write_res)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error); }

   //write struct data
   error = rox_array2d_double_serialize(out, input->homography);
   for ( Rox_Uint each_lvl = 0; each_lvl < input->nb_levels-1; each_lvl++)
   {
      error = rox_array2d_float_serialize(out, input->grids[each_lvl]->u);
      error = rox_array2d_float_serialize(out, input->grids[each_lvl]->v);
   }
   for ( Rox_Uint each_lvl = 0; each_lvl < input->nb_levels; each_lvl++)
   {
      error = rox_array2d_uchar_serialize(out, input->levels[each_lvl]);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_pyramid_npot_uchar_deserialize(Rox_Pyramid_Npot_Uchar output, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint read_res = 0;
   Rox_Uint onb_levels, owidth, oheight;

   //unit tests
   if (!in) 
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }
   if (!output) 
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   //read numeric data
   read_res = (Rox_Sint) fread(&onb_levels, sizeof(Rox_Uint), 1, in);
   if (!read_res)
   {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error); }
   
   read_res = (Rox_Sint) fread(&owidth, sizeof(Rox_Uint), 1, in);
   if (!read_res)
   {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error); }
   
   read_res = (Rox_Sint) fread(&oheight, sizeof(Rox_Uint), 1, in);
   if (!read_res)
   {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error); }

   //test if reading the same type of object
   if ( onb_levels != output->nb_levels || oheight != output->base_height || owidth != output->base_width       )
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }      
   
   // read struct data
   error = rox_array2d_double_deserialize(output->homography, in);
   for ( Rox_Uint each_lvl = 0; each_lvl < output->nb_levels-1; each_lvl++)
   {
      error = rox_array2d_float_deserialize(output->grids[each_lvl]->u, in);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_float_deserialize(output->grids[each_lvl]->v, in);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   for ( Rox_Uint each_lvl = 0; each_lvl < output->nb_levels; each_lvl++)
   {
      error = rox_array2d_uchar_deserialize(output->levels[each_lvl], in);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}
