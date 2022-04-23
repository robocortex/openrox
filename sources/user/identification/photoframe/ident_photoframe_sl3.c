//==============================================================================
//
//    OPENROX   : File ident_photoframe_sl3.c
//
//    Contents  : Implementation of ident_photoframe_sl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ident_photoframe_sl3.h"
#include "ident_photoframe_struct.h"

#include <generated/objset_photoframe_struct.h>
#include <generated/objset_array2d_double.h>

#include <system/memory/memory.h>

#include <baseproc/array/fill/fillval.h>

#include <core/identification/codedframe.h>
#include <core/identification/photoframe_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_ident_photoframe_sl3_new(Rox_Ident_PhotoFrame_SL3 *ident_photoframe_sl3, Rox_Sint image_width, Rox_Sint image_height)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ident_PhotoFrame_SL3 ret = NULL;

   if (ident_photoframe_sl3 == NULL)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }
   *ident_photoframe_sl3 = NULL;

   ret = (Rox_Ident_PhotoFrame_SL3) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   // Init internal variables
   ret->qhom = NULL;
   ret->photoframes = NULL;
   ret->detector = NULL;
   ret->mask = NULL;
   ret->orientation_method = 0; // slowest but more robust
   ret->detection_method   = 0; // fastest but less robust
   ret->score_threshold    = 0.9;

   // Set the quad color to 0 in order to detect white inside black quads by default
   ret->quad_color = 0;

   // Set the minimum length of a segment extracted from the image for quad detection
   ret->quad_segment_length_min = 10;

   error = rox_matsl3_new ( &ret->qhom );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate 10 photoframes
   error = rox_objset_photoframe_new(&ret->photoframes, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_quaddetector_new(&ret->detector, image_width, image_height);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new(&ret->mask, image_height, image_width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *ident_photoframe_sl3 = ret;

function_terminate:
   if (error) rox_ident_photoframe_sl3_del(&ret);

   return error;
}

Rox_ErrorCode rox_ident_photoframe_sl3_del(Rox_Ident_PhotoFrame_SL3 *ident_photoframe_sl3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ident_PhotoFrame_SL3 todel = NULL;

   if(!ident_photoframe_sl3) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *ident_photoframe_sl3;
   *ident_photoframe_sl3 = NULL;

   if(!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ROX_ERROR_CHECK(rox_matsl3_del(&todel->qhom));
   ROX_ERROR_CHECK(rox_imask_del(&todel->mask));
   ROX_ERROR_CHECK(rox_quaddetector_del(&todel->detector));
   ROX_ERROR_CHECK(rox_objset_photoframe_del(&todel->photoframes));

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_photoframe_sl3_addframe(
   Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3,
   Rox_Image model,
   Rox_Sint border_width
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_PhotoFrame toadd = NULL;

   if (!ident_photoframe_sl3 || !model)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   if (border_width < 1)
   {
      error = ROX_ERROR_INVALID_VALUE;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   error = rox_photoframe_new(&toadd, model, border_width);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_objset_photoframe_append(ident_photoframe_sl3->photoframes, toadd);
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   if (error) rox_photoframe_del(&toadd);

   return error;
}

Rox_ErrorCode rox_ident_photoframe_sl3_getcountframes(Rox_Sint *count, Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !ident_photoframe_sl3 || !count )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *count = ident_photoframe_sl3->photoframes->used;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_photoframe_sl3_make(
   Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3,
   Rox_Image input
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Number of detected quad
   Rox_Sint qcount = 0;

   if (!ident_photoframe_sl3 || !input)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //Reset frames to mark them as not detected
   for (Rox_Uint idframe = 0; idframe < ident_photoframe_sl3->photoframes->used; idframe++)
   {
      error = rox_photoframe_reset(ident_photoframe_sl3->photoframes->data[idframe]);
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   error = rox_array2d_uint_fillval(ident_photoframe_sl3->mask, ~0);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_quaddetector_set_quad_color(ident_photoframe_sl3->detector, ident_photoframe_sl3->quad_color);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_quaddetector_set_segment_length_min(ident_photoframe_sl3->detector, ident_photoframe_sl3->quad_segment_length_min);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Detect quads
   // The new detector is rox_quaddetector_process_image_ac
   error = rox_quaddetector_process_image(ident_photoframe_sl3->detector, input, ident_photoframe_sl3->mask);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Get number of detected quads in variable qcount
   error = rox_quaddetector_get_quad_count(&qcount, ident_photoframe_sl3->detector);
   ROX_ERROR_CHECK_TERMINATE(error)

   for (Rox_Sint idquad = 0; idquad < qcount; idquad++)
   {
      // Get homography corresponding to idquad in variable ident_photoframe_sl3->qhom
      error = rox_quaddetector_get_quad_SL3(ident_photoframe_sl3->qhom, ident_photoframe_sl3->detector, idquad);
      ROX_ERROR_CHECK_CONTINUE(error)

      for (Rox_Uint idframe = 0; idframe < ident_photoframe_sl3->photoframes->used; idframe++)
      {
         error = rox_photoframe_set_score_threshold(ident_photoframe_sl3->photoframes->data[idframe], ident_photoframe_sl3->score_threshold);
         ROX_ERROR_CHECK_CONTINUE(error)

         error = rox_photoframe_make_with_fast_orientation_test(ident_photoframe_sl3->photoframes->data[idframe], input, ident_photoframe_sl3->qhom);
         ROX_ERROR_CHECK_CONTINUE(error)
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_photoframe_sl3_getresult(
   Rox_Sint * is_identified,
   Rox_MatSL3 H,
   Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3,
   Rox_Sint id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ident_photoframe_sl3 || !is_identified || !H)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (id > (Rox_Sint) ident_photoframe_sl3->photoframes->used)
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *is_identified = ident_photoframe_sl3->photoframes->data[id]->is_detected;
   if ((*is_identified))
   {
      error = rox_array2d_double_copy(H, ident_photoframe_sl3->photoframes->data[id]->resH);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_photoframe_sl3_set_type (
   Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3,
   Rox_Sint black_to_white
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ident_photoframe_sl3)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   error = rox_quaddetector_set_quad_color(ident_photoframe_sl3->detector, black_to_white);
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   return error;
}


Rox_ErrorCode rox_ident_photoframe_sl3_set_segment_length_min (
   Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3,
   const Rox_Double segment_length_min
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if (!ident_photoframe_sl3)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (segment_length_min > 0.0)
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ident_photoframe_sl3->quad_segment_length_min = segment_length_min;

function_terminate:
   return error;
}


Rox_ErrorCode rox_ident_photoframe_sl3_set_score_threshold (
   Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3,
   Rox_Double score_threshold
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( NULL == ident_photoframe_sl3)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( 0.0 > score_threshold || 1.0 < score_threshold )
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ident_photoframe_sl3->score_threshold = score_threshold;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_photoframe_sl3_getscore (
   Rox_Double * score,
   Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3,
   Rox_Sint id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ident_photoframe_sl3 || !score)
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   if ((Rox_Uint) id > ident_photoframe_sl3->photoframes->used)
   {error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}

   *score = ident_photoframe_sl3->photoframes->data[id]->score;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_photoframe_sl3_set_side_bounds (
   Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3,
   Rox_Double side_min,
   Rox_Double side_max
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ident_photoframe_sl3)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_quaddetector_set_side_bounds ( ident_photoframe_sl3->detector, side_min, side_max );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_photoframe_sl3_set_area_bounds (
   Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3,
   Rox_Double area_min,
   Rox_Double area_max
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ident_photoframe_sl3)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_quaddetector_set_area_bounds ( ident_photoframe_sl3->detector, area_min, area_max );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_photoframe_sl3_get_code (
   Rox_Sint * is_identified,
   Rox_Sint * code_number,
   Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3,
   Rox_Image image,
   Rox_Sint code_bits,
   Rox_Sint id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_CodedFrame codedframe = NULL;
   Rox_MatSL3 H = NULL;

   *code_number = 0;

   error = rox_matsl3_new ( &H );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ident_photoframe_sl3_getresult ( is_identified, H, ident_photoframe_sl3, id );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_codedframe_new ( &codedframe);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (is_identified)
   {
      if (code_bits == 16)
      {
         error = rox_codedframe_make16(codedframe, image, H);
         ROX_ERROR_CHECK_TERMINATE(error)

         *code_number = codedframe->value;
      }

      if (code_bits == 64)
      {
         error = rox_codedframe_make64(codedframe, image, H);
         ROX_ERROR_CHECK_TERMINATE(error)

         *code_number = codedframe->value;
      }
   }

function_terminate:
   rox_matsl3_del(&H);
   rox_codedframe_del(&codedframe);
   return error;
}
