//===============================================================================
//
//    OPENROX   : File identification.c
//
//    Contents  : Implementation of identification module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "identification.h"

#include <system/memory/memory.h>

#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <inout/system/errors_print.h>

#include <user/tracking/tracking_params.h>
#include <user/tracking/tracking.h>

Rox_ErrorCode rox_identification_new(Rox_Identification* ident, Rox_Image itemplate)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Identification ret = NULL;
   Rox_Tracking_Params params = NULL;

   if(!ident || !itemplate)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *ident = NULL;

   // Allocate memory
   ret = (Rox_Identification)rox_memory_allocate(sizeof(*ret), 1);
   if(!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Set default values
   ret->tracker = 0;
   ret->identifier = 0;

   error = rox_ident_texture_sl3_new(&ret->identifier);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ident_texture_sl3_enable_double_size(ret->identifier);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ident_texture_sl3_set_model(ret->identifier, itemplate);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_tracking_params_new(&params);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_tracking_new(&ret->tracker, params, itemplate);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *ident = ret;

function_terminate:
   if(error) rox_identification_del(&ret);

   rox_tracking_params_del(&params);
   return error;
}

Rox_ErrorCode rox_identification_del(Rox_Identification* ident)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Identification todel = NULL;

   if(!ident)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *ident;
   *ident = NULL;

   if(!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_ident_texture_sl3_del(&todel->identifier);
   rox_tracking_del(&todel->tracker);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_identification_make(Rox_Identification ident, Rox_Image current)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double factor = 1.0;
   Rox_MatSL3 H = NULL;
   Rox_Image crop = NULL;

   Rox_Sint is_identified = 0;
   Rox_Uint posu = 0, posv= 0;

   if(!ident || !current)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matsl3_new ( &H );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint rows = 0, cols = 0;
   error = rox_image_get_size(&rows, &cols, current);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint sizu = cols;
   Rox_Uint sizv = rows;

   // Init homographies

   while(sizu > 32 && sizv > 32)
   {
      // Split the current image
      sizu = (Rox_Uint) (cols / factor);
      sizv = (Rox_Uint) (rows / factor);

      for (Rox_Uint i = 0; i < (Rox_Uint)factor; i++)
      {
         for (Rox_Uint j = 0; j < (Rox_Uint)factor; j++)
         {
            posu = j * sizu;
            posv = i * sizv;

            // Get a subview
            error = rox_array2d_uchar_new_subarray2d(&crop, current, posv, posu, sizv, sizu);
            ROX_ERROR_CHECK_TERMINATE ( error );

            // Make detection
            error = rox_ident_texture_sl3_make(&is_identified, H, ident->identifier, crop);

            // Check if the template is found
            if(is_identified)
            {
                // update homography
                error = rox_transformtools_homography_shiftleft(H, posu, posv);
                ROX_ERROR_CHECK_TERMINATE ( error );

                // Set  tracking SL3
                error = rox_tracking_set_homography(ident->tracker, H);
                ROX_ERROR_CHECK_TERMINATE ( error );

                // Refine the homography
                error = rox_tracking_make(ident->tracker, current);

                if(error == ROX_ERROR_NONE)
                {
                    goto function_terminate;
                }
            }

            // Free subimage
            rox_image_del(&crop);
         }
      }

      // With translation
      for (Rox_Uint i = 0; i < (Rox_Uint)factor - 1; i++)
      {
         for (Rox_Uint j = 0; j < (Rox_Uint)factor - 1; j++)
         {
            posu = j * sizu + sizu / 2;
            posv = i * sizv + sizv / 2;

            // Get a subview
            error = rox_array2d_uchar_new_subarray2d(&crop, current, posv, posu, sizv, sizu);
            ROX_ERROR_CHECK_TERMINATE ( error );

            // Make detection
            error = rox_ident_texture_sl3_make(&is_identified, H, ident->identifier, crop);

            // Check if the template is found
            if(is_identified)
            {
                // update homography
                error = rox_transformtools_homography_shiftleft(H, posu, posv);
                ROX_ERROR_CHECK_TERMINATE ( error );

                // Set  tracking SL3
                error = rox_tracking_set_homography(ident->tracker, H);
                ROX_ERROR_CHECK_TERMINATE ( error );

                // Refine the homography
                error = rox_tracking_make(ident->tracker, current);

                if(error == ROX_ERROR_NONE)
                {
                    goto function_terminate;
                }
            }

            // Free subimage
            rox_image_del(&crop);
         }
      }

      // Top border
      for (Rox_Uint i = 0; i < (Rox_Uint)factor - 1; i++)
      {
          posu = i * sizu + sizu / 2;
          posv = 0;

            // Get a subview
            error = rox_array2d_uchar_new_subarray2d(&crop, current, posv, posu, sizv / 2, sizu);
            ROX_ERROR_CHECK_TERMINATE ( error );

            // Make detection
            error = rox_ident_texture_sl3_make(&is_identified, H, ident->identifier, crop);

            // Check if the template is found
            if(is_identified)
            {
                // update homography
                error = rox_transformtools_homography_shiftleft(H, posu, posv);
                ROX_ERROR_CHECK_TERMINATE ( error );

                // Set  tracking SL3
                error = rox_tracking_set_homography(ident->tracker, H);
                ROX_ERROR_CHECK_TERMINATE ( error );

                // Refine the homography
                error = rox_tracking_make(ident->tracker, current);

                if(error == ROX_ERROR_NONE)
                {
                    goto function_terminate;
                }
            }

            // Free subimage
            rox_image_del(&crop);
      }

      // Bottom border
      for (Rox_Uint i = 0; i < (Rox_Uint)factor - 1; i++)
      {
          posu = i * sizu + sizu / 2;
          posv = rows - sizv / 2 - 1;

            // Get a subview
            error = rox_array2d_uchar_new_subarray2d(&crop, current, posv, posu, sizv / 2, sizu);
            ROX_ERROR_CHECK_TERMINATE ( error );

            // Make detection
            error = rox_ident_texture_sl3_make(&is_identified, H, ident->identifier, crop);

            // Check if the template is found
            if(is_identified)
            {
                // update homography
                error = rox_transformtools_homography_shiftleft(H, posu, posv);
                ROX_ERROR_CHECK_TERMINATE ( error );

                // Set  tracking SL3
                error = rox_tracking_set_homography(ident->tracker, H);
                ROX_ERROR_CHECK_TERMINATE ( error );

                // Refine the homography
                error = rox_tracking_make(ident->tracker, current);

                if(error == ROX_ERROR_NONE)
                {
                    goto function_terminate;
                }
            }

            // Free subimage
            rox_image_del(&crop);
      }

      // Left border
      for (Rox_Uint i = 0; i < (Rox_Uint)factor - 1; i++)
      {
          posu = 0;
          posv = sizv / 2 + i * sizv;

            // Get a subview
            error = rox_array2d_uchar_new_subarray2d(&crop, current, posv, posu, sizv, sizu / 2);
            ROX_ERROR_CHECK_TERMINATE ( error );

            // Make detection
            error = rox_ident_texture_sl3_make(&is_identified, H, ident->identifier, crop);

            // Check if the template is found
            if(is_identified)
            {
                // update homography
                error = rox_transformtools_homography_shiftleft(H, posu, posv);
                ROX_ERROR_CHECK_TERMINATE ( error );

                // Set  tracking SL3
                error = rox_tracking_set_homography(ident->tracker, H);
                ROX_ERROR_CHECK_TERMINATE ( error );

                // Refine the homography
                error = rox_tracking_make(ident->tracker, current);

                if(error == ROX_ERROR_NONE)
                {
                    goto function_terminate;
                }
            }

            // Free subimage
            rox_image_del(&crop);
      }

      // Right border
      for (Rox_Uint i = 0; i < (Rox_Uint)factor - 1; i++)
      {
          posu = cols - sizu / 2 - 1;
          posv = sizv / 2 + i * sizv;

            // Get a subview
            error = rox_array2d_uchar_new_subarray2d(&crop, current, posv, posu, sizv, sizu / 2);
            ROX_ERROR_CHECK_TERMINATE ( error );

            // Make detection
            error = rox_ident_texture_sl3_make(&is_identified, H, ident->identifier, crop);

            // Check if the template is found
            if(is_identified)
            {
                // update homography
                error = rox_transformtools_homography_shiftleft(H, posu, posv);
                ROX_ERROR_CHECK_TERMINATE ( error );

                // Set  tracking SL3
                error = rox_tracking_set_homography(ident->tracker, H);
                ROX_ERROR_CHECK_TERMINATE ( error );

                // Refine the homography
                error = rox_tracking_make(ident->tracker, current);

                if(error == ROX_ERROR_NONE)
                {
                    goto function_terminate;
                }
            }

            // Free subimage
            rox_image_del(&crop);
      }

      if(factor > 1.0)
      {
          // Top left corner
          posu = 0;
          posv = 0;

          // Get a subview
          error = rox_array2d_uchar_new_subarray2d(&crop, current, posv, posu, sizv / 2, sizu / 2);
          ROX_ERROR_CHECK_TERMINATE ( error );

          // Make detection
          error = rox_ident_texture_sl3_make(&is_identified, H, ident->identifier, crop);

          // Check if the template is found
          if(is_identified)
          {
              // update homography
              error = rox_transformtools_homography_shiftleft(H, posu, posv);
              ROX_ERROR_CHECK_TERMINATE ( error );

              // Set  tracking SL3
              error = rox_tracking_set_homography(ident->tracker, H);
              ROX_ERROR_CHECK_TERMINATE ( error );

              // Refine the homography
              error = rox_tracking_make(ident->tracker, current);

              if(error == ROX_ERROR_NONE)
              {
                  goto function_terminate;
              }
          }

          // Free subimage
          rox_image_del(&crop);

          // Top right corner
          posu = cols - sizu / 2 - 1;
          posv = 0;

          // Get a subview
          error = rox_array2d_uchar_new_subarray2d(&crop, current, posv, posu, sizv / 2, sizu / 2);
          ROX_ERROR_CHECK_TERMINATE ( error );

          // Make detection
          error = rox_ident_texture_sl3_make(&is_identified, H, ident->identifier, crop);

          // Check if the template is found
          if(is_identified)
          {
              // update homography
              error = rox_transformtools_homography_shiftleft(H, posu, posv);
              ROX_ERROR_CHECK_TERMINATE ( error );

              // Set  tracking SL3
              error = rox_tracking_set_homography(ident->tracker, H);
              ROX_ERROR_CHECK_TERMINATE ( error );

              // Refine the homography
              error = rox_tracking_make(ident->tracker, current);

              if(error == ROX_ERROR_NONE)
              {
                 goto function_terminate;
              }
           }

           // Free subimage
           rox_image_del(&crop);

           // Left bottom
           posu = 0;
           posv = rows - sizv / 2;

           // Get a subview
           error = rox_array2d_uchar_new_subarray2d(&crop, current, posv, posu, sizv / 2, sizu / 2);
           ROX_ERROR_CHECK_TERMINATE ( error );

           // Make detection
           error = rox_ident_texture_sl3_make(&is_identified, H, ident->identifier, crop);

           // Check if the template is found
           if(is_identified)
           {
               // update homography
               error = rox_transformtools_homography_shiftleft(H, posu, posv);
               ROX_ERROR_CHECK_TERMINATE ( error );

               // Set  tracking SL3
               error = rox_tracking_set_homography(ident->tracker, H);
               ROX_ERROR_CHECK_TERMINATE ( error );

               // Refine the homography
               error = rox_tracking_make(ident->tracker, current);

               if(error == ROX_ERROR_NONE)
               {
                   goto function_terminate;
               }
            }

           // Right bottom
           posu = cols - sizu / 2 - 1 ;
           posv = rows - sizv / 2 - 1 ;

           // Get a subview
           error = rox_array2d_uchar_new_subarray2d(&crop, current, posv, posu, sizv / 2, sizu / 2);
           ROX_ERROR_CHECK_TERMINATE ( error );

           // Make detection
           error = rox_ident_texture_sl3_make(&is_identified, H, ident->identifier, crop);

           // Check if the template is found
           if(is_identified)
           {
               // update homography
               error = rox_transformtools_homography_shiftleft(H, posu, posv);ROX_ERROR_CHECK_TERMINATE(error)

               // Set  tracking SL3
               error = rox_tracking_set_homography(ident->tracker, H);
               ROX_ERROR_CHECK_TERMINATE ( error );

               // Refine the homography
               error = rox_tracking_make(ident->tracker, current);

               if(error == ROX_ERROR_NONE)
               {
                  goto function_terminate;
               }
            }
           // Free subimage
           rox_image_del(&crop);
      }
      // Update factor
      factor *= 2.0;
   }

function_terminate:
   rox_image_del(&crop);
   rox_matsl3_del(&H);
   return error;
}

Rox_ErrorCode rox_identification_get_homography(Rox_MatSL3 H, Rox_Identification ident)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!H || !ident) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   error = rox_tracking_get_homography(H, ident->tracker);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
