//==============================================================================
//
//    OPENROX   : File detection_sl3.c
//
//    Contents  : Implementation of bundle_point module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "detection_sl3.h"
#include "detection_sl3_struct.h"

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_detection_sl3_new(Rox_Detection_Sl3 * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Detection_Sl3 ret = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *obj = NULL;

   ret = (Rox_Detection_Sl3)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
    
   //! Pointer to the detected grid
   // error = rox_array2d_double_new(&(ret->_estimated_homography),3,3);
   ret->_estimated_homography = NULL;
   ret->_detected = 0;
   ret->_image = NULL;
   ret->_image_height = 0;
   ret->_image_width = 0;
    
   *obj = ret;

function_terminate:
   if (error) rox_detection_sl3_del(&ret);
   return error;
}

Rox_ErrorCode rox_detection_sl3_del(Rox_Detection_Sl3 * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Detection_Sl3 todel = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (todel->_image) rox_image_del(&todel->_image);
   if (todel->_estimated_homography) rox_array2d_double_del(&todel->_estimated_homography);

   rox_memory_delete(todel);

function_terminate:
   return error;
}
