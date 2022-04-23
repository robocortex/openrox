//==============================================================================
//
//    OPENROX   : File detection_checkerboard.c
//
//    Contents  : Implementation of bundle_point module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//=============================================================================

#include "detection_checkerboard.h"
#include "detection_checkerboard_struct.h"

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_detection_checkerboard_new(Rox_Detection_Checkerboard * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Detection_Checkerboard ret = NULL;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *obj = NULL;

   ret = (Rox_Detection_Checkerboard) rox_memory_allocate(sizeof(struct Rox_Detection_Checkerboard_Struct), 1);
   if (!ret) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
    // !Pointer to the detected grid
    
    ret->_detected_grid    = NULL;
    ret->_detected         = 0;
    ret->_image            = NULL;
    ret->_image_height     = 0;
    ret->_image_width      = 0;
    ret->_n_points         = 0;
    ret->_id               = 0;
   
   *obj = ret;

function_terminate:
   if (error) rox_detection_checkerboard_del(&ret);
   return error;
}

Rox_ErrorCode rox_detection_checkerboard_del(Rox_Detection_Checkerboard * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Detection_Checkerboard todel = NULL;
   
   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (todel->_image) rox_image_del(&(todel->_image));
   
   if (todel->_detected_grid) rox_memory_delete(todel->_detected_grid);
   
   rox_memory_delete(todel);

function_terminate:
   return error;
}
