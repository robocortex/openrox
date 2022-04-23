//==============================================================================
//
//    OPENROX   : File frame.c
//
//    Contents  : Implementation of frame module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "frame.h"
#include "frame_struct.h"

#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>

#include <system/memory/memory.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_frame_new(Rox_Frame *frame)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Frame ret = NULL;

   if(!frame) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *frame = NULL;

   ret = (Rox_Frame) rox_memory_allocate(sizeof(*ret), 1);
   if(!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_new(&ret->pose);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&ret->vt, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&ret->at, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&ret->ar, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&ret->wr, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(ret->pose);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(ret->vt, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(ret->at, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(ret->ar, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(ret->wr, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *frame = ret;

function_terminate:
   if(error) rox_frame_del(&ret);
   return error;
}

Rox_ErrorCode rox_frame_del(Rox_Frame *frame)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Frame todel = NULL;

   if (!frame) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *frame;
   *frame = NULL;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_array2d_double_del(&todel->at);
   rox_array2d_double_del(&todel->vt);
   rox_array2d_double_del(&todel->ar);
   rox_array2d_double_del(&todel->wr);
   rox_array2d_double_del(&todel->pose);
   rox_memory_delete(todel);

function_terminate:   
   return error;
}

Rox_ErrorCode rox_frame_set_pose(Rox_Frame frame, Rox_MatSE3 pose)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if(!frame || !pose ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_matse3_copy(frame->pose, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:   
   return error;
}

Rox_ErrorCode rox_frame_set_translation_velocity(Rox_Frame frame, const Rox_Array2D_Double vt)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if(!frame || !vt) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_array2d_double_copy(frame->vt, vt);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:   
   return error;
}

Rox_ErrorCode rox_frame_set_angular_velocity(Rox_Frame frame, const Rox_Array2D_Double wr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!frame || !wr) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_copy(frame->wr, wr);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:   
   return error;
}

Rox_ErrorCode rox_frame_set_angular_acceleration(Rox_Frame frame, const Rox_Array2D_Double ar)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!frame || !ar ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_copy(frame->ar, ar);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:   
   return error;
}

Rox_ErrorCode rox_frame_set_translation_acceleration(Rox_Frame frame, const Rox_Array2D_Double at)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!frame || !at) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_array2d_double_copy(frame->at, at);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:   
   return error;
}

Rox_ErrorCode rox_frame_get_pose(Rox_MatSE3 pose, const Rox_Frame frame)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!frame || !pose ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_matse3_copy(pose, frame->pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:   
   return error;
}

Rox_ErrorCode  rox_frame_get_velocity(Rox_Array2D_Double vt, Rox_Array2D_Double wr, const Rox_Frame frame)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!frame || !vt || !wr) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_array2d_double_copy(vt, frame->vt);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy(wr, frame->wr);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
function_terminate:   
   return error;
}

Rox_ErrorCode  rox_frame_copy(Rox_Frame frame_out, const Rox_Frame frame_inp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!frame_inp || !frame_out) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_matse3_copy(frame_out->pose, frame_inp->pose);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(frame_out->vt, frame_inp->vt);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy(frame_out->wr, frame_inp->wr);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(frame_out->at, frame_inp->at);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy(frame_out->ar, frame_inp->ar);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
function_terminate:   
   return error;
}
