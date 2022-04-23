//==============================================================================
//
//    OPENROX   : File frame.h
//
//    Contents  : API of frame module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_FRAME__
#define __OPENROX_FRAME__

#include <generated/array2d_double.h>

#include <system/memory/datatypes.h>
#include <system/errors/errors.h>

#include <baseproc/maths/linalg/matse3.h>

//! \ingroup Sensor
//! \addtogroup Frame
//! @{

//! Define the pointer of the Rox_Frame_Struct 
typedef struct Rox_Frame_Struct* Rox_Frame;

//! Create a new frame object.
//! \param [out] frame The frame object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_frame_new(Rox_Frame *frame);

//! Delete frame.
//! \param [in] frame The frame object to delete
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_frame_del(Rox_Frame *frame);

//! Set the pose.
//! \param [out] frame Frame object
//! \param [in]  pose  Matrix in SE3
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_frame_set_pose(Rox_Frame frame, Rox_MatSE3 pose);

//! Set the translation velocity.
//! \param [out] frame Frame object
//! \param [in]  vt translation velocity (3x1 vector)
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_frame_set_translation_velocity(Rox_Frame frame, const Rox_Array2D_Double vt);

//! Set the angular velocity.
//! \param [out] frame Frame object
//! \param [in]  wr Angular velocity (3x1 vector)
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_frame_set_angular_velocity(Rox_Frame frame, const Rox_Array2D_Double wr);

//! Set the translation acceleration.
//! \param [out] frame Frame object
//! \param [in]  at Translation acceleration (3x1 vector)
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_frame_set_translation_acceleration(Rox_Frame frame, const Rox_Array2D_Double at);

//! Set the angular acceleration.
//! \param [out] frame Frame object
//! \param [in]  ar Angular acceleration (3x1 vector)
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_frame_set_angular_acceleration(Rox_Frame frame, const Rox_Array2D_Double ar);

//! Get a copy of the pose.
//! \param [out] pose Matrix in SE3
//! \param [in]  frame Frame object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode  rox_frame_get_pose(Rox_MatSE3 pose, const Rox_Frame frame);

//! Get a copy of the velocity.
//! \param [out]  vt    Translation velocity
//! \param [out]  vr    Rotation velocity
//! \param [in]  frame Frame object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode  rox_frame_get_velocity(Rox_Array2D_Double vt, Rox_Array2D_Double wr, const Rox_Frame frame);

ROX_API Rox_ErrorCode  rox_frame_copy(Rox_Frame frame_out, const Rox_Frame frame_inp);

//! @} 

#endif // __OPENROX_FRAME__ 
