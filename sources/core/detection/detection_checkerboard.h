//==============================================================================
//
//    OPENROX   : File detection_checkerboard.h
//
//    Contents  : API of bundle_point module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//=============================================================================

#ifndef __OPENROX_DETECTION_CHECKERBOARD__
#define __OPENROX_DETECTION_CHECKERBOARD__

#include <baseproc/geometry/point/points_struct.h>
#include <generated/array2d_double.h>
//#include <objset_detection_checkerboard.h>

//! \ingroup Vision
//! \addtogroup Detection
//! @{

//! detection checkerboard object
typedef struct Rox_Detection_Checkerboard_Struct * Rox_Detection_Checkerboard;

//! Create a container object for a detection checkerboard
//! \param  []  obj              the created container pointer
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_checkerboard_new(Rox_Detection_Checkerboard * obj);

//! Delete a container object for a detection checkerboard
//! \param  []  obj              the container pointer to delete
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_checkerboard_del(Rox_Detection_Checkerboard * obj);

//! @}

#endif
