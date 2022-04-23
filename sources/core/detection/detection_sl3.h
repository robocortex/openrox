//==============================================================================
//
//    OPENROX   : File bundle_point.h
//
//    Contents  : API of bundle_point module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DETECTION_SL3__
#define __OPENROX_DETECTION_SL3__

#include <baseproc/geometry/point/points_struct.h>
#include <generated/array2d_double.h>

//#include <objset_detection_checkerboard.h>
//!//! \ingroup Vision
//!//! \addtogroup Bundle
//! @{

//! checkerboard object
typedef struct Rox_Detection_Sl3_Struct * Rox_Detection_Sl3;

//! Create a container object for a bundle point
//! \param  []  obj              the created container pointer
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_sl3_new(Rox_Detection_Sl3 * obj);

//! Delete a container object for a bundle point
//! \param  []  obj              the container pointer to delete
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_sl3_del(Rox_Detection_Sl3 * obj);

//! @}

#endif
