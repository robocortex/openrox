//============================================================================
//
//    OPENROX   : File stereo_calibration_struct.h
//
//    Contents  : API of stereo_calibration module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __STEREO_CALIBRATION_STRUCT__
#define __STEREO_CALIBRATION_STRUCT__

#include <core/calibration/mono/calibration_perspective_struct.h>
#include <generated/array2d_double.h>
#include <generated/objset_array2d_double.h>

//! \ingroup  Sensor
//! \defgroup Stereo Stereo
//! \brief Structure and functions for stereo cameras.

//! \ingroup  Stereo
//! \defgroup Calibration_Stereo_Perspective Stereo Camera Calibration
//! \brief Structure and functions for camera calibration.

//! \addtogroup Calibration_Stereo_Perspective
//! \brief Structure and functions of the perspective stereo calibration
//! @{

//! The Rox_Calibration_Stereo_Perspective_Struct object 
struct Rox_Calibration_Stereo_Perspective_Struct
{
    //! The left camera calibration object 
    Rox_Calibration_Mono_Perspective left;
    
    //! The right camera calibration object 
    Rox_Calibration_Mono_Perspective right;
   
    //! The stereo pose 
    Rox_Array2D_Double rTl;

    //! The pose set 
    Rox_ObjSet_Array2D_Double poses;

    //! The valid flags 
    Rox_DynVec_Uint valid_flags;
};

//! @}

#endif // __OPENROX_STEREO_CALIBRATION_STRUCT__
