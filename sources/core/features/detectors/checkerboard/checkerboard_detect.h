//==============================================================================
//
//    OPENROX   : File checkerboard_detect.h
//
//    Contents  : API of checkerboard_detect module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CHECKERBOARD_DETECT__
#define __OPENROX_CHECKERBOARD_DETECT__

#include <generated/dynvec_uint.h>
#include <generated/dynvec_uint_struct.h>
#include <generated/objset_checkerboard.h>
#include <generated/objset_checkerboard_struct.h>
#include <baseproc/geometry/point/point2d.h>

#include <core/features/detectors/checkerboard/checkercorner_detect.h>

//! checkerboard detector object
struct Rox_CheckerBoard_Detector_Struct
{
   //! Per checkerboard detector
   Rox_DynVec_Uint usedcorners;

   //! checkerboard buffer
   Rox_DynVec_Uint buffer_indices;

   //! detected checkerboards
   Rox_ObjSet_CheckerBoard checkerboards;
};

//! checkerboard detector object
typedef struct Rox_CheckerBoard_Detector_Struct * Rox_CheckerBoard_Detector;

//! Create a container object for checkerboard detection
//! \param checkerboard_detector the created container pointer
//! \return An error code
ROX_API Rox_ErrorCode rox_checkerboard_detector_new(Rox_CheckerBoard_Detector * checkerboard_detector);

//! Delete a container object for checkerboard corner detection
//! \param checkerboard_detector the container pointer to delete
//! \return An error code
ROX_API Rox_ErrorCode rox_checkerboard_detector_del(Rox_CheckerBoard_Detector * checkerboard_detector);

//! Extract checkerboards from corners
//! \param checkerboard_detector    pointer to process
//! \param checkercorner_detector   the corner detector to use for checkerboard extraction (must be processed before)
//! \return An error code
ROX_API Rox_ErrorCode rox_checkerboard_detector_process(Rox_CheckerBoard_Detector checkerboard_detector, Rox_CheckerCorner_Detector checkercorner_detector);

#endif
