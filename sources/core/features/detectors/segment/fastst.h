//==============================================================================
//
//    OPENROX   : File fastst.h
//
//    Contents  : API of fastst module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_FASTST__
#define __OPENROX_FASTST__

#include <generated/dynvec_segment_point.h>
#include <baseproc/image/image.h>


//! \ingroup Detectors
//! \defgroup Fastst Fastst

//! \addtogroup Fastst
//! @{

//! Extract points from an image which successfully pass the accelerated segment test (Modified FAST)
//! \param  [out]  points         the output list of detected points
//! \param  [in ]  source         the input image to extract points from
//! \param  [in ]  barrier        the intensity minimal difference for segment
//! \param  [in ]  level          current image level in the pyramid (may be set to convenience value)
//! \return An error code
ROX_API Rox_ErrorCode rox_fastst_detector ( 
   Rox_DynVec_Segment_Point points,
   const Rox_Image source,
   const Rox_Sint  barrier,
   const Rox_Uint  level 
);

//! @}

#endif
