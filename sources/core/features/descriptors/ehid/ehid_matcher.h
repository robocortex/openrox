//==============================================================================
//
//    OPENROX   : File ehid_matcher.h
//
//    Contents  : API of ehid_matcher module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EHID_MATCHER__
#define __OPENROX_EHID_MATCHER__

#include <generated/objset_ehid_target.h>
#include <generated/dynvec_uint.h>
#include <generated/dynvec_ehid_dbindex.h>
#include <baseproc/maths/linalg/matut3.h>

#include <core/features/descriptors/ehid/ehid.h>
#include <core/features/descriptors/ehid/ehid_database.h>

//! \addtogroup EHID
//! @{

//! Object 
typedef struct Rox_Ehid_Matcher_Struct * Rox_Ehid_Matcher;

//! Create a new empty matcher object
//! \param  [out]  ehid_matcher               The pointer to the newly created object
//! \param  [in ]  max_templates_detected     How many templates we need to find per processing ?
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ehid_matcher_new (
   Rox_Ehid_Matcher * ehid_matcher, 
   const Rox_Uint max_templates_detected
);

//! Delete a matcher object
//! \param  [out]     ehid_matcher              The pointer to the object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ehid_matcher_del (
   Rox_Ehid_Matcher * ehid_matcher
);

//! Given a list of detected features in the runtime image, try to identify a list of target and their pose
//! \param  [out]  ehid_matcher   The matcher object
//! \param  [in ]  db             The database to search on
//! \param  [in ]  detectedfeats  The list of runtime features
//! \param  [in ]  camera_calib   The calibration of the runtime camera
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ehid_matcher_match_se3 (
   Rox_Ehid_Matcher ehid_matcher, 
   const Rox_Ehid_Database db, 
   const Rox_DynVec_Ehid_Point detectedfeats, 
   const Rox_MatUT3 camera_calib
);

//! Given a list of detected features in the runtime image, try to identify a list of target and their homography
//! \param  [out]  ehid_matcher   The matcher object
//! \param  [in ]  db             The database to search on
//! \param  [in ]  detectedfeats  The list of runtime features
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ehid_matcher_match_sl3 (
   Rox_Ehid_Matcher ehid_matcher, 
   const Rox_Ehid_Database db, 
   const Rox_DynVec_Ehid_Point detectedfeats
);

//! @} 

#endif
