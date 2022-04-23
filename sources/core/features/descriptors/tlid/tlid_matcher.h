//==============================================================================
//
//    OPENROX   : File tlid_matcher.h
//
//    Contents  : API of tlid_matcher module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TLID_MATCHER__
#define __OPENROX_TLID_MATCHER__

#include <system/memory/datatypes.h>
#include <core/features/descriptors/tlid/tlid.h>
#include <generated/dynvec_point2d_double_struct.h>
#include <generated/dynvec_uint.h>
#include <generated/dynvec_uint_struct.h>

#define NB_ROTATION_BINS 18

//! \addtogroup TLID
//! @{

//! The Rox_Tlid_Matcher_Struct object 
struct Rox_Tlid_Matcher_Struct
{
   //! To be commented  
   Rox_DynVec_Point2D_Double reference_matched_unfiltered;
   
   //! To be commented  
   Rox_DynVec_Point2D_Double current_matched_unfiltered;

   //! To be commented  
   Rox_DynVec_Point2D_Double reference_matched;

   //! To be commented  
   Rox_DynVec_Point2D_Double current_matched;

   //! To be commented  
   Rox_DynVec_Uint rotation_bins[NB_ROTATION_BINS];
};

//! Define the pointer of the Rox_Tlid_Matcher_Struct 
typedef struct Rox_Tlid_Matcher_Struct * Rox_Tlid_Matcher;

//! Create tlid matcher object
//! \param [out] obj the pointer to the object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_tlid_matcher_new(Rox_Tlid_Matcher * obj);

//! Delete tlid matcher object
//! \param [in] obj the pointer to the object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_tlid_matcher_del(Rox_Tlid_Matcher * obj);

//! Match two descriptors set
//! \param [in] obj the matcher object
//! \param [in] curset the current set
//! \param [in] refset the reference set
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_tlid_matcher_match(Rox_Tlid_Matcher obj, Rox_Tlid curset, Rox_Tlid refset);

//! @} 

#endif

