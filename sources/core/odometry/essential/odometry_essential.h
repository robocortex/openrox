//==============================================================================
//
//    OPENROX   : File odometry_essential.h
//
//    Contents  : API of odometry_essential module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ODOMETRY_ESSENTIAL__
#define __OPENROX_ODOMETRY_ESSENTIAL__

#include <generated/array2d_double.h>
#include <generated/dynvec_point2d_float.h>
#include <generated/dynvec_uint.h>

//! \ingroup Odometry
//! \addtogroup Odometry_Essential
//! @{

//! Essential odometry structure
struct Rox_Odometry_Essential_Struct
{
   //! Estimated pose (Valid up to a scale in translation) 
   Rox_Array2D_Double pose;

   //! Camera calibration (Given by user) 
   Rox_Array2D_Double calib;

   //! Estimated essential matrix 
   Rox_Array2D_Double essential;

   //! Input reference points list 
   Rox_DynVec_Point2D_Float list_reference;

   //! Input current points list 
   Rox_DynVec_Point2D_Float list_current;

   //! Coarse inliers for reference points list 
   Rox_DynVec_Point2D_Float list_reference_coarse_inliers;

   //! Coarse inliers for  current points list 
   Rox_DynVec_Point2D_Float list_current_coarse_inliers;

   //! Corrected (to lie on the epipolar line) coordinates for reference points list 
   Rox_DynVec_Point2D_Float list_reference_corrected;

   //! Corrected (to lie on the epipolar line) coordinates for current points list 
   Rox_DynVec_Point2D_Float list_current_corrected;

   //! For each point pair in the input list, tells if it respect the final epipolar constraint
   Rox_DynVec_Uint list_match_flag;

   //! The total number of consistent matches 
   Rox_Uint count_valid_match;
};

//! Essential odometry structure
typedef struct Rox_Odometry_Essential_Struct * Rox_Odometry_Essential;

//! Create a new odometry with essential constraint object
//! \param [out] obj a pointer to the newly created object
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_essential_new(Rox_Odometry_Essential * obj);

//! Delete an odometry object with essential constraint
//! \param [out] obj a pointer to the created object
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_essential_del(Rox_Odometry_Essential * obj);

//! Set the calibration matrix for this odometry
//! \param [in] obj a pointer to odometry object
//! \param [in] calib a 3*3 matrix with the intrinsic parameters
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_essential_set_calibration(Rox_Odometry_Essential obj, Rox_Array2D_Double calib);

//! Set points raw matches in the odometry object (A point in the reference list at index i must be matched to a point in the current list at index i).
//! \param [in] obj the odometry object
//! \param [in] ref the list of reference matched points in normalized coordinates
//! \param [in] cur the list of current   matched points in normalized coordinates
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_essential_set_rawmatches(Rox_Odometry_Essential obj, Rox_DynVec_Point2D_Float ref, Rox_DynVec_Point2D_Float cur);

//! Process matches to find an optimal odometry robustly.
//! \param [in] obj the odometry object
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_essential_process(Rox_Odometry_Essential obj);

//! Get the estimated pose
//! \param [out]   pose      The pose
//! \param [in]      obj      The odometry object
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_essential_get_pose(Rox_Array2D_Double pose, Rox_Odometry_Essential obj);

//! Get the list of valid matches
//! \param [out]   list_match_flag      The list of valid matches
//! \param [in]      obj                  The odometry object
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_essential_get_valid_matches(Rox_DynVec_Uint list_match_flag, Rox_Odometry_Essential obj);                                                 

//! @} 
 
#endif
