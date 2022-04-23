//==============================================================================
//
//    OPENROX   : File matchsubset_3d2d.h
//
//    Contents  : API of matchsubset_3d2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MATCH_SUBSET_3D2D__
#define __OPENROX_MATCH_SUBSET_3D2D__

#include <generated/dynvec_point2d_float.h>
#include <generated/dynvec_point3d_float.h>
#include <generated/dynvec_sint.h>
#include <generated/dynvec_uint.h>

//! Given a match list, which gives the associated 2d point for a given 3d point index (or -1 if unmatched), create
//! a subset of both sets containing only matches
//! \param  []  subset3d a subset of iniset3d which is only made of matched points
//! \param  []  subset2d a subset of iniset2d which is only made of matched points
//! \param  []  matches a list of n (n=card(iniset3d)) matches (index between -1 and card(ini2dset)) 
//! \param  []  iniset3d initial 3d set
//! \param  []  iniset2d initial 2d set
//! \return an error code
ROX_API Rox_ErrorCode rox_match_float_extract_subset3d2d(Rox_DynVec_Point3D_Float subset3d, Rox_DynVec_Point2D_Float subset2d, Rox_DynVec_Sint matches, Rox_DynVec_Point3D_Float iniset3d, Rox_DynVec_Point2D_Float iniset2d);

//! Given a match list, which gives the associated current 2d point for a given reference point index (or -1 if unmatched), create
//! a subset of both sets containing only matches
//! \param  []  subsetref a subset of inisetref which is only made of matched points
//! \param  []  subsetcur a subset of inisetcur which is only made of matched points
//! \param  []  matches a list of n (n=card(iniset3d)) matches (index between -1 and card(ini2dset)) 
//! \param  []  inisetref initial ref set
//! \param  []  inisetcur initial current set
//! \return an error code
ROX_API Rox_ErrorCode rox_match_float_extract_subset2d2d(Rox_DynVec_Point2D_Float subsetref, Rox_DynVec_Point2D_Float subsetcur, Rox_DynVec_Sint matches, Rox_DynVec_Point2D_Float inisetref, Rox_DynVec_Point2D_Float inisetcur);

//! Given a match list, which gives the associated current 2d point for a given reference point index (or -1 if unmatched), create
//! a subset of both sets containing only matches. Create also an index with the position of the point in the source list.
//! \param  []  subsetref a subset of inisetref which is only made of matched points
//! \param  []  subsetcur a subset of inisetcur which is only made of matched points
//! \param  []  subsetidx a list of indices from subsetcur to inisetcur.
//! \param  []  matches a list of n (n=card(iniset3d)) matches (index between -1 and card(ini2dset))
//! \param  []  inisetref initial ref set
//! \param  []  inisetcur initial current set
//! \return an error code
ROX_API Rox_ErrorCode rox_match_float_extract_subset2d2d_with_indices(Rox_DynVec_Point2D_Float subsetref, Rox_DynVec_Point2D_Float subsetcur, Rox_DynVec_Uint subsetidx, Rox_DynVec_Sint matches, Rox_DynVec_Point2D_Float inisetref, Rox_DynVec_Point2D_Float inisetcur);


#endif
