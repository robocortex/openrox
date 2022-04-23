//==============================================================================
//
//    OPENROX   : File dbident_se3_struct.h
//
//    Contents  : Structure of dbident_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DB_IDENT_SE3_STRUCT__
#define __OPENROX_DB_IDENT_SE3_STRUCT__

#include <generated/array2d_double.h>
#include <generated/dynvec_segment_point.h>
#include <generated/dynvec_segment_point_struct.h>
#include <generated/dynvec_ehid_point.h>
#include <generated/dynvec_ehid_point_struct.h>
#include <generated/dllist_quadtree_item.h>

#include <baseproc/image/pyramid/pyramid_uchar.h>

#include <core/occupancy/quadtree_ref.h>
#include <core/features/descriptors/ehid/ehid_database.h>
#include <core/features/descriptors/ehid/ehid_database_struct.h>
#include <core/features/descriptors/ehid/ehid_matcher.h>

//! \ingroup Identification_Database
//! \addtogroup db_ident_se3
//! @{

//! The Rox_DB_Ident_SE3_Struct object 
struct Rox_DB_Ident_SE3_Struct
{
   //! Input pyramid 
   Rox_Pyramid_Uchar _pyramid;

   //! Raw points
   Rox_DynVec_Segment_Point _fast_points;

   //! Maxima points
   Rox_DynVec_Segment_Point _fast_points_nonmax;

   //! Per level points
   Rox_DynVec_Ehid_Point _curfeats;

   //! All points
   Rox_DynVec_Ehid_Point _curfeats_pyr;

   //! Occupancy checker
   Rox_QuadTree_Ref _quad;

   //! List results
   Rox_Dllist_QuadTree_Item _list_results;

   //! Pointer to externally allocated db
   Rox_Ehid_Database _database;

   //! Matcher
   Rox_Ehid_Matcher _matcher;
};

//! @} 

#endif
