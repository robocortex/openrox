//==============================================================================
//
//    OPENROX   : File dbident_sl3.h
//
//    Contents  : API of dbident_sl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DB_IDENT_SL3__
#define __OPENROX_DB_IDENT_SL3__

#include <generated/array2d_double.h>
#include <generated/dllist_quadtree_item.h>
#include <generated/dynvec_segment_point.h>
#include <generated/dynvec_segment_point_struct.h>
#include <generated/dynvec_ehid_point.h>
#include <generated/dynvec_ehid_point_struct.h>

#include <baseproc/image/pyramid/pyramid_uchar.h>
#include <baseproc/image/image.h>

#include <core/occupancy/quadtree_ref.h>
#include <core/features/descriptors/ehid/ehid_database.h>
#include <core/features/descriptors/ehid/ehid_database_struct.h>
#include <core/features/descriptors/ehid/ehid_matcher.h>

//! \ingroup Identification_Database
//! \addtogroup db_ident_sl3
//! @{

//! The Rox_DB_Ident_SL3_Struct structure
struct Rox_DB_Ident_SL3_Struct
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

//! The Rox_DB_Ident_SL3 object 
typedef struct Rox_DB_Ident_SL3_Struct * Rox_DB_Ident_SL3;

//! Create a new object for identification of a plane on SL(3)
//! \param obj is the newly created identification object
//! \param max_templates_simultaneous how many templates may be found in one image
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_db_ident_sl3_new(Rox_DB_Ident_SL3 * database_ident_sl3, Rox_Uint max_templates_simultaneous);

//! Delete an identification object
//! \param obj is the created identification object to delete
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_db_ident_sl3_del(Rox_DB_Ident_SL3 * database_ident_sl3);

//! Extract points from an image
//! \param obj is the created identification object
//! \param source is the image to extract points into
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_db_ident_sl3_extract(Rox_DB_Ident_SL3 database_ident_sl3, Rox_Image source);

//! Set database to search inside
//! \param database_ident_sl3       The created identification object
//! \param ehid_database            The database to use (shallow copy)
//! \return    An error code
//! \warning   Shallow copy of the database, delete ehid_database after deleting database_ident_sl3
//! \todo      To be tested
ROX_API Rox_ErrorCode rox_db_ident_sl3_set_database(Rox_DB_Ident_SL3 database_ident_sl3, Rox_Ehid_Database ehid_database);

//! Estimate targets homographies
//! \param obj is the created identification object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_db_ident_sl3_estimate_homographies(Rox_DB_Ident_SL3 database_ident_sl3);

//! Set extracted features
//! \param obj is the created identification object
//! \param curfeats is the extracted points to be matched
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_db_ident_sl3_set_extracted_features(Rox_DB_Ident_SL3 database_ident_sl3, Rox_DynVec_Ehid_Point curfeats);

//! Copy the extracted features into buffer
//! \param buffer is the serialization ouptut
//! \param obj is the created identification object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_db_ident_sl3_serialize_extracted_features(char * buffer, Rox_DB_Ident_SL3 database_ident_sl3);

//! Get the octet size of the extracted features
//! \param size is the serialization buffer size
//! \param obj is the created identification object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_db_ident_sl3_get_extracted_features_size(Rox_Uint * size, Rox_DB_Ident_SL3 database_ident_sl3);

//! @}

#endif // __OPENROX_DB_IDENT_SL3__
