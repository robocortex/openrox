//==============================================================================
//
//    OPENROX   : File dbident_se3.h
//
//    Contents  : API of dbident_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DB_IDENT_SE3__
#define __OPENROX_DB_IDENT_SE3__

#include <generated/dynvec_segment_point.h>
#include <generated/dynvec_ehid_point.h>
#include <generated/dllist_quadtree_item.h>

#include <baseproc/image/pyramid/pyramid_uchar.h>
#include <baseproc/image/image.h>
#include <baseproc/maths/linalg/matut3.h>

#include <core/occupancy/quadtree_ref.h>
#include <core/features/descriptors/ehid/ehid_database.h>
#include <core/features/descriptors/ehid/ehid_matcher.h>

//! \ingroup  Vision
//! \defgroup Identification Identification
//! \brief Vision-based identification.

//! \ingroup  Identification
//! \defgroup Identification_Database Identification Database
//! \brief Texture database identification.

//! \ingroup Identification_Database
//! \addtogroup db_ident_se3
//! @{

//! Pointer to the structure*/
typedef struct Rox_DB_Ident_SE3_Struct * Rox_DB_Ident_SE3;

//! Create a new object for identification of a plane on SE(3)
//! \param  [out]  database_ident_se3         The newly created identification object
//! \param  [in ]  max_templates_simultaneous How many templates may be found in one image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_db_ident_se3_new (
   Rox_DB_Ident_SE3 * database_ident_se3, 
   const Rox_Uint max_templates_simultaneous
);

//! Delete an identification object
//! \param  [out]  database_ident_se3         The created identification object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_db_ident_se3_del (
   Rox_DB_Ident_SE3 * database_ident_se3
);

//! Extract points from an image
//! \param  [out]  database_ident_se3         The created identification object
//! \param  [in ]  source                     The image to extract points into
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_db_ident_se3_extract (
   Rox_DB_Ident_SE3 database_ident_se3, 
   const Rox_Image source
);

//! Set database to search inside
//! \warning Shallow copy of the database, delete ehid_database after deleting database_ident_se3
//! \param  [out]  database_ident_se3         The created identification object
//! \param  [in ]  ehid_database              The database to use (shallow copy)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_db_ident_se3_set_database (
   Rox_DB_Ident_SE3 database_ident_se3, 
   const Rox_Ehid_Database ehid_database
);

//! Estimate targets poses
//! \param  [out]  database_ident_se3         The created identification object
//! \param  [in ]  calib_camera               The camera intrinsics paramaters
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_db_ident_se3_estimate_poses (
   Rox_DB_Ident_SE3 database_ident_se3, 
   const Rox_MatUT3 calib_camera
);

//! Set extracted features
//! \param  [out]  database_ident_se3         The created identification object
//! \param  [in ]  curfeats                   The extracted points to be matched
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_db_ident_se3_set_extracted_features (
   Rox_DB_Ident_SE3 database_ident_se3, 
   const Rox_DynVec_Ehid_Point curfeats
);

//! Copy the extracted features into buffer
//! \param  [out]  buffer                      The serialization ouptut
//! \param  [in ]  database_ident_se3          The created identification object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_db_ident_se3_serialize_extracted_features (
   char * buffer, 
   const Rox_DB_Ident_SE3 database_ident_se3
);

//! Get the octet size of the extracted features
//! \param  [out]  size                       The serialization buffer size
//! \param  [in ]  database_ident_se3         The created identification object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_db_ident_se3_get_extracted_features_size (
   Rox_Uint * size, 
   const Rox_DB_Ident_SE3 database_ident_se3
);

//! @}

#endif
