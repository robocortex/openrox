//==============================================================================
//
//    OPENROX   : File database_features.h
//
//    Contents  : API of database_features module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DATABASE_FEATURES__
#define __OPENROX_DATABASE_FEATURES__

#include <generated/dynvec_ehid_point.h>
#include <baseproc/geometry/point/point2d.h>

//! \ingroup  Identification                                                                                                                                              
//! \addtogroup Database_Features
//! @{

//! Define the pointer of the Rox_Database_Features_Struct 
typedef struct Rox_DynVec_Ehid_Point_Struct * Rox_Database_Features;

//! Create a new database_features object
//! \param [out] features        The object to create
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_database_features_new(Rox_Database_Features * features);

//! Delete a database_features object
//! \param [in]   features       The object to delete
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_database_features_del(Rox_Database_Features * features);

//! Save the features data into a binary file
//! \param [out]  filename       The filename to save the object
//! \param [in]   features       The object to save
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_features_save(Rox_Char * filename, Rox_Database_Features features);

//! Load a binary file into a database_features object
//! \param [in]   features       The object to fill
//! \param [in]   filename       The filename to read
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_features_load(Rox_Database_Features features, const Rox_Char * filename);

//! Serialize the database_features object and fill a char buffer
//! \param [in]   buffer         The buffer to serialize the structure
//! \param [in]   features       The object to serialize
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_features_serialize(Rox_Char * buffer, Rox_Database_Features features);

//! Deserialize a buffer into a database_features structure
//! \param [in]   features       The object to fill
//! \param [in]   buffer         The serialized buffer
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_features_deserialize(Rox_Database_Features features, Rox_Char * buffer);

//! Get the database_features structure size in octets
//! \param [out]  size           The structure size in octets
//! \param [in]   features       The database_features object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_features_get_structure_size(Rox_Uint * size, Rox_Database_Features features);

//! Get the database_features number of features
//! \param [out]  size           The number of features
//! \param [in]   features       The database_features object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_features_get_points_size(Rox_Uint * size, Rox_Database_Features database_features);

//! Get the points in database_features
//! \param [out]  points         The points coordinates
//! \param [in]   features       The database_features object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_features_get_points_copy(Rox_Point2D_Double points, Rox_Uint size, Rox_Database_Features database_features);

//! @} 

#endif // __OPENROX_DATABASE_FEATURES__
