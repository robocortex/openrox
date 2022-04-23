//==============================================================================
//
//    OPENROX   : File database_item.h
//
//    Contents  : API of database_item module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DATABASE_ITEM__
#define __OPENROX_DATABASE_ITEM__

#include <generated/dynvec_ehid_point.h>
#include <generated/dynvec_ehid_dbindex.h>

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/image/image.h>

//! \ingroup Identification
//! \defgroup Database

//! \ingroup Database
//! \addtogroup Database_Item
//! @{

//! Define the pointer of the Rox_Database_Item_Struct 
typedef struct Rox_Database_Item_Struct * Rox_Database_Item;

//! Create a new database_item object
//! \param  [out] item              The object to create
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_item_new(Rox_Database_Item * item);

//! Delete a database_item object
//! \param  [out] item 				The object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_item_del(Rox_Database_Item * item);

//! Learn the given template and fill the database_item object
//! \param  [out] item              The object to fill
//! \param  [in]  image_template    The template to learn
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_item_learn_template(Rox_Database_Item item, const Rox_Image image_template);

//! Save the item data into a binary file
//! \param  [out] filename          The filename to save the object
//! \param  [in]  item              The object to save
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_item_save(const Rox_Char * filename, const Rox_Database_Item item);

//! Load a binary file into a database_item object
//! \param  [out] item              The object to fill
//! \param  [in]  filename          The filename to read
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_item_load(Rox_Database_Item item, const Rox_Char * filename);

//! Serialize the database_item object and fill a char buffer
//! \param  [out] buffer            The buffer to serialize the structure
//! \param  [in]  item              The object to serialize
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_item_serialize(Rox_Char * buffer, const Rox_Database_Item item);

//! Deserialize a buffer into a database_item structure
//! \param  [out] item              The object to fill
//! \param  [in]  buffer            The serialized buffer
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_item_deserialize(Rox_Database_Item item, const Rox_Char * buffer);

//! Get the database_item structure size in octets
//! \param  [out] size              The structure size in octets
//! \param  [in]  item              The database_item object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_item_get_structure_size(Rox_Uint * size, const Rox_Database_Item item);

//! Get the database_features number of features
//! \param  [out] size           	The number of features
//! \param  [in]  features       	The database_features object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_item_get_points_size(Rox_Uint * size, Rox_Database_Item database_item);

//! Get the points in database_features
//! \param  [out] points         	The points coordinates
//! \param  [in]  features       	The database_features object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_item_get_points_copy(Rox_Point2D_Double points, Rox_Uint size, Rox_Database_Item database_item);

// TODO for DATABASE Project

//! Set the parameters to learn a template
//! \param  [out] item              The object to set
//! \param  [in]  scales            The increasing scales for learning. Default values are [1,2,3,4,5,6,7,8,9]
//! \param  [in]  angle_max         The maximum angle from the vertical position in degrees.
//! \param  [in]  sigma             The standard deviatio of the Gaussian noise added to the images
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_item_set_params(Rox_Database_Item item, const Rox_Double scales[9], const Rox_Double angle_max, const Rox_Double sigma);

//! @} 

#endif // __OPENROX_DATABASE_ITEM__ 
