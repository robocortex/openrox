//==============================================================================
//
//    OPENROX   : File database.h
//
//    Contents  : API of database module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DATABASE__
#define __OPENROX_DATABASE__

#include <core/features/descriptors/ehid/ehid_database.h>
#include <core/features/descriptors/ehid/ehid_compiler.h>
#include <user/identification/database/database_item.h>

//! \addtogroup Database
//! @{

//! Define the pointer of the Rox_Database_Struct 
typedef struct Rox_Database_Struct * Rox_Database;

//! Create a database object
//! \param [out]  database       The object to create
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_new(Rox_Database * database);

//! Delete a database object
//! \param [out]  database       The object to delete
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_database_del(Rox_Database * database);

//! Add a template into the database set
//! \param [out]  database       The database object
//! \param [in ]  item           The item to add
//! \param [in ]  sizx           The template size in meters along the x-axis
//! \param [in ]  sizy           The template size in meters along the y-axis
//! \return An error code
//! \todo   To be tested, rename rox_database_add_item -> rox_database_add_item_setting_size
ROX_API Rox_ErrorCode rox_database_add_item(Rox_Database database, const Rox_Database_Item item, const Rox_Double sizx, const Rox_Double sizy);

//! Add a template into the database set
//! \param [out]  database       The database object
//! \param [in ]  item           The item to add
//! \return An error code
//! \todo   To be tested, rename rox_database_add_item_auto_size -> rox_database_add_item
ROX_API Rox_ErrorCode rox_database_add_item_default_size(Rox_Database database, const Rox_Database_Item item);

//! Compile added items into a common database for multi target detection (Mandatory before any identification process)
//! \param [out]  database       	The database object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_compile(Rox_Database database);

//! Save into a binary file the compiled database
//! \param [out] 	filename       The filename to save the compiled database
//! \param [in ] 	database       The database object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_save (char * filename, const Rox_Database database);

//! Load a compiled database from a binary file
//! \param [out]  database       The database object to set
//! \param [in ]  filename       The filename .rdb to read
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_load (Rox_Database database, const char * filename);

//! Serialize the compiled database into a char buffer
//! \param [out]  buffer         The char buffer to use for the serialization.
//! \param [in ]  database       The object to serialize
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_serialize (char * buffer, const Rox_Database database);

//! Load a precompiled database to the object from a buffer
//! \param [out]  database       The object to set
//! \param [in ]  buffer         The serialized buffer
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_deserialize (Rox_Database database, const char * buffer);

//! Get the size in octets of the compiled object.
//! \param [out]  size           The compiled structure size in octets
//! \param [in ]  database         The compiled database
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_database_get_structure_size (Rox_Uint * size, const Rox_Database database);

//! @} 

#endif // __OPENROX_DATABASE__ 
