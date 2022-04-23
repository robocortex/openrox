//==============================================================================
//
//    OPENROX   : File ehid_database.h
//
//    Contents  : API of ehid_database module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EHID_DATABASE__
#define __OPENROX_EHID_DATABASE__

#include <stdio.h>

#include <generated/dynvec_ehid_dbnode.h>
#include <generated/dynvec_ehid_point.h>
#include <generated/dynvec_ehid_match.h>
#include <generated/objset_ehid_target.h>

#include <baseproc/image/image.h>

#include <core/features/descriptors/ehid/ehid_dbindex.h>
#include <core/features/descriptors/ehid/ehid_searchtree.h>

//! \addtogroup EHID
//! @{

//! EHID Database structure 
typedef struct Rox_Ehid_Database_Struct * Rox_Ehid_Database;

//! Create a new empty database
//! \param obj a pointer to the newly created object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_database_new(Rox_Ehid_Database * obj);

//! Delete a database
//! \param obj a pointer to the object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_database_del(Rox_Ehid_Database * obj);

//! Reset a database
//! \param obj a pointer to the object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_database_reset(Rox_Ehid_Database obj);

//! Test a database for theorical validity
//! \param score the estimated score for this database (between 0 and 1, 1 being the best score)
//! \param db a pointer to the database object
//! \param source the original image used to compute the database
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_database_test(Rox_Double * score, Rox_Ehid_Database db, Rox_Image source);

//! Test a database for theorical validity (fast and inaccurate)
//! \param pscore a pointer to the estimated score for this database (between 0 and 1, 1 being the best score)
//! \param source the original image used to compute the database
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_database_testfast(Rox_Double * pscore, Rox_Image source);

//! Save a database
//! \param [in] db the database to save
//! \param [in] filename the input file path
//! \return An error code
//! \todo to be tested
 
ROX_API Rox_ErrorCode rox_ehid_database_save(Rox_Ehid_Database db, const Rox_Char * filename);

//! Load a database
//! \param [in] db the database to load into
//! \param [in] filename the input file path
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_database_load(Rox_Ehid_Database db, const Rox_Char * filename);

//! Serialize a database
//! \param [out] ser the stream to write to
//! \param [in] db the database to save
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_database_serialize(char* ser, const Rox_Ehid_Database db);

//! Deserialize a database
//! \param [in] db the loaded db
//! \param [in] ser the stream to load from
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_database_deserialize(Rox_Ehid_Database db, const char * ser);

//! Get the size in octets of the database object.
//! \param [out] size The structure size in octets
//! \param [in] db The database object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_database_get_octet_size(Rox_Uint* size, const Rox_Ehid_Database db);

//! @} 

#endif
