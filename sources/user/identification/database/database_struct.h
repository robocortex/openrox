//==============================================================================
//
//    OPENROX   : File database_struct.h
//
//    Contents  : Structure of database module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DATABASE_STRUCT__
#define __OPENROX_DATABASE_STRUCT__

#include <core/features/descriptors/ehid/ehid_database.h>
#include <core/features/descriptors/ehid/ehid_compiler.h>

//! \addtogroup Database
//! @{

//! The Rox_Database_Struct object 
struct Rox_Database_Struct
{
   //! The database compiler 
   Rox_Ehid_Compiler 	compiler;

   //! The set of items 
   Rox_Ehid_Database 	database;
};

//! @} 

#endif // __OPENROX_DATABASE_STRUCT__ 
