//==============================================================================
//
//    OPENROX   : File database.c
//
//    Contents  : Implementation of database module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "database.h"
#include "database_struct.h"
#include "database_item_struct.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_database_new(Rox_Database * db)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Database ret = NULL;

   if(!db) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   *db = 0;

   ret = (Rox_Database)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->compiler = 0;
   ret->database = 0;

   error = rox_ehid_database_new(&ret->database);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_ehid_compiler_new(&ret->compiler);
   ROX_ERROR_CHECK_TERMINATE(error)

   *db = ret;

function_terminate:
   if(error) rox_database_del(&ret);

   return error;
}

Rox_ErrorCode rox_database_del(Rox_Database * db)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Database todel;

   if (!db)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *db;
   *db = 0;

   if(!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_ehid_database_del(&todel->database);
   rox_ehid_compiler_del(&todel->compiler);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_database_add_item(Rox_Database db, Rox_Database_Item item, const Rox_Double sizx, const Rox_Double sizy)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!db || !item)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (sizx < 0 || sizy < 0)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // TODO: rename to rox_ehid_compiler_add_item
   error = rox_ehid_compiler_add_db(db->compiler, item->dbpoints, item->dbindices, item->cols, item->rows, sizx, sizy);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_database_add_item_default_size(Rox_Database db, Rox_Database_Item item)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!db || !item)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // TODO: rename to rox_ehid_compiler_add_item
   error = rox_ehid_compiler_add_db(db->compiler, item->dbpoints, item->dbindices, item->cols, item->rows, item->cols, item->rows);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_database_compile(Rox_Database db)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!db)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_ehid_compiler_compile(db->database, db->compiler);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_database_save(char * filename, const Rox_Database db)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!filename || !db)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_ehid_database_save(db->database, filename);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_database_load(Rox_Database db, const char * filename)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!filename || !db)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_ehid_database_load(db->database, filename);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_database_serialize(char * buffer, const Rox_Database db)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!buffer || !db)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_ehid_database_serialize(buffer, db->database);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_database_deserialize(Rox_Database db, const char * buffer)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!buffer || !db)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_ehid_database_deserialize(db->database, buffer);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_database_get_structure_size(Rox_Uint * size, const Rox_Database db)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!size || !db)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_ehid_database_get_octet_size(size, db->database);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
