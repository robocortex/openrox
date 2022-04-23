//==============================================================================
//
//    OPENROX   : File image_score.c
//
//    Contents  : Implementation of image_score module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "image_score.h"

#include <core/features/descriptors/ehid/ehid_database.h>

#include <inout/system/errors_print.h>

#include <user/identification/database/database.h>
#include <user/identification/database/database_struct.h>
#include <user/identification/database/database_item.h>


Rox_ErrorCode rox_image_get_quality_score(Rox_Double * score, Rox_Image image)
{
    return rox_ehid_database_testfast(score, image);
}

Rox_ErrorCode rox_image_get_quality_score_precise(Rox_Double * score, Rox_Image image)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    Rox_Database database = NULL;
    Rox_Database_Item item = NULL;

    if (!score || !image) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    error = rox_database_new(&database); 
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_database_item_new(&item); 
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_database_item_learn_template(item, image); 
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_database_add_item_default_size(database, item);
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_database_compile(database); 
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_ehid_database_test(score, database->database, image); 
    ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
    rox_database_del(&database);
    rox_database_item_del(&item);

    return error;
}
