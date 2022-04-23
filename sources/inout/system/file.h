//==============================================================================
//
//    OPENROX   : File file.h
//
//    Contents  : API of print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_FILE__
#define __OPENROX_FILE__

#include <system/memory/datatypes.h>
#include <system/errors/errors.h>

ROX_API Rox_ErrorCode rox_file_count_lines ( Rox_Size * lines_number, const Rox_Char * filename );

#endif