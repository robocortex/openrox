//============================================================================
//
//    OPENROX   : File filepath.h
//
//    Contents  : API of filepath module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_FILEPATH__
#define __OPENROX_FILEPATH__

#include <system/memory/datatypes.h>

//! \ingroup Strings
//! \addtogroup Filepath
//! \brief Filapath string manipulation
//! @{

//! Split file path into directory, name, and extension
//! \param  [out]  dir_           The pointer to the extracted file directory
//! \param  [out]  name_          The pointer to the extracted file name
//! \param  [out]  ext_           The pointer to the extracted file extension
//! \param  [in ]  filepath       The input file path
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_filepath_split(Rox_Char * dir_, Rox_Char *name_, Rox_Char *ext_, const Rox_Char *filepath);

ROX_API Rox_ErrorCode rox_filepath_extract(Rox_Char * dir, Rox_Char * name, Rox_Char * ext, const Rox_Char * filepath);

//! @} 

#endif
