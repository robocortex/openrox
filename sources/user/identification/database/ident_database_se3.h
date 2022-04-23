//==============================================================================
//
//    OPENROX   : File ident_database_se3.h
//
//    Contents  : API of ident_database_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IDENT_DATABASE_SE3__
#define __OPENROX_IDENT_DATABASE_SE3__

#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/maths/linalg/matse3.h>
#include <core/identification/dbident_se3.h>
#include <user/identification/database/database.h>
#include <user/sensor/camera/camera.h>

//! \ingroup Identification
//! \addtogroup ident_database_se3
//! @{

//! Define the pointer of the Rox_DB_Ident_SE3_Struct
typedef struct Rox_DB_Ident_SE3_Struct * Rox_Ident_Database_SE3;

//! Create a new object for identification of a plane on SE(3)
//! \param  [out]  ident       The object to create
//! \param  [in ]  max_templates_simultaneous How many templates may be found in one image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_se3_new (
   Rox_Ident_Database_SE3 * ident, 
   const Rox_Uint max_templates_simultaneous
);

//! Delete an identification object
//! \param  [out]  ident       	 The object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_se3_del (
   Rox_Ident_Database_SE3 * ident
);

//! Set database to search inside
//! \param  [out]  ident          The created identification object
//! \param  [in ]  database       The database to use (shallow copy)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_se3_set_database (
   Rox_Ident_Database_SE3 ident,
   Rox_Database database
);

//! Process the identification
//! \param  [in ]  ident          The created identification object
//! \param  [in ]  camera         The camera containing the image where the template is to be found
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_se3_make (
   Rox_Ident_Database_SE3 ident, 
   const Rox_Camera camera
);

//! Extract the features of the current image
//! \param  [in ]  features       The extraction result
//! \param  [in ]  ident          The created identification object
//! \param  [in ]  camera         The camera containing the image where the template is to be found
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_se3_extract_features (
   Rox_DynVec_Ehid_Point features, 
   const Rox_Ident_Database_SE3 ident, 
   const Rox_Camera camera
);

//! Process identification from a set of features.
//! \param  [in ]  ident          The created identification object
//! \param  [in ]  features       Set of features extracted from the current image to found in the database
//! \param  [in ]  calib_camera   The camera classic calibration
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_se3_make_features (
   Rox_Ident_Database_SE3 ident, 
   const Rox_DynVec_Ehid_Point features, 
   const Rox_Matrix calib_camera
);

//! Get the identification results, the pose translation is scaled using the sizex and size y in the database
//! \param  [out]  is_identified  The pointer to the result boolean (0 if not identified, otherwise identified)
//! \param  [out]  pose           If the template is identified, contains the pose
//! \param  [in ]  ident          The identification object
//! \param  [in ]  id             The sequential template identification
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_se3_getresult (
   Rox_Sint * is_identified, 
   Rox_MatSE3 pose, 
   const Rox_Ident_Database_SE3 ident, 
   const Rox_Uint id
);

//! Get the identification results rescaling the pose with a desired size along x-axis 
//! \param  [out]  is_identified  The pointer to the result boolean (0 if not identified, otherwise identified)
//! \param  [out]  pose           If the template is identified, contains the pose
//! \param  [in ]  ident          The identification object
//! \param  [in ]  sizex          The desired size of the 3D plane along the x axis
//! \param  [in ]  id             The sequential template identification
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_se3_get_result_force_sizex (
   Rox_Sint * is_identified, 
   Rox_MatSE3 pose, 
   const Rox_Ident_Database_SE3 ident, 
   const Rox_Double sizex, Rox_Uint id
);

//! Get the identification results rescaling the pose with a desired size along y-axis 
//! \param  [out]  is_identified  The pointer to the result boolean (0 if not identified, otherwise identified)
//! \param  [out]  pose           If the template is identified, contains the pose
//! \param  [in ]  ident          The identification object
//! \param  [in ]  sizey          The desired size of the 3D plane along the y axis
//! \param  [in ]  id             The sequential template identification
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_se3_get_result_force_sizey (
   Rox_Sint * is_identified, 
   Rox_MatSE3 pose, 
   const Rox_Ident_Database_SE3 ident, 
   const Rox_Double sizey, 
   const Rox_Uint id
);

//! Get the identification score for a given target id
//! \param  [out]  score          The score of identification for this target
//! \param  [in ]  ident          The identification object
//! \param  [in ]  id             The sequential template identification
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_se3_getscore (
   Rox_Double * score, 
   const Rox_Ident_Database_SE3 ident, 
   const Rox_Uint id
);

//! Retrieve number of template added
//! \param  [out]  count          The pointer to the result counter
//! \param  [in ]  ident          The created identification object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_se3_getcountframes (
   Rox_Uint * count, 
   const Rox_Ident_Database_SE3 ident
);

//! @}

#endif // __OPENROX_IDENT_DATABASE_SE3__
