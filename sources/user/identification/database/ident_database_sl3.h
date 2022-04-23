//==============================================================================
//
//    OPENROX   : File ident_database_sl3.h
//
//    Contents  : API of ident_database_sl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IDENT_DATABASE_SL3__
#define __OPENROX_IDENT_DATABASE_SL3__

#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/image/image.h>
#include <user/identification/database/database.h>

//! \ingroup Identification
//! \addtogroup ident_database_sl3
//! @{

//! Define the pointer of the Rox_DB_Ident_SL3_Struct
typedef struct Rox_DB_Ident_SL3_Struct * Rox_Ident_Database_SL3;

//! Create a new object for identification of a plane on SL(3)
//! \param  [out] ident 						   The object to create
//! \param  [in]  max_templates_simultaneous How many templates may be found in one image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_sl3_new(Rox_Ident_Database_SL3 * ident, Rox_Uint max_templates_simultaneous);

//! Delete an identification object
//! \param [out]	   ident 						The object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_sl3_del(Rox_Ident_Database_SL3 * ident);

//! Set database to search inside
//! \param [out]  	ident 						The created identification object
//! \param [in]   	db 							The database to use (shallow copy)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_sl3_set_database(Rox_Ident_Database_SL3 ident, Rox_Database db);

//! Process the detection
//! \param  [in] 	ident 						   The created identification object
//! \param  [in] 	source 						   The image where the template is to be found
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_sl3_make(Rox_Ident_Database_SL3 ident, Rox_Image source);

//! Extract the features of the current image
//! \param  [in] 	features 					The extraction result
//! \param  [in] 	ident 						The created identification object
//! \param  [in] 	image 						The image where the template is to be found
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_sl3_extract_features(Rox_DynVec_Ehid_Point features, Rox_Ident_Database_SL3 ident, Rox_Image image);

//! Process detection from a set of features.
//! \param  [in] 	ident 						The created identification object
//! \param  [in] 	features 					Set of features extracted from the current image to found in the database
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_sl3_make_features(Rox_Ident_Database_SL3 ident, Rox_DynVec_Ehid_Point features);

//! Get the detection results
//! \param  [out] is_identified 		     The pointer to the result boolean (0 if not identified, otherwise identified)
//! \param  [out] homography 			     If the template is identified, contains the homography
//! \param  [in] 	ident 				     The identification object
//! \param  [in] 	id 					     The sequential template identification
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_sl3_getresult(Rox_Sint * is_identified, Rox_MatSL3 homography, Rox_Ident_Database_SL3 ident, Rox_Uint id);

//! Get the identification results rescaling the homography with a desired size along x-axis
//! \param  [out] is_identified        The pointer to the result boolean (0 if not identified, otherwise identified)
//! \param  [out] homography           If the template is identified, contains the pose
//! \param  [in]  ident                The identification object
//! \param  [in]  sizeu                The desired size of the 3D plane along the x axis
//! \param  [in]  id                   The sequential template identification
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_sl3_get_result_force_sizeu(Rox_Sint * is_identified, Rox_MatSL3 homography, Rox_Ident_Database_SL3 ident, Rox_Double sizeu, Rox_Uint id);

//! Get the identification results rescaling the homography with a desired size along y-axis
//! \param  [out] is_identified     The pointer to the result boolean (0 if not identified, otherwise identified)
//! \param  [out] homography        If the template is identified, contains the pose
//! \param  [in]  ident             The identification object
//! \param  [in]  sizev             The desired size of the 3D plane along the y axis
//! \param  [in]  id                The sequential template identification
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_sl3_get_result_force_sizev(Rox_Sint * is_identified, Rox_MatSL3 homography, Rox_Ident_Database_SL3 ident, Rox_Double sizev, Rox_Uint id);

//! Get the detection score for a given target id
//! \param [out]    score          				The score of detection for this target
//! \param [in]     ident          				The identification object
//! \param [in]    	id             				The sequential template identification
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_sl3_getscore(Rox_Double * score, Rox_Ident_Database_SL3 ident, Rox_Uint id);

//! Retrieve number of template added
//! \param [out] 	count 						The pointer to the result counter
//! \param [in] 	ident 						The created identification object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_database_sl3_getcountframes(Rox_Uint *count, Rox_Ident_Database_SL3 ident);

//! @}

#endif // __OPENROX_IDENT_DATABASE_SL3__

