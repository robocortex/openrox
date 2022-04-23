//==============================================================================
//
//    OPENROX   : File caofile_edges.h
//
//  	Contents  : API of caofile_edges module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_CAOFILE_EDGES__
#define __OPENROX_CAOFILE_EDGES__

#include <generated/array2d_double.h>
#include <baseproc/geometry/segment/segment3d_struct.h>
#include <baseproc/geometry/ellipse/ellipse3d.h>
#include <baseproc/geometry/ellipse/ellipse3d_struct.h>
#include <baseproc/geometry/cylinder/cylinder3d.h>
#include <baseproc/geometry/cylinder/cylinder3d_struct.h>

#include <system/memory/datatypes.h>

//!\addtogroup 3dfile
//!@{

typedef struct Rox_CaoFile_Edges_Struct * Rox_CaoFile_Edges;

//!Create a new caofile edges structure
//!\param [in,out] obj the new object created
//!\return an error code
ROX_API Rox_ErrorCode rox_caofile_edges_new(Rox_CaoFile_Edges * obj);

//!Create a new caofile edges structure
//!\param [in,out] obj the object deleted
//!\return an error code
ROX_API Rox_ErrorCode rox_caofile_edges_del(Rox_CaoFile_Edges * obj);

//!Load model in structure
//!\param [in,out] obj the caofile structure
//!\param [in] filename the model filename
//!\return an error code
ROX_API Rox_ErrorCode rox_caofile_edges_load_cao(Rox_CaoFile_Edges obj, const Rox_Char * filepath);

//!Load model in structure from string
//!\param [in,out] obj the caofile structure
//!\param [in] inline_model_string the string containing the raw model
//!\return an error code
ROX_API Rox_ErrorCode rox_caofile_edges_load_cao_from_string(Rox_CaoFile_Edges obj, const Rox_Char * inline_model_string);


//!For debug purposes
//!\param [in] obj the caofile structure
//!\return an error code
ROX_API Rox_ErrorCode rox_caofile_edges_print(Rox_CaoFile_Edges obj);

//! \brief Estimate the visible 3D subsegments from projected segments in the image
//! Use rox_caofile_edges_get_contours to get the resulted contours
//! \param  [out] obj 							The caofile structure
//! \param  [in ] pose 						The pose matrix used to get this 2d segmetns
//! \param  [in ] fu 							The focal length in pixels
//! \param  [in ] fv 							The focal length in pixels
//! \param  [in ] cu 							The offaxis u parameters
//! \param  [in ] cv 							The offaxis v parameters
//! \param  [in ] minimal_segment_size 	The minimal size of a segment
//! \return An error code
ROX_API Rox_ErrorCode rox_caofile_edges_estimate_segments(Rox_CaoFile_Edges obj, Rox_Array2D_Double pose, Rox_Double fu, Rox_Double fv, Rox_Double cu, Rox_Double cv, Rox_Uint minimal_segment_size);

//! \brief Adds all 3D subsegments to visible contours (mainly for debug purposes)
//! Use rox_caofile_edges_get_contours to get the resulted contours
//! \param [in/out] obj 							The caofile structure
//! \return An error code
ROX_API Rox_ErrorCode rox_caofile_edges_estimate_all_segments(Rox_CaoFile_Edges caofile_edges);

//! Retrieve contours data computed by rox_caofile_edges_estimate_segments or rox_caofile_edges_estimate_all_segments
//! \param  [out]  contours       3D contours
//! \param  [out]  csize          Contours count
//! \param  [in ]  obj            The caofile structure
//! \return An error code
ROX_API Rox_ErrorCode rox_caofile_edges_get_contours (
   Rox_Segment3D_Struct ** contours, 
   Rox_Uint * csize, 
   Rox_CaoFile_Edges obj
);

//! \brief Estimate the visible 3D ellipses from projected ellipses in the image
//! \param [in] obj 							The caofile structure
//! \param [in] pose 						The pose matrix used to get this 2d segmetns
//! \param [in] fu 							The focal length in pixels
//! \param [in] fv 							The focal length in pixels
//! \param [in] cu 							The offaxis u parameters
//! \param [in] cv 							The offaxis v parameters
//! \param [in] minimal_ellipse_size 	The minimal size of a segment
//! \return An error code
ROX_API Rox_ErrorCode rox_caofile_edges_estimate_visible_ellipses(Rox_CaoFile_Edges obj, Rox_Array2D_Double pose, Rox_Double fu, Rox_Double fv, Rox_Double cu, Rox_Double cv, Rox_Uint minimal_ellipse_size);

//! Retrieve computed ellipses data
//! \param [out]	ellipses 	The 3D ellipses
//! \param [out] 	size 			The number of visible ellipses
//! \param [in] 	obj 			The caofile_edges structure
//! \return an error code
ROX_API Rox_ErrorCode rox_caofile_edges_get_visible_ellipses(Rox_Ellipse3D ** ellipses, Rox_Uint * size, Rox_CaoFile_Edges obj);

//! \brief Estimate the visible 3D cylinders from projected cylinders in the image
//! \param [in] obj 							The caofile structure
//! \param [in] pose 						The pose matrix used to get this 2d segmetns
//! \param [in] fu 							The focal length in pixels
//! \param [in] fv 							The focal length in pixels
//! \param [in] cu 							The offaxis u parameters
//! \param [in] cv 							The offaxis v parameters
//! \param [in] minimal_ellipse_size 	The minimal size of a segment
//! \return An error code
ROX_API Rox_ErrorCode rox_caofile_edges_estimate_visible_cylinders(Rox_CaoFile_Edges obj, Rox_Array2D_Double pose, Rox_Double fu, Rox_Double fv, Rox_Double cu, Rox_Double cv, Rox_Uint minimal_segment_size);

//! Retrieve computed cylinders data
//! \param [out]	cylinders 	The 3D cylinders
//! \param [out] 	size 			The number of visible cylinders
//! \param [in] 	obj 			The caofile_edges structure
//! \return an error code
ROX_API Rox_ErrorCode rox_caofile_edges_get_visible_cylinders(Rox_Cylinder3D ** cylinders, Rox_Uint * size, Rox_CaoFile_Edges obj);

/*
//! Serialize a cao file container
//! \param [out] ser the stream to write to
//! \param [in] obj the cao file container to save
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_caofile_edges_serialize(char* ser, const Rox_CaoFile_Edges obj);

//! Deserialize a cao file container
//! \param [in] obj the loaded cao file container
//! \param [in] ser the stream to load from
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_caofile_edges_deserialize(Rox_CaoFile_Edges obj, const char * ser);

//! Get the size in octets of the cao file container object.
//! \param [out] size The structure size in bytes
//! \param [in] obj the cao file container object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_caofile_edges_get_byte_size(Rox_Uint* size, const Rox_CaoFile_Edges obj);
//! @}
*/
#endif //__ROX_CAOFILE_EDGES__
