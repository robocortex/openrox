//==============================================================================
//
//    OPENROX   : File tlid.h
//
//    Contents  : API of tlid module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TLID__
#define __OPENROX_TLID__

#include <system/memory/datatypes.h>
#include <generated/dynvec_tlid_segment.h>
#include <generated/dynvec_tlid_segment_struct.h>
#include <generated/dynvec_segment2d.h>
#include <generated/array2d_point2d_sshort.h>
#include <generated/dynvec_point2d_double.h>

//! \ingroup Descriptors
//! \defgroup TLID TLID
//! \brief SRAID edges descriptor.

//! \addtogroup TLID
//! @{

//! The Rox_Tlid_Struct object

struct Rox_Tlid_Struct
{
	 //! Width 
    Rox_Sint width;
	 //! Height 
    Rox_Sint height;
 	 //! Segments 
    Rox_DynVec_TLID_Segment segments;
};

//! Define the pointer of the Rox_Tlid_Struct 
typedef struct Rox_Tlid_Struct * Rox_Tlid;

//! Create tlid descriptor object
//! \param [out] obj the pointer to the object
//! \param [in] width the width
//! \param [in] height the height
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_tlid_new(Rox_Tlid * obj, Rox_Sint width, Rox_Sint height);

//! Delete tlid  object
//! \param [in] obj the pointer to the object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_tlid_del(Rox_Tlid * obj);

//! Process edge tlid descriptor on edges
//! \param [in] obj the pointer to the object
//! \param [in] segments the list of segments
//! \param [in] gradients the image of gradients
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_tlid_process(Rox_Tlid obj, Rox_DynVec_Segment2D segments, Rox_Array2D_Point2D_Sshort gradients);

//! @} 

#endif // __OPENROX_TLID__

