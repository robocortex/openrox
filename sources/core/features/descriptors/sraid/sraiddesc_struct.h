//==============================================================================
//
//    OPENROX   : File sraiddesc_struct.h
//
//    Contents  : API of sraiddesc_struct module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SRAID_DESC_STRUCT__
#define __OPENROX_SRAID_DESC_STRUCT__

//! \addtogroup SRAID
//! @{

//! Define the SRAID descriptor size
#define ROX_SRAID_DESCRIPTOR_SIZE 128

//! SRAID Feature description
struct Rox_SRAID_Feature_Struct
{
   //! Description vector for the feature
   //! \return define

   Rox_Ushort ROX_STATIC_ALIGN(16) descriptor[ROX_SRAID_DESCRIPTOR_SIZE];

   //! Optional object ID
   Rox_Sint object_id;

   //! Optional object ID
   Rox_Sint is_minimal;

   //! X coordinate in image space
   Rox_Float x;

   //! Y coordinate in image space
   Rox_Float y;

   //! orientation of the feature
   Rox_Float ori;

   //! detection level of the feature
   Rox_Sint level;
};

//! typedef struct SRAID Feature description
typedef struct Rox_SRAID_Feature_Struct Rox_SRAID_Feature_Struct;

//! typedef struct SRAID Feature description
typedef struct Rox_SRAID_Feature_Struct * Rox_SRAID_Feature;

//! @}

#endif
