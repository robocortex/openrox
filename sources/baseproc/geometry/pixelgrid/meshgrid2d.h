//==============================================================================
//
//    OPENROX   : File meshgrid2d.h
//
//    Contents  : API of meshgrid2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MESHGRID2D__
#define __OPENROX_MESHGRID2D__

#include <inout/system/errors_print.h>

//! \addtogroup MeshGrid2D
//! @{

//! Define the Rox_MeshGrid2D_Float object as a pointer to the Rox_MeshGrid2D_Float structure
typedef struct Rox_MeshGrid2D_Float_Struct * Rox_MeshGrid2D_Float;

//! Define the Rox_MeshGrid2D_Sshort object as a pointer to the Rox_MeshGrid2D_Sshort structure
typedef struct Rox_MeshGrid2D_Sshort_Struct * Rox_MeshGrid2D_Sshort;

//! Constructor for meshgrid2d structure
//! \param  [out]  meshgrid2d     MeshGrid2D object
//! \param  [in ]  rows           Image height in pixels
//! \param  [in ]  cols           Image width in pixels
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_meshgrid2d_float_new ( 
   Rox_MeshGrid2D_Float * meshgrid2d, 
   const Rox_Sint rows, 
   const Rox_Sint cols 
);

//! Destructor for meshgrid2d structure
//! \param  [in ]  meshgrid2d     The object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_meshgrid2d_float_del (
   Rox_MeshGrid2D_Float * meshgrid2d
);

//! Get the size of a meshgrid2d (rows and cols)
//! \param  [out]  rows           The copy of the image rows
//! \param  [out]  cols           The copy of the image cols
//! \param  [in ]  meshgrid2d     The meshgrid2d object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_meshgrid2d_float_get_size (
   Rox_Sint * rows, 
   Rox_Sint * cols, 
   const Rox_MeshGrid2D_Float meshgrid2d
);

ROX_API Rox_ErrorCode rox_meshgrid2d_float_check_size (    
   const Rox_MeshGrid2D_Float meshgrid2d,
   const Rox_Sint rows, 
   const Rox_Sint cols 
); 

// -----------------------------------------------------------

//! Constructor for meshgrid2d structure
//! \param  [out]  meshgrid2d     MeshGrid2D object
//! \param  [in ]  rows           Image height in pixels
//! \param  [in ]  cols           Image width in pixels
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_meshgrid2d_sshort_new ( 
   Rox_MeshGrid2D_Sshort * meshgrid2d, 
   const Rox_Sint rows, 
   const Rox_Sint cols 
);

//! Destructor for meshgrid2d structure
//! \param  [in ]  meshgrid2d     The object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_meshgrid2d_sshort_del (
   Rox_MeshGrid2D_Sshort * meshgrid2d
);

//! Get the size of a meshgrid2d (rows and cols)
//! \param  [out]  rows           The copy of the image rows
//! \param  [out]  cols           The copy of the image cols
//! \param  [in ]  meshgrid2d     The meshgrid2d object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_meshgrid2d_sshort_get_size (
   Rox_Sint * rows, 
   Rox_Sint * cols, 
   const Rox_MeshGrid2D_Sshort meshgrid2d
);

ROX_API Rox_ErrorCode rox_meshgrid2d_sshort_check_size (    
   const Rox_MeshGrid2D_Sshort meshgrid2d,
   const Rox_Sint rows, 
   const Rox_Sint cols 
); 




//! @}

#endif // __OPENROX_MESHGRID2D__
