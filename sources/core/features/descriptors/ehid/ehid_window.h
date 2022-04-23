//==============================================================================
//
//    OPENROX   : File ehid_window.h
//
//    Contents  : API of ehid_window module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EHID_WINDOW__
#define __OPENROX_EHID_WINDOW__

#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point2d.h>

//! \addtogroup EHID
//! @{

//! Ehid Window structure 
typedef struct Rox_Ehid_Window_Struct * Rox_Ehid_Window;

//! Create a new Ehid Window (to cluster features inside)
//! \param  [out]  ehid_window    The pointer to the object created
//! \param  [in ]  top            The top row
//! \param  [in ]  left           The left column
//! \param  [in ]  height         The height of the window
//! \param  [in ]  width          The width of the window
//! \param  [in ]  maxpts         The max points accepted
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ehid_window_new(Rox_Ehid_Window * ehid_window, Rox_Sint top, Rox_Sint left, Rox_Sint height, Rox_Sint width, Rox_Uint maxpts);

//! Delete a Ehid Window (to cluster features inside)
//! \param  [out]  ehid_window    The pointer to the object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ehid_window_del(Rox_Ehid_Window * ehid_window);

//! Begin a new local view for this window (Points detected on a virtual frame)
//! \param  [out]  ehid_window    The window object
//! \return An error code
//! \todo  To be tested
ROX_API Rox_ErrorCode rox_ehid_window_begin_view(Rox_Ehid_Window ehid_window);

//! Terminate the local view for this window (Points detected on a virtual frame)
//! \param  [out]  ehid_window    The window object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ehid_window_terminate_view(Rox_Ehid_Window ehid_window);

//! Append a point if compatible to the local view for this window.
//! \param  [out]  ehid_window    The window object
//! \param  [in ]  refpt          The reference frame coordinates of the point
//! \param  [in ]  viewpt         The virtual view frame defined feature
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_ehid_window_append_point (
   Rox_Ehid_Window ehid_window, 
   Rox_Point2D_Double refpt, 
   Rox_Point2D_Double viewpt);

//! Cluster points of this window.
//! \param [in] ehid_window the window object
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_ehid_window_cluster(Rox_Ehid_Window ehid_window);

//! Delete an Ehid Window
//! \param [in] obj the window object
//! \return An error code
//! \todo To be tested
// ROX_API Rox_ErrorCode rox_ehid_window_del(Rox_Ehid_Window * obj);

//! @} 

#endif
