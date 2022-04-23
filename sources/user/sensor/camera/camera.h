//==============================================================================
//
//    OPENROX   : File camera.h
//
//    Contents  : API of camera module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CAMERA__
#define __OPENROX_CAMERA__

#include <generated/array2d_double.h>

#include "baseproc/image/image.h"
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matse3.h>

//! \defgroup Sensor Sensor
//! \brief Sensor structures and methods.

//! \ingroup  Sensor
//! \defgroup Camera Camera
//! \brief Camera structures and methods.

//! \addtogroup Camera
//! @{

//! Define the Rox_Camera object as a pointer to the camera structure
typedef struct Rox_Camera_Struct * Rox_Camera;

//! Constructor for camera structure
//! \param  [out]  camera           Camera object
//! \param  [in ]  cols             Image width in pixels
//! \param  [in ]  rows             Image height in pixels
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_new ( Rox_Camera * camera, const Rox_Sint cols, const Rox_Sint rows );

//! Instanciate camera and load image data to camera instance
//! \param  [in ]  camera           Camera object
//! \param  [in ]  filename         Path to a file containing a pgm image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_new_read_pgm ( Rox_Camera * camera, const char * filename );

//! Destructor for camera structure
//! \param  [in ]  camera           The object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_del ( Rox_Camera * camera );

//! Load image data to camera object
//! \param  [out]  camera           Camera object
//! \param  [in ]  path             Path to a file containing a pnm image (.pgm or .ppm)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_read_pgm ( Rox_Camera camera, const char * path);

//! Save the current image of the camera object
//! \param  [out]  filename         The image filename
//! \param  [in ]  camera           Camera object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_save_pgm ( const char * filename, const Rox_Camera camera);

//! Set distortion parameters
//! \param  [out]  camera           Camera object with intrinsics camera parameters
//! \param  [in ]  K                Perspective camera intrinsic parameters matrix
//! \param  [in ]  radial           A 3*1 vector with radial parameters (Cf. Bouguet Matlab calibration toolbox, k1,k2,k5).
//! \param  [in ]  tangential       A 2*1 vector with tangential distortion parameters (Cf k3,k4 in Matlab calibration toolbox).
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_set_params_undistort ( Rox_Camera camera, const Rox_MatUT3 K, const Rox_Array2D_Double radial, const Rox_Array2D_Double tangential);

//! Set perspective parameters
//! \param  [out]  camera           Camera object with intrinsics camera parameters
//! \param  [in ]  fu               Focal length in pixels along the u axis
//! \param  [in ]  fv               Focal length in pixels along the v axis
//! \param  [in ]  cu               Principal point u coordinate
//! \param  [in ]  cv               Principal point v coordinate
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_set_pinhole_params ( Rox_Camera camera, const Rox_Double fu, const Rox_Double fv, const Rox_Double cu, const Rox_Double cv);

//! Set the camera image
//! \param  [out]  camera           The camera object
//! \param  [in ]  image            The image to copy
//! \return An error code
//! \warning The image is copied
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_set_image ( Rox_Camera camera, const Rox_Image image );

//! Set the camera image data
//! \param  [out]  camera           The camera object
//! \param  [in ]  data             The image to copy
//! \param  [in ]  bytesPerRow      The
//! \param  [in ]  format           The input image format
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_set_image_data ( Rox_Camera camera, const Rox_Uchar * data, const Rox_Sint bytesPerRow, const enum Rox_Image_Format format);

//! Get the camera image data
//! \param  [in ]  data             The pointer to the image data
//! \param  [out]  camera           The camera object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_get_image_data ( Rox_Uchar * data, const Rox_Camera camera );

//! Compute an undistorted image accordingly to intrinsics camera parameters and given distortion parameters
//! \param  [out]  camera           The camera object with intrinsics camera parameters
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_undistort_image ( Rox_Camera camera );

//! Get a copy of intrinsic parameters
//! \param  [out]  calib            The copy of the intrinsic parameters
//! \param  [in ]  camera           The camera object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_get_intrinsic_parameters ( Rox_MatUT3 calib, const Rox_Camera camera );


//! Get a copy of intrinsic parameters
//! \param  [out]  calib            The copy of the intrinsic parameters
//! \param  [in ]  camera           The camera object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_get_intrinsic_parameters_pointer ( Rox_MatUT3 * calib, const Rox_Camera camera );


//! Set the intrinsic parameters (camera calibration)
//! \param  [in ]  camera           The camera object
//! \param  [in ]  calib            The intrinsic parameters to copy
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_set_intrinsic_parameters ( Rox_Camera camera, const Rox_MatUT3 calib );

//! Set the extrinsic parameters (camera pose)
//! \param  [in ]  camera           The camera object
//! \param  [in ]  calib            The extrinsic parameters to copy
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_set_extrinsic_parameters ( Rox_Camera camera, const Rox_MatSE3 cTo );

//! Get a copy of the image rows
//! \param  [out]  rows             The copy of the image rows
//! \param  [in ]  camera           The camera object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_get_rows ( Rox_Sint * rows, const Rox_Camera camera );

//! Get a copy of the image cols
//! \param  [out]  cols             The copy of the image cols
//! \param  [in ]  camera           The camera object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_get_cols ( Rox_Sint * cols, Rox_Camera const camera);

//! Get a copy of the image size (rows and cols)
//! \param  [out]  rows             The copy of the image rows
//! \param  [out]  cols             The copy of the image cols
//! \param  [in ]  camera           The camera object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_get_size ( Rox_Sint * rows, Rox_Sint * cols, const Rox_Camera camera);

//! Set the image buffer
//! \param  [in ]  camera              The camera object created with size rows * cols
//! \param  [in ]  grays_image_buffer  The grayscale image buffer of size rows * cols
//! \return An error code
//! \warning The image is copied
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_set_grays_image_buffer(Rox_Camera camera, const Rox_Uchar * grays_image_buffer);

//! Get the pointer on the 2D array of image data
//! \param  [out]  rowsptr             The pointer on the 2D array 
//! \param  [in ]  camera              The camera object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_get_image_rowsptr(Rox_Uchar *** rowsptr, const Rox_Camera camera);

//! @}

#endif // __OPENROX_CAMERA__
