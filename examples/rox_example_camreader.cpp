//============================================================================
//
//    OPENROX   : File rox_example_camreader.pp
//
//    Contents  : Implementation of rox_example_camreader example
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== HEADERS   ================================================================

#include <rox_ffmpeg.h>
#include <vizwindow.h>

extern "C"
{
   #include <inout/system/errors_print.h>
   #include <system/errors/errors.h>
}

#include <string>
#include <cstring>

// Windows camera names
//#define CAMERA_NAME "Logitech Webcam Pro 9000"
//#define CAMERA_NAME "Logitech HD Pro Webcam C920"
//#define CAMERA_NAME "Integrated Webcam"
// Linux camera names
#define CAMERA_NAME "/dev/video0"
//#define CAMERA_NAME "/dev/video1"
#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 480
#define CAMERA_FPS 30

int main(int argc, char * argv[])
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_FFmpeg_Media camera = NULL;
   Rox_VizWindow viewer = NULL;
   std::string video_size = std::to_string(CAMERA_WIDTH) + "x" + std::to_string(CAMERA_HEIGHT);

   error = rox_ffmpeg_new(&camera);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_vizwindow_new(&viewer);
   ROX_ERROR_CHECK_TERMINATE ( error );


   error = rox_ffmpeg_set_str_opt(camera, "media::video_size", video_size.c_str());
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_ffmpeg_open(camera, CAMERA_NAME);

   error = rox_ffmpeg_set_str_opt(camera, "video::transform::pixel_format", "rgba");
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_vizwindow_setimagesize(viewer, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_WIDTH*CAMERA_HEIGHT*4);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   while (true)
   {
      unsigned char* data = NULL;
      error = rox_ffmpeg_readnext(camera);
      ROX_ERROR_CHECK_TERMINATE ( error );

      data = rox_ffmpeg_get_video_data(camera);

      error = rox_vizwindow_setimage(viewer, data, CAMERA_WIDTH, CAMERA_HEIGHT);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_vizwindow_update(viewer);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   rox_error_print(error);
   ROX_ERROR_CHECK(rox_vizwindow_del(&viewer));
   ROX_ERROR_CHECK(rox_ffmpeg_del(&camera));

   return error;
}