//==============================================================================
//
//    OPENROX   : File line3d_print.c
//
//    Contents  : Implementation of line3d print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include <stdio.h>
#include "line3d_print.h"
#include <baseproc/geometry/line/line3d_struct.h>
#include <inout/geometry/plane/plane3d_print.h>

#include <inout/system/print.h>
#include <system/errors/errors.h>

Rox_ErrorCode rox_line3d_planes_print ( const Rox_Line3D_Planes line3d )
{
    rox_log ( "line3d planes:\n" );
    rox_plane3d_print ( &line3d->planes[0] );
    rox_plane3d_print ( &line3d->planes[1] );

    return ROX_ERROR_NONE;
}
