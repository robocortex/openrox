//==============================================================================
//
//    OPENROX   : File rectangle.c
//
//    Contents  : Implementation of rectangle module
//
//    Author(s) : R&D department leaded by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linear7pts.h"

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/point/point3d.h>

void makeMt(Rox_Double **Mt, Rox_Point2D_Double  ac, Rox_Point3D_Double bc, Rox_Point2D_Double ar, Rox_Point3D_Double br, Rox_Double x, Rox_Double y, Rox_Double z)
{
   double ac1_0 = ac[0].u;
   double ac1_1 = ac[0].v;
   double ac2_0 = ac[1].u;
   double ac2_1 = ac[1].v;
   double ac3_0 = ac[2].u;
   double ac3_1 = ac[2].v;
   double ac4_0 = ac[3].u;
   double ac4_1 = ac[3].v;
   double ac5_0 = ac[4].u;
   double ac5_1 = ac[4].v;
   double ac6_0 = ac[5].u;
   double ac6_1 = ac[5].v;
   double ac7_0 = ac[6].u;
   double ac7_1 = ac[6].v;

   double bc1_0 = bc[0].X;
   double bc1_1 = bc[0].Y;
   double bc1_2 = bc[0].Z;
   double bc2_0 = bc[1].X;
   double bc2_1 = bc[1].Y;
   double bc2_2 = bc[1].Z;
   double bc3_0 = bc[2].X;
   double bc3_1 = bc[2].Y;
   double bc3_2 = bc[2].Z;
   double bc4_0 = bc[3].X;
   double bc4_1 = bc[3].Y;
   double bc4_2 = bc[3].Z;
   double bc5_0 = bc[4].X;
   double bc5_1 = bc[4].Y;
   double bc5_2 = bc[4].Z;
   double bc6_0 = bc[5].X;
   double bc6_1 = bc[5].Y;
   double bc6_2 = bc[5].Z;
   double bc7_0 = bc[6].X;
   double bc7_1 = bc[6].Y;
   double bc7_2 = bc[6].Z;

   double ar1_0 = ar[0].u;
   double ar1_1 = ar[0].v;
   double ar2_0 = ar[1].u;
   double ar2_1 = ar[1].v;
   double ar3_0 = ar[2].u;
   double ar3_1 = ar[2].v;
   double ar4_0 = ar[3].u;
   double ar4_1 = ar[3].v;
   double ar5_0 = ar[4].u;
   double ar5_1 = ar[4].v;
   double ar6_0 = ar[5].u;
   double ar6_1 = ar[5].v;
   double ar7_0 = ar[6].u;
   double ar7_1 = ar[6].v;

   double br1_0 = br[0].X;
   double br1_1 = br[0].Y;
   double br1_2 = br[0].Z;
   double br2_0 = br[1].X;
   double br2_1 = br[1].Y;
   double br2_2 = br[1].Z;
   double br3_0 = br[2].X;
   double br3_1 = br[2].Y;
   double br3_2 = br[2].Z;
   double br4_0 = br[3].X;
   double br4_1 = br[3].Y;
   double br4_2 = br[3].Z;
   double br5_0 = br[4].X;
   double br5_1 = br[4].Y;
   double br5_2 = br[4].Z;
   double br6_0 = br[5].X;
   double br6_1 = br[5].Y;
   double br6_2 = br[5].Z;
   double br7_0 = br[6].X;
   double br7_1 = br[6].Y;
   double br7_2 = br[6].Z;

   Mt[0][0] = ac1_1 - ar1_1 - 2 * x + ac1_0 * z + ar1_0 * z - ac1_1 * x * x
              + ar1_1 * x * x - 2 * ac1_1 * ar1_1 * x + ac1_0 * ar1_1 * y + ac1_1 *
              ar1_0 * y + ac1_0 * x * y - ar1_0 * x * y - ac1_0 * ar1_1 * x * z +
              ac1_1 * ar1_0 * x * z;
   Mt[0][1] = ar1_0 - ac1_0 - 2 * y + ac1_1 * z + ar1_1 * z + ac1_0 * y * y
              - ar1_0 * y * y + ac1_0 * ar1_1 * x + ac1_1 * ar1_0 * x - 2 * ac1_0 *
              ar1_0 * y - ac1_1 * x * y + ar1_1 * x * y - ac1_0 * ar1_1 * y * z +
              ac1_1 * ar1_0 * y * z;
   Mt[0][2] = y * (ar1_1 + x - ar1_0 * z) + x * (ar1_0 - y + ar1_1 * z) +
              ac1_0 * (ar1_1 + x - ar1_0 * z - z * (ar1_0 - y + ar1_1 * z)) - ac1_1 *
              (ar1_0 - y + ar1_1 * z + z * (ar1_1 + x - ar1_0 * z));
   Mt[0][3] = 2 * ac1_1 * br1_2 * x - 2 * ar1_1 * bc1_2 * x - 2 * ac1_0 *
              br1_2 * y + 2 * ar1_0 * bc1_2 * y + 2 * ac1_0 * br1_1 * z - 2 * ac1_1 *
              br1_0 * z - ac1_0 * br1_0 * y * y - ar1_0 * bc1_0 * y * y + ac1_1 *
              br1_1 * y * y + ar1_1 * bc1_1 * y * y - ar1_0 * bc1_0 * z * z - ac1_0 *
              br1_0 * z * z - ac1_1 * br1_1 * z * z - ar1_1 * bc1_1 * z * z + 2 *
              ac1_0 * br1_1 * x * y + 2 * ac1_1 * br1_0 * x * y + 2 * ar1_0 * bc1_1 *
              x * y + 2 * ar1_1 * bc1_0 * x * y + 2 * ac1_0 * br1_2 * x * z + 2 *
              ar1_0 * bc1_2 * x * z + 2 * ac1_1 * br1_2 * y * z + 2 * ar1_1 * bc1_2 *
              y * z - 2 * ar1_0 * bc1_1 * z + 2 * ar1_1 * bc1_0 * z + 2 * br1_0 * x *
              z + 2 * bc1_0 * x * z + 2 * bc1_1 * y * z + 2 * br1_1 * y * z + ac1_0 *
              br1_0 * x * x + ar1_0 * bc1_0 * x * x - ac1_1 * br1_1 * x * x - ar1_1 *
              bc1_1 * x * x + 2 * bc1_1 * x - 2 * br1_1 * x - 2 * bc1_0 * y + 2 *
              br1_0 * y - bc1_2 * x * x - br1_2 * x * x - bc1_2 * y * y - br1_2 * y *
              y + bc1_2 * z * z + br1_2 * z * z;
   Mt[1][0] = ac2_1 - ar2_1 - 2 * x + ac2_0 * z + ar2_0 * z - ac2_1 * x * x
              + ar2_1 * x * x - 2 * ac2_1 * ar2_1 * x + ac2_0 * ar2_1 * y + ac2_1 *
              ar2_0 * y + ac2_0 * x * y - ar2_0 * x * y - ac2_0 * ar2_1 * x * z +
              ac2_1 * ar2_0 * x * z;
   Mt[1][1] = ar2_0 - ac2_0 - 2 * y + ac2_1 * z + ar2_1 * z + ac2_0 * y * y
              - ar2_0 * y * y + ac2_0 * ar2_1 * x + ac2_1 * ar2_0 * x - 2 * ac2_0 *
              ar2_0 * y - ac2_1 * x * y + ar2_1 * x * y - ac2_0 * ar2_1 * y * z +
              ac2_1 * ar2_0 * y * z;
   Mt[1][2] = y * (ar2_1 + x - ar2_0 * z) + x * (ar2_0 - y + ar2_1 * z) +
              ac2_0 * (ar2_1 + x - ar2_0 * z - z * (ar2_0 - y + ar2_1 * z)) - ac2_1 *
              (ar2_0 - y + ar2_1 * z + z * (ar2_1 + x - ar2_0 * z));
   Mt[1][3] = 2 * ar2_1 * bc2_0 * x * y + 2 * ac2_0 * br2_1 * x * y + 2 *
              ac2_1 * br2_0 * x * y + 2 * ar2_0 * bc2_1 * x * y - 2 * ac2_0 * br2_2 *
              y + 2 * ac2_1 * br2_2 * x - 2 * ar2_1 * bc2_2 * x + 2 * br2_0 * y -
              ac2_0 * br2_0 * z * z - ar2_0 * bc2_0 * z * z + 2 * ar2_0 * bc2_2 * y +
              2 * ac2_0 * br2_1 * z - 2 * ac2_1 * br2_0 * z - 2 * ar2_0 * bc2_1 * z +
              2 * ar2_1 * bc2_0 * z + 2 * bc2_0 * x * z + 2 * br2_0 * x * z + 2 *
              bc2_1 * y * z + 2 * br2_1 * y * z + ac2_0 * br2_0 * x * x + ar2_0 *
              bc2_0 * x * x - ac2_1 * br2_1 * x * x - ar2_0 * bc2_0 * y * y - ar2_1 *
              bc2_1 * x * x - ac2_0 * br2_0 * y * y + ar2_1 * bc2_1 * y * y + ac2_1 *
              br2_1 * y * y + 2 * bc2_1 * x - 2 * br2_1 * x - 2 * bc2_0 * y - bc2_2 *
              x * x - br2_2 * x * x - bc2_2 * y * y - br2_2 * y * y + bc2_2 * z * z +
              br2_2 * z * z - ac2_1 * br2_1 * z * z - ar2_1 * bc2_1 * z * z + 2 *
              ac2_0 * br2_2 * x * z + 2 * ar2_0 * bc2_2 * x * z + 2 * ac2_1 * br2_2 *
              y * z + 2 * ar2_1 * bc2_2 * y * z;
   Mt[2][0] = ac3_1 - ar3_1 - 2 * x + ac3_0 * z + ar3_0 * z - ac3_1 * x * x
              + ar3_1 * x * x - 2 * ac3_1 * ar3_1 * x + ac3_0 * ar3_1 * y + ac3_1 *
              ar3_0 * y + ac3_0 * x * y - ar3_0 * x * y - ac3_0 * ar3_1 * x * z +
              ac3_1 * ar3_0 * x * z;
   Mt[2][1] = ar3_0 - ac3_0 - 2 * y + ac3_1 * z + ar3_1 * z + ac3_0 * y * y
              - ar3_0 * y * y + ac3_0 * ar3_1 * x + ac3_1 * ar3_0 * x - 2 * ac3_0 *
              ar3_0 * y - ac3_1 * x * y + ar3_1 * x * y - ac3_0 * ar3_1 * y * z +
              ac3_1 * ar3_0 * y * z;
   Mt[2][2] = y * (ar3_1 + x - ar3_0 * z) + x * (ar3_0 - y + ar3_1 * z) +
              ac3_0 * (ar3_1 + x - ar3_0 * z - z * (ar3_0 - y + ar3_1 * z)) - ac3_1 *
              (ar3_0 - y + ar3_1 * z + z * (ar3_1 + x - ar3_0 * z));
   Mt[2][3] = -ar3_0 * bc3_0 * y * y + ac3_1 * br3_1 * y * y + ar3_1 *
              bc3_1 * y * y - ac3_0 * br3_0 * z * z - ar3_0 * bc3_0 * z * z - ac3_1 *
              br3_1 * z * z - ar3_1 * bc3_1 * z * z + 2 * bc3_1 * y * z + 2 * ac3_1 *
              br3_2 * x - 2 * ar3_1 * bc3_2 * x - 2 * ac3_0 * br3_2 * y + 2 * ar3_0 *
              bc3_2 * y - 2 * ac3_1 * br3_0 * z + 2 * ac3_0 * br3_1 * z - 2 * ar3_0 *
              bc3_1 * z + 2 * ar3_1 * bc3_0 * z + 2 * bc3_0 * x * z + 2 * br3_0 * x *
              z + 2 * ac3_0 * br3_1 * x * y + 2 * ac3_1 * br3_0 * x * y + 2 * ar3_0 *
              bc3_1 * x * y + 2 * ar3_1 * bc3_0 * x * y + 2 * ac3_0 * br3_2 * x * z +
              2 * ar3_0 * bc3_2 * x * z + 2 * ac3_1 * br3_2 * y * z + 2 * ar3_1 *
              bc3_2 * y * z + 2 * br3_1 * y * z + ac3_0 * br3_0 * x * x + ar3_0 *
              bc3_0 * x * x - ac3_1 * br3_1 * x * x - ar3_1 * bc3_1 * x * x - ac3_0 *
              br3_0 * y * y + 2 * bc3_1 * x - 2 * br3_1 * x - 2 * bc3_0 * y + 2 *
              br3_0 * y - bc3_2 * x * x - br3_2 * x * x - bc3_2 * y * y - br3_2 * y *
              y + bc3_2 * z * z + br3_2 * z * z;
   Mt[3][0] = ac4_1 - ar4_1 - 2 * x + ac4_0 * z + ar4_0 * z - ac4_1 * x * x
              + ar4_1 * x * x - 2 * ac4_1 * ar4_1 * x + ac4_0 * ar4_1 * y + ac4_1 *
              ar4_0 * y + ac4_0 * x * y - ar4_0 * x * y - ac4_0 * ar4_1 * x * z +
              ac4_1 * ar4_0 * x * z;
   Mt[3][1] = ar4_0 - ac4_0 - 2 * y + ac4_1 * z + ar4_1 * z + ac4_0 * y * y
              - ar4_0 * y * y + ac4_0 * ar4_1 * x + ac4_1 * ar4_0 * x - 2 * ac4_0 *
              ar4_0 * y - ac4_1 * x * y + ar4_1 * x * y - ac4_0 * ar4_1 * y * z +
              ac4_1 * ar4_0 * y * z;
   Mt[3][2] = y * (ar4_1 + x - ar4_0 * z) + x * (ar4_0 - y + ar4_1 * z) +
              ac4_0 * (ar4_1 + x - ar4_0 * z - z * (ar4_0 - y + ar4_1 * z)) - ac4_1 *
              (ar4_0 - y + ar4_1 * z + z * (ar4_1 + x - ar4_0 * z));
   Mt[3][3] = -ac4_0 * br4_0 * y * y - ar4_0 * bc4_0 * y * y + ac4_1 *
              br4_1 * y * y + ar4_1 * bc4_1 * y * y - ac4_0 * br4_0 * z * z - ar4_0 *
              bc4_0 * z * z - ar4_1 * bc4_1 * z * z - ac4_1 * br4_1 * z * z + 2 *
              ar4_0 * bc4_2 * x * z + 2 * ac4_1 * br4_2 * y * z + 2 * ar4_1 * bc4_2 *
              y * z + 2 * bc4_1 * x - 2 * br4_1 * x - 2 * bc4_0 * y + 2 * br4_0 * y -
              bc4_2 * x * x - br4_2 * x * x - bc4_2 * y * y - br4_2 * y * y + bc4_2 *
              z * z + br4_2 * z * z + 2 * ac4_0 * br4_1 * x * y + 2 * ac4_1 * br4_0 *
              x * y + 2 * ar4_0 * bc4_1 * x * y + 2 * ar4_1 * bc4_0 * x * y + 2 *
              ac4_0 * br4_2 * x * z - ar4_1 * bc4_1 * x * x - 2 * ac4_1 * br4_0 * z +
              2 * ac4_1 * br4_2 * x - 2 * ar4_1 * bc4_2 * x - 2 * ac4_0 * br4_2 * y +
              2 * ar4_0 * bc4_2 * y + 2 * ac4_0 * br4_1 * z - 2 * ar4_0 * bc4_1 * z +
              2 * ar4_1 * bc4_0 * z + 2 * bc4_0 * x * z + 2 * br4_0 * x * z + 2 *
              bc4_1 * y * z + 2 * br4_1 * y * z + ac4_0 * br4_0 * x * x + ar4_0 *
              bc4_0 * x * x - ac4_1 * br4_1 * x * x;
   Mt[4][0] = ac5_1 - ar5_1 - 2 * x + ac5_0 * z + ar5_0 * z - ac5_1 * x * x
              + ar5_1 * x * x - 2 * ac5_1 * ar5_1 * x + ac5_0 * ar5_1 * y + ac5_1 *
              ar5_0 * y + ac5_0 * x * y - ar5_0 * x * y - ac5_0 * ar5_1 * x * z +
              ac5_1 * ar5_0 * x * z;
   Mt[4][1] = ar5_0 - ac5_0 - 2 * y + ac5_1 * z + ar5_1 * z + ac5_0 * y * y
              - ar5_0 * y * y + ac5_0 * ar5_1 * x + ac5_1 * ar5_0 * x - 2 * ac5_0 *
              ar5_0 * y - ac5_1 * x * y + ar5_1 * x * y - ac5_0 * ar5_1 * y * z +
              ac5_1 * ar5_0 * y * z;
   Mt[4][2] = y * (ar5_1 + x - ar5_0 * z) + x * (ar5_0 - y + ar5_1 * z) +
              ac5_0 * (ar5_1 + x - ar5_0 * z - z * (ar5_0 - y + ar5_1 * z)) - ac5_1 *
              (ar5_0 - y + ar5_1 * z + z * (ar5_1 + x - ar5_0 * z));
   Mt[4][3] = br5_2 * z * z - 2 * ac5_1 * br5_0 * z + 2 * ac5_1 * br5_2 * x
              - 2 * ar5_1 * bc5_2 * x - 2 * ac5_0 * br5_2 * y + 2 * ar5_0 * bc5_2 * y
              + 2 * ac5_0 * br5_1 * z - ar5_1 * bc5_1 * z * z + 2 * ac5_0 * br5_1 * x
              * y + 2 * ac5_1 * br5_0 * x * y + 2 * ar5_0 * bc5_1 * x * y + 2 * ar5_1
              * bc5_0 * x * y + 2 * ac5_0 * br5_2 * x * z + 2 * ar5_0 * bc5_2 * x * z
              + 2 * ac5_1 * br5_2 * y * z + 2 * ar5_1 * bc5_2 * y * z + 2 * bc5_1 * x
              - 2 * br5_1 * x - 2 * bc5_0 * y + 2 * br5_0 * y - bc5_2 * x * x - br5_2
              * x * x - bc5_2 * y * y - br5_2 * y * y + bc5_2 * z * z - 2 * ar5_0 *
              bc5_1 * z + 2 * ar5_1 * bc5_0 * z + 2 * bc5_0 * x * z + 2 * br5_0 * x *
              z + 2 * bc5_1 * y * z + ar5_0 * bc5_0 * x * x + 2 * br5_1 * y * z +
              ac5_0 * br5_0 * x * x - ac5_1 * br5_1 * x * x - ac5_0 * br5_0 * y * y -
              ar5_1 * bc5_1 * x * x - ar5_0 * bc5_0 * y * y + ac5_1 * br5_1 * y * y +
              ar5_1 * bc5_1 * y * y - ac5_0 * br5_0 * z * z - ac5_1 * br5_1 * z * z -
              ar5_0 * bc5_0 * z * z;
   Mt[5][0] = ac6_1 - ar6_1 - 2 * x + ac6_0 * z + ar6_0 * z - ac6_1 * x * x
              + ar6_1 * x * x - 2 * ac6_1 * ar6_1 * x + ac6_0 * ar6_1 * y + ac6_1 *
              ar6_0 * y + ac6_0 * x * y - ar6_0 * x * y - ac6_0 * ar6_1 * x * z +
              ac6_1 * ar6_0 * x * z;
   Mt[5][1] = ar6_0 - ac6_0 - 2 * y + ac6_1 * z + ar6_1 * z + ac6_0 * y * y
              - ar6_0 * y * y + ac6_0 * ar6_1 * x + ac6_1 * ar6_0 * x - 2 * ac6_0 *
              ar6_0 * y - ac6_1 * x * y + ar6_1 * x * y - ac6_0 * ar6_1 * y * z +
              ac6_1 * ar6_0 * y * z;
   Mt[5][2] = y * (ar6_1 + x - ar6_0 * z) + x * (ar6_0 - y + ar6_1 * z) +
              ac6_0 * (ar6_1 + x - ar6_0 * z - z * (ar6_0 - y + ar6_1 * z)) - ac6_1 *
              (ar6_0 - y + ar6_1 * z + z * (ar6_1 + x - ar6_0 * z));
   Mt[5][3] = ar6_0 * bc6_0 * x * x - ar6_1 * bc6_1 * x * x - ac6_1 * br6_1
              * x * x + ac6_1 * br6_1 * y * y - ac6_0 * br6_0 * y * y + ar6_1 * bc6_1
              * y * y - ar6_0 * bc6_0 * z * z - ar6_0 * bc6_0 * y * y - ar6_1 * bc6_1
              * z * z - ac6_0 * br6_0 * z * z - ac6_1 * br6_1 * z * z - 2 * ar6_1 *
              bc6_2 * x + 2 * ac6_1 * br6_2 * x - 2 * ac6_0 * br6_2 * y + 2 * ar6_0 *
              bc6_2 * y + 2 * ac6_0 * br6_1 * z - 2 * ac6_1 * br6_0 * z - 2 * ar6_0 *
              bc6_1 * z + 2 * ar6_1 * bc6_0 * z + 2 * bc6_0 * x * z + 2 * br6_0 * x *
              z + 2 * bc6_1 * y * z + 2 * br6_1 * y * z + ac6_0 * br6_0 * x * x + 2 *
              ac6_1 * br6_2 * y * z + 2 * ar6_1 * bc6_2 * y * z + 2 * bc6_1 * x - 2 *
              br6_1 * x - 2 * bc6_0 * y + 2 * br6_0 * y - bc6_2 * x * x - br6_2 * x *
              x - bc6_2 * y * y - br6_2 * y * y + bc6_2 * z * z + br6_2 * z * z + 2 *
              ar6_0 * bc6_2 * x * z + 2 * ac6_0 * br6_1 * x * y + 2 * ac6_1 * br6_0 *
              x * y + 2 * ar6_0 * bc6_1 * x * y + 2 * ar6_1 * bc6_0 * x * y + 2 *
              ac6_0 * br6_2 * x * z;
   Mt[6][0] = ac7_1 - ar7_1 - 2 * x + ac7_0 * z + ar7_0 * z - ac7_1 * x * x
              + ar7_1 * x * x - 2 * ac7_1 * ar7_1 * x + ac7_0 * ar7_1 * y + ac7_1 *
              ar7_0 * y + ac7_0 * x * y - ar7_0 * x * y - ac7_0 * ar7_1 * x * z +
              ac7_1 * ar7_0 * x * z;
   Mt[6][1] = ar7_0 - ac7_0 - 2 * y + ac7_1 * z + ar7_1 * z + ac7_0 * y * y
              - ar7_0 * y * y + ac7_0 * ar7_1 * x + ac7_1 * ar7_0 * x - 2 * ac7_0 *
              ar7_0 * y - ac7_1 * x * y + ar7_1 * x * y - ac7_0 * ar7_1 * y * z +
              ac7_1 * ar7_0 * y * z;
   Mt[6][2] = y * (ar7_1 + x - ar7_0 * z) + x * (ar7_0 - y + ar7_1 * z) +
              ac7_0 * (ar7_1 + x - ar7_0 * z - z * (ar7_0 - y + ar7_1 * z)) - ac7_1 *
              (ar7_0 - y + ar7_1 * z + z * (ar7_1 + x - ar7_0 * z));
   Mt[6][3] = ac7_1 * br7_1 * y * y + ar7_1 * bc7_1 * y * y - ar7_0 * bc7_0
              * z * z - ac7_0 * br7_0 * z * z - ac7_1 * br7_1 * z * z - ar7_1 * bc7_1
              * z * z - br7_2 * y * y + ar7_0 * bc7_0 * x * x - ac7_1 * br7_1 * x * x
              - ar7_1 * bc7_1 * x * x - ac7_0 * br7_0 * y * y - ar7_0 * bc7_0 * y * y
              + 2 * ac7_1 * br7_2 * x - 2 * ar7_1 * bc7_2 * x - 2 * ac7_0 * br7_2 * y
              + 2 * ar7_0 * bc7_2 * y + 2 * ar7_1 * bc7_0 * z - 2 * ac7_1 * br7_0 * z
              + 2 * ac7_0 * br7_1 * z - 2 * ar7_0 * bc7_1 * z + 2 * br7_0 * x * z + 2
              * bc7_0 * x * z + 2 * bc7_1 * y * z + 2 * br7_1 * y * z + ac7_0 * br7_0
              * x * x + 2 * bc7_1 * x - 2 * br7_1 * x - 2 * bc7_0 * y + 2 * br7_0 * y
              - bc7_2 * x * x - br7_2 * x * x - bc7_2 * y * y + bc7_2 * z * z + br7_2
              * z * z + 2 * ac7_0 * br7_1 * x * y + 2 * ac7_1 * br7_0 * x * y + 2 *
              ar7_0 * bc7_1 * x * y + 2 * ar7_1 * bc7_0 * x * y + 2 * ac7_0 * br7_2 *
              x * z + 2 * ar7_0 * bc7_2 * x * z + 2 * ac7_1 * br7_2 * y * z + 2 *
              ar7_1 * bc7_2 * y * z;
}
