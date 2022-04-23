//==============================================================================
//
//    OPENROX   : File linear7pts.h
//
//    Contents  : API of linear7pts module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINEAR7PTS__
#define __OPENROX_LINEAR7PTS__

#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point3d_struct.h>

void cdet4(Rox_Double * ce, Rox_Point2D_Double_Struct * ac, Rox_Point3D_Double_Struct *bc, Rox_Point2D_Double_Struct *ar, Rox_Point3D_Double_Struct *br);

void make_equations(Rox_Double ** eqs, Rox_Point2D_Double_Struct * ac, Rox_Point3D_Double_Struct *bc, Rox_Point2D_Double_Struct *ar, Rox_Point3D_Double_Struct *br);

void makeMt(Rox_Double **Mt, Rox_Point2D_Double_Struct * ac, Rox_Point3D_Double_Struct *bc, Rox_Point2D_Double_Struct *ar, Rox_Point3D_Double_Struct *br, Rox_Double x, Rox_Double y, Rox_Double z);

#endif