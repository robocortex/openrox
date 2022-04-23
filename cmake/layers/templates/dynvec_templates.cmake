# Dynamic vectors of simple structures (i.e. structures that do not contains dynamically allocated data)

# Macro for a simple one line declaration
macro(dynvec_generator includename structname typename_lowercase typename typename_struct)
   SET (DYNVECTYPE_INCLUDE          ${includename})
   SET (DYNVECTYPE_INCLUDE_STRUCT   ${structname})
   SET (LDYNVECTYPE                 ${typename_lowercase})
   SET (DYNVECTYPE                  ${typename})
   SET (SDYNVECTYPE                 ${typename_struct})
   configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
   configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
   configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)
endmacro()

dynvec_generator( "<system/memory/datatypes.h>"               "<system/memory/datatypes.h>"               "uint"               "Uint"                      "Uint"                      )
dynvec_generator( "<system/memory/datatypes.h>"               "<system/memory/datatypes.h>"               "uchar"              "Uchar"                     "Uchar"                     )
dynvec_generator( "<system/memory/datatypes.h>"               "<system/memory/datatypes.h>"               "sint"               "Sint"                      "Sint"                      )
dynvec_generator( "<system/memory/datatypes.h>"               "<system/memory/datatypes.h>"               "slint"              "Slint"                     "Slint"                     )
dynvec_generator( "<system/memory/datatypes.h>"               "<system/memory/datatypes.h>"               "ushort"             "Ushort"                    "Ushort"                    )
dynvec_generator( "<system/memory/datatypes.h>"               "<system/memory/datatypes.h>"               "sshort"             "Sshort"                    "Sshort"                    )
dynvec_generator( "<system/memory/datatypes.h>"               "<system/memory/datatypes.h>"               "float"              "Float"                     "Float"                     )
dynvec_generator( "<system/memory/datatypes.h>"               "<system/memory/datatypes.h>"               "double"             "Double"                    "Double"                    )
dynvec_generator( "<system/memory/datatypes.h>"               "<system/memory/datatypes.h>"               "sparse_value"       "Sparse_Value"       "Sparse_Value_Struct"       )
dynvec_generator( "<system/memory/array2d.h>"                 "<system/memory/array2d_struct.h>"          "array2d"            "Array2D"                   "Array2D"                   )

############### THE FOLLOWING ARE NOT ALLOWED ???? ###############
dynvec_generator( "<generated/array2d_double.h>"                        "<generated/array2d_double.h>"                        "array2d_double"     "Array2D_Double"            "Array2D_Double"            )
dynvec_generator( "<generated/array2d_float.h>"                         "<generated/array2d_float.h>"                         "array2d_float"      "Array2D_Float"             "Array2D_Float"             )
dynvec_generator( "<generated/array2d_uint.h>"                          "<generated/array2d_uint.h>"                          "array2d_uint"       "Array2D_Uint"              "Array2D_Uint"              )
##################################################################

dynvec_generator( "<core/tracking/edge/edge_point_site.h>"    "<core/tracking/edge/edge_point_site.h>"    "edge_point_site"    "Edge_Point_Site"    "Edge_Point_Site_Struct"    )
dynvec_generator( "<core/tracking/edge/edge_segment_site.h>"  "<core/tracking/edge/edge_segment_site.h>"  "edge_segment_site"  "Edge_Segment_Site"  "Edge_Segment_Site_Struct"  )
dynvec_generator( "<core/tracking/edge/edge_ellipse_site.h>"  "<core/tracking/edge/edge_ellipse_site.h>"  "edge_ellipse_site"  "Edge_Ellipse_Site"  "Edge_Ellipse_Site_Struct"  )
dynvec_generator( "<core/tracking/edge/edge_cylinder_site.h>" "<core/tracking/edge/edge_cylinder_site.h>" "edge_cylinder_site" "Edge_Cylinder_Site" "Edge_Cylinder_Site_Struct" )

# Dynamic vector of sraid features
SET (DYNVECTYPE_INCLUDE          "<core/features/descriptors/sraid/sraiddesc_struct.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<core/features/descriptors/sraid/sraiddesc_struct.h>")
SET (LDYNVECTYPE                 "sraiddesc")
SET (DYNVECTYPE                  "SRAID_Feature")
SET (SDYNVECTYPE                 "SRAID_Feature_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of dog features
SET (DYNVECTYPE_INCLUDE          "<core/features/detectors/dog/dog.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<core/features/detectors/dog/dog.h>")
SET (LDYNVECTYPE                 "dog")
SET (DYNVECTYPE                  "Dog_Feature")
SET (SDYNVECTYPE                 "Dog_Feature_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of Rox_Rect_Sint_Struct
SET (DYNVECTYPE_INCLUDE          "<baseproc/geometry/rectangle/rectangle.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<baseproc/geometry/rectangle/rectangle_struct.h>")
SET (LDYNVECTYPE                 "rect_sint")
SET (DYNVECTYPE                  "Rect_Sint")
SET (SDYNVECTYPE                 "Rect_Sint_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of point_double
SET (DYNVECTYPE_INCLUDE          "<baseproc/geometry/point/points_struct.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<baseproc/geometry/point/points_struct.h>")
SET (LDYNVECTYPE                 "point_double")
SET (DYNVECTYPE                  "Point_Double")
SET (SDYNVECTYPE                 "Point_Double_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of point2d_float
SET (DYNVECTYPE_INCLUDE          "<baseproc/geometry/point/point2d.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<baseproc/geometry/point/point2d_struct.h>")
SET (LDYNVECTYPE                 "point2d_float")
SET (DYNVECTYPE                  "Point2D_Float")
SET (SDYNVECTYPE                 "Point2D_Float_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of point2d_double
SET (DYNVECTYPE_INCLUDE          "<baseproc/geometry/point/point2d.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<baseproc/geometry/point/point2d_struct.h>")
SET (LDYNVECTYPE                 "point2d_double")
SET (DYNVECTYPE                  "Point2D_Double")
SET (SDYNVECTYPE                 "Point2D_Double_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of point2d_sint
SET (DYNVECTYPE_INCLUDE          "<baseproc/geometry/point/point2d.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<baseproc/geometry/point/point2d_struct.h>")
SET (LDYNVECTYPE                 "point2d_sint")
SET (DYNVECTYPE                  "Point2D_Sint")
SET (SDYNVECTYPE                 "Point2D_Sint_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of point2d_uint
SET (DYNVECTYPE_INCLUDE          "<baseproc/geometry/point/point2d.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<baseproc/geometry/point/point2d_struct.h>")
SET (LDYNVECTYPE                 "point2d_uint")
SET (DYNVECTYPE                  "Point2D_Uint")
SET (SDYNVECTYPE                 "Point2D_Uint_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of point2d_sshort
SET (DYNVECTYPE_INCLUDE          "<baseproc/geometry/point/point2d.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<baseproc/geometry/point/point2d_struct.h>")
SET (LDYNVECTYPE                 "point2d_sshort")
SET (DYNVECTYPE                  "Point2D_Sshort")
SET (SDYNVECTYPE                 "Point2D_Sshort_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of point3d_double
SET (DYNVECTYPE_INCLUDE          "<baseproc/geometry/point/point3d_struct.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<baseproc/geometry/point/point3d_struct.h>")
SET (LDYNVECTYPE                 "point3d_double")
SET (DYNVECTYPE                  "Point3D_Double")
SET (SDYNVECTYPE                 "Point3D_Double_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of point3d_sint
SET (DYNVECTYPE_INCLUDE          "<baseproc/geometry/point/point3d_struct.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<baseproc/geometry/point/point3d_struct.h>")
SET (LDYNVECTYPE                 "point3d_sint")
SET (DYNVECTYPE                  "Point3D_Sint")
SET (SDYNVECTYPE                 "Point3D_Sint_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of point3d_float
SET (DYNVECTYPE_INCLUDE          "<baseproc/geometry/point/point3d_struct.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<baseproc/geometry/point/point3d_struct.h>")
SET (LDYNVECTYPE                 "point3d_float")
SET (DYNVECTYPE                  "Point3D_Float")
SET (SDYNVECTYPE                 "Point3D_Float_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of segment2d
SET (DYNVECTYPE_INCLUDE          "<baseproc/geometry/segment/segment2d_struct.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<baseproc/geometry/segment/segment2d_struct.h>")
SET (LDYNVECTYPE                 "segment2d")
SET (DYNVECTYPE                  "Segment2D")
SET (SDYNVECTYPE                 "Segment2D_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of Quadrilateral
SET (DYNVECTYPE_INCLUDE          "<core/features/detectors/quad/quad_struct.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<core/features/detectors/quad/quad_struct.h>")
SET (LDYNVECTYPE                 "quad")
SET (DYNVECTYPE                  "Quad")
SET (SDYNVECTYPE                 "Quad_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of Quadrilateral segment
SET (DYNVECTYPE_INCLUDE          "<core/features/detectors/quad/quad_segment2d_struct.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<core/features/detectors/quad/quad_segment2d_struct.h>")
SET (LDYNVECTYPE                 "quad_segment2d")
SET (DYNVECTYPE                  "Quad_Segment2D")
SET (SDYNVECTYPE                 "Quad_Segment2D_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of Orientedpoints
SET (DYNVECTYPE_INCLUDE          "<core/features/detectors/quad/oriented_image_point_struct.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<core/features/detectors/quad/oriented_image_point_struct.h>")
SET (LDYNVECTYPE                 "orientedimagepoint")
SET (DYNVECTYPE                  "OrientedImagePoint")
SET (SDYNVECTYPE                 "OrientedImagePoint_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of edges points
SET (DYNVECTYPE_INCLUDE          "<core/features/detectors/edges/edgepath.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<core/features/detectors/edges/edgepath.h>")
SET (LDYNVECTYPE                 "edgeturn")
SET (DYNVECTYPE                  "EdgeTurn")
SET (SDYNVECTYPE                 "EdgeTurn_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of edges points
SET (DYNVECTYPE_INCLUDE          "<core/features/detectors/edges/edgepath.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<core/features/detectors/edges/edgepath.h>")
SET (LDYNVECTYPE                 "edgel")
SET (DYNVECTYPE                  "Edgel")
SET (SDYNVECTYPE                 "Edgel_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of edges points
SET (DYNVECTYPE_INCLUDE          "<core/features/detectors/edges/edgepath.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<core/features/detectors/edges/edgepath.h>")
SET (LDYNVECTYPE                 "segment_part")
SET (DYNVECTYPE                  "Segment_Part")
SET (SDYNVECTYPE                 "Segment_Part_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of segment points
SET (DYNVECTYPE_INCLUDE          "<core/features/detectors/segment/segmentpoint.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<core/features/detectors/segment/segmentpoint_struct.h>")
SET (LDYNVECTYPE                 "segment_point")
SET (DYNVECTYPE                  "Segment_Point")
SET (SDYNVECTYPE                 "Segment_Point_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of brief
SET (DYNVECTYPE_INCLUDE          "<core/features/descriptors/brief/brief_point.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<core/features/descriptors/brief/brief_point_struct.h>")
SET (LDYNVECTYPE                 "brief_point")
SET (DYNVECTYPE                  "Brief_Point")
SET (SDYNVECTYPE                 "Brief_Point_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of ehid
SET (DYNVECTYPE_INCLUDE          "<core/features/descriptors/ehid/ehid_point.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<core/features/descriptors/ehid/ehid_point_struct.h>")
SET (LDYNVECTYPE                 "ehid_point")
SET (DYNVECTYPE                  "Ehid_Point")
SET (SDYNVECTYPE                 "Ehid_Point_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of ehid_dbindex
SET (DYNVECTYPE_INCLUDE          "<core/features/descriptors/ehid/ehid_dbindex.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<core/features/descriptors/ehid/ehid_dbindex_struct.h>")
SET (LDYNVECTYPE                 "ehid_dbindex")
SET (DYNVECTYPE                  "Ehid_DbIndex")
SET (SDYNVECTYPE                 "Ehid_DbIndex_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of ehid_match
SET (DYNVECTYPE_INCLUDE          "<core/features/descriptors/ehid/ehid_match.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<core/features/descriptors/ehid/ehid_match_struct.h>")
SET (LDYNVECTYPE                 "ehid_match")
SET (DYNVECTYPE                  "Ehid_Match")
SET (SDYNVECTYPE                 "Ehid_Match_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of ehid db node
SET (DYNVECTYPE_INCLUDE          "<core/features/descriptors/ehid/ehid_dbnode_struct.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<core/features/descriptors/ehid/ehid_dbnode_struct.h>")
SET (LDYNVECTYPE                 "ehid_dbnode")
SET (DYNVECTYPE                  "Ehid_DbNode")
SET (SDYNVECTYPE                 "Ehid_DbNode_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of tlid segment
SET (DYNVECTYPE_INCLUDE          "<core/features/descriptors/tlid/tlid_struct.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<core/features/descriptors/tlid/tlid_struct.h>")
SET (LDYNVECTYPE                 "tlid_segment")
SET (DYNVECTYPE                  "TLID_Segment")
SET (SDYNVECTYPE                 "TLID_Segment_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

#dynamic vector of fpsm feature
SET (DYNVECTYPE_INCLUDE          "<core/features/descriptors/fpsm/fpsm_feature_struct.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<core/features/descriptors/fpsm/fpsm_feature_struct.h>")
SET (LDYNVECTYPE                 "fpsm_feature")
SET (DYNVECTYPE                  "Fpsm_Feature")
SET (SDYNVECTYPE                 "Fpsm_Feature_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of fpsm template
SET (DYNVECTYPE_INCLUDE          "<core/features/descriptors/fpsm/fpsm_template_struct.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<core/features/descriptors/fpsm/fpsm_template_struct.h>")
SET (LDYNVECTYPE                 "fpsm_template")
SET (DYNVECTYPE                  "Fpsm_Template")
SET (SDYNVECTYPE                 "Fpsm_Template_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of triangles
SET (DYNVECTYPE_INCLUDE          "<baseproc/geometry/triangle/triangle_struct.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<baseproc/geometry/triangle/triangle_struct.h>")
SET (LDYNVECTYPE                 "triangle_index")
SET (DYNVECTYPE                  "Triangle_Index")
SET (SDYNVECTYPE                 "Triangle_Index_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of triangle double
SET (DYNVECTYPE_INCLUDE          "<baseproc/geometry/triangle/triangle_struct.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<baseproc/geometry/triangle/triangle_struct.h>")
SET (LDYNVECTYPE                 "triangle_double")
SET (DYNVECTYPE                  "Triangle_Double")
SET (SDYNVECTYPE                 "Triangle_Double_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of checker corners
SET (DYNVECTYPE_INCLUDE          "<core/features/detectors/checkerboard/checkercorner_struct.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<core/features/detectors/checkerboard/checkercorner_struct.h>")
SET (LDYNVECTYPE                 "checkercorner")
SET (DYNVECTYPE                  "CheckerCorner")
SET (SDYNVECTYPE                 "CheckerCorner_Struct")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)

# Dynamic vector of bundle measures
SET (DYNVECTYPE_INCLUDE          "<core/bundle/bundle_measure.h>")
SET (DYNVECTYPE_INCLUDE_STRUCT   "<core/bundle/bundle_measure.h>")
SET (LDYNVECTYPE                 "bundle_measure")
SET (DYNVECTYPE                  "Bundle_Measure")
SET (SDYNVECTYPE                 "Bundle_Measure")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.h           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dynvec_template.c           ${OPENROX_BINARY_DIR}/generated/dynvec_${LDYNVECTYPE}.c @ONLY)
