#IO LAYER DEFINITION

set(IO_SOURCES_DIR ${OPENROX_CODE_DIR}/inout)

set(IO_LAYER_DRAWING_SOURCES
)

set(IO_LAYER_IMAGE_SOURCES
      ${IO_SOURCES_DIR}/image/pgm/pgmfile.c
      ${IO_SOURCES_DIR}/image/ppm/ppmfile.c
)

set(IO_LAYER_MASK_SOURCES
      ${IO_SOURCES_DIR}/mask/pgm/mask_pgmfile.c
)

set(IO_LAYER_NUMERIC_SOURCES
      ${IO_SOURCES_DIR}/numeric/scalar_save.c
      ${IO_SOURCES_DIR}/numeric/array_save.c
      ${IO_SOURCES_DIR}/numeric/array2d_print.c
      ${IO_SOURCES_DIR}/numeric/array2d_save.c
      ${IO_SOURCES_DIR}/numeric/array2d_serialize.c
      ${IO_SOURCES_DIR}/numeric/complex_print.c

      ${IO_SOURCES_DIR}/numeric/objset_array2d_print.c
      ${IO_SOURCES_DIR}/numeric/objset_array2d_serialize.c
      ${IO_SOURCES_DIR}/numeric/dynvec_print.c
      ${IO_SOURCES_DIR}/numeric/dynvec_serialize.c
      ${IO_SOURCES_DIR}/numeric/objset_dynvec_serialize.c
      ${IO_SOURCES_DIR}/numeric/objset_serialize.c

      # ANSI functions
      ${IO_SOURCES_DIR}/numeric/ansi_array_print.c
      ${IO_SOURCES_DIR}/numeric/ansi_array_save.c
      ${IO_SOURCES_DIR}/numeric/ansi_array2d_print.c
      ${IO_SOURCES_DIR}/numeric/ansi_array2d_save.c
)

set(IO_LAYER_LUT_SOURCES
      ${IO_SOURCES_DIR}/lut/txt/array2d_point2d_float_txtfile.c
)

set(IO_LAYER_FEATURES_SOURCES
      ${IO_SOURCES_DIR}/features/sdwm_serialize.c
      ${IO_SOURCES_DIR}/features/sdwm_object_serialize.c
      ${IO_SOURCES_DIR}/features/pyramid_npot_serialize.c
      ${IO_SOURCES_DIR}/features/fpsm_serialize.c
      ${IO_SOURCES_DIR}/features/edge_serialize.c
)

set(IO_LAYER_GEOMETRY_SOURCES
   ${IO_SOURCES_DIR}/geometry/point/point2d_print.c
   ${IO_SOURCES_DIR}/geometry/point/point3d_print.c
   ${IO_SOURCES_DIR}/geometry/point/point2d_save.c
   ${IO_SOURCES_DIR}/geometry/point/point3d_save.c
   ${IO_SOURCES_DIR}/geometry/point/array2d_point2d_print.c
   ${IO_SOURCES_DIR}/geometry/point/dynvec_point2d_print.c
   ${IO_SOURCES_DIR}/geometry/point/dynvec_point3d_print.c
   ${IO_SOURCES_DIR}/geometry/point/dynvec_point2d_save.c
   ${IO_SOURCES_DIR}/geometry/point/dynvec_point3d_save.c
   ${IO_SOURCES_DIR}/geometry/point/objset_dynvec_point2d_print.c
   ${IO_SOURCES_DIR}/geometry/point/objset_dynvec_point3d_print.c
   ${IO_SOURCES_DIR}/geometry/line/line2d_print.c
   ${IO_SOURCES_DIR}/geometry/line/line3d_print.c
   ${IO_SOURCES_DIR}/geometry/line/line3d_save.c
   ${IO_SOURCES_DIR}/geometry/plane/plane3d_print.c
   ${IO_SOURCES_DIR}/geometry/segment/segment3d_print.c
   ${IO_SOURCES_DIR}/geometry/segment/segment3d_save.c
)


set (IO_LAYER_SERIALIZATION_SOURCES
   ${IO_SOURCES_DIR}/serialization/dynvec_ehid_point_serialization.c
   ${IO_SOURCES_DIR}/serialization/dynvec_ehid_dbindex_serialization.c
)

set (IO_LAYER_SYSTEM_SOURCES
   ${IO_SOURCES_DIR}/system/errors_print.c
   ${IO_SOURCES_DIR}/system/memory_print.c
   ${IO_SOURCES_DIR}/system/print.c
   ${IO_SOURCES_DIR}/system/file.c
)

#Add sources
set (IO_LAYER_SOURCES
   ${IO_LAYER_IMAGE_SOURCES}
   ${IO_LAYER_MASK_SOURCES}
   ${IO_LAYER_NUMERIC_SOURCES}
   ${IO_LAYER_DRAWING_SOURCES}
   ${IO_LAYER_VISUALIZATION_SOURCES}
   ${IO_LAYER_VIDEO_SOURCES}
   ${IO_LAYER_CAPTURE_SOURCES}
   ${IO_LAYER_LUT_SOURCES}
   ${IO_LAYER_FEATURES_SOURCES}
   ${IO_LAYER_SERIALIZATION_SOURCES}
   ${IO_LAYER_SYSTEM_SOURCES}
   ${IO_LAYER_GEOMETRY_SOURCES}
)

#define hierarchical organization for ide projects
source_group("inout\\image"            FILES    ${IO_LAYER_IMAGE_SOURCES})
source_group("inout\\mask"             FILES    ${IO_LAYER_MASK_SOURCES})
source_group("inout\\numeric"          FILES    ${IO_LAYER_NUMERIC_SOURCES})
source_group("inout\\drawing"          FILES    ${IO_LAYER_DRAWING_SOURCES})
source_group("inout\\visualization"    FILES    ${IO_LAYER_VISUALIZATION_SOURCES})
source_group("inout\\video"            FILES    ${IO_LAYER_VIDEO_SOURCES})
source_group("inout\\capture"          FILES    ${IO_LAYER_CAPTURE_SOURCES})
source_group("inout\\lut"              FILES    ${IO_LAYER_LUT_SOURCES})
source_group("inout\\features"         FILES    ${IO_LAYER_FEATURES_SOURCES})
source_group("inout\\serialization"    FILES    ${IO_LAYER_SERIALIZATION_SOURCES})
source_group("inout\\system"           FILES    ${IO_LAYER_SYSTEM_SOURCES})
source_group("inout\\geometry"         FILES    ${IO_LAYER_GEOMETRY_SOURCES})
