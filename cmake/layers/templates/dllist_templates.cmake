# Dynamic linked list definitions

SET (DLLISTTYPE_INCLUDE    "<system/memory/datatypes.h>")
SET (DLLIST_STRUCT_INCLUDE "<system/memory/datatypes.h>")
SET (LDLLISTTYPE     "uint")
SET (DLLISTTYPE      "Uint")
SET (DLLISTTYPENAME  "UINT")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dllist_template_struct.h    ${OPENROX_BINARY_DIR}/generated/dllist_${LDLLISTTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dllist_template.h           ${OPENROX_BINARY_DIR}/generated/dllist_${LDLLISTTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dllist_template.c           ${OPENROX_BINARY_DIR}/generated/dllist_${LDLLISTTYPE}.c @ONLY)

SET (DLLISTTYPE_INCLUDE    "<core/occupancy/quadtree_item.h>")
SET (DLLIST_STRUCT_INCLUDE "<core/occupancy/quadtree_item_struct.h>")
SET (LDLLISTTYPE           "quadtree_item")
SET (DLLISTTYPE            "QuadTree_Item")
SET (DLLISTTYPENAME        "QUADTREE_ITEM")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dllist_template_struct.h  ${OPENROX_BINARY_DIR}/generated/dllist_${LDLLISTTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dllist_template.h  ${OPENROX_BINARY_DIR}/generated/dllist_${LDLLISTTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/dllist_template.c  ${OPENROX_BINARY_DIR}/generated/dllist_${LDLLISTTYPE}.c @ONLY)


