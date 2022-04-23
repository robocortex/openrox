#EXTERN LAYER DEFINITION

set(EXTERN_SOURCES_DIR ${OPENROX_CODE_DIR}/extern)

set(EXTERN_LAYER_CAD_MODEL
      ${EXTERN_SOURCES_DIR}/caofile/caofile_edges.c
)

#Add sources
set (EXTERN_LAYER_SOURCES
   ${EXTERN_LAYER_CAD_MODEL}
)

#define hierarchical organization for ide projects
source_group("extern\\cad_model"            FILES    ${EXTERN_LAYER_CAD_MODEL})
   
