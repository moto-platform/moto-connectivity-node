# Application component: everything under src/ plus the generated moto-vehicle-defs
# code for this node (only the CL250 DID table is used today; this node is not on the
# platform bus yet, so platform.c / moto_e2e.c are not linked).
set(DEFS_GEN ${CMAKE_SOURCE_DIR}/external/moto-vehicle-defs/gen/c/conn)
if(NOT EXISTS ${DEFS_GEN}/vehicle_cl250.c)
    message(FATAL_ERROR "Missing ${DEFS_GEN}. Run: git submodule update --init")
endif()

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)
idf_component_register(
    SRCS ${app_sources} ${DEFS_GEN}/vehicle_cl250.c
    INCLUDE_DIRS "." ${DEFS_GEN}
)
