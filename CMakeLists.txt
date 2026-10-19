cmake_minimum_required(VERSION 3.20)
project(rockchip_vaapi_driver LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBVA REQUIRED libva)
pkg_check_modules(LIBVA_DRM REQUIRED libva-drm)
pkg_check_modules(LIBDRM REQUIRED libdrm)

# Ensure we build shared library for the VAAPI driver
add_library(rockchip_drv_video SHARED
    src/driver.cpp
    src/mpp_decoder.cpp
    src/util/log.cpp
)

add_executable(test_mpp_decode tools/test_mpp_decode.cpp)
add_executable(vaapi_decode_test tools/vaapi_decode_test.cpp)
add_executable(gecko_vaapi_probe tools/gecko_vaapi_probe.cpp)
add_executable(panthor_egl_import_test tools/panthor_egl_import_test.cpp)
add_executable(test_slice_framing tools/test_slice_framing.cpp)

target_link_libraries(test_mpp_decode PRIVATE
    dl
    pthread
)

target_link_libraries(vaapi_decode_test PRIVATE
    ${LIBVA_LIBRARIES}
    ${LIBVA_DRM_LIBRARIES}
    dl
    pthread
)

target_link_libraries(gecko_vaapi_probe PRIVATE
    ${LIBVA_LIBRARIES}
    ${LIBVA_DRM_LIBRARIES}
    ${LIBDRM_LIBRARIES}
    dl
    pthread
)

target_link_libraries(panthor_egl_import_test PRIVATE
    ${LIBDRM_LIBRARIES}
    dl
    pthread
)

target_include_directories(test_mpp_decode PRIVATE
    src
)

target_include_directories(vaapi_decode_test PRIVATE
    ${LIBVA_INCLUDE_DIRS}
    ${LIBVA_DRM_INCLUDE_DIRS}
    ${LIBDRM_INCLUDE_DIRS}
    src
)

target_include_directories(gecko_vaapi_probe PRIVATE
    ${LIBVA_INCLUDE_DIRS}
    ${LIBVA_DRM_INCLUDE_DIRS}
    ${LIBDRM_INCLUDE_DIRS}
    src
)

target_include_directories(panthor_egl_import_test PRIVATE
    ${LIBDRM_INCLUDE_DIRS}
    src
)

target_include_directories(test_slice_framing PRIVATE
    ${LIBVA_INCLUDE_DIRS}
    src
)

# The VA-API driver naming convention expects <driver>_drv_video.so (no lib prefix).
set_target_properties(rockchip_drv_video PROPERTIES
    PREFIX ""
    OUTPUT_NAME "rockchip_drv_video"
)

target_compile_definitions(rockchip_drv_video PRIVATE "_GNU_SOURCE")

# The driver is a VA-API -> MPP bridge.  It may only use the installed MPP
# public headers and shared library (the MPI ABI), never MPP source-tree
# internals.  Keep the search paths explicit so a MPP checkout can never leak
# into the build through an accidental -I.
find_library(ROCKCHIP_MPP_LIB NAMES rockchip_mpp PATHS /usr/lib /usr/local/lib
             PATH_SUFFIXES aarch64-linux-gnu NO_DEFAULT_PATH)
find_path(ROCKCHIP_MPP_INCLUDE NAMES rk_mpi.h PATHS /usr/include /usr/local/include
          PATH_SUFFIXES rockchip NO_DEFAULT_PATH)

if (NOT ROCKCHIP_MPP_LIB OR NOT ROCKCHIP_MPP_INCLUDE)
    message(FATAL_ERROR "rockchip-mpp public headers/library not found; install librockchip-mpp")
endif()
message(STATUS "Using installed rockchip-mpp: ${ROCKCHIP_MPP_LIB} (${ROCKCHIP_MPP_INCLUDE})")
set(MPP_INCLUDE_DIR ${ROCKCHIP_MPP_INCLUDE})
set(MPP_LIB ${ROCKCHIP_MPP_LIB})

target_include_directories(rockchip_drv_video PRIVATE
    ${MPP_INCLUDE_DIR}/..
    ${MPP_INCLUDE_DIR}
    ${LIBVA_INCLUDE_DIRS}
    ${LIBDRM_INCLUDE_DIRS}
    src
)

target_link_libraries(rockchip_drv_video PRIVATE
    ${LIBVA_LIBRARIES}
    ${LIBDRM_LIBRARIES}
    dl
    pthread
)

target_link_libraries(rockchip_drv_video PRIVATE ${MPP_LIB})
target_link_libraries(test_mpp_decode PRIVATE ${MPP_LIB})

# Fail the build if anything in the bridge starts depending on MPP internals.
add_custom_target(check-mpp-boundary
    COMMAND bash ${CMAKE_SOURCE_DIR}/tools/check_mpp_boundary.sh ${CMAKE_SOURCE_DIR}
    COMMENT "Checking MPP public-ABI boundary"
    VERBATIM)
add_dependencies(rockchip_drv_video check-mpp-boundary)

# Installation
install(TARGETS rockchip_drv_video
    LIBRARY DESTINATION lib/aarch64-linux-gnu/dri
    RUNTIME DESTINATION bin
)

# Deploy script should be executable
install(PROGRAMS deploy.sh DESTINATION .)
