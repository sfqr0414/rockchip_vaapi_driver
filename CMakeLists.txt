cmake_minimum_required(VERSION 3.20)
project(rockchip_vaapi_driver LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBVA REQUIRED libva)
pkg_check_modules(LIBVA_DRM REQUIRED libva-drm)

# Ensure we build shared library for the VAAPI driver
add_library(rockchip_drv_video SHARED
    src/driver.cpp
    src/mpp_decoder.cpp
    src/util/log.cpp
)

add_executable(test_mpp_decode tools/test_mpp_decode.cpp)
add_executable(vaapi_decode_test tools/vaapi_decode_test.cpp)

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

target_include_directories(vaapi_decode_test PRIVATE
    ${LIBVA_INCLUDE_DIRS}
    ${LIBVA_DRM_INCLUDE_DIRS}
)

# The VA-API driver naming convention expects <driver>_drv_video.so (no lib prefix).
set_target_properties(rockchip_drv_video PROPERTIES
    PREFIX ""
    OUTPUT_NAME "rockchip_drv_video"
)

target_compile_definitions(rockchip_drv_video PRIVATE "_GNU_SOURCE")

# Link against Rockchip MPP and system libraries
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBVA REQUIRED libva)
pkg_check_modules(LIBVA_DRM REQUIRED libva-drm)

# Prefer installed rockchip-mpp (RK3588) if available.
find_library(ROCKCHIP_MPP_LIB NAMES rockchip_mpp)
find_path(ROCKCHIP_MPP_INCLUDE NAMES rk_mpi.h PATH_SUFFIXES rockchip)

if (ROCKCHIP_MPP_LIB AND ROCKCHIP_MPP_INCLUDE)
    message(STATUS "Using system rockchip-mpp: ${ROCKCHIP_MPP_LIB}")
    set(MPP_INCLUDE_DIR ${ROCKCHIP_MPP_INCLUDE})
    set(MPP_LIB ${ROCKCHIP_MPP_LIB})
    set(MPP_FOUND TRUE)
else()
    message(STATUS "rockchip-mpp not found; falling back to local stub implementation")
    set(MPP_FOUND FALSE)
    set(MPP_INCLUDE_DIR "${CMAKE_SOURCE_DIR}")
endif()

target_include_directories(rockchip_drv_video PRIVATE
    ${MPP_INCLUDE_DIR}
    ${LIBVA_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
    src
)

target_link_libraries(rockchip_drv_video PRIVATE
    ${LIBVA_LIBRARIES}
    dl
    pthread
)

if (MPP_FOUND)
    target_link_libraries(rockchip_drv_video PRIVATE ${MPP_LIB})
    target_link_libraries(test_mpp_decode PRIVATE ${MPP_LIB})
else()
    message(FATAL_ERROR "rockchip-mpp not found; please install librockchip-mpp or set MPP_INCLUDE_DIR and MPP_LIB")
endif()

# Installation
install(TARGETS rockchip_drv_video
    LIBRARY DESTINATION lib/aarch64-linux-gnu/dri
    RUNTIME DESTINATION bin
)

# Deploy script should be executable
install(PROGRAMS deploy.sh DESTINATION .)
