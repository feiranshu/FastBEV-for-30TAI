cmake_minimum_required(VERSION 3.16)
project(FastBEV_Pipeline LANGUAGES CXX C)

message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Source dir: ${PROJECT_SOURCE_DIR}")

set(TARGET_CHIP "ZG" CACHE STRING "Target chip backend: BY (Buyi) or ZG (ZG330)")
set_property(CACHE TARGET_CHIP PROPERTY STRINGS "BY" "ZG")
string(TOUPPER "${TARGET_CHIP}" TARGET_CHIP_UPPER)

if(TARGET_CHIP_UPPER STREQUAL "BY")
    add_compile_definitions(USE_BUYI_BACKEND)
    message(STATUS "Backend: Buyi (BY)")
elseif(TARGET_CHIP_UPPER STREQUAL "ZG")
    add_compile_definitions(USE_ZG330_BACKEND)
    message(STATUS "Backend: ZG330 (ZG)")
else()
    message(FATAL_ERROR "Invalid TARGET_CHIP: ${TARGET_CHIP}. Choose 'BY' or 'ZG'.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(DEPS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps)

set(THIRDPARTY_INCLUDES
    ${DEPS_DIR}/thirdparty/include
    ${DEPS_DIR}/thirdparty/include/yaml-cpp
    ${DEPS_DIR}/thirdparty/include/opencv2
    ${DEPS_DIR}/thirdparty/include/Eigen-3.4.0
    ${DEPS_DIR}/thirdparty/include/Eigen-3.4.0/Eigen
    ${DEPS_DIR}/thirdparty/include/fmt
    ${DEPS_DIR}/thirdparty/include/spdlog
    ${DEPS_DIR}/thirdparty/include/onnxruntime
    ${DEPS_DIR}/thirdparty/include/OpenEXR
    ${DEPS_DIR}/thirdparty/include/ffmpeg
)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)

link_directories(
    ${DEPS_DIR}/thirdparty/a
    ${DEPS_DIR}/thirdparty/so
)

link_libraries(
    avformat avdevice avfilter avcodec avutil swscale swresample
    opencv_imgcodecs opencv_imgproc opencv_core tegra_hal
    libjpeg-turbo ittnotify libpng libwebp libtiff libopenjp2
    IlmImf-2_3_s Iex-2_3_s Half-2_3_s IexMath-2_3_s
    IlmImfUtil-2_3_s IlmThread-2_3_s Imath-2_3_s
    fmt pthread dl zlib ssl crypto atomic yaml-cpp
)

find_package(Icraft-BuyiBackend REQUIRED)
find_package(Icraft-HostBackend REQUIRED)
find_package(icraft-zg330backend REQUIRED)

function(configure_fastbev_target target_name)
    target_include_directories(${target_name} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${DEPS_DIR}/modelzoo_utils/include
        ${THIRDPARTY_INCLUDES}
    )
    target_link_libraries(${target_name} PRIVATE
        Icraft::HostBackend
        Icraft::BuyiBackend
        Icraft::ZG330Backend
        stdc++fs
    )
    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
            $<TARGET_FILE:${target_name}>
            ${CMAKE_CURRENT_SOURCE_DIR}/${target_name}
    )
endfunction()

# NuScenes synchronous pipeline with PC web push plus FPGA LED / USB audio alerts.
add_executable(fastbev_pipeline_audio
    src/fastbev_pipeline_audio.cpp
    src/alert_manager.cpp
    src/alert_config.cpp
    src/cJSON.c
    src/filter.cpp
    src/nms.cpp
)
configure_fastbev_target(fastbev_pipeline_audio)

# NuScenes experimental coarse pipeline. Uses local result/parameter export.
add_executable(fastbev_pipeline_async
    src/fastbev_pipeline_async.cpp
    src/cJSON.c
    src/filter.cpp
    src/nms.cpp
)
configure_fastbev_target(fastbev_pipeline_async)

# NuScenes Group4/SA validation pipeline. Uses local result/parameter export.
add_executable(fastbev_pipeline_SA
    src/fastbev_pipeline_SA.cpp
    src/cJSON.c
    src/filter.cpp
    src/nms.cpp
)
configure_fastbev_target(fastbev_pipeline_SA)

# CARLA / BEV1 persistent inference service.
add_executable(fastbev_pipeline_carla
    src/fastbev_pipeline_carla.cpp
    src/filter.cpp
    src/nms.cpp
)
configure_fastbev_target(fastbev_pipeline_carla)

# CARLA diagnostic service: legacy FP32 Part2 PL-PS-PL bridge into FP16 Part3,
# with non-blocking USB audio alerts (no LED access).
add_executable(fastbev_pipeline_carla_fp16
    src/fastbev_pipeline_carla_fp16.cpp
    src/alert_manager.cpp
    src/alert_config.cpp
    src/filter.cpp
    src/nms.cpp
)
configure_fastbev_target(fastbev_pipeline_carla_fp16)

# CARLA FP16 production candidate: new Part2 writes NCHWc16 directly into
# Decoder view(12) v244 Runtime PLDDR. Audio alerts are enabled; LED is unused.
add_executable(fastbev_pipeline_carla_new
    src/fastbev_pipeline_carla_new.cpp
    src/alert_manager.cpp
    src/alert_config.cpp
    src/filter.cpp
    src/nms.cpp
)
configure_fastbev_target(fastbev_pipeline_carla_new)

# CARLA modular auto-drive service. Keeps the carla_new FP16 direct Conv
# perception path on 5200 and adds a high-rate control-only path on 5201.
add_executable(fastbev_pipeline_carla_new_modular
    src/fastbev_pipeline_carla_new_modular.cpp
    src/alert_manager.cpp
    src/alert_config.cpp
    src/filter.cpp
    src/nms.cpp
)
configure_fastbev_target(fastbev_pipeline_carla_new_modular)

# Vehicle single-frame local pipeline. Uses local sample images, Vehicle Part2
# v362 direct Conv input, Vehicle postprocess, and visualize_vehicle output.
add_executable(fastbev_pipeline_vehicle
    src/fastbev_pipeline_vehicle.cpp
    src/fastbev_vehicle_postprocess.cpp
    src/nms.cpp
)
configure_fastbev_target(fastbev_pipeline_vehicle)

# Vehicle live Raw BGR service. Receives six 640x480 BGR camera frames from PC,
# runs Vehicle Part1/Part2/Part3, and returns Vehicle 9-column detections as JSON.
add_executable(fastbev_pipeline_vehicle_live
    src/fastbev_pipeline_vehicle_live.cpp
    src/fastbev_vehicle_postprocess.cpp
    src/nms.cpp
)
configure_fastbev_target(fastbev_pipeline_vehicle_live)

# MatrixVT NuScenes serial pipeline. It uses one NPU model and local visualize;
# no FastBEV Part1/Part2/Part3 split and no PL/LUT register access.
add_executable(fastbev_pipeline_matrixvt
    src/fastbev_pipeline_matrixvt.cpp
    src/cJSON.c
)
configure_fastbev_target(fastbev_pipeline_matrixvt)

# MatrixVT native serial verifier. It implements MatrixVT-style decode and PNG
# rendering internally, independent of the shared FastBEV visualize executable.
add_executable(fastbev_pipeline_matrixvt_native
    src/fastbev_pipeline_matrixvt_native.cpp
    src/cJSON.c
)
configure_fastbev_target(fastbev_pipeline_matrixvt_native)

# PointPillars two-stage LiDAR runtime: CPU preprocess/scatter/postprocess plus
# PFE and BEV-heads Icraft sessions.
add_executable(pointpillars
    src/pointpillars.cpp
)
configure_fastbev_target(pointpillars)


# TCP CARLA / TCP1 persistent inference service on Icraft ZG330.
# tcp.cpp is self-contained: TCP1 protocol + JPEG preprocessing +
# Icraft forward + Beta/PID/fusion + TCP result.
add_executable(tcp
    src/tcp.cpp
)
configure_fastbev_target(tcp)

# Local PNG renderer used by the NuScenes local-visualize pipelines.
add_executable(visualize
    src/visualize.cpp
)
target_include_directories(visualize PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${THIRDPARTY_INCLUDES}
)
target_link_libraries(visualize PRIVATE stdc++fs)

# Vehicle sandbox-style local PNG renderer.
add_executable(visualize_vehicle
    src/visualize_vehicle.cpp
)
target_include_directories(visualize_vehicle PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${THIRDPARTY_INCLUDES}
)
target_link_libraries(visualize_vehicle PRIVATE stdc++fs)
