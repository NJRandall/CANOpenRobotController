cmake_minimum_required(VERSION 3.5)
project(CORC C CXX)

# Export compile commands so VSCode IntelliSense can find all headers automatically.
# After running cmake, point c_cpp_properties.json compileCommands at
# build/compile_commands.json for accurate squiggle-free editing.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build for BeagleBone (ARM):
#   mkdir -p build && cd build
#   cmake -DCMAKE_TOOLCHAIN_FILE=../armhf.cmake ..
#   make -j$(nproc)
#   cd .. && ./script/uploadBB.sh
# For aarch64 (e.g. Jetson): use aarch64.cmake instead of armhf.cmake

################################## USER FLAGS ##################################

### Select the application by setting the state machine app.cmake to use
include(../MTRobotMachine/app.cmake)
#include(src/apps/ExoTestMachine/app.cmake)
#include(src/apps/FITHVExoDemoMachine/app.cmake)
#include(src/apps/M1DemoMachine/app.cmake)
#include(src/apps/M1DemoMachineROS/app.cmake)
#include(src/apps/M2DemoMachine/app.cmake)
#include(src/apps/M2ProDemoMachine/app.cmake)
#include(src/apps/M3DemoMachine/app.cmake)
#include(src/apps/X2DemoMachine/app.cmake)
#include(src/apps/X2ROS2DemoMachine/app.cmake)
#include(src/apps/LoggingDevice/app.cmake)
#include(../myStateMachineApp/app.cmake) ## example only, need to be defined

## Comment to use actual hardware, uncomment for a nor robot (virtual) app
#set(NO_ROBOT ON)

## Select desired logging level (Options: TRACE, DEBUG, INFO, WARN, ERROR, CRITICAL, OFF)
## INFO is the recommended level in normal operation
set(CORC_LOGGING_LEVEL DEBUG)

################################################################################

## CORC internal cmake logic
include(corc.cmake)

## LCD1602 background thread requires pthread
target_link_libraries(${APP_NAME} pthread)

## Copy MTR_params.yaml to build/config/ for cross-compile builds where
## BASE_DIRECTORY="." and the binary looks for config/ relative to itself.
## (For native builds BASE_DIRECTORY=CMAKE_SOURCE_DIR and the file is found
## directly in the source tree at config/MTR_params.yaml.)
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/config/MTR_params.yaml")
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/config")
    configure_file(
        "${CMAKE_CURRENT_LIST_DIR}/config/MTR_params.yaml"
        "${CMAKE_BINARY_DIR}/config/MTR_params.yaml"
        COPYONLY
    )
endif()

add_custom_command(TARGET ${APP_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        "$<TARGET_FILE_DIR:${APP_NAME}>/config"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_LIST_DIR}/config/MTR_params.yaml"
        "$<TARGET_FILE_DIR:${APP_NAME}>/config/MTR_params.yaml"
)