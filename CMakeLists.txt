# ----------------------------------------------------------------------------
#  Root CMake file for Camera Capture Code
# ----------------------------------------------------------------------------
#
# To perform an out of source build do the following on the command line:
# 1. navigate to the root project directory (in this case DfD2)
# 2. mkdir build
# 3. cd build
# 4. depending on your system (windows/linux) type the following
# Example CMake build line for windows & VS2015
# windows: cmake -G "Visual Studio 14 2015 Win64" -DUSE_AVX_INSTRUCTIONS=ON ..
# linux: cmake -DUSE_AVX_INSTRUCTIONS=ON ..
#
# 5. To compile:
# windows: cmake --build . --config Release
# linux: cmake --build . --config Release -- -j4
#


cmake_minimum_required(VERSION 2.8.12)

PROJECT(Cam_Capture_Project)
set(exe_name "cam_cap")

set(PLATFORM "$ENV{PLATFORM}" CACHE INTERNAL "Copied from environment variable")

message("Platform: " ${PLATFORM})

if(${PLATFORM} STREQUAL "Laptop")
    set(COMMON_ROOT "D:/Common")

elseif(${PLATFORM} STREQUAL "MainGear")
    set(COMMON_ROOT "~/Common")

elseif(${PLATFORM} STREQUAL "HPC")
    set(COMMON_ROOT "$HOME/Common")
endif()

include(${COMMON_ROOT}/cmake_utils/use_cpp_11.cmake)
include(${COMMON_ROOT}/cmake_utils/use_sse_avx.cmake)


file(GLOB_RECURSE SOURCES "src/*.c*")

set(project_includes "include" "${COMMON_ROOT}/include")

include(${COMMON_ROOT}/cmake_utils/find_ftdi.cmake)
if(FTDI_FOUND)
    message(STATUS "Found FTDI Library: " ${FTDI_LIBS})
    set(project_includes ${project_includes} ${FTDI_INCLUDE_DIRS})
    set(project_libs ${project_libs} ${FTDI_LIBS})
endif()

include(${COMMON_ROOT}/cmake_utils/find_flycapture.cmake)
if(FC2_FOUND)
    message(STATUS "Found FlyCapture Library: " ${FC2_LIBS})
    set(project_includes ${project_includes} ${FC2_INCLUDE_DIRS})
    set(project_libs ${project_libs} ${FC2_LIBS})
endif()

message(" ")
message("--------------------------------------------------------")
message("Source Files: ")
foreach(S ${SOURCES})
    message("  " ${S})
endforeach()
message("--------------------------------------------------------")
message(" ")


# find OpenCV
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
	set(project_includes ${project_includes} ${OpenCV_INCLUDE_DIRS})
    
    message(" ")
    message("--------------------------------------------------------")
    message("Project Includes: ")
    foreach(S ${project_includes})
        message("  " ${S})
    endforeach()
    message("--------------------------------------------------------")
    message(" ")
    
    include_directories(${project_includes})
	add_executable(${exe_name} ${SOURCES})
	#target_link_libraries(${exe_name} ${OpenCV_LIBS} ${FTDI_LIBS} ${FC2_LIBS})
	target_link_libraries(${exe_name} ${OpenCV_LIBS} ${project_libs})

else()
	message("OpenCV not found, so we won't build the project.")
endif()

if(WIN32)
    # uncomment for MSVC and cmake greater than 3.6.0
    set(default_build_type "Release")
    if(MSVC AND (${CMAKE_VERSION} VERSION_GREATER "3.6.0"))
        target_compile_options(${exe_name} PRIVATE "/MP")
        #set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} /MP)
        #set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} /MP)
        set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${exe_name})
    endif()

endif()

message(" ")
message(STATUS "Build Complete!")
message(" ")



