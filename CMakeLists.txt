cmake_minimum_required(VERSION 3.10)
project(kalibrate-hydrasdr CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_SOURCE_DIR}/src)

# ==============================================================================
# Helper: Print Library Status
# ==============================================================================
function(print_lib_status name source lib inc)
    message(STATUS "---------------------------------------------------")
    message(STATUS "[${name}] Status:")
    message(STATUS "   + Source:  ${source}")
    message(STATUS "   + Library: ${lib}")
    message(STATUS "   + Include: ${inc}")
    message(STATUS "---------------------------------------------------")
endfunction()

message(STATUS "=============================================================")
message(STATUS "Configuring kalibrate-hydrasdr")
message(STATUS "System : ${CMAKE_SYSTEM_NAME} (${CMAKE_SYSTEM_PROCESSOR})")
message(STATUS "Source : ${CMAKE_SOURCE_DIR}")
message(STATUS "Prefix : ${CMAKE_PREFIX_PATH}")
message(STATUS "=============================================================")

find_package(PkgConfig QUIET)

# ==============================================================================
# 1. LibUSB-1.0
# ==============================================================================
message(STATUS ">> Searching for LibUSB-1.0...")

if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_LIBUSB QUIET libusb-1.0)
endif()

find_path(LIBUSB_INCLUDE_DIR
    NAMES libusb.h
    HINTS ${PC_LIBUSB_INCLUDE_DIRS} ${CMAKE_PREFIX_PATH}
    PATH_SUFFIXES include/libusb-1.0 include
)

find_library(LIBUSB_LIBRARY
    NAMES usb-1.0 libusb-1.0
    HINTS ${PC_LIBUSB_LIBRARY_DIRS} ${CMAKE_PREFIX_PATH}
    PATH_SUFFIXES lib bin
)

if(NOT LIBUSB_INCLUDE_DIR OR NOT LIBUSB_LIBRARY)
    message(FATAL_ERROR "[FAIL] LibUSB-1.0 not found.")
endif()

set(LIBUSB_FOUND TRUE)
set(LIBUSB_INCLUDE_DIRS ${LIBUSB_INCLUDE_DIR})
set(LIBUSB_LIBRARIES ${LIBUSB_LIBRARY})

set(LIBUSB_SOURCE
    $<IF:$<BOOL:${PC_LIBUSB_FOUND}>,PkgConfig,Manual Search>
)

get_filename_component(LIBUSB_LIBRARY_DIR "${LIBUSB_LIBRARY}" DIRECTORY)

print_lib_status("LibUSB" "${LIBUSB_SOURCE}" "${LIBUSB_LIBRARIES}" "${LIBUSB_INCLUDE_DIRS}")

# Force cache for HydraSDRConfig compatibility
set(LIBUSB_INCLUDE_DIR "${LIBUSB_INCLUDE_DIRS}" CACHE PATH "" FORCE)
set(LIBUSB_LIBRARY     "${LIBUSB_LIBRARIES}"    CACHE FILEPATH "" FORCE)


# ==============================================================================
# 2. FFTW3
# ==============================================================================
message(STATUS ">> Searching for FFTW3...")

if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_FFTW3 QUIET fftw3)
endif()

if(NOT PC_FFTW3_FOUND)
    find_path(FFTW3_INCLUDE_DIRS NAMES fftw3.h
        HINTS ${CMAKE_PREFIX_PATH} ${PC_FFTW3_INCLUDE_DIRS}
    )
    find_library(FFTW3_LIBRARIES NAMES fftw3 libfftw3-3
        HINTS ${CMAKE_PREFIX_PATH} ${PC_FFTW3_LIBRARY_DIRS}
    )
else()
    set(FFTW3_INCLUDE_DIRS ${PC_FFTW3_INCLUDE_DIRS})
    set(FFTW3_LIBRARIES    ${PC_FFTW3_LIBRARIES})
endif()

# macOS: fix pkg-config returning "fftw3" instead of a real path
if(APPLE)
    # If library is not an actual file, resolve it manually.
    if(NOT EXISTS "${FFTW3_LIBRARIES}")
        message(STATUS "macOS: pkg-config returned '${FFTW3_LIBRARIES}', resolving actual file...")
        
        # Use a DISTINCT variable name to avoid shadowing the pkg-config local variable
        find_library(FFTW3_RESOLVED_PATH
            NAMES fftw3 libfftw3-3
            HINTS ${CMAKE_PREFIX_PATH} /opt/homebrew/lib /usr/local/lib
            PATH_SUFFIXES lib
        )

        if(FFTW3_RESOLVED_PATH)
            # Overwrite the local variable with the absolute path found
            set(FFTW3_LIBRARIES "${FFTW3_RESOLVED_PATH}")
            message(STATUS "macOS: Resolved to '${FFTW3_LIBRARIES}'")
        else()
            message(FATAL_ERROR "[FAIL] Could not resolve absolute path for fftw3 on macOS.")
        endif()
    endif()
endif()

if(NOT FFTW3_LIBRARIES)
    message(FATAL_ERROR "[FAIL] FFTW3 not found.")
endif()

set(FFTW3_SOURCE
    $<IF:$<BOOL:${PC_FFTW3_FOUND}>,PkgConfig,Manual Search>
)

get_filename_component(FFTW3_LIBRARY_DIR "${FFTW3_LIBRARIES}" DIRECTORY)

print_lib_status("FFTW3" "${FFTW3_SOURCE}" "${FFTW3_LIBRARIES}" "${FFTW3_INCLUDE_DIRS}")

# ==============================================================================
# 3. LibIIO
# ==============================================================================
message(STATUS ">> Searching for LibIIO...")

if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_LIBIIO QUIET libiio)
endif()

find_path(LIBIIO_INCLUDE_DIR
    NAMES iio.h
    HINTS ${PC_LIBIIO_INCLUDE_DIRS} ${CMAKE_PREFIX_PATH}
    PATH_SUFFIXES include
)

find_library(LIBIIO_LIBRARY
    NAMES iio libiio
    HINTS ${PC_LIBIIO_LIBRARY_DIRS} ${CMAKE_PREFIX_PATH}
    PATH_SUFFIXES lib bin
)

if(NOT LIBIIO_INCLUDE_DIR OR NOT LIBIIO_LIBRARY)
    message(FATAL_ERROR "[FAIL] LibIIO not found.")
endif()

set(LIBIIO_INCLUDE_DIRS ${LIBIIO_INCLUDE_DIR})
set(LIBIIO_LIBRARIES ${LIBIIO_LIBRARY})

print_lib_status("LibIIO" "Manual/PkgConfig" "${LIBIIO_LIBRARIES}" "${LIBIIO_INCLUDE_DIRS}")

# ==============================================================================
# Build Target
# ==============================================================================
file(GLOB SOURCES "src/*.cc")
list(FILTER SOURCES EXCLUDE REGEX "hydrasdr_source\\.cc$")
file(GLOB HEADERS "src/*.h")

add_executable(kal ${SOURCES} ${HEADERS})

target_include_directories(kal PRIVATE
    ${LIBIIO_INCLUDE_DIRS}
    ${FFTW3_INCLUDE_DIRS}
)

set(KAL_LIBS
    ${LIBIIO_LIBRARIES}
    ${FFTW3_LIBRARIES}
)

# ==============================================================================
# Platform Specific Configuration
# ==============================================================================
if(WIN32 AND MINGW)
    target_compile_options(kal PRIVATE -O3)
    target_link_options(kal PRIVATE -static -static-libgcc -static-libstdc++)
elseif(WIN32 AND MSVC)
    target_compile_definitions(kal PRIVATE _WIN32 _CRT_SECURE_NO_WARNINGS _USE_MATH_DEFINES)
elseif(APPLE)
    target_compile_options(kal PRIVATE -O3)
elseif(UNIX)
    target_compile_options(kal PRIVATE -O3 -pthread)
    target_link_options(kal PRIVATE -pthread)
    list(APPEND KAL_LIBS m)
endif()

target_link_libraries(kal PRIVATE ${KAL_LIBS})


# ==============================================================================
# DLL Copy (Windows)
# ==============================================================================
if(WIN32)
    message(STATUS ">> Configuring DLL copy steps...")

    if(DEFINED ENV{MSYSTEM_PREFIX})
        set(MSYS2_BIN "$ENV{MSYSTEM_PREFIX}/bin")
    elseif(LIBUSB_LIBRARY_DIR)
        get_filename_component(MSYS2_ROOT "${LIBUSB_LIBRARY_DIR}" DIRECTORY)
        set(MSYS2_BIN "${MSYS2_ROOT}/bin")
    else()
        set(MSYS2_BIN "/mingw64/bin")
    endif()

    function(copy_dll_if_exists src target_dir)
        if(EXISTS "${src}")
            message(STATUS "   Will copy: ${src}")
            add_custom_command(TARGET kal POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different "${src}" "${target_dir}"
            )
        endif()
    endfunction()

    get_filename_component(LIBIIO_LIB_DIR "${LIBIIO_LIBRARY}" DIRECTORY)
    copy_dll_if_exists("${LIBIIO_LIB_DIR}/libiio.dll" $<TARGET_FILE_DIR:kal>)
    copy_dll_if_exists("${LIBIIO_LIB_DIR}/../bin/libiio.dll" $<TARGET_FILE_DIR:kal>)

    copy_dll_if_exists("${MSYS2_BIN}/libusb-1.0.dll"                  $<TARGET_FILE_DIR:kal>)
endif()


# ==============================================================================
# Installation
# ==============================================================================
include(GNUInstallDirs)
install(TARGETS kal RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
