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
# 3. HydraSDR
# ==============================================================================
message(STATUS ">> Searching for HydraSDR...")

set(USE_MANUAL_SEARCH FALSE)
set(HYDRASDR_DISPLAY_LIB "")
set(HYDRASDR_DISPLAY_INC "")

# A. Try CMake Config
find_package(HydraSDR QUIET NO_CMAKE_SYSTEM_PACKAGE_REGISTRY NO_CMAKE_PACKAGE_REGISTRY)

if(HydraSDR_FOUND)
    foreach(tgt HydraSDR::HydraSDR HydraSDR::hydrasdr hydrasdr)
        if(TARGET ${tgt})
            set(HYDRASDR_LIBS ${tgt})
            set(HYDRASDR_SOURCE "CMake Config (Target: ${tgt})")
            set(HYDRASDR_TARGET ${tgt})
            break()
        endif()
    endforeach()

    if(NOT HYDRASDR_TARGET)
        message(WARNING "[!] HydraSDRConfig found but no usable target.")
        set(USE_MANUAL_SEARCH TRUE)
    else()
        get_target_property(HYDRASDR_DISPLAY_LIB ${HYDRASDR_TARGET} IMPORTED_LOCATION)
        if(NOT HYDRASDR_DISPLAY_LIB)
            get_target_property(HYDRASDR_DISPLAY_LIB ${HYDRASDR_TARGET} IMPORTED_LOCATION_RELEASE)
        endif()
        get_target_property(HYDRASDR_DISPLAY_INC ${HYDRASDR_TARGET} INTERFACE_INCLUDE_DIRECTORIES)
    endif()
else()
    set(USE_MANUAL_SEARCH TRUE)
endif()

# B. Manual search
if(USE_MANUAL_SEARCH)
    if(NOT CMAKE_LIBRARY_ARCHITECTURE AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
        execute_process(
            COMMAND dpkg-architecture -qDEB_HOST_MULTIARCH
            OUTPUT_VARIABLE CMAKE_LIBRARY_ARCHITECTURE
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
    endif()

    find_library(HYDRASDR_LIB
        NAMES hydrasdr libhydrasdr
        PATHS ${CMAKE_SOURCE_DIR}/lib
        HINTS $ENV{SDK_ROOT} ${CMAKE_PREFIX_PATH}
        PATH_SUFFIXES
            lib lib64 bin
            lib/${CMAKE_LIBRARY_ARCHITECTURE}
            lib/x86_64-linux-gnu
            lib/aarch64-linux-gnu
    )

    find_path(HYDRASDR_INCLUDE_DIR
        NAMES hydrasdr.h
        PATHS ${CMAKE_SOURCE_DIR}/include
        HINTS $ENV{SDK_ROOT} ${CMAKE_PREFIX_PATH}
        PATH_SUFFIXES include include/libhydrasdr
    )

    if(NOT HYDRASDR_LIB OR NOT HYDRASDR_INCLUDE_DIR)
        message(FATAL_ERROR
            "[FAIL] HydraSDR not found.\n"
            "Checked: ${CMAKE_SOURCE_DIR}/lib, ENV{SDK_ROOT}, CMAKE_PREFIX_PATH"
        )
    endif()

    set(HYDRASDR_LIBS ${HYDRASDR_LIB})
    set(HYDRASDR_SOURCE "Manual Search")
    set(HYDRASDR_DISPLAY_LIB ${HYDRASDR_LIB})
    set(HYDRASDR_DISPLAY_INC ${HYDRASDR_INCLUDE_DIR})
    include_directories(${HYDRASDR_INCLUDE_DIR})
endif()

print_lib_status("HydraSDR"
    "${HYDRASDR_SOURCE}"
    "${HYDRASDR_DISPLAY_LIB}"
    "${HYDRASDR_DISPLAY_INC}"
)


# ==============================================================================
# Build Target
# ==============================================================================
file(GLOB SOURCES "src/*.cc")
file(GLOB HEADERS "src/*.h")

add_executable(kal ${SOURCES} ${HEADERS})

target_include_directories(kal PRIVATE
    ${HYDRASDR_INCLUDE_DIR}
    ${FFTW3_INCLUDE_DIRS}
)

set(KAL_LIBS
    ${HYDRASDR_LIBS}
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

    if(HYDRASDR_LIBRARY_DIR)
        get_filename_component(SDK_ROOT "${HYDRASDR_LIBRARY_DIR}" DIRECTORY)
        copy_dll_if_exists("${SDK_ROOT}/bin/libhydrasdr.dll"   $<TARGET_FILE_DIR:kal>)
        copy_dll_if_exists("${HYDRASDR_LIBRARY_DIR}/libhydrasdr.dll" $<TARGET_FILE_DIR:kal>)
    endif()

    copy_dll_if_exists("${CMAKE_SOURCE_DIR}/libhydrasdr.dll"          $<TARGET_FILE_DIR:kal>)
    copy_dll_if_exists("${CMAKE_SOURCE_DIR}/lib/libhydrasdr.dll"      $<TARGET_FILE_DIR:kal>)
    copy_dll_if_exists("${MSYS2_BIN}/libusb-1.0.dll"                  $<TARGET_FILE_DIR:kal>)
endif()


# ==============================================================================
# Installation
# ==============================================================================
include(GNUInstallDirs)
install(TARGETS kal RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
