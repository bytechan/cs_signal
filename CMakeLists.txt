cmake_minimum_required(VERSION 3.18.0 FATAL_ERROR)

cmake_policy(VERSION 3.18.0..3.29.6)

if (${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.19.0")
   # allows spaces in ctest names
   cmake_policy(SET CMP0110 NEW)
endif()

if (${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.20.0")
   # enable RTTI on MSVC
   cmake_policy(SET CMP0117 OLD)
endif()

project(cs_signal)

set(BUILD_MAJOR "1")
set(BUILD_MINOR "4")
set(BUILD_MICRO "2")

set(BUILD_COMPONENTS "cs_signal")

# catch2 set up
option(BUILD_TESTS "Enables building the Catch2 unit tests" OFF)

include(CheckCXXCompilerFlag)
include(CheckCXXSourceCompiles)
include(CheckIncludeFile)
include(CheckIncludeFiles)
include(CheckTypeSize)

# location for install or package
if (CMAKE_SYSTEM_NAME MATCHES "Darwin")
   include(GNUInstallDirs)
   set(CMAKE_INSTALL_RPATH "@executable_path")

elseif(CMAKE_SYSTEM_NAME MATCHES "(Linux|OpenBSD|FreeBSD|NetBSD|DragonFly)")
   include(GNUInstallDirs)
   set(CMAKE_INSTALL_RPATH "\$ORIGIN")

elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
   set(CMAKE_INSTALL_BINDIR bin)
   set(CMAKE_INSTALL_LIBDIR lib)
   set(CMAKE_INSTALL_INCLUDEDIR include)

endif()

set(PACKAGE               "cs_signal")
set(PACKAGE_NAME          "CsSignal")
set(PACKAGE_VERSION       "${BUILD_MAJOR}.${BUILD_MINOR}.${BUILD_MICRO}")
set(PACKAGE_STRING        "cs_signal ${BUILD_MAJOR}.${BUILD_MINOR}.${BUILD_MICRO}")
set(PACKAGE_TARNAME       "cs_signal")
set(PACKAGE_BUGREPORT     "info@copperspice.com")
set(PACKAGE_URL           "https://www.copperspice.com/")

set(CPACK_PACKAGE_NAME    ${PROJECT_NAME} )
set(CPACK_PACKAGE_VENDOR  "CopperSpice")
set(CPACK_PACKAGE_CONTACT "info@copperspice.com")

set(CPACK_PACKAGE_VERSION_MAJOR ${BUILD_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${BUILD_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${BUILD_MICRO})

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Library for thread aware Signal/Slot delivery")
set(CPACK_PACKAGE_DESCRIPTION_FILE    "${CMAKE_SOURCE_DIR}/README.md")

set(CPACK_SOURCE_IGNORE_FILES         "/build/;/.git;${CPACK_SOURCE_IGNORE_FILES}")
set(CPACK_PACKAGE_INSTALL_DIRECTORY   ${CMAKE_INSTALL_PREFIX})
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY  OFF)

include(CPack)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_INCLUDE_DIRECTORIES_BEFORE ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)

if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
   set(CMAKE_EXE_LINKER_FLAGS    "${CMAKE_EXE_LINKER_FLAGS}    -Wl,-undefined,error")
   set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-undefined,error")
   set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -Wl,-undefined,error")

elseif(CMAKE_SYSTEM_NAME MATCHES "(OpenBSD|FreeBSD|NetBSD|DragonFly)")
   set(CMAKE_EXE_LINKER_FLAGS    "${CMAKE_EXE_LINKER_FLAGS}    -Wl,--no-undefined")
   set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ")
   set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -Wl,--no-undefined")

elseif(MSVC)
   string (REGEX REPLACE "/W3" "" CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}"  )
   string (REGEX REPLACE "/W3" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

   add_compile_options("/utf-8")

else()
   # Linux, Windows (MinGW)

   set(CMAKE_EXE_LINKER_FLAGS    "${CMAKE_EXE_LINKER_FLAGS}    -Wl,--no-undefined")
   set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
   set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -Wl,--no-undefined")
endif()

# start output messages
message("")
message("-- Searching for required packages\n")

find_package(CsLibGuarded QUIET)

if (CsLibGuarded_FOUND)
   message(STATUS "CsLibGuarded was found \n"
      " * Path is ${CsLibGuarded_INCLUDES}\n")
else()
   message(STATUS "CsLibGuarded was not found, bundled library will be used\n")
endif()

# destination for cmake export files
if (CMAKE_SYSTEM_NAME MATCHES "Windows")
   set(PKG_PREFIX "cmake/CsSignal")

else()
   set(PKG_PREFIX "${CMAKE_INSTALL_LIBDIR}/cmake/CsSignal")

endif()

# catch2 set up
if (BUILD_TESTS)
   find_package(Catch2)
   if(NOT Catch2_FOUND)
      message(FATAL_ERROR "Catch2 was not found")
   endif()

   enable_testing()
   add_subdirectory(test)
endif()

configure_file(
   ${CMAKE_SOURCE_DIR}/cmake/CsSignalConfig.cmake
   ${CMAKE_BINARY_DIR}/CsSignalConfig.cmake
   @ONLY
)

configure_file(
   ${CMAKE_SOURCE_DIR}/cmake/CsSignalConfigVersion.cmake
   ${CMAKE_BINARY_DIR}/CsSignalConfigVersion.cmake
   @ONLY
)

install(
   FILES
      ${CMAKE_BINARY_DIR}/CsSignalConfig.cmake
      ${CMAKE_BINARY_DIR}/CsSignalConfigVersion.cmake
   DESTINATION ${PKG_PREFIX}
)

# file locations for building
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include(src/signal/signal.cmake)

if (${CMAKE_SIZEOF_VOID_P} EQUAL 4)
   set(TARGETBITS 32)
else()
   set(TARGETBITS 64)
endif()

message("")
message("CsSignal configured to run on:  ${CMAKE_SYSTEM_NAME} ${TARGETBITS} bit, ${CMAKE_BUILD_TYPE} Mode")
message("CsSignal will be built in:      ${CMAKE_BINARY_DIR}")
message("CsSignal will be installed in:  ${CMAKE_INSTALL_PREFIX}")
message("\n")

