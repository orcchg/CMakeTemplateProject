cmake_minimum_required( VERSION 2.8 )
PROJECT( ProjectTemplate )

# -----------------------------------------------------------------------------
# --------------------------- Common header -----------------------------------
# -----------------------------------------------------------------------------
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_BINARY_DIR} )  # for config.h
INCLUDE_DIRECTORIES( include )
SET( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules" )

if( CMAKE_COMPILER_IS_MINGW )
elseif( (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX) AND NOT CMAKE_COMPILER_IS_MINGW )
    ADD_DEFINITIONS( -fPIC )
    ADD_DEFINITIONS( -fvisibility=hidden -Wall )
    ADD_DEFINITIONS( -std=c++11 )
elseif( MSVC )
    ADD_DEFINITIONS( /MP )  # multi-core compilation with MSVC
endif()
if( CMAKE_COMPILER_IS_GNUCXX )
    SET( LIBSTDC++_LIBRARIES -lstdc++ )
endif()
if( "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" )
    ADD_DEFINITIONS( -Wno-missing-braces )
    ADD_DEFINITIONS( -Wno-enum-compare )
    ADD_DEFINITIONS( -Wno-deprecated )
    ADD_DEFINITIONS( -Wno-deprecated-declarations )
    ADD_DEFINITIONS( -Wno-deprecated-register )
endif()

# If this is an in-source build (CMAKE_SOURCE_DIR == CMAKE_BINARY_DIR),
# write the library/executable files to the respective directories in the
# source tree. During an out-of-source build, however, do not litter this
# directory, since that is probably what the user wanted to avoid.
if( CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR )
    SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_HOME_DIRECTORY}/lib" )
    SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_HOME_DIRECTORY}/lib" )
    SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_HOME_DIRECTORY}/bin" )
endif( CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR )

# Cache these to allow the user to override them manually.
SET( TVHISTOGRAM_LIB_INSTALL_DIR "lib" CACHE PATH "Path the built library files are installed to." )
SET( TVHISTOGRAM_INCLUDE_INSTALL_DIR "include" CACHE PATH "Path the header files are installed to." )
SET( TVHISTOGRAM_BIN_INSTALL_DIR "bin" CACHE PATH "Path the tool executables are installed to." )

# Allow the user to build a shared or static library
OPTION ( BUILD_SHARED_LIBS "Build a shared version of the library" ON )

# -----------------------------------------------------------------------------
# Search for external dependencies
#FIND_PACKAGE( Eigen3 REQUIRED )
#FIND_PACKAGE( OpenCL REQUIRED )
#FIND_PACKAGE( OpenCV REQUIRED )
#if( ${OpenCV_VERSION} VERSION_LESS 2.4.8 )
#    MESSAGE( FATAL_ERROR "OpenCV version is not compatible : ${OpenCV_VERSION}" )
#endif()

# -----------------------------------------------------------------------------
ADD_SUBDIRECTORY( app )
ADD_SUBDIRECTORY( src )

# -----------------------------------------------------------------------------
# Documentation building
FIND_PROGRAM(DOXYGEN doxygen)
if ( DOXYGEN )
  set(DoxyfilePattern "${PROJECT_SOURCE_DIR}/docs/Doxyfile.in.pattern")
  set(Doxyfile "${CMAKE_BINARY_DIR}/Doxyfile.in")
  message(STATUS "Building documentation based on (${Doxyfile}) ...")

  FILE(READ ${DoxyfilePattern} DOXYFILE_CONTENT)
  set(SEARCH_REGEX "DOXYFILE_PROJECT_ROOT")
  STRING(REGEX REPLACE "${SEARCH_REGEX}" "${PROJECT_SOURCE_DIR}" MODIFIED_FILE_CONTENT "${DOXYFILE_CONTENT}")
  FILE(WRITE ${Doxyfile} "${MODIFIED_FILE_CONTENT}")

  execute_process(COMMAND ${DOXYGEN} ${Doxyfile})
endif()

