#
# Project settings
#

# Minimum required version of cmake
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

# Project name and programming languages used
PROJECT( acado_example_ws CXX )

# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

#
# Prerequisites
#
FIND_PACKAGE( ACADO REQUIRED )

#
# Include directories
#
INCLUDE_DIRECTORIES( include ${ACADO_INCLUDE_DIRS} )

#
# Libraries
#
ADD_LIBRARY(acado_math_functions src/acado_math/acado_math_functions.cpp)

#
# Build executables
#
ADD_EXECUTABLE(        generate_primitives src/generate_primitives.cpp )
TARGET_LINK_LIBRARIES( generate_primitives acado_math_functions ${ACADO_SHARED_LIBRARIES})
SET_TARGET_PROPERTIES( generate_primitives
PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )
