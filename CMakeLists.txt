### GLOBAL INFORMATION ON PROJECT ####
### A CmakeLists.txt on the project root directory is mandatory for some IDE ###
### It is a wrapper for cmake/ directory ####

MESSAGE(STATUS "\n_____ CMakeLists.txt ____________________________________________\n" )

# Project name, used for all subsequent variables
project(OPENROX C CXX)

# Cmake minimal version required to use this script
cmake_minimum_required(VERSION 3.5.1)

# Cmake policy adapted to minimal version
cmake_policy(VERSION 3.5.1)

# Inner configuration files
include(cmake/openrox.cmake)
