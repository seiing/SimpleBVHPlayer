cmake_minimum_required(VERSION 2.8.6)
project(bvh)
set(OpenGL_GL_PREFERENCE LEGACY)
set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: Debug Release
RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

add_compile_options(-fPIC)
add_compile_options(-ggdb)
add_compile_options(-Wdeprecated-declarations)

add_compile_options(-DHAVE_CSTDDEF)
add_definitions(-DBVH_DIR="${CMAKE_HOME_DIRECTORY}")
set(CMAKE_MODULE_PATH "${CMAKE_HOME_DIRECTORY}/cmake")

find_package(PythonInterp 3.6 EXACT REQUIRED)
find_package(PythonLibs 3.6 EXACT REQUIRED)

add_subdirectory( core )
add_subdirectory( render )