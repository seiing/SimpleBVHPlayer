cmake_minimum_required(VERSION 2.8.6)
project(render)

add_compile_options(-std=gnu++11)
add_compile_options(-Wdeprecated-declarations)
SET(CMAKE_BUILD_TYPE Release CACHE STRING
	"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
#	FORCE
	)

link_directories(../core/)
include_directories(../core/)

find_package(PythonLibs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

add_compile_options(-DHAVE_CSTDDEF)
include_directories(${PYTHON_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})

file(GLOB srcs "*.h" "*.cpp")
add_executable(render ${srcs})
target_link_libraries(render core ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})