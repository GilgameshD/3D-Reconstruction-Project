cmake_minimum_required(VERSION 2.8.3)
project(scan)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL 1.8 REQUIRED)
find_package(OpenGL REQUIRED)
FIND_PACKAGE( OpenCV REQUIRED )

# Find glfw header
find_path(GLFW_INCLUDE_DIR NAMES GLFW/glfw3.h
        	PATHS /usr/X11R6/include
              	      /usr/include/X11
                      /opt/graphics/OpenGL/include
                      /opt/graphics/OpenGL/contrib/libglfw
                      /usr/local/include
                      /usr/include/GL
                      /usr/include)
# Find glfw library
find_library(GLFW_LIBRARIES NAMES glfw glfw3
            	PATHS /usr/lib64
                      /usr/lib
                      /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
                      /usr/local/lib64
                      /usr/local/lib
                      /usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}
                      /usr/X11R6/lib)

# Find realsense library
find_library(REALSENSE_LIBRARIES NAMES realsense
            	PATHS /usr/lib64
                      /usr/lib
                      /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
                      /usr/local/lib64
                      /usr/local/lib
                      /usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}
                      /usr/X11R6/lib)

include_directories(${GLFW_INCLUDE_DIR})
include_directories(./third_party)

# for PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# for G2O
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/opt/g2o )
FIND_PACKAGE( G2O REQUIRED )
FIND_PACKAGE( CSparse REQUIRED )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

SET(G2O_LIBS ${G2O_CORE_LIBRARY}   ${G2O_STUFF_LIBRARY} ${G2O_CLI_LIBRARY}
       ${G2O_TYPES_DATA}     ${G2O_TYPES_ICP}     ${G2O_SOLVER_CHOLMOD}
       ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION}
       ${G2O_SOLVER_DENSE}   ${G2O_SOLVER_PCG}    ${G2O_SOLVER_SLAM2D_LINEAR}
       ${G2O_SOLVER_STRUCTURE_ONLY}  ${G2O_SOLVER_EIGEN}
       ${G2O_TYPES_DATA}     ${G2O_TYPES_ICP}     ${G2O_TYPES_SBA}
       ${G2O_TYPES_SCLAM2D}  ${G2O_TYPES_SIM3}    ${G2O_TYPES_SLAM2D}
       ${G2O_TYPES_SLAM3D})

## add files
add_executable(readRGBandDEPTH readRGBandDEPTH.cpp)
target_link_libraries(readRGBandDEPTH ${REALSENSE_LIBRARIES} ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} ${LIBUSB1_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} )

add_executable(scan_from_file scan_from_file.cpp)
target_link_libraries(scan_from_file ${REALSENSE_LIBRARIES} ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} ${LIBUSB1_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${G2O_LIBS} ${CSPARSE_LIBRARY})

