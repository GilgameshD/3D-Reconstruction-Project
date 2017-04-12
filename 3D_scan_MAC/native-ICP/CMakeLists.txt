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

add_executable(scan scan.cpp)
target_link_libraries(scan ${REALSENSE_LIBRARIES} ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} ${LIBUSB1_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(readFromRealsense readFromRealsense.cpp)
target_link_libraries(readFromRealsense ${REALSENSE_LIBRARIES} ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} ${LIBUSB1_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} )

add_executable(scan_from_file scan_from_file.cpp)
target_link_libraries(scan_from_file ${REALSENSE_LIBRARIES} ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} ${LIBUSB1_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} )

add_executable(readRGBandDEPTH readRGBandDEPTH.cpp)
target_link_libraries(readRGBandDEPTH ${REALSENSE_LIBRARIES} ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} ${LIBUSB1_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} )





