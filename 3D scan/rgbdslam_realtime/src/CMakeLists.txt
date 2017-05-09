# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04

# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )

# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

# 添加g2o的依赖
# 因为g2o不是常用库，要添加它的findg2o.cmake文件
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

# Find realsense library
find_library(REALSENSE_LIBRARIES NAMES realsense
                PATHS /usr/lib64
                      /usr/lib
                      /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
                      /usr/local/lib64
                      /usr/local/lib
                      /usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}
                      /usr/X11R6/lib)


ADD_LIBRARY( slambase slamBase.cpp )
TARGET_LINK_LIBRARIES( slambase
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} 
    ${LIBUSB1_LIBRARIES}
    ${REALSENSE_LIBRARIES} )

ADD_EXECUTABLE( slam slam.cpp )
TARGET_LINK_LIBRARIES( slam
    slambase
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${G2O_LIBS} 
    ${CSPARSE_LIBRARY})


