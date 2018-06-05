cmake_minimum_required( VERSION 2.8)
project(recons_slam)

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )


############################################################
# OpenCV
find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
#target_link_libraries(recons_slam ${OpenCV_LIBS})

# Sophus 
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

#ceres
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
#target_link_libraries(recons_slam ${CERES_LIBRARIES})

# G2O
#find_package( G2O REQUIRED )
#message(STATUS "   G2O  include path:   ${G2O_INCLUDE_DIR}")
#include_directories( ${G2O_INCLUDE_DIR} )list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(G2O REQUIRED)
message(STATUS "   G2O  include path:   ${G2O_INCLUDE_DIR}")
include_directories(${G20_INCLUDE_DIRS})
#target_link_libraries(recons_slam g2o_core g2o_stuff)

#PCL
find_package(PCL REQUIRED COMPONENT common io)
message(STATUS "    PCL include path:  ${PCL_INCLUDE_DIRS}")
include_directories(${PCL_INCLUDE_DIRS})
#add_definitions(${PCL_DEFINITIONS})
#target_link_libraries(recons_slam ${PCL_LIBRARIES})

set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    ${PCL_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba
)
##############################################################

include_directories( ${PROJECT_SOURCE_DIR}/Include )
include_directories( ${PROJECT_SOURCE_DIR}/Include/slam_class )

add_subdirectory(src)

###############################################################
add_executable(recons_slam_main main.cpp)
target_link_libraries(recons_slam_main recons_slam)


