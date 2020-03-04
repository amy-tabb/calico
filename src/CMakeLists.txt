cmake_minimum_required(VERSION 3.0)

project(calico)

# set the flags, may want to add/change debugging or optimization (-O3) flags as appropriate
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -Wall -pedantic -Wextra -std=c++11 ")
# no OpenMP version below. 
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra -std=c++11 ")

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)

# OpenCV required, can specify version.  OpenCV 4.0 not strictly necessary, see the README for more details.
find_package( OpenCV REQUIRED ) 


add_executable(calico
calico.cpp
calico.hpp
Includes.hpp
DirectoryFunctions.cpp
DirectoryFunctions.hpp
camera_calibration.cpp
camera_calibration.hpp
multicamera.cpp
multicamera.hpp
camera_visualization.cpp
camera_visualization.hpp
solving_structure.cpp
solving_structure.hpp
helper.cpp
helper.hpp
)

target_link_libraries(calico ${CERES_LIBRARIES} Eigen3::Eigen gomp ${OpenCV_LIBS})

# no OpenMP version below
# target_link_libraries(calico ${CERES_LIBRARIES} Eigen3::Eigen ${OpenCV_LIBS})

# install for Docker build 
set_property(TARGET calico PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS calico  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )

