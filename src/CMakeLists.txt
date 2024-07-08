cmake_minimum_required(VERSION 3.16)

#set(CMAKE_C_COMPILER "gcc-8")
#set(CMAKE_CXX_COMPILER "g++-8")

project(calico-jul24)

include(CMakePrintHelpers)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Ceres REQUIRED)

find_package(apriltags REQUIRED) 

find_package(PkgConfig)
pkg_search_module(apriltag REQUIRED apriltag)

# OpenCV required, can specify version.  OpenCV 4.0 not strictly necessary, see the README for more details.
# OpenCV 4.4 and greater not compatable. 
find_package( OpenCV 4.3.0 REQUIRED COMPONENTS core imgproc imgcodecs aruco calib3d )
# alternate method, with find any installed OpenCV and use the latest version. Note calico will not
# build with OpenCV > 4.3.0, but you can find which versions you have installed this way. 
#find_package( OpenCV REQUIRED COMPONENTS core imgproc imgcodecs aruco calib3d ) 

link_directories(${apriltag_LIBDIR})

set(my_USUAL_FLAGS  -Wextra -Wall -g3 -O3 -fopenmp CACHE STRING "; non-profiling compile flags")
set(my_USUAL_LINKER_FLAGS CACHE STRING "; non-profiling linker flags")

set(CALICO_EXE calico-jul24)

set(COMPERROR_EXE compute-jul24)

set(calico_SOURCES
calico.cpp
calico.hpp
Includes.hpp
DirectoryFunctions.cpp
DirectoryFunctions.hpp
camera-calibration.cpp
camera-calibration.hpp
pattern-parameters.cpp
pattern-parameters.hpp
patterns.cpp
patterns.hpp
local-charuco.hpp
local-charuco.cpp
multicamera.cpp
multicamera.hpp
camera-visualization.cpp
camera-visualization.hpp
solving-structure.cpp
solving-structure.hpp
helper.cpp
helper.hpp
helper-cali.cpp
helper-cali.hpp
apriltag-file.cpp
)

set(compute_error_SOURCES
compute-error.cpp
helper.cpp
helper.hpp
camera-visualization.cpp
camera-visualization.hpp
)

###################### compute error

set(OBJECT_DIR ${CMAKE_BINARY_DIR}/CMakeFiles/${COMPERROR_EXE}.dir)
message("-- Object files will be output to: ${OBJECT_DIR}")

add_executable(${COMPERROR_EXE} ${compute_error_SOURCES})

target_include_directories(${COMPERROR_EXE} PRIVATE ${Eigen_INCLUDE_DIRS}  ${apriltag_INCLUDE_DIRS})

target_compile_features(${COMPERROR_EXE} PRIVATE cxx_std_17)

target_compile_options(${COMPERROR_EXE} PRIVATE ${my_USUAL_FLAGS})

target_link_libraries(${COMPERROR_EXE} ${CERES_LIBRARIES} Eigen3::Eigen gomp ${OpenCV_LIBS} ${apriltags_LIBRARIES} ${apriltag_LIBRARIES})

################# calico #############################

# Create OBJECT_DIR variable
set(OBJECT_DIR ${CMAKE_BINARY_DIR}/CMakeFiles/${CALICO_EXE}.dir)
message("-- Object files will be output to: ${OBJECT_DIR}")

add_executable(${CALICO_EXE} ${calico_SOURCES})

target_include_directories(${CALICO_EXE} PRIVATE ${Eigen_INCLUDE_DIRS}  ${apriltag_INCLUDE_DIRS})

target_compile_features(${CALICO_EXE} PRIVATE cxx_std_17)

target_compile_options(${CALICO_EXE} PRIVATE ${my_USUAL_FLAGS})

target_link_libraries(${CALICO_EXE} ${CERES_LIBRARIES} Eigen3::Eigen gomp ${OpenCV_LIBS} ${apriltags_LIBRARIES} ${apriltag_LIBRARIES})

################ installs #############################

install(TARGETS ${CALICO_EXE} DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

install(TARGETS ${COMPERROR_EXE} DESTINATION ${CMAKE_INSTALL_PREFIX}/bin) 



