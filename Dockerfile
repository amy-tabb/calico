FROM ubuntu:22.04
MAINTAINER Amy Tabb

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
	build-essential \
	cmake	\
	git \
	libgtk2.0-dev \
	pkg-config \
	libavcodec-dev \
	libavformat-dev 

RUN apt-get install -y libswscale-dev\
	libtbb2 \
	libtbb-dev \
	libjpeg-dev \
	libpng-dev \
	libtiff-dev 

RUN apt-get install -y libeigen3-dev \
	liblapack-dev \
	libatlas-base-dev \
	libgomp1 

RUN apt-get install -y libgoogle-glog-dev libgflags-dev

RUN apt-get install -y libsuitesparse-dev

RUN apt-get install -y wget unzip

RUN apt-get install -y g++


## opencv

RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.3.0.zip
RUN wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.3.0.zip

RUN unzip opencv.zip

RUN unzip opencv_contrib.zip

WORKDIR /opencv-build 

RUN cmake -D CMAKE_BUILD_TYPE=Release -D OPENCV_EXTRA_MODULES_PATH=/opencv_contrib-4.3.0/modules CMAKE_INSTALL_PREFIX=/usr/local  -DBUILD_LIST=core,imgproc,imgcodecs,aruco,calib3d,highgui,videoio /opencv-4.3.0

RUN make -j8

RUN make install

RUN ldconfig

## ceres

WORKDIR /

RUN git clone https://ceres-solver.googlesource.com/ceres-solver

WORKDIR /ceres-build/

RUN cmake /ceres-solver

RUN make -j4

RUN make install

RUN ldconfig

WORKDIR /

RUN git clone https://github.com/amy-tabb/apriltags-lib.git

WORKDIR /apriltags-lib/build 

RUN apt-get install -y libv4l-dev

RUN cmake /apriltags-lib

RUN make -j8

RUN make install

RUN ldconfig

WORKDIR /

RUN git clone https://github.com/AprilRobotics/apriltag.git

WORKDIR /apriltag/build/

RUN cmake -B build -DCMAKE_BUILD_TYPE=Release /apriltag

RUN cmake --build build --target install /apriltag

RUN ldconfig

WORKDIR /

RUN git clone https://github.com/amy-tabb/calico.git

WORKDIR /calico/build/

RUN cmake /calico/src

RUN make -j4

RUN ldconfig

WORKDIR /docker_dir/

WORKDIR /calico/build/

