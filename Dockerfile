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

## ceres

RUN git clone https://ceres-solver.googlesource.com/ceres-solver

WORKDIR /ceres-build/

RUN cmake ../ceres-solver

RUN make -j4

RUN make install

RUN ldconfig

WORKDIR /

## opencv

RUN git clone https://github.com/opencv/opencv.git

RUN git clone https://github.com/opencv/opencv_contrib.git

WORKDIR /opencv/build 

RUN cmake -D CMAKE_BUILD_TYPE=Release -D OPENCV_EXTRA_MODULES_PATH=/opencv_contrib/modules CMAKE_INSTALL_PREFIX=/usr/local /opencv

RUN make -j8

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

RUN git clone -b updated-release-dec-2023 https://github.com/amy-tabb/calico.git

WORKDIR /calico/build/

RUN cmake ../src

RUN make -j4

RUN make install

RUN ldconfig
