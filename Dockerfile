FROM amytabb/ubuntu18-ceres-opencv4-contrib-all:latest

MAINTAINER Amy Tabb

#cass
WORKDIR /installed_libs/

RUN git clone https://github.com/amy-tabb/calico.git

WORKDIR /installed_libs/calico/build/

RUN cmake ../src

RUN make

RUN make install

## exiftools

RUN apt-get -y install exiftool

WORKDIR /host_dir/





