FROM nvidia/cuda:10.2-devel-ubuntu18.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update \
  && apt-get upgrade -y \
  # install dependencies
  && apt-get install --no-install-recommends -y \
  software-properties-common \
  wget nano build-essential \
  git curl libeigen3-dev ca-certificates \
  libtbb-dev libboost-all-dev libgtest-dev libmetis-dev \
  libomp-dev libglm-dev libglfw3-dev libpng-dev libopencv-dev \
  # setup the latest cmake
  && apt-get purge -y cmake \
  && wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null \
  && echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ bionic main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null \
  && apt-get update \
  && apt-get install --no-install-recommends -y cmake \
  # install ROS
  && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
  && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
  && apt-get update \
  && apt-get install --no-install-recommends -y \
  ros-melodic-ros-base ros-melodic-cv-bridge \
  ros-melodic-image-transport-plugins ros-melodic-tf2-ros \
  # clean
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# install CUB
WORKDIR /root
RUN wget https://github.com/NVIDIA/cub/archive/refs/tags/1.8.0.tar.gz
RUN tar xzvf 1.8.0.tar.gz
RUN mv cub-1.8.0/cub /usr/local/cuda/include/

# install GTSAM
WORKDIR /root
RUN git clone https://github.com/borglab/gtsam.git
WORKDIR /root/gtsam/build
RUN cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_WITH_TBB=OFF \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
RUN make -j$(nproc) && make install

# install viewer
WORKDIR /root
RUN git clone https://github.com/koide3/iridescence.git --recursive
WORKDIR /root/iridescence/build
RUN cmake ..
RUN make -j$(nproc) && make install

RUN echo "#!/bin/bash" >> /ros_entrypoint.sh \
  && echo "set -e" >> /ros_entrypoint.sh \
  && echo "source /opt/ros/melodic/setup.bash" >> /ros_entrypoint.sh \
  && echo "source /root/catkin_ws/devel/setup.bash" >> /ros_entrypoint.sh \
  && echo 'exec "$@"' >> /ros_entrypoint.sh \
  && chmod a+x /ros_entrypoint.sh

WORKDIR /root/catkin_ws/src
COPY glim /root/catkin_ws/src/glim
COPY glim_ros1 /root/catkin_ws/src/glim_ros1

WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=ON -DBUILD_WITH_CUDA_MULTIARCH=OFF -DBUILD_WITH_VIEWER=ON -DBUILD_WITH_MARCH_NATIVE=OFF'

WORKDIR /root/catkin_ws/src/glim

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
