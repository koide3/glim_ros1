FROM koide3/gtsam_docker:focal_cuda11.2

RUN apt-get update \
  && apt-get upgrade -y \
  # install dependencies
  && apt-get install --no-install-recommends -y \
    curl libopencv-dev \
  # install ROS
  && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
  && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
  && apt-get update \
  && apt-get install --no-install-recommends -y \
    ros-noetic-ros-base ros-noetic-cv-bridge \
    ros-noetic-image-transport-plugins ros-noetic-tf2-ros \
  # clean
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

RUN echo "#!/bin/bash" >> /ros_entrypoint.sh \
  && echo "set -e" >> /ros_entrypoint.sh \
  && echo "source /opt/ros/noetic/setup.bash" >> /ros_entrypoint.sh \
  && echo "source /root/catkin_ws/devel/setup.bash" >> /ros_entrypoint.sh \
  && echo 'exec "$@"' >> /ros_entrypoint.sh \
  && chmod a+x /ros_entrypoint.sh

WORKDIR /root/catkin_ws/src
COPY glim /root/catkin_ws/src/glim
COPY glim_ros1 /root/catkin_ws/src/glim_ros1

WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=ON -DBUILD_WITH_CUDA_MULTIARCH=ON -DBUILD_WITH_VIEWER=ON'

WORKDIR /root/catkin_ws/src/glim

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
