FROM ros:melodic-ros-core-bionic

# init catkin workspace
RUN mkdir -p /catkin_ws/src/vector_ros_driver

# install required dependencies
RUN apt-get update && apt-get install -y \
    python3-yaml \
    python3-pip  \
    expect \
    python-catkin-tools \
    python3-dev \
    python3-catkin-pkg-modules \
    python3-numpy \
    libopencv-dev \
    ros-melodic-tf \
    ros-melodic-xacro \
    ros-melodic-robot-state-publisher \
    ros-melodic-joint-state-publisher

RUN pip3 install \
    rospkg \
    catkin_pkg

# install up-to-date rosunit so we'll get the patch for Python3
RUN cd /catkin_ws/src/ && \
    git clone https://github.com/ros/ros && \
    cd .. && \
    /ros_entrypoint.sh catkin_make install --pkg rosunit

# setup Anki's SDK
RUN python3 -m pip install --user anki_vector

# build cv_bridge for Python3.6
RUN /ros_entrypoint.sh /bin/bash -c "mkdir /cv_bridge_build_ws && \
                                     cd /cv_bridge_build_ws && \
                                     catkin init && \
                                     catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3.6 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so && \
                                     catkin config --install && \
                                     git clone https://github.com/ros-perception/vision_opencv.git src/vision_opencv && \
                                     cd src/vision_opencv/ && \
                                     git checkout melodic && \
                                     cd ../../ && \
                                     catkin build cv_bridge"

# copy pakcage source code
COPY . /catkin_ws/src/vector_ros_driver

# clone vector_ros repo
RUN git clone https://github.com/betab0t/vector_ros /catkin_ws/src/vector_ros

# clone diff_drive package
RUN git clone https://github.com/merose/diff_drive /catkin_ws/src/diff_drive

# build all packages in catkin_ws
RUN cd /catkin_ws && \
    /ros_entrypoint.sh catkin_make

WORKDIR /catkin_ws

COPY vector_entrypoint.sh /vector_entrypoint.sh
ENTRYPOINT ["/vector_entrypoint.sh"]

CMD ["roslaunch", "vector_ros_driver", "driver.launch"]