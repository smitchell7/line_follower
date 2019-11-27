FROM osrf/ros:kinetic-desktop-full

RUN apt-get update

RUN apt-get install -y ros-kinetic-kobuki-node
RUN apt-get install -y ros-kinetic-usb-cam
RUN apt-get install -y ros-kinetic-kobuki-gazebo

COPY wait-for-it.sh /

RUN mkdir -p /catkin_ws && ln -s /data /catkin_ws/src
RUN echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc

WORKDIR /catkin_ws


#ENTRYPOINT bash -i -c "source /opt/ros/kinetic/setup.bash && catkin_make"
#CMD /bin/bash
