FROM osrf/ros:noetic-desktop-focal
ENV DEBIAN_FRONTEND=noninteractive
ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1

SHELL ["/bin/bash", "-c"]

MAINTAINER Luca Di Giammarino <digiammarino@diag.uniroma1.it>
MAINTAINER Leonardo Brizi <brizi@diag.uniroma1.it>

ARG STANDARD_PKG="git vim"

ARG LIBRARIES="libeigen3-dev \
 libsuitesparse-dev \
 libqglviewer-dev-qt5 \
 freeglut3-dev \
 libpcl-dev" 

# installing all the packages
RUN apt-get update && \
    apt-get dist-upgrade -y && \
    apt-get install -y ${STANDARD_PKG} ${LIBRARIES}

RUN apt-get install -y python3-pip python3-catkin-tools ros-noetic-grid-map-msgs
  
#create workspace
RUN mkdir -p /catkin_ws/src

# clone useful pkgs
RUN cd /catkin_ws/src && git clone https://github.com/digiamm/md_slam.git
# RUN cd /catkin_ws/src && git clone https://gitlab.com/srrg-software/md_slam.git
RUN cd /catkin_ws/src && git clone https://gitlab.com/srrg-software/srrg_cmake_modules.git 
RUN cd /catkin_ws/src && git clone https://gitlab.com/srrg-software/srrg_hbst.git 
RUN cd /catkin_ws/src && git clone https://gitlab.com/srrg-software/srrg2_core.git && cd srrg2_core && git checkout c747aa854a2d1f7fdad6516474c4a4d3a543ea47 
RUN cd /catkin_ws/src && git clone https://gitlab.com/srrg-software/srrg2_solver.git && cd srrg2_solver && git checkout eb34f226733532ab67d5e45e7de21b284599af89 
# remove useless stuff from solver
RUN cd /catkin_ws/src/srrg2_solver && rm -rf srrg2_solver_gui srrg2_solver_star srrg2_solver_experiments

RUN source /opt/ros/noetic/setup.bash && cd /catkin_ws && catkin init && catkin build 

COPY ./dl.conf /dl.conf

WORKDIR /
#COPY ./ros_entrypoint.sh ros_entrypoint.sh
#ENTRYPOINT ["/ros_entrypoint.sh"]
#RUN bash -c "source '/catkin_ws/devel/setup.bash' && echo 'source /catkin_ws/devel/setup.bash' >> /.bashrc"
RUN bash -c "source /opt/ros/noetic/setup.bash \
    && echo 'source /catkin_ws/devel/setup.bash' >> ~/.bashrc"
#CMD ["bash"]

