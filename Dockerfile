FROM ubuntu:18.04

ADD l_openvino_toolkit* /openvino/

ARG INSTALL_DIR=/opt/intel/openvino 

RUN apt-get update && apt-get -y upgrade && apt-get autoremove -y

#Install needed dependences
RUN apt-get install -y --no-install-recommends \
        build-essential \
        cpio \
        curl \
        git \
        lsb-release \
        pciutils \
        python3.6 \
        python3.6-dev \
        python3-pip \
        python3-setuptools \
        sudo

# installing OpenVINO dependencies
RUN cd /openvino/ && \
    ./install_openvino_dependencies.sh

RUN pip3 install numpy

# installing OpenVINO itself
RUN cd /openvino/ && \
    sed -i 's/decline/accept/g' silent.cfg && \
    ./install.sh --silent silent.cfg

# Model Optimizer
RUN cd $INSTALL_DIR/deployment_tools/model_optimizer/install_prerequisites && \
    ./install_prerequisites.sh

RUN apt-get install -y gnupg2 && \
        echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
        apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
        apt update && \
        DEBIAN_FRONTEND=noninteractive apt install -y ros-melodic-desktop-full && \
        rosdep init && \
        rosdep update && \
        echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
        apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential && \
        echo "ROS1 melodic installed"

# clean up 
RUN apt autoremove -y && \
    rm -rf /openvino /var/lib/apt/lists/*

RUN /bin/bash -c "source $INSTALL_DIR/bin/setupvars.sh"

RUN echo "source $INSTALL_DIR/bin/setupvars.sh" >> /root/.bashrc

CMD ["/bin/bash"]
