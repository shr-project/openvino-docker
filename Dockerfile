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
        python3-yaml \
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

ADD download-model-pipeline-people.sh /openvino/download-model-pipeline-people.sh
RUN cd /opt/intel/openvino/deployment_tools/ && \
        git clone https://github.com/opencv/open_model_zoo.git open_model_zoo_git && \
        sh /openvino/download-model-pipeline-people.sh && \
        export MODELS=/opt/intel/openvino/models && \
        mkdir -p ${MODELS}/Transportation/object_detection/face/pruned_mobilenet_reduced_ssd_shared_weights/dldt && \
        mkdir -p ${MODELS}/Retail/object_attributes/age_gender/dldt && \
        mkdir -p ${MODELS}/Transportation/object_attributes/headpose/vanilla_cnn/dldt && \
        mkdir -p ${MODELS}/Retail/object_attributes/emotions_recognition/0003/dldt && \
        mkdir -p ${MODELS}/Transportation/object_attributes/facial_landmarks/custom-35-facial-landmarks/dldt && \
        ln -snf ${MODELS}/intel/face-detection-adas-0001/FP16 ${MODELS}/Transportation/object_detection/face/pruned_mobilenet_reduced_ssd_shared_weights/dldt/ && \
        ln -snf ${MODELS}/intel/age-gender-recognition-retail-0013/FP16 ${MODELS}/Retail/object_attributes/age_gender/dldt/ && \
        ln -snf ${MODELS}/intel/head-pose-estimation-adas-0001/FP16 ${MODELS}/Transportation/object_attributes/headpose/vanilla_cnn/dldt/ && \
        ln -snf ${MODELS}/intel/emotions-recognition-retail-0003/FP16 ${MODELS}/Retail/object_attributes/emotions_recognition/0003/dldt/ && \
        ln -snf ${MODELS}/intel/facial-landmarks-35-adas-0002/FP16 ${MODELS}/Transportation/object_attributes/facial_landmarks/custom-35-facial-landmarks/dldt/ && \
        echo "Pipeline people model fetched"

RUN apt install -y libgtk-3-dev && \
        cd /opt/intel/openvino/deployment_tools/open_model_zoo/demos/ && \
        HOME=/opt/intel/openvino/deployment_tools/open_model_zoo/demos ./build_demos.sh && \
        echo "export F=cam MODELS=/opt/intel/openvino/models; /opt/intel/openvino/deployment_tools/open_model_zoo/demos/omz_demos_build/intel64/Release/interactive_face_detection_demo -i $F -m $MODELS/Transportation/object_detection/face/pruned_mobilenet_reduced_ssd_shared_weights/dldt/FP16/face-detection-adas-0001.xml -m_ag $MODELS/Retail/object_attributes/age_gender/dldt/FP16/age-gender-recognition-retail-0013.xml -m_hp $MODELS/Transportation/object_attributes/headpose/vanilla_cnn/dldt/FP16/head-pose-estimation-adas-0001.xml -m_em $MODELS/Retail/object_attributes/emotions_recognition/0003/dldt/FP16/emotions-recognition-retail-0003.xml -m_lm $MODELS/Transportation/object_attributes/facial_landmarks/custom-35-facial-landmarks/dldt/FP16/facial-landmarks-35-adas-0002.xml -d cpu" > /opt/intel/openvino/deployment_tools/open_model_zoo/demos/omz_demos_build/interactive_face_detection_demo-launch.sh && \
        echo "OMZ demos built in /opt/intel/openvino/deployment_tools/open_model_zoo/demos/omz_demos_build"

# clean up 
RUN apt autoremove -y && \
    rm -rf /openvino /var/lib/apt/lists/*

RUN /bin/bash -c "source $INSTALL_DIR/bin/setupvars.sh"

RUN echo "source $INSTALL_DIR/bin/setupvars.sh" >> /root/.bashrc

CMD ["/bin/bash"]
