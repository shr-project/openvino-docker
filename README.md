# openvino-docker

Intel® OpenVINO™ Toolkit environment

This Dockerfile will provide you with a base environment to run your inference models with OpenVINO™.  

## Building the Docker Image

### Download Intel® OpenVINO™ Toolkit

The firt thing you need is to download the OpenVINO(tm) toolkit.

You can register and download it from the following link (Linux version):
[https://software.intel.com/en-us/openvino-toolkit/choose-download/free-download-linux](https://software.intel.com/en-us/openvino-toolkit/choose-download/free-download-linux)

Or use wget to get the package directly (Latest version is 2019 R2.0.1 by the time writing this guide)

``` bash
wget http://registrationcenter-download.intel.com/akdlm/irc_nas/15792/l_openvino_toolkit_p_2019.2.275.tgz
```

### Extract the file in the root folder  

``` bash
tar -xf l_openvino_toolkit*
```

### Build the image

``` bash
docker build -t openvino-18.04 .
```

## Using the image

### Run a container

You can directly run a container based on this image or use this image across other images.

To run a container based on this image:

``` bash
docker run -ti openvino-18.04-2019.2 /bin/bash
```

### Use the image in another container

You can use this Docker image as a base image and use it in multiple Dockerfiles. An example of how to do this has been provided:

Move to sample-app directory and build the image

``` bash
cd sample-app
docker build -t openvino-app-18.04 .
```

### Run the the container with X enabled (Linux)

Additionally, for running a sample application that displays an image, you need to share the host display to be accessed from guest Docker container.

The following flags needs to be added to the docker run command:

* --net=host
* --env="DISPLAY"
* --volume="$HOME/.Xauthority:/root/.Xauthority:rw"
* --device /dev/video0

To run the docker-app image with the display enabled:

``` bash
docker run --net=host --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" --device /dev/video0 -ti openvino-app-18.04 /bin/bash
```

### Run two demos

Once inside the container, go to the Inference Engine demo directory:

``` bash
cd /opt/intel/openvino/deployment_tools/demo
```

Run the Image Classification demo:

``` bash
./demo_squeezenet_download_convert_run.sh
```

Run the inference pipeline demo:

``` bash
./demo_security_barrier_camera.sh
```

Run interactive face detection demo:

``` bash
sh /opt/intel/openvino/deployment_tools/open_model_zoo/demos/omz_demos_build/interactive_face_detection_demo-launch.sh
```

Run ros_openvino
``` bash
cd ~/catkin_ws/
source devel/setup.bash
sed 's/pipeline_people_myriad.yaml/pipeline_people_cpu_no_rviz.yaml/g; /^  <node name="rviz"/,+2d; /^  <arg name="default_rviz"/d; /^        <remap from =".openvino_toolkit.images/d' /opt/openvino_toolkit/ros_openvino_toolkit/vino_launch/launch/pipeline_people_myriad.launch > /opt/openvino_toolkit/ros_openvino_toolkit/vino_launch/launch/pipeline_people_cpu_no_rviz.launch
sed 's/pipeline_people_myriad.yaml/pipeline_people_cpu_no_rviz.yaml/g; /^  <node name="rviz"/,+2d; /^  <arg name="default_rviz"/d; /^        <remap from =".openvino_toolkit.images/d' ./src/ros_openvino_toolkit/vino_launch/launch/pipeline_people_myriad.launch > ./src/ros_openvino_toolkit/vino_launch/launch/pipeline_people_cpu_no_rviz.launch
sed 's/MYRIAD/CPU/g; s/,RViz//g; s/ImageWindow, //g' /opt/openvino_toolkit/ros_openvino_toolkit/vino_launch/param/pipeline_people_myriad.yaml > /opt/openvino_toolkit/ros_openvino_toolkit/vino_launch/param/pipeline_people_cpu_no_rviz.yaml
sed 's/MYRIAD/CPU/g; s/,RViz//g; s/ImageWindow, //g' ./src/ros_openvino_toolkit/vino_launch/param/pipeline_people_myriad.yaml > ./src/ros_openvino_toolkit/vino_launch/param/pipeline_people_cpu_no_rviz.yaml
cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/intel/openvino_2019.2.275/deployment_tools/intel/emotions-recognition-retail-0003/FP16/
cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/intel/computer_vision_sdk/deployment_tools/intel_models/emotions-recognition-retail-0003/FP16/

roslaunch vino_launch pipeline_people_cpu_no_rviz.launch
```
