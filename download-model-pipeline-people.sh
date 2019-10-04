#!/bin/sh

DOWNLOADER="python3 /opt/intel/openvino/deployment_tools/open_model_zoo_git/tools/downloader/downloader.py --precisions FP16 --name"

${DOWNLOADER} face-detection-adas-0001
${DOWNLOADER} age-gender-recognition-retail-0013
${DOWNLOADER} emotions-recognition-retail-0003
${DOWNLOADER} head-pose-estimation-adas-0001
${DOWNLOADER} person-detection-retail-0013
${DOWNLOADER} person-reidentification-retail-0076
${DOWNLOADER} facial-landmarks-35-adas-0002
