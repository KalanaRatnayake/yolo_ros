#---------------------------------------------------------------------------------------------------------------------------
#----
#----   Start base image
#----
#---------------------------------------------------------------------------------------------------------------------------

FROM ros:humble-ros-base-jammy as base

## Parameters
ENV WORKSPACE_ROOT=/yolo

#############################################################################################################################
#####
#####   Install Dependencies
#####
#############################################################################################################################

WORKDIR /

RUN apt-get update -y
RUN apt-get install -y --no-install-recommends  ros-dev-tools \
                                                ros-$ROS_DISTRO-rmw-cyclonedds-cpp

RUN pip3 install torch==1.12.0+cu102 torchvision==0.13.0+cu102 torchaudio==0.12.0 --extra-index-url https://download.pytorch.org/whl/cu102

RUN apt-get clean

#############################################################################################################################
#####
#####   Install Kobuki packages
#####
#############################################################################################################################

WORKDIR ${WORKSPACE_ROOT}/src

RUN git clone https://github.com/AIResearchLab/yolo_ros.git
RUN git clone https://github.com/AIResearchLab/yolo_ros_msgs.git

RUN cd yolo_ros && \
    pip3 install -r requirements.txt && \
    cd ..

RUN rosdep init && rosdep update && rosdep install --from-paths ${WORKSPACE_ROOT}/src -y --ignore-src


#############################################################################################################################
#####
#####   Build Kobuki packages
#####
#############################################################################################################################

WORKDIR ${WORKSPACE_ROOT}

RUN . /opt/ros/humble/setup.sh && colcon build
WORKDIR /


#############################################################################################################################
#####
#####   Remove workspace source and build files that are not relevent to running the system
#####
#############################################################################################################################

RUN rm -rf ${WORKSPACE_ROOT}/src
RUN rm -rf ${WORKSPACE_ROOT}/log
RUN rm -rf ${WORKSPACE_ROOT}/build
RUN rm -rf /var/lib/apt/lists/*
RUN rm -rf /tmp/*

RUN apt-get clean


#---------------------------------------------------------------------------------------------------------------------------
#----
#----   Start final release image
#----
#---------------------------------------------------------------------------------------------------------------------------


FROM ros:humble-ros-base-jammy as final

## Parameters
ENV WORKSPACE_ROOT=/yolo
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV YOLO_MODEL=yolov9t.pt
ENV INPUT_TOPIC=/camera/color/image_raw
ENV PUBLISH_ANNOTATED_IMAGE=False
ENV OUTPUT_ANNOTATED_TOPIC=/yolo_ros/annotated_image
ENV OUTPUT_DETAILED_TOPIC=/yolo_ros/detection_result
ENV CONFIDENCE_THRESHOLD=0.25
ENV DEVICE='0' 

WORKDIR /

COPY --from=base / /

COPY docker/workspace_entrypoint.sh /workspace_entrypoint.sh

RUN chmod +x /workspace_entrypoint.sh

ENTRYPOINT [ "/workspace_entrypoint.sh" ]

WORKDIR ${WORKSPACE_ROOT}
