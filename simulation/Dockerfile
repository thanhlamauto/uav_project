# Build after deploy/Dockerfile.gpu; all extra dependencies stay in Docker.
ARG RUNTIME_IMAGE=uav-navigation:humble-gpu
ARG SIM_BUILD_IMAGE=uav-navigation:simulation-build
FROM ${SIM_BUILD_IMAGE} AS build
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY simulation/bridge/include /sim_ws/bridge/include
COPY simulation/ros /sim_ws/ros
WORKDIR /sim_ws
RUN source /opt/ros/humble/setup.bash && \
    colcon build --base-paths ros --merge-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
FROM ${RUNTIME_IMAGE}
COPY --from=build /sim_ws/install /simulation_install
# libjpeg is installed through the base image OpenCV dependency.
COPY simulation/config/runtime.yaml /simulation/runtime.yaml
ENV LD_LIBRARY_PATH=/jetson-driver:/opt/tensorrt/lib:/opt/cuda/lib:/opt/opencv/lib
ENV ROS_LOCALHOST_ONLY=1
CMD ["bash", "-c", "source /simulation_install/setup.bash && exec ros2 launch uav_simulation forest.launch.py"]
