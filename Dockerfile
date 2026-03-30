# tbuggy Visual Localization — OpenVINS Monocular VIO
# Base: osrf/ros:humble-desktop (Ubuntu 22.04 + ROS2 Humble + OpenCV + cv_bridge)
#
# Build:
#   docker build -t uditsinghparihar/tbuggy_vio:latest .
#
# Push to Docker Hub:
#   docker push uditsinghparihar/tbuggy_vio:latest
#
# Run (see utils/docker_run_log01.sh for convenience scripts):
#   docker run --rm -it --network=host \
#     -v /path/to/bags:/data \
#     -v $(pwd)/output:/colcon_ws/utils/results \
#     uditsinghparihar/tbuggy_vio:latest bash

FROM osrf/ros:humble-desktop

# Avoid interactive prompts during apt installs
ENV DEBIAN_FRONTEND=noninteractive

# ── Layer 1: System dependencies ────────────────────────────────────────────
# OpenCV, Eigen3, cv_bridge, image_transport already in base image.
# Ceres solver and its deps must be added explicitly.
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    libeigen3-dev \
    libceres-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# ── Layer 2: Python packages ─────────────────────────────────────────────────
# rosbags: read ROS2 .db3 bags without a running ROS2 node (used by all utils/)
# evo:     trajectory evaluation (evo_ape, evo_rpe)
# numpy+scipy installed together so scipy wheels match the numpy ABI;
# avoids the system scipy/pip-numpy binary incompatibility.
RUN pip3 install --no-cache-dir \
    rosbags \
    matplotlib \
    "numpy<1.25.0" \
    scipy \
    evo

# ── Layer 3: Clone repo + submodule ──────────────────────────────────────────
# Pulls parent repo + open_vins submodule (your fork with v3 tuning configs).
# The exact submodule commit hash is pinned in .gitmodules → reproducible.
RUN git clone --recurse-submodules \
    https://github.com/UditSinghParihar/tbuggy_localization.git \
    /colcon_ws

# ── Layer 4: Build OpenVINS with colcon ──────────────────────────────────────
# Sequential build to avoid OOM on memory-constrained machines.
# The built install/ directory is baked into the image — reviewer never compiles.
WORKDIR /colcon_ws
RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    MAKEFLAGS='-j4' colcon build \
      --symlink-install \
      --executor sequential \
      --parallel-workers 1"

# ── Layer 5: Shell environment ────────────────────────────────────────────────
# Source both ROS2 and workspace in every interactive shell session.
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /colcon_ws/install/setup.bash" >> /root/.bashrc

# ── Layer 6: Entrypoint ───────────────────────────────────────────────────────
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Create results directory so volume mounts work cleanly
RUN mkdir -p /colcon_ws/utils/results

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
