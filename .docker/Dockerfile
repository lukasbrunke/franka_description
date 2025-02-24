FROM osrf/ros:humble-desktop
ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user
WORKDIR /workspaces

# Install all system dependencies first (as root)
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-visualization-msgs \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages separately
RUN pip3 install pin

# Set up user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir -p -m 0700 /run/user/"${USER_UID}" \
    && mkdir -p -m 0700 /run/user/"${USER_UID}"/gdm \
    && chown user:user /run/user/"${USER_UID}" \
    && chown user:user /workspaces \
    && chown user:user /run/user/"${USER_UID}"/gdm

# Create the visualization package
RUN mkdir -p /workspaces/src/franka_vis_tools/franka_vis_tools \
    && mkdir -p /workspaces/src/franka_vis_tools/scripts \
    && mkdir -p /workspaces/src/franka_vis_tools/resource

# Copy the package files
COPY ./franka_vis_tools/setup.py /workspaces/src/franka_vis_tools/
COPY ./franka_vis_tools/package.xml /workspaces/src/franka_vis_tools/
COPY ./franka_vis_tools/franka_vis_tools/sphere_publisher.py /workspaces/src/franka_vis_tools/franka_vis_tools/
COPY ./franka_vis_tools/franka_vis_tools/custom_marker_publisher.py /workspaces/src/franka_vis_tools/franka_vis_tools/
COPY ./franka_vis_tools/resource/franka_vis_tools /workspaces/src/franka_vis_tools/resource/

# Create the scripts and make them executable (while still root)
RUN touch /workspaces/src/franka_vis_tools/franka_vis_tools/__init__.py
# RUN echo '#!/usr/bin/env python3' > /workspaces/src/franka_vis_tools/scripts/sphere_publisher
# RUN echo 'from franka_vis_tools.sphere_publisher import main' >> /workspaces/src/franka_vis_tools/scripts/sphere_publisher
# RUN echo 'if __name__ == "__main__":' >> /workspaces/src/franka_vis_tools/scripts/sphere_publisher
# RUN echo '    main()' >> /workspaces/src/franka_vis_tools/scripts/sphere_publisher
COPY ./franka_vis_tools/scripts/ /workspaces/src/franka_vis_tools/scripts
# COPY ./franka_vis_tools/scripts/sphere_publisher.py /workspaces/src/franka_vis_tools/scripts
# COPY ./franka_vis_tools/scripts/custom_marker_publisher.py /workspaces/src/franka_vis_tools/scripts
# RUN ls /workspaces/src/franka_vis_tools/scripts
RUN chmod +x /workspaces/src/franka_vis_tools/scripts/sphere_publisher.py
RUN chmod +x /workspaces/src/franka_vis_tools/scripts/custom_marker_publisher.py
RUN chmod +x /workspaces/src/franka_vis_tools/franka_vis_tools/sphere_publisher.py
RUN chmod +x /workspaces/src/franka_vis_tools/franka_vis_tools/custom_marker_publisher.py

# Set ownership of all files to the user
RUN chown -R $USERNAME:$USERNAME /workspaces

# Now switch to non-root user
USER $USERNAME
ENV XDG_RUNTIME_DIR=/run/user/"${USER_UID}"

RUN echo "source /ros_entrypoint.sh" >>~/.bashrc
ARG MAX_ROS_DOMAIN_ID=232
RUN echo "export ROS_DOMAIN_ID=100" >>~/.bashrc
RUN echo "set +e" >>~/.bashrc
