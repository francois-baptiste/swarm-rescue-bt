# Built FROM the official Nav2 CI image (ghcr.io/ros-navigation/navigation2:humble)
# instead of apt-installing ros-humble-navigation2, because this sandbox's
# network blocks packages.ros.org (confirmed: DNS/proxy returns 403/refused)
# but github.com/ghcr.io are reachable. This image ships BehaviorTree.CPP v3,
# Gazebo and every other Nav2 system dependency as pre-installed .deb
# packages - but /opt/overlay_ws/src/navigation2 in the image turned out to
# contain only package.xml stubs (no CMakeLists.txt/source), so the real
# navigation2 source is fetched fresh from GitHub (humble branch) instead.
FROM ghcr.io/ros-navigation/navigation2:humble

SHELL ["/bin/bash", "-c"]

WORKDIR /opt/overlay_ws
RUN rm -rf src/navigation2 && \
    git clone --depth 1 --branch humble https://github.com/ros-navigation/navigation2.git src/navigation2

# Split into two RUN layers on purpose: swarm_sar_bt's package.xml depends
# on nav2_bringup, which transitively pulls in most of navigation2 (every
# planner/controller/costmap plugin it can launch) - that build takes
# ~20 minutes. Building it here, BEFORE swarm_sar_bt's source is copied in,
# means Docker caches this layer and iterating on swarm_sar_bt itself only
# re-runs the fast second RUN below.
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --packages-up-to nav2_bringup \
      --event-handlers console_direct+ \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY src/swarm_sar_bt src/swarm_sar_bt

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --packages-select swarm_sar_bt \
      --event-handlers console_direct+ \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

CMD ["/bin/bash"]
