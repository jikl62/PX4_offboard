set -eo pipefail
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH
source /opt/ros/humble/setup.bash
source /home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs/share/px4_msgs/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
colcon --log-base /home/jikl62/.cache/fc01-release-20261112-ros-706_0cxh/log build --base-paths /home/jikl62/.cache/fc01-release-20261112-8r2k_twx/actual-package/actual-release-home-ry9c6qhj/FC01_OrangePi_Navigation_20261112_release/native/src --packages-select jikl_fc01_system --build-base /home/jikl62/.cache/fc01-release-20261112-ros-706_0cxh/build --install-base /home/jikl62/.cache/fc01-release-20261112-ros-706_0cxh/install --event-handlers console_direct+ --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF
