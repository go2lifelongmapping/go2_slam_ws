#!/usr/bin/env bash

set -e

# 1. ROS core — luôn có
source /opt/ros/"${ROS_DISTRO}"/setup.bash

# 2. Workspace của mình — chỉ source nếu đã build.
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

# 3. CycloneDDS — dùng để ghim DDS vào đúng card mạng nối với Go2/Ouster.
if [ -f /ws/docker/cyclonedds.xml ]; then
    export CYCLONEDDS_URI="file:///ws/docker/cyclonedds.xml"
fi

# 4. 'docker compose exec slam bash' KHÔNG chạy qua entrypoint này.
grep -q "go2_slam bootstrap" ~/.bashrc 2>/dev/null || cat >> ~/.bashrc <<'RC'

# --- go2_slam bootstrap ---
source /opt/ros/foxy/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
[ -f /ws/docker/cyclonedds.xml ] && export CYCLONEDDS_URI="file:///ws/docker/cyclonedds.xml"
alias cb='cd /ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
PS1="\[\e[1;32m\][go2_slam:foxy]\[\e[0m\] \w\$ "
# --- end go2_slam bootstrap ---
RC

# 5. exec thay thế tiến trình shell bằng lệnh được truyền vào.
exec "$@"
