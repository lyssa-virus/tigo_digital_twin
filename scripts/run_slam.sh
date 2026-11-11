#!/usr/bin/env bash
# SLAM: TF estático base_link->laser_link + odom->base_link (de /odom) + slam_toolbox.
# Pré-requisitos na simulação: run_simulation.sh, run_bridge.sh e run_sim_odom.sh já rodando.
# No robô real: /odom vem do tigo_base (e use_sim_time deve ser false).
# Instalar: sudo apt install -y ros-humble-slam-toolbox
source /opt/ros/humble/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"

# Posição do lidar em relação ao base_link (TEM que ser igual ao tigo.sdf: x y z)
ros2 run tf2_ros static_transform_publisher --x 0.1355 --y 0 --z 0.123 \
  --frame-id base_link --child-frame-id laser_link --ros-args -p use_sim_time:=true &
TF_STATIC=$!

# odom -> base_link a partir de /odom
python3 "$SCRIPT_DIR/odom_to_tf.py" --ros-args -p use_sim_time:=true &
TF_ODOM=$!

trap 'kill $TF_STATIC $TF_ODOM 2>/dev/null' EXIT

ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file "$ROOT/config/slam_toolbox.yaml"
