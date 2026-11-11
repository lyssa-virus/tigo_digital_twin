#!/usr/bin/env bash
# Câmera: TFs da câmera + nuvem de pontos (/camera/points) + scan feito com a câmera (/camera/scan).
# Pré-requisitos na simulação: run_simulation.sh e run_bridge.sh já rodando.
# Posição da câmera: TEM que ser igual ao tigo.sdf (x y z em relação ao base_link).
source /opt/ros/humble/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# base_link -> camera_link (x à frente, z para cima)
ros2 run tf2_ros static_transform_publisher --x 0.165 --y 0 --z 0.049 \
  --frame-id base_link --child-frame-id camera_link --ros-args -p use_sim_time:=true &
TF1=$!

# camera_link -> camera_optical_link (convenção óptica: z à frente, x à direita, y para baixo)
ros2 run tf2_ros static_transform_publisher --roll -1.5707963 --pitch 0 --yaw -1.5707963 \
  --frame-id camera_link --child-frame-id camera_optical_link --ros-args -p use_sim_time:=true &
TF2=$!

trap 'kill $TF1 $TF2 2>/dev/null' EXIT

python3 "$SCRIPT_DIR/depth_to_points.py" --ros-args -p use_sim_time:=true
