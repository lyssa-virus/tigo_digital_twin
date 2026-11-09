#!/usr/bin/env bash
# SÓ NA SIMULAÇÃO: publica /odom com erro realista (substitui os encoders do robô real).
# Uso: ./scripts/run_sim_odom.sh            (erro padrão)
#      NOISE=0 ./scripts/run_sim_odom.sh    (odometria perfeita)
#      NOISE=2 SEED=7 ./scripts/run_sim_odom.sh
source /opt/ros/humble/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "$SCRIPT_DIR/sim_odometry.py" --ros-args -p noise_scale:="${NOISE:-1.0}" -p seed:="${SEED:-1}"
