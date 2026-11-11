#!/usr/bin/env bash
# Inicia a simulação do TIGO no Ignition Gazebo já despausada (-r)
# Uso:
#   ./scripts/run_simulation.sh               # laboratório vazio (worlds/lacea.sdf)
#   ./scripts/run_simulation.sh obstaculos    # laboratório + obstáculos de teste da câmera

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

case "${1:-lacea}" in
  lacea)       WORLD="worlds/lacea.sdf" ;;
  obstaculos)  WORLD="worlds/lacea_obstaculos.sdf" ;;
  *) echo "Mundo desconhecido: $1 (use: lacea ou obstaculos)"; exit 1 ;;
esac

cd "$PROJECT_ROOT"
echo "Iniciando Ignition Gazebo com $WORLD..."
ign gazebo -r "$WORLD"
