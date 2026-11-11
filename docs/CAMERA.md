# Testando a câmera na simulação

Sensores virtuais (valores do datasheet):

| | Profundidade | RGB |
|---|---|---|
| Tópico | `/ascamera_hp60c/camera_publisher/depth0/image_raw` | `/ascamera_hp60c/camera_publisher/rgb0/image_raw` (nome suposto) |
| Resolução | 640x360 (real: 1920x1080 por lente) | 640x360 |
| Taxa | 30 Hz | 15 Hz |
| Campo de visão | 80° horizontal | 80° horizontal |
| Frame | `camera_optical_link` | `camera_optical_link` |

Posição no robô: lente a 16,5 cm à frente da origem do modelo 3D e 9,2 cm do piso (tirada da oval da câmera no modelo).

## Rodar

1. `./scripts/run_simulation.sh obstaculos` (mundo com mesa, bloco baixo e poste)
2. `./scripts/run_bridge.sh`
3. `./scripts/run_camera.sh` (TFs da câmera, `/camera/points` e `/camera/scan`)
4. `rviz2 -d config/tigo_camera.rviz`

## O que isso permite validar

- Se a câmera enxerga o que o lidar não enxerga (tampo da mesa, objetos baixos).
- Se o `/camera/scan` (câmera como lidar) concorda com o `/scan` do lidar nas paredes.
- O alcance e o campo de visão úteis da câmera na posição em que ela foi instalada.

## Limites da simulação

- A profundidade virtual é perfeita (sem ruído, sem buracos). A estéreo real tem ruído crescente com a distância e falha em superfícies sem textura.
- O alcance de profundidade (0,10 a 10 m) é suposição; ver o datasheet.
- `depth_to_points.py` aceita profundidade 32FC1 (metros) e 16UC1 (milímetros), então roda igual com a câmera real.
