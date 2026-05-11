# follower_limo

Código desenvolvido para o Trabalho de Conclusão de Curso na plataforma AgileX LIMO. O sistema tem dois modos de operação: um modo de seguimento de alvo por cor/LiDAR, e um modo de tarefas onde comandos em linguagem natural são traduzidos em ações do robô via LLM.

---

## Estrutura

```
src/
  color_detector.py     detecção de alvo por segmentação HSV
  lidar_reader.py       consulta de distância no LiDAR para o ângulo do alvo
  sensor_fusion.py      combina câmera e LiDAR em uma estimativa de pose do alvo
  follower_controller.py  controle proporcional que gera comandos /cmd_vel
  yolo_detector.py      detecção de objetos por categoria usando YOLOv8
  robot_primitives.py   biblioteca de ações primitivas (andar, girar, buscar, etc.)
  task_executor.py      executa sequências de ações recebidas em JSON
  llm_commander.py      traduz linguagem natural em ações usando uma LLM local

config/
  color_detector.yaml   faixas HSV e área mínima de contorno
  lidar_reader.yaml     limites de distância e ângulo válidos
  sensor_fusion.yaml    suavização exponencial e distância alvo
  follower_controller.yaml  ganhos do controlador e parâmetros de recovery
  yolo_detector.yaml    classe alvo e threshold de confiança do YOLO
  task_executor.yaml    tópico cmd_vel
  llm_commander.yaml    modelo LLM e configuração de device

launch/
  follower.launch       modo seguidor (pipeline completo)
  task_mode.launch      modo de tarefas com YOLO e LLM
  test_camera.launch    só câmera e color_detector
  test_lidar.launch     só lidar_reader
  test_fusion.launch    câmera + LiDAR + fusão, sem controlador

tools/
  hsv_calibrator.py     ferramenta gráfica para calibrar faixas HSV
```

---

## Instalação

Clone dentro do workspace catkin:

```bash
cd ~/catkin_ws/src
git clone <url-do-repositorio> follower_limo
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Dependências Python para o modo de tarefas:

```bash
pip install ultralytics
pip install transformers torch accelerate
```

---

## Modo 1: Seguidor de alvo (por cor)

O robô detecta um objeto de cor específica, estima a distância com o LiDAR e segue o alvo mantendo uma distância configurável.

### Pipeline

```
color_detector -> lidar_reader -> sensor_fusion -> follower_controller -> /cmd_vel
```

### Como rodar

```bash
roslaunch follower_limo follower.launch
```

### Calibrar a cor do alvo

O `color_detector.py` usa segmentação HSV para encontrar o alvo. Para ajustar os valores, use o calibrador:

```bash
# Com uma imagem de referência:
python tools/hsv_calibrator.py --image tools/captura0047.jpg

# Com a câmera ao vivo:
python tools/hsv_calibrator.py --camera 0
```

A janela mostra sliders para H_min, H_max, S_min, S_max, V_min, V_max e uma prévia da máscara em tempo real. Quando a máscara isolar bem o objeto, pressione `p` para imprimir os valores no terminal e `q` para sair. Copie os valores para `config/color_detector.yaml`.

### Parâmetros principais

`config/color_detector.yaml` — faixas HSV e área mínima de contorno para considerar detecção válida.

`config/sensor_fusion.yaml` — `target_distance` define em quantos metros o robô tenta ficar do alvo.

`config/follower_controller.yaml` — `kp_angular` e `kp_linear` controlam a agressividade do controlador. Se o robô oscilar muito, reduza os ganhos.

---

## Modo 2: Tarefas com linguagem natural (YOLO + LLM)

O usuário descreve uma tarefa em linguagem comum e o robô a executa. A LLM traduz o texto em uma sequência de ações primitivas, o YOLO identifica os objetos e o executor roda as ações em ordem.

### Arquitetura

```
/task/command (texto)
       |
  llm_commander  <-->  modelo HuggingFace (local)
       |
/task/actions (JSON)
       |
  task_executor
    |         |
    |     yolo_detector  <--  /camera/rgb/image_raw
    |         |
    |    /yolo/detection + /yolo/class
    |
  robot_primitives  -->  /cmd_vel
```

O `lidar_reader` também é iniciado com remapeamento: recebe `/yolo/detection` em vez de `/target/detection`, então a distância medida pelo LiDAR continua disponível para as ações de aproximação.

### Como rodar

```bash
roslaunch follower_limo task_mode.launch
```

Na primeira execução, o modelo LLM é baixado automaticamente (~1 GB para o Qwen2.5-0.5B). Aguarde o terminal imprimir `modelo pronto` antes de enviar comandos.

### Enviando comandos

```bash
rostopic pub /task/command std_msgs/String '{"data": "encontre a bola e me avise"}'
```

Acompanhe o status da execução:

```bash
rostopic echo /task/status
```

### Exemplos de comandos

```
"gire 90 graus e ande 1 metro"
"encontre a bola e se aproxime dela"
"vasculhe a área e me avise o que encontrou"
"procure uma pessoa e se aproxime a 1 metro"
```

### Ações primitivas disponíveis

| Ação | Parâmetros | Descrição |
|---|---|---|
| `go_forward` | `distance_m`, `speed` | Avança a distância especificada (estimativa por tempo) |
| `rotate` | `degrees`, `speed` | Gira no lugar; positivo = esquerda, negativo = direita |
| `scan_area` | `speed` | Rotação de 360 para varrer a área |
| `search_object` | `class_name`, `timeout` | Gira devagar até encontrar o objeto via YOLO |
| `approach_object` | `class_name`, `target_dist`, `timeout` | P-controller para aproximar do objeto detectado |
| `stop` | — | Para o robô imediatamente |
| `alert` | `message` | Publica uma mensagem em `/task/alert` e loga no terminal |

Os nomes de classe seguem o dataset COCO, que o YOLOv8 usa por padrão. Exemplos: `"sports ball"`, `"person"`, `"cup"`, `"chair"`.

### Testando sem a LLM

Para testar as ações diretamente (útil para verificar cada primitiva antes de integrar a LLM):

```bash
rosrun follower_limo task_executor.py

# Em outro terminal:
rostopic pub /task/actions std_msgs/String \
  '{"data": "[{\"action\":\"rotate\",\"degrees\":90}]"}'
```

### Escolha do modelo LLM

Edite `config/llm_commander.yaml`:

```yaml
# Jetson Xavier NX (8 GB de RAM):
model_name: "microsoft/Phi-3-mini-4k-instruct"

# Jetson Nano (4 GB) ou para ser mais leve:
model_name: "Qwen/Qwen2.5-0.5B-Instruct"
```

Se o Jetson não tiver RAM suficiente para rodar LLM e YOLO simultaneamente, o `llm_commander.py` pode ser rodado em um laptop na mesma rede ROS. Basta mudar `ROS_MASTER_URI` no laptop para apontar para o IP do Limo; os outros nós continuam rodando no robô.

---

## Visualização com Foxglove

Todos os launch files já iniciam o `foxglove_bridge` automaticamente na porta 8765. Para visualizar:

1. Abra o Foxglove Studio (app ou navegador em `https://studio.foxglove.dev`)
2. Clique em "Open connection" e selecione "Rosbridge / Foxglove WebSocket"
3. Coloque o endereço: `ws://IP_DO_LIMO:8765`
4. Para descobrir o IP do Limo: `hostname -I` no terminal do robô

### Modo seguidor

Adicione um painel de imagem e selecione o tópico `/target/image_debug`. Ele mostra o frame da câmera com o bounding box desenhado ao redor do alvo detectado, o ângulo calculado e a área do contorno. Quando não há alvo, escreve "SEM ALVO" na imagem.

### Modo de tarefas (YOLO + LLM)

O yolo_detector publica em `/yolo/image_debug` com bounding boxes de todos os objetos detectados, mostrando classe, confiança e ângulo. Adicione um painel de imagem apontando para esse tópico.

Para acompanhar o que o executor está fazendo, adicione um painel "Raw Messages" nos tópicos:

- `/task/status` — estado atual: `IDLE`, `RUNNING: search_object`, `ERROR: ...`
- `/task/alert` — mensagens de alerta publicadas pela ação `alert`
- `/task/actions` — JSON com a sequência de ações gerada pela LLM (aparece uma vez por comando)

Se quiser ver os dois modos ao mesmo tempo em telas diferentes, o Foxglove suporta layouts com múltiplos painéis — basta arrastar e redimensionar.

---

## Tópicos ROS

| Tópico | Tipo | Publicado por |
|---|---|---|
| `/target/detection` | Float32MultiArray | color_detector |
| `/lidar/distance_at_angle` | Float32MultiArray | lidar_reader |
| `/target/pose` | Float32MultiArray | sensor_fusion |
| `/cmd_vel` | Twist | follower_controller ou task_executor |
| `/yolo/detection` | Float32MultiArray | yolo_detector |
| `/yolo/class` | String | yolo_detector |
| `/task/command` | String | usuário |
| `/task/actions` | String (JSON) | llm_commander |
| `/task/status` | String | task_executor |
| `/task/alert` | String | task_executor |

O formato de `/yolo/detection` é idêntico ao de `/target/detection`: `[found, cx_norm, cy_norm, area_px2, angle_deg]`.
