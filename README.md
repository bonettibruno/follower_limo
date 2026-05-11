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
  robot_primitives.py   biblioteca de ações primitivas com freio de segurança e campos potenciais
  task_executor.py      executa sequências de ações em JSON; assina /odom, /scan, /yolo/*
  task_logger.py        grava dados dos sensores em CSV durante cada tarefa
  llm_commander.py      traduz linguagem natural em ações usando uma LLM local (HuggingFace)

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
  plot_task.py          gera gráficos PNG a partir de um CSV do task_logger

logs/
  task_*.csv            gerados automaticamente pelo task_logger (não commitados)
  task_*.png            gerados pelo plot_task.py (não commitados)
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

O `color_detector.py` usa segmentação HSV para encontrar o alvo. O fluxo de calibração é: tirar uma foto no Limo, transferir para o PC e rodar o calibrador localmente (ele abre uma janela gráfica que não funciona bem via SSH sem configuração extra).

**1. Tirar a foto no Limo**

Conecte via SSH e capture um frame da câmera com o objeto alvo visível em condições reais de iluminação:

```bash
ssh agilex@IP_DO_LIMO

# Captura um frame e salva em /tmp
python -c "
import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

rospy.init_node('capture', anonymous=True)
bridge = CvBridge()
msg = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=5)
frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
cv2.imwrite('/tmp/captura.jpg', frame)
print('Salvo em /tmp/captura.jpg')
" --wait
```

Ou, se a câmera já estiver rodando como dispositivo V4L2:

```bash
python -c "import cv2; cap=cv2.VideoCapture(0); ret,f=cap.read(); cv2.imwrite('/tmp/captura.jpg',f); cap.release(); print('ok')"
```

**2. Transferir para o PC**

No terminal do seu PC (não no SSH):

```bash
scp agilex@IP_DO_LIMO:/tmp/captura.jpg ./tools/
```

**3. Rodar o calibrador no PC**

```bash
python tools/hsv_calibrator.py --image tools/captura.jpg
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
| `go_forward` | `distance_m`, `speed` | Avança a distância especificada (estimativa por tempo); para se o freio de segurança disparar |
| `rotate` | `degrees`, `speed` | Gira no lugar; positivo = esquerda, negativo = direita |
| `scan_area` | `speed` | Rotação de 360 para varrer a área |
| `search_object` | `class_name`, `timeout` | Gira devagar até encontrar o objeto via YOLO |
| `approach_object` | `class_name`, `target_dist`, `timeout` | Aproximação com campos potenciais (atração ao alvo + repulsão de obstáculos); para se o freio de segurança disparar |
| `stop` | — | Para o robô imediatamente |
| `alert` | `message` | Publica uma mensagem em `/task/alert` e loga no terminal |

Os nomes de classe seguem o dataset COCO, que o YOLOv8 usa por padrão. Exemplos: `"sports ball"`, `"person"`, `"cup"`, `"chair"`.

### Freio de segurança e campos potenciais

O `task_executor` assina `/scan` (LiDAR bruto) e calcula continuamente a distância mínima no setor frontal (±30°). Dois mecanismos usam esse valor:

**Freio de segurança** — ativo em `go_forward` e `approach_object`. Se a distância mínima frontal cair abaixo de 20 cm, o robô para imediatamente e a ação é interrompida. O limite pode ser ajustado pela constante `SAFETY_DIST` em `robot_primitives.py`.

**Campos potenciais em `approach_object`** — a velocidade linear resulta da combinação de:
- Força atrativa: `kp_linear * (distancia_atual - distancia_alvo)` — puxa o robô em direção ao alvo
- Força repulsiva: `KR * (1/d - 1/d0)` quando `d < d0` — empurra o robô para longe de obstáculos próximos

Quando um obstáculo aparece entre o robô e o alvo, a repulsão reduz ou reverte a velocidade de avanço, evitando colisão. Os parâmetros `KR` (ganho repulsivo) e `REPULSIVE_INFLUENCE_DIST` (distância de influência, padrão 0.5 m) ficam no topo de `robot_primitives.py`.

### Odometria

O `task_executor` assina `/odom` automaticamente. Quando o Limo publica odometria, `go_forward` e `rotate` passam a usar distância e ângulo reais medidos pelos encoders das rodas, em vez de estimativa por tempo. Se `/odom` não estiver disponível (tópico silencioso), as funções caem automaticamente para o modo por tempo — nenhuma configuração manual é necessária.

### Logging e geração de gráficos

O `task_logger` grava automaticamente os dados de cada tarefa em um arquivo CSV enquanto o executor está rodando.

**Iniciar o logger junto com o task mode:**

```bash
roslaunch follower_limo task_mode.launch
# Em outro terminal:
rosrun follower_limo task_logger.py
```

Os CSVs são salvos em `logs/task_YYYYMMDD_HHMMSS.csv`. Um novo arquivo é criado a cada tarefa. Os arquivos de log não são commitados no git.

**Gerar os gráficos:**

```bash
python tools/plot_task.py logs/task_20240101_120000.csv
```

Gera um PNG no mesmo diretório com 6 painéis:

- Distância ao alvo ao longo do tempo (com linha da distância configurada)
- Distância mínima frontal do LiDAR (com linhas do freio de segurança e da zona de influência repulsiva)
- Forças atrativa e repulsiva dos campos potenciais e velocidade resultante
- Comandos `/cmd_vel` (linear e angular) enviados ao robô
- Ângulo do alvo detectado pelo YOLO (mostra convergência para o centro)
- Trajetória do robô no plano XY via odometria

Linhas verticais tracejadas nos gráficos marcam quando cada ação começou.

**Para transferir os logs para o PC e gerar os gráficos lá:**

```bash
# No PC:
scp agilex@IP_DO_LIMO:~/catkin_ws/src/follower_limo/logs/*.csv ./logs/
pip install matplotlib
python tools/plot_task.py logs/task_20240101_120000.csv
```

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

## Acesso via SSH

O fluxo normal de trabalho é: PC conectado na mesma rede Wi-Fi do Limo, acesso via SSH para rodar os nós, e Foxglove para visualizar.

### Conectar

```bash
ssh agilex@IP_DO_LIMO
```

Para descobrir o IP do Limo quando ele já estiver na rede:

```bash
# No terminal do próprio Limo (se tiver monitor conectado):
hostname -I

# Ou no seu PC, escaneie a rede:
nmap -sn 192.168.1.0/24   # ajuste a faixa para a sua rede
```

### Abrir múltiplos terminais no mesmo Limo

Cada comando ROS (roslaunch, rostopic, etc.) precisa de um terminal separado. Em vez de abrir várias janelas SSH, use `tmux` diretamente no Limo:

```bash
ssh agilex@IP_DO_LIMO
tmux new -s ros          # cria sessão chamada "ros"

# Dentro do tmux:
# Ctrl+B, %   → divide a tela verticalmente
# Ctrl+B, "   → divide horizontalmente
# Ctrl+B, seta → navega entre painéis
# Ctrl+B, D   → desanexa (sessão continua rodando)
# tmux attach -t ros → reanexar depois
```

### Transferir arquivos entre PC e Limo

```bash
# Do Limo para o PC:
scp agilex@IP_DO_LIMO:/caminho/no/limo ./destino/local/

# Do PC para o Limo:
scp ./arquivo/local agilex@IP_DO_LIMO:/caminho/no/limo/

# Pasta inteira:
scp -r ./pasta agilex@IP_DO_LIMO:/destino/
```

### ROS via SSH

Se quiser rodar nós no PC que se comuniquem com o ROS do Limo (por exemplo, rodar o `llm_commander` no PC e o resto no Limo):

```bash
# No PC, antes de qualquer comando ROS:
export ROS_MASTER_URI=http://IP_DO_LIMO:11311
export ROS_IP=IP_DO_SEU_PC
source /opt/ros/noetic/setup.bash   # ou melodic
```

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
