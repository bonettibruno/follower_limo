# Roteiro de Testes — Modo de Tarefas (YOLO + Campos Potenciais)

Objetivo: coletar os dados (CSV → 6 gráficos) e imagens que validam o Modo 2 para
o relatório (Seção 4.3).

---

## 1. O que levar

**Objetos-alvo** (a câmera do LIMO é bem baixa, ~10 cm do chão — objetos no chão
detectam melhor). Leve mais de um e teste a confiança no início:

| Prioridade | Objeto | Classe YOLO (`class_name`) | Observação |
|---|---|---|---|
| 1 (principal) | Mochila | `backpack` | Grande, silhueta distinta, na altura da câmera |
| 2 (confiável) | Uma pessoa parada | `person` | Detecção mais robusta de todas (plano B) |
| 3 (temático) | Bola de futebol | `sports ball` | É o alvo "de projeto"; teste a confiança antes |
| Obstáculo (exp. 2) | Cadeira ou caixa | — | Para bloquear o caminho e acionar o freio |

Dica: prefira objeto de **cor que contraste com o chão** e teste com **boa
iluminação** (você viu que pouca luz derruba a confiança do YOLOv3-tiny).

**Material:**
- Trena (essencial — para medir distância real inicial e final = ground truth)
- Caderno/celular para anotar qual CSV é qual experimento
- Fita crepe / marcador para marcar no chão a posição inicial do robô e do objeto

---

## 2. Ligar e conectar

1. Ligue o robô, conecte na mesma rede Wi-Fi.
2. SSH: `ssh agilex@IP_DO_LIMO`
3. Abra o tmux: `tmux new -s testes`
   - Dividir painel: `Ctrl+B` depois `%` (vertical) ou `"` (horizontal)
   - Navegar: `Ctrl+B` depois seta

---

## 3. Inicialização (uma coisa por painel do tmux)

> Rode na ordem. Cada comando fica rodando no seu painel.

**Painel 1 — base + LiDAR + odometria:**
```bash
roslaunch limo_bringup limo_start.launch
```

**Painel 2 — câmera de profundidade (o nó RGB morre, tudo bem):**
```bash
roslaunch astra_camera astrapro.launch
```

**Painel 3 — câmera RGB via libuvc (publica em /image_raw):**
```bash
rosrun libuvc_camera camera_node _vendor:=0x2bc5 _product:=0x050e \
  _width:=640 _height:=480 _frame_rate:=30 _video_mode:=yuyv
```

**Painel 4 — YOLO (troque o `_target_class` conforme o objeto):**
```bash
rosrun follower_limo yolo_detector.py \
  _target_class:=backpack _confidence_threshold:=0.3 \
  /camera/rgb/image_raw:=/image_raw
```

**Painel 5 — lidar_reader (remapeado para o YOLO):**
```bash
rosrun follower_limo lidar_reader.py /target/detection:=/yolo/detection
```

**Painel 6 — executor de tarefas:**
```bash
rosrun follower_limo task_executor.py
```

**Painel 7 — logger (grava CSV automaticamente a cada tarefa):**
```bash
rosrun follower_limo task_logger.py
```

**Painel 8 — livre**, para enviar comandos e acompanhar o status.

---

## 4. Verificações antes de testar

No painel livre:

```bash
# A câmera RGB está publicando?
rostopic hz /image_raw            # deve dar ~30 Hz

# O YOLO está detectando o objeto? (aponte a câmera pro objeto)
rostopic echo /yolo/class         # deve aparecer "backpack"

# O LiDAR está medindo a distância do alvo?
rostopic echo /lidar/distance_at_angle

# Odometria viva?
rostopic hz /odom
```

Deixe rodando em segundo plano para acompanhar o estado da tarefa:
```bash
rostopic echo /task/status
```

---

## 5. Teste de sanidade (antes dos experimentos)

Confirma movimento + odometria + freio, sem objeto. **Deixe espaço livre à frente.**
```bash
rostopic pub -1 /task/actions std_msgs/String \
  '{"data": "[{\"action\":\"rotate\",\"degrees\":90},{\"action\":\"go_forward\",\"distance_m\":0.5}]"}'
```
Esperado: gira 90° e anda 0,5 m. Se algo estiver a <20 cm na frente, o freio para.

---

## 6. Experimentos

> Para cada um: **meça com a trena** a distância inicial (robô→objeto) e a final.
> Anote o horário/nome do CSV. Cada tarefa gera um `logs/task_AAAAMMDD_HHMMSS.csv`.

### Experimento 1 — Aproximação limpa
Objeto a ~2 m, centralizado, **sem obstáculo**. Marque a posição inicial do robô.
```bash
rostopic pub -1 /task/actions std_msgs/String \
  '{"data": "[{\"action\":\"approach_object\",\"class_name\":\"backpack\",\"target_dist\":0.5,\"timeout\":40}]"}'
```
- [ ] Distância inicial medida: ______ m
- [ ] Distância final medida (trena): ______ m
- [ ] CSV: ______________________
- Valida: convergência à distância-alvo (0,5 m) e centralização.

### Experimento 2 — Aproximação com obstáculo
Mesmo comando. **Depois que o robô começar a andar, coloque a cadeira/caixa entre
o robô e o objeto.** Prova o freio + a força repulsiva.
```bash
rostopic pub -1 /task/actions std_msgs/String \
  '{"data": "[{\"action\":\"approach_object\",\"class_name\":\"backpack\",\"target_dist\":0.5,\"timeout\":40}]"}'
```
- [ ] Distância do obstáculo quando parou (trena): ______ m (esperado ~0,20 m)
- [ ] CSV: ______________________
- Valida: freio de segurança e repulsão dos campos potenciais.

### Experimento 3 — Busca + aproximação
Objeto fora do campo de visão inicial (ex.: 90° à direita). Robô varre, acha, aproxima.
```bash
rostopic pub -1 /task/actions std_msgs/String \
  '{"data": "[{\"action\":\"scan_area\"},{\"action\":\"search_object\",\"class_name\":\"backpack\",\"timeout\":25},{\"action\":\"approach_object\",\"class_name\":\"backpack\",\"target_dist\":0.5,\"timeout\":40}]"}'
```
- [ ] Encontrou? Em quanto tempo (aprox.): ______
- [ ] Distância final (trena): ______ m
- [ ] CSV: ______________________
- Valida: busca ativa + trajetória rica na odometria.

**Repita cada experimento 2–3 vezes** (dá consistência e permite escolher o melhor).

---

## 7. Imagens a capturar

- Print do `/yolo/image_debug` no Foxglove (bounding box no objeto)
- Foto/vídeo de cada cena (robô, objeto, obstáculo) — para ilustrar o relatório
- `rqt_graph` dos nós rodando (1 comando, boa figura de arquitetura):
  ```bash
  rosrun rqt_graph rqt_graph
  ```

---

## 8. Parar / cancelar com segurança

- O freio para sozinho a <20 cm de um obstáculo frontal.
- Para **interromper no meio** de uma ação: `Ctrl+C` no painel do `task_executor`
  (o comando `cancel` só é checado entre ações, não no meio de uma).
- Mantenha espaço livre e esteja pronto para desligar se necessário.

---

## 9. Coletar os dados no fim

Os CSVs ficam em `~/agilex_ws/src/follower_limo/logs/` no robô.

No seu **PC** (não no SSH):
```bash
scp agilex@IP_DO_LIMO:~/agilex_ws/src/follower_limo/logs/*.csv ./logs/
python tools/plot_task.py logs/task_AAAAMMDD_HHMMSS.csv
```
Isso gera o PNG com os 6 painéis para cada experimento.

---

## Resumo da validação (o que cada dado prova)

| Dado coletado | O que valida no relatório |
|---|---|
| Trena inicial/final vs alvo 0,5 m | Erro de posicionamento (ex.: parou a 0,53 m → erro 3 cm) |
| Painel distância ao alvo | Convergência ao setpoint (tempo de acomodação) |
| Painel ângulo YOLO | Centralização do objeto (ângulo → 0°) |
| Painel forças + LiDAR mín. | Freio e repulsão agindo (Exp. 2) |
| Painel trajetória XY (odom) | Caminho percorrido até o objeto |
| Odometria vs trena | Consistência entre sensores (triangulação) |
