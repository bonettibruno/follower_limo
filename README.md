# Follower LIMO - Projeto de Seguidor de Alvos

Este repositório contém o código desenvolvido para o Trabalho de Graduação focado na plataforma **AgileX LIMO**. O objetivo é implementar um sistema de seguimento de alvos utilizando fusão de dados entre **LiDAR** e **Câmera**.

## 📂 Estrutura do Projeto

* `src/`: Scripts Python com a lógica de detecção e controle.
* `launch/`: Arquivos de inicialização do ROS.
* `config/`: Parâmetros de calibração (Cores HSV e ganhos do controlador).

## 🚀 Como Executar no LIMO

### 1. Pré-requisitos
Certifique-se de que o LIMO está com o ROS instalado e o workspace configurado.
* Dependências: `python-opencv`, `sensor_msgs`, `geometry_msgs`.

### 2. Instalação
Clone este repositório dentro do seu `catkin_ws/src`:
```bash
cd ~/catkin_ws/src
git clone [https://github.com/SEU_USUARIO/follower_limo.git](https://github.com/SEU_USUARIO/follower_limo.git)
cd ..
catkin_make
source devel/setup.bash

