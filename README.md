# ROS2 7-DOF Manipulator Simulation & Control System

Sistema completo de simulação e controle para um manipulador robótico de 7 graus de liberdade (DOF), desenvolvido em ROS2 Jazzy com integração Gazebo, controle dinâmico avançado e interface gráfica interativa.

## Sumário

- [Visão Geral](#visão-geral)
- [Arquitetura do Sistema](#arquitetura-do-sistema)
- [Pré-requisitos](#pré-requisitos)
- [Instalação](#instalação)
- [Compilação](#compilação)
- [Execução](#execução)
- [Pacotes do Projeto](#pacotes-do-projeto)
- [Estratégias de Controle](#estratégias-de-controle)
- [Interface Gráfica (HMI)](#interface-gráfica-hmi)
- [Tópicos ROS2](#tópicos-ros2)
- [Estrutura do Projeto](#estrutura-do-projeto)
- [Docker](#docker)
- [Contribuição](#contribuição)
- [Licença](#licença)

## Visão Geral

Este projeto implementa um sistema modular de controle para um manipulador robótico de 7 DOF, incluindo:

- **Simulação em Gazebo** com física realista
- **Controle de Torque Computado** com compensação de gravidade
- **Cinemática Inversa** via biblioteca KDL (Orocos)
- **Controladores de Posição e Velocidade** configuráveis
- **Interface Gráfica PyQt5** para teleoperação

### Características Principais

| Recurso | Descrição |
|---------|-----------|
| 7 Graus de Liberdade | Juntas rotativas J1-J7 |
| Controle Dinâmico | Controlador de torque computado (500 Hz) |
| Cinemática Inversa | Solver Levenberg-Marquardt via KDL |
| Compensação de Gravidade | Modelo dinâmico com RNE |
| HMI Interativa | Controle por juntas e cartesiano |
| Simulação | Gazebo (Ignition) integrado |

## Arquitetura do Sistema

```
┌─────────────────────────────────────────────────────────────────┐
│                           HMI (PyQt5)                           │
│         ┌─────────────────┬─────────────────────────┐           │
│         │  Controle Joint │   Controle Cartesiano   │           │
│         └────────┬────────┴───────────┬─────────────┘           │
└──────────────────┼────────────────────┼─────────────────────────┘
                   │                    │
                   ▼                    ▼
          /velocity_controller    /desired_pose
             /commands                  │
                   │                    │
                   │         ┌──────────┴──────────┐
                   │         │   Kinematics KDL    │
                   │         │    (IK Solver)      │
                   │         └──────────┬──────────┘
                   │                    │
                   ▼                    ▼
         ┌─────────────────────────────────────────┐
         │         Controller Manager              │
         │  ┌──────────────┬──────────────────┐    │
         │  │   Position   │ Computed Torque  │    │
         │  │  Controller  │   Controller     │    │
         │  └──────────────┴──────────────────┘    │
         └───────────────────┬─────────────────────┘
                             │
                             ▼
         ┌─────────────────────────────────────────┐
         │              Gazebo Sim                 │
         │         (7-DOF Manipulator)             │
         └─────────────────────────────────────────┘
```

## Pré-requisitos

### Sistema Operacional
- Ubuntu 24.04 LTS (Recomendado)
- Ubuntu 22.04 LTS (Compatível)

### Dependências Principais
- **ROS2 Jazzy Jalisco**
- **Gazebo (Ignition Gazebo)**
- **Python 3.10+**
- **PyQt5**

### Instalação de Dependências ROS2

```bash
# Adicionar repositório ROS2
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Instalar ROS2 Jazzy
sudo apt update
sudo apt install -y ros-jazzy-desktop

# Instalar dependências do projeto
sudo apt install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-hardware-interface \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-position-controllers \
    ros-jazzy-velocity-controllers \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-kdl-parser \
    ros-jazzy-orocos-kdl-vendor \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt* \
    python3-pyqt5 \
    libeigen3-dev

# Ferramentas de build
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

## Instalação

### Clonar o Repositório

```bash
mkdir -p ~/ros2_ws
cd ~/ros2_ws
git clone https://github.com/ThamirisCosta/manipulator_ws2.git manipulator_ws
cd manipulator_ws
```

### Inicializar rosdep (se ainda não feito)

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Compilação

### Compilação Padrão

```bash
cd ~/ros2_ws/manipulator_ws
colcon build
```

### Compilação com Otimizações

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Compilação de Pacote Específico

```bash
colcon build --packages-select dynamic_controller_plugin
```

### Após a Compilação

```bash
source install/setup.bash
```

> **Dica**: Adicione ao seu `~/.bashrc`:
> ```bash
> echo "source ~/ros2_ws/manipulator_ws/install/setup.bash" >> ~/.bashrc
> ```

## Execução

### 1. Sistema Completo (Recomendado)

Inicia simulação Gazebo + Controlador de Torque Computado + Cinemática Inversa + HMI:

```bash
ros2 launch manipulator_launch_pkg effort_manipulator_launch.launch.py
```

### 2. Simulação com Controladores Padrão

Simulação com controladores de posição e velocidade:

```bash
ros2 launch manipulator_launch_pkg manipulator.launch.py
```

### 3. Apenas Controladores (Gazebo Mínimo)

Setup mínimo para testes:

```bash
ros2 launch manipulator_launch_pkg gazebo_controllers_launch.launch.py
```

### 4. Apenas Descrição do Robô (RViz)

Para visualização sem simulação física:

```bash
ros2 launch manipulator_launch_pkg description.launch.py
```

### 5. Executar HMI Separadamente

Em um terminal separado (após iniciar a simulação):

```bash
ros2 run hmi_pkg hmi
```

### Verificação do Sistema

```bash
# Listar tópicos ativos
ros2 topic list

# Verificar estado dos controladores
ros2 control list_controllers

# Monitorar estados das juntas
ros2 topic echo /joint_states

# Monitorar pose do end-effector
ros2 topic echo /end_effector_pose
```

## Pacotes do Projeto

### manipulator_description_pkg

Definição do modelo do robô em URDF com meshes STL.

| Arquivo | Descrição |
|---------|-----------|
| `manipulator_description.urdf` | Modelo principal com ros2_control |
| `manipulator_description_gazebo.urdf` | Variante para Gazebo |
| `meshes/*.stl` | Malhas 3D das juntas |

**Especificações do Robô:**
- 7 juntas rotativas (Link_1 a Link_7)
- Limite de torque: 50 N·m por junta
- Limite de velocidade: 5.0 rad/s por junta
- Limites de posição: ±3.14 rad ou ±1.57 rad (conforme junta)

### manipulator_controllers_pkg

Configurações de controladores em YAML.

| Arquivo | Taxa | Descrição |
|---------|------|-----------|
| `controllers.yaml` | 100 Hz | Posição + Velocidade |
| `effort_controllers.yaml` | 500 Hz | Torque Computado |

**Parâmetros do Controlador de Torque Computado:**
```yaml
Kp: [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]  # Ganhos proporcionais
Kd: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]         # Ganhos derivativos
gravity: [0.0, 0.0, -9.81]                       # Vetor gravidade
```

### kinematics_kdl

Solver de cinemática inversa em C++ usando biblioteca KDL.

**Funcionalidades:**
- Cinemática inversa via Levenberg-Marquardt
- Cinemática direta para publicação da pose
- Controle proporcional de velocidade (10 Hz)

**Tópicos:**
- Subscribe: `/desired_pose` (geometry_msgs/PoseStamped)
- Publish: `/end_effector_pose` (geometry_msgs/PoseStamped)
- Publish: `/velocity_controller/commands` (std_msgs/Float64MultiArray)

### dynamic_controller_plugin

Plugin de controlador de torque computado com compensação de gravidade.

**Características:**
- Controle não-linear baseado em modelo dinâmico
- Solver Newton-Euler recursivo (RNE)
- Arquitetura pluginlib para ros2_control
- Suporte a prioridade SCHED_FIFO (tempo real)

### hmi_pkg

Interface gráfica PyQt5 para controle manual do manipulador.

**Abas da Interface:**

1. **Controle por Juntas**
   - 7 sliders para velocidade (-5 a +5 rad/s)
   - Controle individual de cada junta

2. **Controle Cartesiano**
   - Pose absoluta (x, y, z, qx, qy, qz, qw)
   - Deslocamento relativo (dx, dy, dz)
   - Exibição em tempo real da pose atual
   - Exibição das posições das juntas

### manipulator_gazebo_pkg

Ambiente de simulação Gazebo.

| Arquivo | Descrição |
|---------|-----------|
| `manipulator_world.sdf` | Mundo Gazebo com física e iluminação |

### manipulator_launch_pkg

Arquivos de lançamento para diferentes cenários.

| Launch File | Descrição |
|-------------|-----------|
| `effort_manipulator_launch.launch.py` | Sistema completo com torque computado |
| `manipulator.launch.py` | Simulação com controladores padrão |
| `gazebo_controllers_launch.launch.py` | Setup mínimo Gazebo |
| `description.launch.py` | Apenas visualização |
| `manipulator_v1.launch.py` | Variante com controle de velocidade |

## Estratégias de Controle

### 1. Controle de Posição

```bash
# Enviar comando de posição
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [0.0, 0.5, -0.5, 0.3, 0.0, 0.2, 0.0]"
```

### 2. Controle de Velocidade

```bash
# Enviar comando de velocidade
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

### 3. Controle Cartesiano (via IK)

```bash
# Enviar pose desejada para o solver IK
ros2 topic pub /desired_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.5, y: 0.0, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

## Interface Gráfica (HMI)

A HMI oferece duas modalidades de controle:

### Aba de Controle por Juntas

- Sliders para cada uma das 7 juntas
- Faixa: -5.0 a +5.0 rad/s
- Atualização em tempo real

### Aba de Controle Cinemático

- **Pose Absoluta**: Definir posição (x, y, z) e orientação (quaternion)
- **Deslocamento Relativo**: Incrementos em dx, dy, dz
- **Feedback**: Visualização da pose atual do end-effector e posições das juntas

## Tópicos ROS2

### Tópicos Publicados

| Tópico | Tipo | Descrição |
|--------|------|-----------|
| `/joint_states` | sensor_msgs/JointState | Estado das juntas |
| `/end_effector_pose` | geometry_msgs/PoseStamped | Pose do TCP |
| `/velocity_controller/commands` | std_msgs/Float64MultiArray | Comandos de velocidade |
| `/robot_description` | std_msgs/String | URDF do robô |

### Tópicos Subscritos

| Tópico | Tipo | Descrição |
|--------|------|-----------|
| `/desired_pose` | geometry_msgs/PoseStamped | Pose cartesiana desejada |
| `/command` | trajectory_msgs/JointTrajectoryPoint | Referência para torque computado |

## Estrutura do Projeto

```
manipulator_ws/
├── Dockerfile                              # Imagem Docker ROS2 Jazzy
├── build.sh                                # Script de build Docker
├── run.sh                                  # Script de execução Docker
├── README.md                               # Este arquivo
└── src/
    ├── manipulator_description_pkg/        # Modelo URDF + Meshes
    │   ├── urdf/
    │   │   ├── manipulator_description.urdf
    │   │   └── manipulator_description_gazebo.urdf
    │   └── meshes/
    │       └── *.stl
    │
    ├── manipulator_controllers_pkg/        # Configurações de controle
    │   └── controllers/
    │       ├── controllers.yaml
    │       └── effort_controllers.yaml
    │
    ├── manipulator_launch_pkg/             # Arquivos de lançamento
    │   └── launch/
    │       ├── effort_manipulator_launch.launch.py
    │       ├── manipulator.launch.py
    │       └── ...
    │
    ├── kinematics_kdl/                     # Solver IK/FK (C++)
    │   └── src/
    │       └── main.cpp
    │
    ├── dynamic_controller_plugin/          # Controlador de torque (C++)
    │   ├── include/
    │   │   └── computed_torque_controller.hpp
    │   ├── src/
    │   │   └── computed_torque_controller.cpp
    │   └── plugins.xml
    │
    ├── hmi_pkg/                            # Interface gráfica (Python)
    │   └── hmi_pkg/
    │       └── hmi_control.py
    │
    └── manipulator_gazebo_pkg/             # Ambiente Gazebo
        └── worlds/
            └── manipulator_world.sdf
```

## Docker

### Construir Imagem

```bash
# Criar arquivo TOKEN.txt com credenciais Git (se necessário)
echo "seu_token_github" > TOKEN.txt

# Construir imagem
bash build.sh
```

### Executar Container

```bash
# Executa com detecção automática de GPU NVIDIA
bash run.sh
```

O container inclui:
- ROS2 Jazzy completo
- Gazebo e ferramentas de simulação
- RViz2 e rqt
- Suporte a GPU NVIDIA (se disponível)

## Contribuição

1. Fork o repositório
2. Crie uma branch para sua feature (`git checkout -b feature/nova-feature`)
3. Commit suas mudanças (`git commit -m 'Adiciona nova feature'`)
4. Push para a branch (`git push origin feature/nova-feature`)
5. Abra um Pull Request

## Licença

- **dynamic_controller_plugin**: Apache-2.0
- **Demais pacotes**: Consulte arquivos `package.xml` individuais

---

**Desenvolvido por**: Thamiris Costa
**Repositório**: https://github.com/ThamirisCosta/manipulator_ws2.git
