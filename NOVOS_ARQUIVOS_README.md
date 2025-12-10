# Documentação dos Novos Arquivos Criados para Suporte a Hardware Real

## Arquivos Criados (NÃO alteram arquivos existentes)

### 1. Interface de Seleção de Modo
**Arquivo:** `src/manipulator_launch_pkg/manipulator_launch_pkg/mode_selector.py`
- Interface gráfica PyQt5 para escolher entre Simulação ou Hardware Real
- Botões interativos com confirmações de segurança
- Lança automaticamente o sistema apropriado

### 2. URDF para Hardware Real
**Arquivo:** `src/manipulator_description_pkg/urdf/manipulator_description_real.urdf`
- URDF idêntico ao de simulação, mas com plugin `mock_components/GenericSystem`
- Remove o plugin do Gazebo (`gz_ros2_control/GazeboSimSystem`)
- Mantém todas as definições de juntas, links e geometria
- Comentários detalhados sobre como substituir por drivers reais:
  - Dynamixel
  - UR (Universal Robots)
  - EtherCAT
  - Serial customizado

### 3. Launch File para Hardware Real
**Arquivo:** `src/manipulator_launch_pkg/launch/real_hardware.launch.py`
- Launch completo sem Gazebo
- Inicia: Robot State Publisher, Controller Manager, RViz, Controladores, Cinemática KDL, HMI
- Configurado com `use_sim_time=false`
- Timers para inicialização sequencial dos componentes

### 4. Scripts de Lançamento
**Arquivos:** 
- `launch_system.sh` - Script completo com verificações
- `start_launcher.sh` - Script simplificado

Ambos:
- Configuram ambiente ROS2
- Lançam a interface de seleção de modo
- Verificam pacotes necessários

### 5. Setup Atualizado (Referência)
**Arquivo:** `src/manipulator_launch_pkg/setup_new.py`
- Adiciona entry point para o mode_selector
- Permite executar via `ros2 run manipulator_launch_pkg mode_selector`
- **NOTA:** Este é um arquivo de referência. Para usar, renomeie para `setup.py`

## Como Usar

### Método 1: Script Shell (Recomendado)
```bash
cd /home/thamiris/Desktop/ISI/ROS_Simulations/manipulator_ws
./start_launcher.sh
```

### Método 2: Diretamente
```bash
cd /home/thamiris/Desktop/ISI/ROS_Simulations/manipulator_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/manipulator_launch_pkg/manipulator_launch_pkg/mode_selector.py
```

### Método 3: Via ROS2 (após recompilar com setup_new.py)
```bash
# Substituir setup.py pelo setup_new.py e recompilar
mv src/manipulator_launch_pkg/setup.py src/manipulator_launch_pkg/setup_old.py
mv src/manipulator_launch_pkg/setup_new.py src/manipulator_launch_pkg/setup.py
colcon build --packages-select manipulator_launch_pkg
source install/setup.bash

# Executar
ros2 run manipulator_launch_pkg mode_selector
```

## Fluxo de Operação

1. **Execução do Launcher**
   - `start_launcher.sh` configura ambiente e abre a GUI

2. **Seleção do Modo**
   - **Simulação (Gazebo)**: Lança `gazebo_controllers_launch.launch.py` (arquivo existente)
   - **Hardware Real**: Lança `real_hardware.launch.py` (arquivo novo)

3. **Sistema Iniciado**
   - Todos os componentes necessários são carregados
   - HMI, RViz, controladores, cinemática funcionam idênticos

## Configuração para Hardware Real

Para conectar ao hardware real de verdade, edite:
```xml
# Arquivo: manipulator_description_real.urdf
# Linha ~470

<hardware>
    <!-- Substitua este mock pelo driver real -->
    <plugin>mock_components/GenericSystem</plugin>
    
    <!-- Exemplo para Dynamixel: -->
    <plugin>dynamixel_hardware/DynamixelHardware</plugin>
    <param name="usb_port">/dev/ttyUSB0</param>
    <param name="baud_rate">1000000</param>
</hardware>
```

## Arquivos NÃO Alterados

✓ Todos os arquivos existentes permanecem intactos:
- `manipulator_description.urdf`
- `manipulator_description_gazebo.urdf`
- `gazebo_controllers_launch.launch.py`
- `manipulator.launch.py`
- `controllers.yaml`
- Todos os pacotes de controle e cinemática

## Vantagens da Implementação

1. **Modular**: Escolha o modo sem alterar código
2. **Seguro**: Avisos de segurança para hardware real
3. **Compatível**: Sistema de simulação permanece inalterado
4. **Escalável**: Fácil adicionar novos modos ou drivers
5. **Testável**: Mock permite testar sem hardware físico

## Próximos Passos para Usar Hardware Real

1. Identificar o tipo de hardware (servos, drivers, protocolo)
2. Instalar o driver ROS2 apropriado
3. Editar `manipulator_description_real.urdf` com o plugin correto
4. Configurar parâmetros de comunicação (porta, IP, baudrate)
5. Testar comunicação básica
6. Executar via interface de seleção
