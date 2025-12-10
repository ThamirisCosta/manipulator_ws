# Correções Aplicadas - Mode Selector

## Problema Identificado
O `mode_selector.py` estava executando comandos no **host** em vez de dentro do **container Docker**.

## Solução Implementada

### 1. Atualização do `mode_selector.py`
- ✅ Agora usa `docker exec ros-jazzy-container` para executar comandos
- ✅ Segue o mesmo padrão do `start.sh` 
- ✅ Compila o projeto antes de lançar (`colcon build`)
- ✅ Usa os mesmos launch files XML

### 2. Atualização do `start_launcher.sh`
- ✅ Configura X11 para GUI (Gazebo, RViz, HMI)
- ✅ Verifica se o container está rodando
- ✅ Inicia o container automaticamente se necessário
- ✅ Lança a interface de seleção após garantir que o container está ativo

### 3. Novo arquivo: `real_hardware.launch.xml`
- ✅ Launch file XML para hardware real (consistente com o padrão do projeto)
- ✅ Carrega: description, controllers, RViz, HMI, cinemática
- ✅ Sem Gazebo

## Como Usar

```bash
cd /home/thamiris/Desktop/ISI/ROS_Simulations/manipulator_ws
./start_launcher.sh
```

### Fluxo de Execução

1. **Script `start_launcher.sh`:**
   - Configura X11
   - Inicia container Docker (se não estiver rodando)
   - Abre a interface gráfica de seleção

2. **Interface de Seleção:**
   - Botão "SIMULAÇÃO (Gazebo)" → Executa dentro do container
   - Botão "HARDWARE REAL" → Executa dentro do container

3. **Comando Docker executado:**
   ```bash
   # Para Simulação:
   docker exec ros-jazzy-container bash -c "cd /ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash && ros2 launch manipulator_launch_pkg manipulator_launch.launch.xml start_controller_manager:=false"
   
   # Para Hardware Real:
   docker exec ros-jazzy-container bash -c "cd /ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash && ros2 launch manipulator_launch_pkg real_hardware.launch.xml"
   ```

## Arquivos Modificados

1. ✏️ `src/manipulator_launch_pkg/manipulator_launch_pkg/mode_selector.py`
2. ✏️ `start_launcher.sh`
3. ➕ `src/manipulator_launch_pkg/launch/real_hardware.launch.xml` (novo)

## Diferenças dos Modos

### Simulação (Gazebo)
- Usa `manipulator_launch.launch.xml`
- Carrega Gazebo com física completa
- Plugin: `gz_ros2_control/GazeboSimSystem`

### Hardware Real
- Usa `real_hardware.launch.xml`
- Sem Gazebo
- Plugin: `mock_components/GenericSystem` (pode ser substituído por driver real)
- Mesma HMI, RViz, controladores e cinemática

## Testes

Execute:
```bash
./start_launcher.sh
```

Selecione **SIMULAÇÃO** → Deve abrir Gazebo + RViz + HMI dentro do container
Selecione **HARDWARE REAL** → Deve abrir RViz + HMI (sem Gazebo) dentro do container

## Arquivos NÃO Alterados
✓ `start.sh` - permanece intacto
✓ `manipulator_launch.launch.xml` - permanece intacto
✓ Todos os arquivos URDF de simulação
✓ Todos os controladores
