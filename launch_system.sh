#!/bin/bash
# ==============================================================================
# Sistema de Controle do Manipulador 7-DOF
# Launcher Principal - Seleção de Modo (Simulação ou Hardware Real)
# ==============================================================================
# Autor: Sistema de Controle do Manipulador
# Data: 2025-12-10
# 
# Este script lança a interface de seleção de modo que permite escolher entre:
# - Simulação no Gazebo
# - Hardware Real
# ==============================================================================

set -e  # Parar execução em caso de erro

# Cores para output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Banner
echo -e "${BLUE}"
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                                                                ║"
echo "║        Sistema de Controle do Manipulador 7-DOF               ║"
echo "║                   ROS2 Jazzy - 2025                            ║"
echo "║                                                                ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Verificar se está no workspace correto
if [ ! -f "install/setup.bash" ]; then
    echo -e "${RED}[ERRO] Não foi possível encontrar o workspace ROS2.${NC}"
    echo -e "${YELLOW}Execute este script a partir do diretório do workspace (manipulator_ws).${NC}"
    exit 1
fi

# Configurar ambiente ROS2
echo -e "${GREEN}[INFO] Configurando ambiente ROS2...${NC}"

# Source do ROS2 base
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo -e "${GREEN}✓ ROS2 Jazzy carregado${NC}"
else
    echo -e "${RED}[ERRO] ROS2 Jazzy não encontrado em /opt/ros/jazzy${NC}"
    exit 1
fi

# Source do workspace
source install/setup.bash
echo -e "${GREEN}✓ Workspace carregado${NC}"

# Verificar se os pacotes necessários existem
echo -e "${GREEN}[INFO] Verificando pacotes...${NC}"

required_packages=(
    "manipulator_launch_pkg"
    "manipulator_description_pkg"
    "manipulator_controllers_pkg"
    "hmi_pkg"
    "kinematics_kdl"
)

for pkg in "${required_packages[@]}"; do
    if ros2 pkg list | grep -q "^${pkg}$"; then
        echo -e "${GREEN}✓ ${pkg}${NC}"
    else
        echo -e "${RED}✗ ${pkg} não encontrado${NC}"
        echo -e "${YELLOW}Execute: colcon build --packages-select ${pkg}${NC}"
        exit 1
    fi
done

# Lançar interface de seleção de modo
echo ""
echo -e "${BLUE}[INFO] Iniciando interface de seleção de modo...${NC}"
echo ""

# Executar o nó Python da interface de seleção
python3 src/manipulator_launch_pkg/manipulator_launch_pkg/mode_selector.py

echo ""
echo -e "${GREEN}[INFO] Sistema encerrado.${NC}"
