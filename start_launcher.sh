#!/bin/bash
# ==============================================================================
# Script de Lançamento do Sistema com Seleção de Modo
# ==============================================================================
# Este script inicia o container Docker e lança a interface de seleção de modo
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Cores
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Banner
echo -e "${BLUE}"
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║     Sistema de Controle do Manipulador 7-DOF - Launcher       ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Configurar X11
echo -e "${GREEN}[INFO] Configurando permissões X11...${NC}"
xhost +local:docker 2>/dev/null || echo -e "${YELLOW}[WARN] Falha ao configurar xhost${NC}"

# Verificar e iniciar container
if ! docker ps | grep -q ros-jazzy-container; then
    echo -e "${GREEN}[INFO] Iniciando container Docker...${NC}"
    docker compose up -d
    sleep 3
    
    if docker ps | grep -q ros-jazzy-container; then
        echo -e "${GREEN}[INFO] Container iniciado com sucesso!${NC}"
    else
        echo -e "${RED}[ERRO] Falha ao iniciar container${NC}"
        exit 1
    fi
else
    echo -e "${GREEN}[INFO] Container já está em execução${NC}"
fi

# Lançar interface de seleção
echo -e "${BLUE}[INFO] Iniciando interface de seleção...${NC}"
echo ""

# Executar o script Python da interface de seleção
python3 src/manipulator_launch_pkg/manipulator_launch_pkg/mode_selector.py

echo ""
echo -e "${GREEN}[INFO] Interface encerrada.${NC}"
