#!/bin/bash

# Script de inicialização do projeto ROS2 Manipulator
# Uso: ./start.sh [build|run|stop|clean]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Cores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Função para configurar X11
setup_x11() {
    print_status "Configurando permissões X11..."
    xhost +local:docker 2>/dev/null || print_warning "Falha ao configurar xhost (pode já estar configurado)"
}

# Função para iniciar o container
start_container() {
    print_status "Iniciando container Docker..."
    docker compose up -d
    sleep 3

    if docker ps | grep -q ros-jazzy-container; then
        print_status "Container iniciado com sucesso!"
    else
        print_error "Falha ao iniciar container"
        exit 1
    fi
}

# Função para compilar o projeto
build_project() {
    print_status "Compilando projeto ROS2..."
    docker exec ros-jazzy-container bash -c "cd /ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build"
    print_status "Compilação concluída!"
}

# Função para executar a simulação
run_simulation() {
    print_status "Iniciando simulação..."
    docker exec ros-jazzy-container bash -c "cd /ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch manipulator_launch_pkg manipulator_launch.launch.xml"
}

# Função para parar o container
stop_container() {
    print_status "Parando container..."
    docker stop ros-jazzy-container 2>/dev/null || true
    docker rm ros-jazzy-container 2>/dev/null || true
    print_status "Container parado!"
}

# Função para limpar tudo
clean_all() {
    print_status "Limpando ambiente..."
    stop_container
    docker compose down --volumes --remove-orphans 2>/dev/null || true
    print_status "Ambiente limpo!"
}

# Função para abrir terminal interativo
open_terminal() {
    print_status "Abrindo terminal interativo no container..."
    docker exec -it ros-jazzy-container bash
}

# Função principal
case "${1:-run}" in
    build)
        setup_x11
        start_container
        build_project
        print_status "Projeto compilado! Execute './start.sh run' para iniciar a simulação."
        ;;
    run)
        setup_x11
        start_container
        build_project
        run_simulation
        ;;
    stop)
        stop_container
        ;;
    clean)
        clean_all
        ;;
    terminal|shell)
        setup_x11
        start_container
        open_terminal
        ;;
    help|--help|-h)
        echo "Uso: $0 [comando]"
        echo ""
        echo "Comandos disponíveis:"
        echo "  build    - Inicia container e compila o projeto"
        echo "  run      - Inicia container, compila e executa a simulação (padrão)"
        echo "  stop     - Para o container"
        echo "  clean    - Para e remove o container e volumes"
        echo "  terminal - Abre terminal interativo no container"
        echo "  help     - Mostra esta ajuda"
        echo ""
        echo "Exemplos:"
        echo "  ./start.sh          # Executa build e run"
        echo "  ./start.sh build    # Apenas compila"
        echo "  ./start.sh terminal # Abre shell interativo"
        ;;
    *)
        print_error "Comando desconhecido: $1"
        echo "Use '$0 help' para ver os comandos disponíveis."
        exit 1
        ;;
esac
