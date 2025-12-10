#!/usr/bin/env python3
"""
Interface de Seleção de Modo - Simulação ou Hardware Real
Autor: Sistema de Controle do Manipulador
Data: 2025-12-10
"""

import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QPushButton, QLabel, QMessageBox)
from PyQt5.QtCore import Qt, QProcess
from PyQt5.QtGui import QFont, QIcon
import os
from ament_index_python.packages import get_package_share_directory


class ModeSelectorWindow(QMainWindow):
    """Janela de seleção do modo de operação do manipulador."""
    
    def __init__(self):
        super().__init__()
        self.process = None
        self.init_ui()
    
    def init_ui(self):
        """Inicializa a interface gráfica."""
        self.setWindowTitle('Sistema de Controle do Manipulador - Seleção de Modo')
        self.setGeometry(100, 100, 600, 400)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f5f5f5;
            }
            QLabel {
                color: #2b2b2b;
            }
            QPushButton {
                background-color: #0d7377;
                color: white;
                border: none;
                padding: 15px;
                font-size: 16px;
                font-weight: bold;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #14a0a6;
            }
            QPushButton:pressed {
                background-color: #0a5456;
            }
        """)
        
        # Widget central
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Layout principal
        layout = QVBoxLayout()
        layout.setSpacing(20)
        layout.setContentsMargins(40, 40, 40, 40)
        
        # Título
        title_label = QLabel('Sistema de Controle do Manipulador 7-DOF')
        title_font = QFont('Arial', 20, QFont.Bold)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)
        
        # Subtítulo
        subtitle_label = QLabel('Selecione o modo de operação:')
        subtitle_font = QFont('Arial', 14)
        subtitle_label.setFont(subtitle_font)
        subtitle_label.setAlignment(Qt.AlignCenter)
        subtitle_label.setStyleSheet("color: #555555; margin-top: 10px;")
        layout.addWidget(subtitle_label)
        
        # Espaçador
        layout.addStretch()
        
        # Botão Simulação
        self.sim_button = QPushButton('🖥️  SIMULAÇÃO (Gazebo)')
        self.sim_button.setMinimumHeight(80)
        self.sim_button.clicked.connect(self.launch_simulation)
        layout.addWidget(self.sim_button)
        
        # Descrição simulação
        sim_desc = QLabel('Executa o manipulador em ambiente simulado com Gazebo e RViz')
        sim_desc.setAlignment(Qt.AlignCenter)
        sim_desc.setStyleSheet("color: #666666; font-size: 12px; margin-bottom: 20px;")
        layout.addWidget(sim_desc)
        
        # Botão Hardware Real
        self.real_button = QPushButton('🤖  HARDWARE REAL')
        self.real_button.setMinimumHeight(80)
        self.real_button.setStyleSheet("""
            QPushButton {
                background-color: #d97706;
            }
            QPushButton:hover {
                background-color: #f59e0b;
            }
            QPushButton:pressed {
                background-color: #b45309;
            }
        """)
        self.real_button.clicked.connect(self.launch_real_hardware)
        layout.addWidget(self.real_button)
        
        # Descrição hardware real
        real_desc = QLabel('Conecta-se ao manipulador físico via interface de hardware')
        real_desc.setAlignment(Qt.AlignCenter)
        real_desc.setStyleSheet("color: #666666; font-size: 12px;")
        layout.addWidget(real_desc)
        
        # Espaçador
        layout.addStretch()
        
        # Rodapé
        footer_label = QLabel('ROS2 Jazzy • 7-DOF Manipulator Control System')
        footer_label.setAlignment(Qt.AlignCenter)
        footer_label.setStyleSheet("color: #999999; font-size: 10px; margin-top: 20px;")
        layout.addWidget(footer_label)
        
        central_widget.setLayout(layout)
    
    def launch_simulation(self):
        """Lança o sistema em modo de simulação."""
        reply = QMessageBox.question(
            self,
            'Confirmar Simulação',
            'Deseja iniciar o sistema em modo de SIMULAÇÃO (Gazebo)?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes
        )
        
        if reply == QMessageBox.Yes:
            self.disable_buttons()
            self.launch_ros_system('simulation')
    
    def launch_real_hardware(self):
        """Lança o sistema em modo de hardware real."""
        reply = QMessageBox.warning(
            self,
            'Confirmar Hardware Real',
            'ATENÇÃO: Você está prestes a conectar ao HARDWARE REAL.\n\n'
            'Certifique-se de que:\n'
            '• O robô está conectado corretamente\n'
            '• A área de trabalho está livre de obstáculos\n'
            '• Você está pronto para parada de emergência\n\n'
            'Deseja continuar?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self.disable_buttons()
            self.launch_ros_system('real')
    
    def launch_ros_system(self, mode):
        """
        Lança o sistema ROS no modo especificado dentro do container Docker.
        
        Args:
            mode (str): 'simulation' ou 'real'
        """
        try:
            # Limpar processos ROS2 anteriores
            cleanup_cmd = (
                'docker exec ros-jazzy-container bash -c '
                '"pkill -9 -f \\"ros2|rviz2|gz sim|gzserver|controller_manager|hmi|main|joint_state\\" 2>/dev/null || true"'
            )
            print("Limpando processos ROS2 anteriores...")
            os.system(cleanup_cmd)
            
            # Pequeno delay para garantir que os processos foram encerrados
            import time
            time.sleep(2)
            
            # Determinar qual comando executar
            if mode == 'simulation':
                info_msg = 'Iniciando sistema em modo SIMULAÇÃO (Gazebo)...'
                launch_file = 'manipulator_launch.launch.xml'
                # Usa o mesmo comando do start.sh para simulação
                docker_cmd = (
                    'docker exec ros-jazzy-container bash -c '
                    '"cd /ros2_ws && '
                    'source /opt/ros/jazzy/setup.bash && '
                    'colcon build && '
                    'source install/setup.bash && '
                    f'ros2 launch manipulator_launch_pkg {launch_file} start_controller_manager:=false"'
                )
            else:
                info_msg = 'Iniciando sistema em modo HARDWARE REAL...'
                launch_file = 'real_hardware.launch.xml'
                # Para hardware real, usa o launch file XML específico
                docker_cmd = (
                    'docker exec ros-jazzy-container bash -c '
                    '"cd /ros2_ws && '
                    'source /opt/ros/jazzy/setup.bash && '
                    'colcon build && '
                    'source install/setup.bash && '
                    f'ros2 launch manipulator_launch_pkg {launch_file}"'
                )
            
            # Mostrar mensagem
            QMessageBox.information(
                self,
                'Iniciando Sistema',
                f'{info_msg}\n\n'
                'O sistema será iniciado no container Docker.\n'
                'Verifique o terminal original para acompanhar a execução.'
            )
            
            # Executar o comando Docker
            os.system(docker_cmd)
            
            # Fechar a janela de seleção
            self.close()
            
        except Exception as e:
            QMessageBox.critical(
                self,
                'Erro ao Iniciar',
                f'Erro ao iniciar o sistema:\n{str(e)}'
            )
            self.enable_buttons()
    
    def disable_buttons(self):
        """Desabilita os botões durante o lançamento."""
        self.sim_button.setEnabled(False)
        self.real_button.setEnabled(False)
    
    def enable_buttons(self):
        """Habilita os botões."""
        self.sim_button.setEnabled(True)
        self.real_button.setEnabled(True)


def main():
    """Função principal."""
    app = QApplication(sys.argv)
    window = ModeSelectorWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
