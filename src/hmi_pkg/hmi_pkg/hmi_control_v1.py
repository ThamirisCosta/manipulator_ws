#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QSlider, QLabel, QPushButton, QTabWidget, QLineEdit, QTextEdit
)
from PyQt5.QtCore import Qt, QTimer
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

# ==========================================
# Nó ROS2 principal da IHM
# ==========================================
class ManipulatorGUI(Node):
    def __init__(self):
        super().__init__('manipulator_gui')

        # Publishers
        # Velocidade por juntas (seu controller de velocidade)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        # Pose desejada (cartesiano) para o nó IK
        self.pose_pub = self.create_publisher(PoseStamped, '/desired_pose', 10)

        # Subscribers
        # Estado das juntas (para exibição)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        # Pose atual do end-effector publicada pelo nó KDL
        self.pose_sub = self.create_subscription(PoseStamped, '/end_effector_pose', self.pose_callback, 10)

        # Estados
        self.joint_values = [0.0] * 7
        self.current_joint_positions = [0.0] * 7
        self.current_pose = PoseStamped()
        # Inicializa frame_id para evitar leitura de zeros até primeira mensagem
        self.current_pose.header.frame_id = "base_link"

        self.get_logger().info("IHM inicializada (modo velocidade + cinemática)")

    def joint_callback(self, msg: JointState):
        if len(msg.position) >= len(self.current_joint_positions):
            self.current_joint_positions = list(msg.position[:7])

    def pose_callback(self, msg: PoseStamped):
        # Atualiza a pose atual do end-effector (publicada pelo KDL)
        self.current_pose = msg

# ==========================================
# Aba 1: Controle por juntas (velocidade)
# ==========================================
class JointControlTab(QWidget):
    def __init__(self, node: ManipulatorGUI):
        super().__init__()
        self.node = node
        layout = QVBoxLayout()

        layout.addWidget(QLabel("Controle por juntas (modo velocidade)"))

        self.sliders = []
        for i in range(7):
            lbl = QLabel(f"Junta {i+1}")
            slider = QSlider(Qt.Horizontal)
            slider.setRange(-100, 100)
            slider.setValue(0)
            slider.valueChanged.connect(lambda val, idx=i: self.update_joint(idx, val))
            layout.addWidget(lbl)
            layout.addWidget(slider)
            self.sliders.append(slider)

        send_btn = QPushButton("Enviar velocidades")
        send_btn.clicked.connect(self.publish)
        layout.addWidget(send_btn)

        self.setLayout(layout)

    def update_joint(self, idx, val):
        # escala -100..100 -> -5..5 rad/s
        self.node.joint_values[idx] = val / 20.0

    def publish(self):
        msg = Float64MultiArray()
        msg.data = self.node.joint_values
        self.node.vel_pub.publish(msg)
        self.node.get_logger().info(f"[VELOCITY] Enviado: {msg.data}")

# ==========================================
# Aba 2: Controle Cinemático (Pose + visualização)
# ==========================================
class KinematicControlTab(QWidget):
    def __init__(self, node: ManipulatorGUI):
        super().__init__()
        self.node = node

        layout = QVBoxLayout()
        layout.addWidget(QLabel("Controle por cinemática inversa (deslocamento relativo)"))

        # Entradas de deslocamento relativo
        self.inputs = {}
        for field in ['dx', 'dy', 'dz']:
            hl = QHBoxLayout()
            lbl = QLabel(f"{field} [m]:")
            edit = QLineEdit()
            edit.setText("0.0")
            hl.addWidget(lbl)
            hl.addWidget(edit)
            layout.addLayout(hl)
            self.inputs[field] = edit

        send_btn = QPushButton("Enviar deslocamento")
        send_btn.clicked.connect(self.publish_relative)
        layout.addWidget(send_btn)

        # Grupo de exibição de pose atual
        self.pose_display = QTextEdit()
        self.pose_display.setReadOnly(True)
        self.pose_display.setMinimumHeight(130)
        layout.addWidget(QLabel("Pose atual do end-effector:"))
        layout.addWidget(self.pose_display)

        # Grupo de exibição de juntas
        self.joint_display = QTextEdit()
        self.joint_display.setReadOnly(True)
        self.joint_display.setMinimumHeight(100)
        layout.addWidget(QLabel("Posição atual das juntas [rad]:"))
        layout.addWidget(self.joint_display)

        # Timer para atualizar exibição
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(300)  # atualiza a cada 300 ms

        self.setLayout(layout)

    # Atualiza visor com dados atuais
    def update_display(self):
        pose = self.node.current_pose.pose
        # Pode estar vazio nas primeiras mensagens — proteger
        try:
            self.pose_display.setPlainText(
                f"x: {pose.position.x:.4f}\n"
                f"y: {pose.position.y:.4f}\n"
                f"z: {pose.position.z:.4f}\n"
                f"qx: {pose.orientation.x:.4f}\n"
                f"qy: {pose.orientation.y:.4f}\n"
                f"qz: {pose.orientation.z:.4f}\n"
                f"qw: {pose.orientation.w:.4f}"
            )
        except Exception:
            self.pose_display.setPlainText("Pose indefinida (aguardando mensagem)")

        joints_text = "\n".join([f"J{i+1}: {val:.4f}" for i, val in enumerate(self.node.current_joint_positions)])
        self.joint_display.setPlainText(joints_text)

    # Publica deslocamento relativo em X/Y/Z (mantém orientação atual)
    def publish_relative(self):
        try:
            dx = float(self.inputs['dx'].text())
            dy = float(self.inputs['dy'].text())
            dz = float(self.inputs['dz'].text())
        except ValueError:
            self.node.get_logger().error("Entradas inválidas: use apenas números.")
            return

        # Cria nova pose deslocada
        new_pose = PoseStamped()
        new_pose.header.stamp = self.node.get_clock().now().to_msg()
        new_pose.header.frame_id = "base_link"

        current = self.node.current_pose.pose
        # Se ainda não houve mensagem, usar zeros
        try:
            cur_x = current.position.x
            cur_y = current.position.y
            cur_z = current.position.z
            cur_orient = current.orientation
        except Exception:
            cur_x = cur_y = cur_z = 0.0
            cur_orient = new_pose.pose.orientation  # zeros

        new_pose.pose.position.x = cur_x + dx
        new_pose.pose.position.y = cur_y + dy
        new_pose.pose.position.z = cur_z + dz
        # mantém orientação atual (importante para IK se levar orientação em conta)
        new_pose.pose.orientation = cur_orient

        self.node.pose_pub.publish(new_pose)

        self.node.get_logger().info(
            f"[POSE RELATIVE] Δx={dx:.3f}, Δy={dy:.3f}, Δz={dz:.3f} => "
            f"Nova pose: x={new_pose.pose.position.x:.3f}, y={new_pose.pose.position.y:.3f}, z={new_pose.pose.position.z:.3f}"
        )

# ==========================================
# Interface principal
# ==========================================
class GUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("IHM Manipulador - Velocidade / Cinemática")
        self.resize(450, 600)

        layout = QVBoxLayout()
        tabs = QTabWidget()
        tabs.addTab(JointControlTab(node), "Controle por Juntas")
        tabs.addTab(KinematicControlTab(node), "Controle por Cinemática")
        layout.addWidget(tabs)
        self.setLayout(layout)

# ==========================================
# Main
# ==========================================
def main():
    rclpy.init()
    node = ManipulatorGUI()

    app = QApplication(sys.argv)
    gui = GUI(node)
    gui.show()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            app.processEvents()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
