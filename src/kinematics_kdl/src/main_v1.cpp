#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "kdl_parser/kdl_parser.hpp"
#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"

using namespace std::placeholders;

class LegIK : public rclcpp::Node
{
public:
    LegIK() : Node("inverse_kinematics"), first_iteration_(true), initial_x_(0.0), initial_y_(0.0), initial_z_(0.0)
    {
        // Publisher: pose atual do end-effector (para exibição na IHM)
        current_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/end_effector_pose", 10);

        // Publisher: COMANDOS DE VELOCIDADE para o controller
        velocity_commands_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/velocity_controller/commands", 10);

        // Subscrição: pose desejada vinda da IHM
        position_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/desired_pose", 10, std::bind(&LegIK::positionCallback, this, _1));

        // Subscrição: robot_description (URDF) — transient_local para pegar latched param
        robot_description_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "robot_description",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            std::bind(&LegIK::robotDescriptionCallback, this, _1));

        // Subscrição: estados das juntas para inicializar solver / FK
        joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&LegIK::jointStateCallback, this, _1));

        // Timer para controle de velocidade contínuo
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz
            std::bind(&LegIK::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "LegIK node has been started.");
    }

private:
    void positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!solver_) {
            RCLCPP_WARN(this->get_logger(), "KDL solver not initialized yet.");
            return;
        }

        // Recebe a pose desejada inteira (posição + orientação)
        const auto& position = msg->pose.position;
        const auto& orientation = msg->pose.orientation;
        RCLCPP_INFO(this->get_logger(), "Received desired pose: x=%.3f y=%.3f z=%.3f qx=%.3f qy=%.3f qz=%.3f qw=%.3f",
            position.x, position.y, position.z,
            orientation.x, orientation.y, orientation.z, orientation.w);

        // monta KDL::Frame com posição + rotação (quaternion)
        KDL::Rotation R = KDL::Rotation::Quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w);
        KDL::Frame desired_frame(R, KDL::Vector(position.x, position.y, position.z));

        // usa previous_joint_angles_ (lido de /joint_states) como q_init
        KDL::JntArray q_init(chain_.getNrOfJoints());
        for (size_t i = 0; i < previous_joint_angles_.size(); ++i) {
            q_init(i) = previous_joint_angles_[i];
        }

        KDL::JntArray q_out(chain_.getNrOfJoints());
        int result = solver_->CartToJnt(q_init, desired_frame, q_out);

        if (result < 0) {
            RCLCPP_ERROR(this->get_logger(), "Inverse kinematics solver failed with error code: %d", result);
            return;
        }

        // converte q_out para vector<double> - ESTA É A NOVA POSIÇÃO DESEJADA DAS JUNTAS
        desired_joint_positions_.assign(q_out.data.data(), q_out.data.data() + q_out.data.size());

        RCLCPP_INFO(this->get_logger(), "IK solution computed, starting velocity control");
        
        // Ativa o controle de velocidade
        control_active_ = true;
        last_control_time_ = this->now();
    }

    void controlLoop()
    {
        if (!control_active_ || desired_joint_positions_.empty()) {
            return;
        }

        auto current_time = this->now();
        double dt = (current_time - last_control_time_).seconds();
        
        if (dt <= 0) {
            return;
        }

        // Calcula erros de posição
        std::vector<double> position_errors(chain_.getNrOfJoints());
        for (size_t i = 0; i < previous_joint_angles_.size(); ++i) {
            position_errors[i] = desired_joint_positions_[i] - previous_joint_angles_[i];
        }

        // Verifica se alcançamos a posição desejada
        bool reached_target = true;
        for (size_t i = 0; i < position_errors.size(); ++i) {
            if (std::fabs(position_errors[i]) > position_tolerance_) {
                reached_target = false;
                break;
            }
        }

        if (reached_target) {
            RCLCPP_INFO(this->get_logger(), "Target position reached");
            control_active_ = false;
            
            // Publica velocidades zero
            std_msgs::msg::Float64MultiArray stop_msg;
            stop_msg.data.assign(chain_.getNrOfJoints(), 0.0);
            velocity_commands_publisher_->publish(stop_msg);
            return;
        }

        // Controlador P simples para velocidade
        std_msgs::msg::Float64MultiArray velocity_msg;
        for (size_t i = 0; i < position_errors.size(); ++i) {
            double velocity = kp_ * position_errors[i];
            
            // Limita a velocidade máxima
            if (velocity > max_velocity_) velocity = max_velocity_;
            if (velocity < -max_velocity_) velocity = -max_velocity_;
            
            velocity_msg.data.push_back(velocity);
        }

        velocity_commands_publisher_->publish(velocity_msg);
        last_control_time_ = current_time;

        // Atualiza pose atual para visualização
        publishCurrentPoseFromFK(previous_joint_angles_);
    }

    void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string urdf = msg->data;
        if (!kdl_parser::treeFromString(urdf, tree_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF from robot description.");
            return;
        }

        std::string last_link = getLastLink(tree_);
        if (last_link.empty() || !tree_.getChain("base_link", last_link, chain_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract KDL chain from base_link to %s", last_link.c_str());
            return;
        }

        solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);
        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

        joint_names_ = getJointNames(chain_);
        previous_joint_angles_.assign(chain_.getNrOfJoints(), 0.0);
        desired_joint_positions_.assign(chain_.getNrOfJoints(), 0.0);

        // Inicializa parâmetros do controlador
        kp_ = 2.0;  // Ganho proporcional
        max_velocity_ = 5.0; // Velocidade máxima rad/s
        position_tolerance_ = 0.01; // Tolerância de posição em radianos
        control_active_ = false;

        RCLCPP_INFO(this->get_logger(), "KDL chain created with %u joints.", chain_.getNrOfJoints());
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Atualiza previous_joint_angles_ com base nos nomes das juntas
        for (size_t i = 0; i < msg->name.size(); ++i) {
            auto it = std::find(joint_names_.begin(), joint_names_.end(), msg->name[i]);
            if (it != joint_names_.end()) {
                previous_joint_angles_[std::distance(joint_names_.begin(), it)] = msg->position[i];
            }
        }

        // Atualiza pose atual via FK (para visualização)
        calculateCurrentPose();
    }

    // calcula FK com o vetor previous_joint_angles_ e publica /end_effector_pose
    void calculateCurrentPose()
    {
        if (!fk_solver_) {
            RCLCPP_WARN(this->get_logger(), "KDL forward kinematics solver is not initialized.");
            return;
        }

        KDL::JntArray joint_positions(chain_.getNrOfJoints());
        for (size_t i = 0; i < previous_joint_angles_.size(); ++i) {
            joint_positions(i) = previous_joint_angles_[i];
        }

        KDL::Frame current_pose;
        if (fk_solver_->JntToCart(joint_positions, current_pose) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Forward kinematics solver failed.");
            return;
        }

        const auto& pos = current_pose.p;
        const auto& rot = current_pose.M;

        if (first_iteration_) {
            first_iteration_ = false;
            initial_x_ = pos.x();
            initial_y_ = pos.y();
            initial_z_ = pos.z();
            RCLCPP_INFO(this->get_logger(), "First iteration: Saving initial position.");
        }

        RCLCPP_DEBUG(this->get_logger(), "Current pose FK: x=%.3f y=%.3f z=%.3f", pos.x(), pos.y(), pos.z());

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "base_link";
        pose_msg.pose.position.x = pos.x();
        pose_msg.pose.position.y = pos.y();
        pose_msg.pose.position.z = pos.z();

        double qx, qy, qz, qw;
        rot.GetQuaternion(qx, qy, qz, qw);
        pose_msg.pose.orientation.x = qx;
        pose_msg.pose.orientation.y = qy;
        pose_msg.pose.orientation.z = qz;
        pose_msg.pose.orientation.w = qw;

        current_pose_publisher_->publish(pose_msg);
    }

    static std::string getLastLink(const KDL::Tree& tree)
    {
        for (const auto& segment : tree.getSegments()) {
            if (segment.second.children.empty()) {
                return segment.first;
            }
        }
        return {};
    }

    static std::vector<std::string> getJointNames(const KDL::Chain& chain)
    {
        std::vector<std::string> joint_names;
        for (const auto& segment : chain.segments) {
            std::string jname = segment.getJoint().getName();
            if (!jname.empty()) {
                joint_names.push_back(jname);
            }
        }
        return joint_names;
    }

    void publishCurrentPoseFromFK(const std::vector<double>& joint_angles)
    {
        (void)joint_angles;
        // Apenas chama calculateCurrentPose que usa previous_joint_angles_
        calculateCurrentPose();
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_commands_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    KDL::Tree tree_;
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> solver_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

    std::vector<std::string> joint_names_;
    std::vector<double> previous_joint_angles_;
    std::vector<double> desired_joint_positions_;

    // Parâmetros do controlador de velocidade
    double kp_;
    double max_velocity_;
    double position_tolerance_;
    bool control_active_;
    rclcpp::Time last_control_time_;

    bool first_iteration_;
    double initial_x_, initial_y_, initial_z_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegIK>());
    rclcpp::shutdown();
    return 0;
}