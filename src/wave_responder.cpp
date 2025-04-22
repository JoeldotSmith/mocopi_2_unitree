#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <thread>
#include <chrono>
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "unitree_hg/msg/motor_cmd.hpp"
#include "common/motor_crc_hg.h"

#define INFO_IMU 0
#define INFO_MOTOR 0
#define HIGH_FREQ 1

enum PRorAB
{
    PR = 0,
    AB = 1
};

using std::placeholders::_1;

const int G1_NUM_MOTOR = 29;

enum G1JointIndex
{
    LeftHipPitch = 0,
    LeftHipRoll = 1,
    LeftHipYaw = 2,
    LeftKnee = 3,
    LeftAnklePitch = 4,
    LeftAnkleB = 4,
    LeftAnkleRoll = 5,
    LeftAnkleA = 5,
    RightHipPitch = 6,
    RightHipRoll = 7,
    RightHipYaw = 8,
    RightKnee = 9,
    RightAnklePitch = 10,
    RightAnkleB = 10,
    RightAnkleRoll = 11,
    RightAnkleA = 11,
    WaistYaw = 12,
    WaistRoll = 13,  // NOTE INVALID for g1 23dof/29dof with waist locked
    WaistA = 13,     // NOTE INVALID for g1 23dof/29dof with waist locked
    WaistPitch = 14, // NOTE INVALID for g1 23dof/29dof with waist locked
    WaistB = 14,     // NOTE INVALID for g1 23dof/29dof with waist locked
    LeftShoulderPitch = 15,
    LeftShoulderRoll = 16,
    LeftShoulderYaw = 17,
    LeftElbow = 18,
    LeftWristRoll = 19,
    LeftWristPitch = 20, // NOTE INVALID for g1 23dof
    LeftWristYaw = 21,   // NOTE INVALID for g1 23dof
    RightShoulderPitch = 22,
    RightShoulderRoll = 23,
    RightShoulderYaw = 24,
    RightElbow = 25,
    RightWristRoll = 26,
    RightWristPitch = 27, // NOTE INVALID for g1 23dof
    RightWristYaw = 28    // NOTE INVALID for g1 23dof
};

class WaveResponder : public rclcpp::Node
{
public:
    WaveResponder() : Node("wave_responder")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>("/wave_detector", 10, std::bind(&WaveResponder::wave_callback, this, std::placeholders::_1));

        auto topic_name = "lf/lowstate";
        if (HIGH_FREQ)
        {
            topic_name = "lowstate";
        }

        lowstate_subscriber_ = this->create_subscription<unitree_hg::msg::LowState>(topic_name, 10, std::bind(&WaveResponder::LowStateHandler, this, _1));
        lowcmd_publisher_ = this->create_publisher<unitree_hg::msg::LowCmd>("/lowcmd", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_dt), std::bind(&WaveResponder::Control, this));

        time_ = 0;
        duration_ = 3.0; // 3 s
        wave_detected_ = false;
        finished_action_ = true;
        handshake_detected_ = false;
        RCLCPP_INFO(this->get_logger(), "Wave Responder Node Initialized");
    }

private:
    void wave_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "Wave detected" && !handshake_detected_)
        // if (msg->data == "Handshake detected" && !wave_detected_)
        {
            RCLCPP_INFO(this->get_logger(), "Wave detected! Waving back :)");
            finished_action_ = false;
            action_iterator = 0.0;
            wave_detected_ = true;
            wave_start_time_ = this->now().seconds();
        }
        else if (msg->data == "Handshake detected" && !wave_detected_)
        // else if (msg->data == "Wave detected")
        {
            RCLCPP_INFO(this->get_logger(), "Handshake detected! Shaking hand :)");
            finished_action_ = false;
            action_iterator = 0.0;
            handshake_detected_ = !handshake_detected_;
            handshake_start_time_ = this->now().seconds();
        }
    }
    void Control()
    {
        time_ += control_dt_;
        action_iterator += control_dt_;
        low_command.mode_pr = mode_;
        low_command.mode_machine = mode_machine;
        for (int i = 0; i < G1_NUM_MOTOR; ++i)
        {
            low_command.motor_cmd[i].mode = 1; // 1:Enable, 0:Disable
            low_command.motor_cmd[i].tau = 0.0;
            low_command.motor_cmd[i].q = 0.0;
            low_command.motor_cmd[i].dq = 0.0;
            low_command.motor_cmd[i].kp = (i < 13) ? 100.0 : 50.0;
            low_command.motor_cmd[i].kd = 1.0;
        }

        if (wave_detected_ && !handshake_detected_ && !finished_action_ && time_ > duration_) // Wave Action
        {
            wave();
        }
        else if (!wave_detected_ && handshake_detected_ && !finished_action_ && time_ > duration_) // Handshake Action
        {
            handshake();
        }

        else // go to initial position
        {
            goToInitialPosition();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        get_crc(low_command);
        lowcmd_publisher_->publish(low_command);
    }

    void goToInitialPosition()
    {
        for (int i = 0; i < G1_NUM_MOTOR; ++i)
        {
            double ratio = clamp(time_ / duration_, 0.0, 1.0);
            low_command.motor_cmd[i].q = (1. - ratio) * motor[i].q;
        }
    }

    void wiggleAnkles()
    {
        mode_ = PRorAB::PR; // Enable PR mode
        // generate sin/cos trajectory
        double max_P = 0.25; // [rad]
        double max_R = 0.25; // [rad]
        double t = time_ - duration_;
        double L_P_des = max_P * std::cos(2.0 * M_PI * t);
        double L_R_des = max_R * std::sin(2.0 * M_PI * t);
        double R_P_des = max_P * std::cos(2.0 * M_PI * t);
        double R_R_des = -max_R * std::sin(2.0 * M_PI * t);

        // update ankle joint position targets
        float Kp_Pitch = 80;
        float Kd_Pitch = 1;
        float Kp_Roll = 80;
        float Kd_Roll = 1;

        low_command.motor_cmd[G1JointIndex::LeftAnklePitch].q = L_P_des;
        low_command.motor_cmd[G1JointIndex::LeftAnklePitch].dq = 0;
        low_command.motor_cmd[G1JointIndex::LeftAnklePitch].kp = Kp_Pitch;
        low_command.motor_cmd[G1JointIndex::LeftAnklePitch].kd = Kd_Pitch;
        low_command.motor_cmd[G1JointIndex::LeftAnklePitch].tau = 0;
        low_command.motor_cmd[G1JointIndex::LeftAnkleRoll].q = L_R_des;
        low_command.motor_cmd[G1JointIndex::LeftAnkleRoll].dq = 0;
        low_command.motor_cmd[G1JointIndex::LeftAnkleRoll].kp = Kp_Roll;
        low_command.motor_cmd[G1JointIndex::LeftAnkleRoll].kd = Kd_Roll;
        low_command.motor_cmd[G1JointIndex::LeftAnkleRoll].tau = 0;
        low_command.motor_cmd[G1JointIndex::RightAnklePitch].q = R_P_des;
        low_command.motor_cmd[G1JointIndex::RightAnklePitch].dq = 0;
        low_command.motor_cmd[G1JointIndex::RightAnklePitch].kp = Kp_Pitch;
        low_command.motor_cmd[G1JointIndex::RightAnklePitch].kd = Kd_Pitch;
        low_command.motor_cmd[G1JointIndex::RightAnklePitch].tau = 0;
        low_command.motor_cmd[G1JointIndex::RightAnkleRoll].q = R_R_des;
        low_command.motor_cmd[G1JointIndex::RightAnkleRoll].dq = 0;
        low_command.motor_cmd[G1JointIndex::RightAnkleRoll].kp = Kp_Roll;
        low_command.motor_cmd[G1JointIndex::RightAnkleRoll].kd = Kd_Roll;
        low_command.motor_cmd[G1JointIndex::RightAnkleRoll].tau = 0;

        double max_wrist_roll_angle = 0.5;
        double WristRoll_des = max_wrist_roll_angle * std::sin(2.0 * M_PI * t);
        low_command.motor_cmd[G1JointIndex::LeftWristRoll].q = WristRoll_des;
        low_command.motor_cmd[G1JointIndex::LeftWristRoll].dq = 0;
        low_command.motor_cmd[G1JointIndex::LeftWristRoll].kp = 50;
        low_command.motor_cmd[G1JointIndex::LeftWristRoll].kd = 1;
        low_command.motor_cmd[G1JointIndex::LeftWristRoll].tau = 0;

        low_command.motor_cmd[G1JointIndex::RightWristRoll].q = WristRoll_des;
        low_command.motor_cmd[G1JointIndex::RightWristRoll].dq = 0;
        low_command.motor_cmd[G1JointIndex::RightWristRoll].kp = 50;
        low_command.motor_cmd[G1JointIndex::RightWristRoll].kd = 1;
        low_command.motor_cmd[G1JointIndex::RightWristRoll].tau = 0;
        std::cout << "______________________" << std::endl;
        std::cout << "Left Ankle Des" << L_P_des << std::endl;
    }

    void wave()
    {
        mode_ = PRorAB::PR; // Enable PR mode
        // generate sin/cos trajectory
        double max_shoulder_roll = 1.74533; // [rad]
        double max_shoulder_yaw = 1.0472;   // [rad]
        double left_shoulder_roll_target = max_shoulder_roll * 2 * std::sin(2.0 * M_PI * action_iterator);
        double left_shoulder_yaw_target = max_shoulder_yaw * 2 * std::sin(2.0 * M_PI * action_iterator);
        left_shoulder_roll_target = clamp(left_shoulder_roll_target, 0, max_shoulder_roll);
        left_shoulder_yaw_target = clamp(left_shoulder_yaw_target, 0, max_shoulder_yaw);

        float Kp = 30;
        float Kd = 1;

        // ROLL
        low_command.motor_cmd[G1JointIndex::LeftShoulderRoll].q = left_shoulder_roll_target;
        low_command.motor_cmd[G1JointIndex::LeftShoulderRoll].dq = 0.05;
        low_command.motor_cmd[G1JointIndex::LeftShoulderRoll].kp = Kp;
        low_command.motor_cmd[G1JointIndex::LeftShoulderRoll].kd = Kd;
        low_command.motor_cmd[G1JointIndex::LeftShoulderRoll].tau = 0;

        // YAW
        low_command.motor_cmd[G1JointIndex::LeftShoulderYaw].q = left_shoulder_yaw_target;
        low_command.motor_cmd[G1JointIndex::LeftShoulderYaw].dq = 0.05;
        low_command.motor_cmd[G1JointIndex::LeftShoulderYaw].kp = Kp;
        low_command.motor_cmd[G1JointIndex::LeftShoulderYaw].kd = Kd;
        low_command.motor_cmd[G1JointIndex::LeftShoulderYaw].tau = 0;

        std::cout << "______________________" << std::endl;
        std::cout << "finished_action?: " << finished_action_ << std::endl;
        std::cout << "time: " << action_iterator << std::endl;
        std::cout << "Left Shoulder Target: " << left_shoulder_roll_target << std::endl;
        if (std::abs(left_shoulder_roll_target) < 0.001)
        {
            finished_action_ = true;
        }
    }

    void handshake()
    {
        mode_ = PRorAB::PR; // Enable PR mode
        // generate sin/cos trajectory
        double max_shoulder_pitch = 0.523599; // [rad]
        double right_shoulder_pitch_target = max_shoulder_pitch * 2 * std::sin(2.0 * M_PI * clamp(action_iterator, 0, 0.25));
        right_shoulder_pitch_target = clamp(right_shoulder_pitch_target, 0, max_shoulder_pitch);

        double max_elbow = 0.261799612195482; // [rad]
        double right_elbo = max_elbow * 2 * std::sin(2.0 * M_PI * clamp(action_iterator, 0, 0.25));
        right_elbo = clamp(right_elbo, 0, max_elbow);

        float Kp = 30;
        float Kd = 1;

        // Pitch
        low_command.motor_cmd[G1JointIndex::RightShoulderPitch].q = -right_shoulder_pitch_target;
        low_command.motor_cmd[G1JointIndex::RightShoulderPitch].dq = 0.05;
        low_command.motor_cmd[G1JointIndex::RightShoulderPitch].kp = Kp;
        low_command.motor_cmd[G1JointIndex::RightShoulderPitch].kd = Kd;
        low_command.motor_cmd[G1JointIndex::RightShoulderPitch].tau = 0;

        low_command.motor_cmd[G1JointIndex::RightElbow].q = right_elbo;
        low_command.motor_cmd[G1JointIndex::RightElbow].dq = 0.05;
        low_command.motor_cmd[G1JointIndex::RightElbow].kp = Kp;
        low_command.motor_cmd[G1JointIndex::RightElbow].kd = Kd;
        low_command.motor_cmd[G1JointIndex::RightElbow].tau = 0;

        std::cout << "______________________" << std::endl;
        std::cout << "finished_action?: " << finished_action_ << std::endl;
        std::cout << "time: " << action_iterator << std::endl;
        std::cout << "right Shoulder Target: " << right_shoulder_pitch_target << std::endl;
    }

    void LowStateHandler(unitree_hg::msg::LowState::SharedPtr message)
    {
        mode_machine = (int)message->mode_machine;
        imu = message->imu_state;
        for (int i = 0; i < G1_NUM_MOTOR; i++)
        {
            motor[i] = message->motor_state[i];
        }

        if (INFO_IMU)
        {
            // Info IMU states
            // RPY euler angle(ZYX order respected to body frame)
            // Quaternion
            // Gyroscope (raw data)
            // Accelerometer (raw data)
            RCLCPP_INFO(this->get_logger(), "Euler angle -- roll: %f; pitch: %f; yaw: %f", imu.rpy[0], imu.rpy[1], imu.rpy[2]);
            RCLCPP_INFO(this->get_logger(), "Quaternion -- qw: %f; qx: %f; qy: %f; qz: %f", imu.quaternion[0], imu.quaternion[1], imu.quaternion[2], imu.quaternion[3]);
            RCLCPP_INFO(this->get_logger(), "Gyroscope -- wx: %f; wy: %f; wz: %f", imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2]);
            RCLCPP_INFO(this->get_logger(), "Accelerometer -- ax: %f; ay: %f; az: %f",
                        imu.accelerometer[0], imu.accelerometer[1], imu.accelerometer[2]);
        }
        if (INFO_MOTOR)
        {
            // Info motor states
            // q: angluar (rad)
            // dq: angluar velocity (rad/s)
            // ddq: angluar acceleration (rad/(s^2))
            // tau_est: Estimated external torque
            for (int i = 0; i < G1_NUM_MOTOR; i++)
            {
                motor[i] = message->motor_state[i];
                RCLCPP_INFO(this->get_logger(), "Motor state -- num: %d; q: %f; dq: %f; ddq: %f; tau: %f", i, motor[i].q, motor[i].dq, motor[i].ddq, motor[i].tau_est);
            }
        }
    }

    double clamp(double value, double low, double high)
    {
        if (value < low)
            return low;
        if (value > high)
            return high;
        return value;
    }

    rclcpp::TimerBase::SharedPtr timer_;                                             // ROS2 timer
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_publisher_;         // ROS2 Publisher
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_subscriber_; // ROS2 Subscriber
    unitree_hg::msg::LowCmd low_command;                                             // Unitree hg lowcmd message
    unitree_hg::msg::IMUState imu;                                                   // Unitree hg IMU message
    unitree_hg::msg::MotorState motor[G1_NUM_MOTOR];                                 // Unitree hg motor state message
    double control_dt_ = 0.002;                                                      // 2ms
    int timer_dt = control_dt_ * 1000;
    double time_; // Running time count
    double duration_;
    double action_iterator;
    bool finished_action_;
    bool wave_detected_;
    bool handshake_detected_;
    double wave_start_time_;
    double handshake_start_time_;
    PRorAB mode_ = PRorAB::PR;
    int mode_machine;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::TimerBase::SharedPtr timer_;
    auto node = std::make_shared<WaveResponder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}