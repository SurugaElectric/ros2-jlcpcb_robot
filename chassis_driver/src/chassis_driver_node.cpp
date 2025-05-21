#include "chassis_driver/chassis_driver_node.hpp"

#include "utilities/data_utils.hpp"
#include "utilities/utils.hpp"
#include <cmath>

using namespace utils;

namespace chassis_driver{

ChassisDriver::ChassisDriver(const rclcpp::NodeOptions& options) : ChassisDriver("", options) {}

ChassisDriver::ChassisDriver(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("chassis_driver_node", name_space, options),
wheel_radius(get_parameter("wheel_radius").as_double()),
attached_direction(get_parameter("attached_direction").as_double()),
stop_canid(static_cast<uint32_t>(get_parameter("stop_canid").as_int())),
restart_canid(static_cast<uint32_t>(get_parameter("restart_canid").as_int())),
velocity_canid(static_cast<uint32_t>(get_parameter("velocity_canid").as_int())),
linear_max_vel(get_parameter("linear_max_vel").as_double()),
angular_max_vel(dtor(get_parameter("angular_max_vel").as_double())),
sqrt2over2(std::sqrt(2.0) / 2.0)
{
    _subscription_vel = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        _qos,
        std::bind(&ChassisDriver::_subscriber_callback_vel, this, std::placeholders::_1)
    );
    _subscription_stop = this->create_subscription<std_msgs::msg::Empty>(
        "stop",
        _qos,
        std::bind(&ChassisDriver::_subscriber_callback_stop, this, std::placeholders::_1)
    );
    _subscription_restart = this->create_subscription<std_msgs::msg::Empty>(
        "restart",
        _qos,
        std::bind(&ChassisDriver::_subscriber_callback_restart, this, std::placeholders::_1)
    );
    _subscription_selfpose = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "self_pose",
        _qos,
        std::bind(&ChassisDriver::_subscriber_callback_selfpose, this, std::placeholders::_1)
    );

    publisher_can = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);

    RCLCPP_INFO(this->get_logger(), "車輪半径:%f  取り付け距離:%f", wheel_radius, attached_direction);
    RCLCPP_INFO(this->get_logger(), "最大並進速度:%f  最大回転速度:%f", linear_max_vel, angular_max_vel);
}

void ChassisDriver::_subscriber_callback_vel(const geometry_msgs::msg::Twist::SharedPtr msg){
    if(mode == Mode::stop) return;
    mode = Mode::cmd;

    /*座標系変換*/
    this->prime_vel = *msg;
    rotate_vector(msg->linear.x, msg->linear.y, current_yaw); // poseが手に入ったらmap座標系での角度を入力
    // この後にprime_velを操作すること

    /*処理に使う一時データ*/
    const double omega = constrain(msg->angular.z, -angular_max_vel, angular_max_vel);
    // 並進の処理
    const double linear_length = std::sqrt(this->prime_vel.linear.x*this->prime_vel.linear.x + this->prime_vel.linear.y*this->prime_vel.linear.y);
    if(linear_length > linear_max_vel){
        const double scale = linear_max_vel / linear_length;
        this->prime_vel.linear.x *= scale;
        this->prime_vel.linear.y *= scale;
    }
    // 上のスカラー制限処理で問題ないが，最終的なフィルターとして置いておく
    const double vx_p = constrain(this->prime_vel.linear.x, -linear_max_vel, linear_max_vel);
    const double vy_p = constrain(this->prime_vel.linear.y, -linear_max_vel, linear_max_vel);

    std::array<double, 4> wheel_vel;

    /*速度の計算*/
    wheel_vel[0] = (vx_p*sqrt2over2 + vy_p*sqrt2over2 - attached_direction * omega);
    wheel_vel[1] = (-vx_p*sqrt2over2 + vy_p*sqrt2over2 - attached_direction * omega);
    wheel_vel[2] = (-vx_p*sqrt2over2 - vy_p*sqrt2over2 - attached_direction * omega);
    wheel_vel[3] = (vx_p*sqrt2over2 - vy_p*sqrt2over2 - attached_direction * omega);

    int index = 0;
    for (auto &vel : wheel_vel){
        vel = vel / wheel_radius;
        send_motorvel(index, vel);
        index++;
    }
}

/*自己位置のコールバック*/
void ChassisDriver::_subscriber_callback_selfpose(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    this->current_yaw = std::atan2(
        2.0 * (msg->pose.orientation.w * msg->pose.orientation.z),
        1.0 - 2.0 * (msg->pose.orientation.z * msg->pose.orientation.z));
}

void ChassisDriver::rotate_vector(const double vx, const double vy, const double theta){
    const double cos = std::cos(theta);
    const double sin = std::sin(theta);
    const double vx_p = -vy;
    const double vy_p = vx;
    this->prime_vel.linear.x = vx_p * cos + vy_p * sin;
    this->prime_vel.linear.y = -vx_p * sin + vy_p * cos;
}

/*CANモータ設定*/
void ChassisDriver::send_motorvel(const int motor_num, const double vel){
    // 出版
    auto msg_can = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_can->canid = velocity_canid; // ベースID
    msg_can->canid = (msg_can->canid & 0xF0F) | (motor_num << 4);   // モータ番号によって変更
    msg_can->candlc = 4;

    uint8_t _candata[8];
    float_to_bytes(_candata, static_cast<float>(vel));

    for(int i=0; i<msg_can->candlc; i++) msg_can->candata[i]=_candata[i];
    publisher_can->publish(*msg_can);

    // RCLCPP_INFO(this->get_logger(), "canid:%X  vel:%f", msg_can->canid, vel);
}

/*基幹入力*/
void ChassisDriver::_subscriber_callback_stop(const std_msgs::msg::Empty::SharedPtr msg){
    mode = Mode::stop;
    // 出版
    for(int motor_num=0; motor_num<4; motor_num++){
        auto msg_can = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_can->canid = stop_canid; // ベースID
        msg_can->canid = (msg_can->canid & 0xF0F) | (motor_num << 4);   // モータ番号によって変更
        msg_can->candlc = 0;
        publisher_can->publish(*msg_can);
    }

    RCLCPP_INFO(this->get_logger(), "停止");
}
void ChassisDriver::_subscriber_callback_restart(const std_msgs::msg::Empty::SharedPtr msg){
    mode = Mode::stay;
    // 出版
    for(int motor_num=0; motor_num<4; motor_num++){
        auto msg_can = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_can->canid = restart_canid; // ベースID
        msg_can->canid = (msg_can->canid & 0xF0F) | (motor_num << 4);   // モータ番号によって変更
        msg_can->candlc = 0;
        publisher_can->publish(*msg_can);
    }

    RCLCPP_INFO(this->get_logger(), "再稼働");
}

}  // namespace chassis_driver
