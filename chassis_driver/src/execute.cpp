#include <rclcpp/rclcpp.hpp>
#include "chassis_driver/chassis_driver_node.hpp"
#include "socketcan_interface/socketcan_interface_node.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    rclcpp::NodeOptions nodes_option;
    nodes_option.allow_undeclared_parameters(true);
    nodes_option.automatically_declare_parameters_from_overrides(true);

    auto socketcan_node = std::make_shared<socketcan_interface::SocketcanInterface>(nodes_option);
    auto chassis_driver_node = std::make_shared<chassis_driver::ChassisDriver>(nodes_option);
    exec.add_node(socketcan_node);
    exec.add_node(chassis_driver_node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
