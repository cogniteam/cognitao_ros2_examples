

#include <Ros2ActionWaitServer.h>

/**
 * @brief 
 * @param argv argv[1] is the name of the action (ros2 runner)
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = nullptr;
  node = rclcpp::Node::make_shared(argv[1]);

  auto action_server = std::make_shared<Ros2ActionWaitServer>(node, argv[1]);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}