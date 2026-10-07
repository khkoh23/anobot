#include <memory>
#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "anobot_ui/main_window.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  QApplication application(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("anobot_ui", options);
  anobot_ui::MainWindow window(node);
  window.show();
  const int result = application.exec();
  rclcpp::shutdown();
  return result;
}