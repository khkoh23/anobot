#include "anobot_ui/rviz_widget.hpp"

#include <filesystem>
#include <string>
#include <QTimer>
#include <QVBoxLayout>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

namespace anobot_ui{

RvizWidget::RvizWidget(QWidget * parent) : QWidget(parent){
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  rviz_ros_node_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("anobot_ui_rviz");
  frame_ = new rviz_common::VisualizationFrame(rviz_ros_node_, this);
  frame_->setWindowFlags(frame_->windowFlags() & ~Qt::Window);
  frame_->setWindowFlags(frame_->windowFlags() | Qt::Widget);
  frame_->setAttribute(Qt::WA_DeleteOnClose, false);
  frame_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  layout->addWidget(frame_);
}

void RvizWidget::showEvent(QShowEvent * event){
  QWidget::showEvent(event);
  if (!initialized_){
    QTimer::singleShot(0, this, &RvizWidget::initialize);
  }
}

void RvizWidget::initialize(){
  if (initialized_ || frame_ == nullptr){
    return;
  }
  try {
    const std::string package_share = ament_index_cpp::get_package_share_directory("anobot_moveit_config");
    const std::string rviz_config = package_share + "/config/moveit.rviz";
    RCLCPP_INFO(rclcpp::get_logger("RvizWidget"), "Loading embedded RViz config: %s", rviz_config.c_str());
    if (!std::filesystem::exists(rviz_config)){
      RCLCPP_ERROR(rclcpp::get_logger("RvizWidget"), "RViz config not found: %s", rviz_config.c_str());
      return;
    }
    frame_->setSplashPath("");
    frame_->initialize(rviz_ros_node_, QString::fromStdString(rviz_config));
    frame_->show();
    frame_->raise();
    if (frame_->layout() != nullptr) {
      frame_->layout()->activate();
    }
    frame_->updateGeometry();
    frame_->repaint();
    initialized_ = true;
    RCLCPP_INFO(rclcpp::get_logger("RvizWidget"), "Embedded RViz initialized");
  }
  catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("RvizWidget"), "Embedded RViz initialization failed: %s", error.what());
    initialized_ = false;
  }
}

}  // namespace anobot_ui