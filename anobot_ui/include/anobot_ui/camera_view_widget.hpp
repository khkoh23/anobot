#ifndef ANOBOT_UI__CAMERA_VIEW_WIDGET_HPP_
#define ANOBOT_UI__CAMERA_VIEW_WIDGET_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <QImage>
#include <QLabel>
#include <QString>
#include <QTimer>
#include <QWidget>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace anobot_ui {

class CameraViewWidget : public QWidget{
  Q_OBJECT

public:
  explicit CameraViewWidget(const rclcpp::Node::SharedPtr & node, QWidget * parent = nullptr);
  ~CameraViewWidget() override = default;
  void setTopic(const QString & topic);

Q_SIGNALS:
  void streamStateChanged(bool ready, const QString & detail);

private Q_SLOTS:
  void updateImage(const QImage & image);
  void checkStreamTimeout();

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & message);
  rclcpp::Node::SharedPtr node_;
  image_transport::Subscriber subscription_;
  QLabel * image_label_{nullptr};
  QTimer * health_timer_{nullptr};
  QString current_topic_;
  std::chrono::steady_clock::time_point last_image_time_;
  bool image_received_{false};
  bool stream_ready_{false};
  double maximum_image_age_s_{1.0};
  std::chrono::steady_clock::time_point  last_display_time_;
  double maximum_display_rate_hz_{5.0};
};

}  // namespace anobot_ui

#endif  // ANOBOT_UI__CAMERA_VIEW_WIDGET_HPP_