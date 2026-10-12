#include "anobot_ui/camera_view_widget.hpp"

#include <functional>
#include <QMetaObject>
#include <QPixmap>
#include <QVBoxLayout>
#include <opencv2/imgproc.hpp>
#include <rmw/qos_profiles.h>

namespace anobot_ui{

CameraViewWidget::CameraViewWidget(const rclcpp::Node::SharedPtr & node, QWidget * parent): QWidget(parent), node_(node){
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  image_label_ = new QLabel(this);
  image_label_->setAlignment(Qt::AlignCenter);
  image_label_->setMinimumSize(480, 360);
  image_label_->setStyleSheet("background-color: #252525;" "color: #dddddd;" "border: 1px solid #555555;");
  image_label_->setText("Waiting for RealSense / ArUco image...");
  layout->addWidget(image_label_);
  health_timer_ = new QTimer(this);
  connect(health_timer_, &QTimer::timeout, this, &CameraViewWidget::checkStreamTimeout);
  health_timer_->start(250);
}

void CameraViewWidget::setTopic(const QString & topic){
  if (topic.isEmpty()) {
    return;
  }
  current_topic_ = topic;
  if (subscription_) {
    subscription_.shutdown();
  }
  image_received_ = false;
  stream_ready_ = false;
  image_label_->clear();
  image_label_->setText("Waiting for image topic:\n" + current_topic_);
  RCLCPP_INFO(node_->get_logger(), "Camera widget subscribing to %s", current_topic_.toStdString().c_str());
  auto qos_profile = rmw_qos_profile_sensor_data;
  qos_profile.depth = 1;
  subscription_ = image_transport::create_subscription(
    node_.get(),
    current_topic_.toStdString(),
    std::bind(&CameraViewWidget::imageCallback, this, std::placeholders::_1),
    "raw",
    qos_profile);
}

void CameraViewWidget::imageCallback(
  const sensor_msgs::msg::Image::
    ConstSharedPtr & message)
{
  const auto now =
    std::chrono::steady_clock::now();

  // Record every received frame for stream-health
  // monitoring, even if this frame is not displayed.
  last_image_time_ = now;
  image_received_ = true;

  if (!stream_ready_) {
    stream_ready_ = true;

    Q_EMIT streamStateChanged(
      true,
      "RealSense color image is updating");
  }

  const double display_interval_s =
    1.0 / maximum_display_rate_hz_;

  if (
    last_display_time_.time_since_epoch().count() != 0)
  {
    const double elapsed_s =
      std::chrono::duration<double>(
      now - last_display_time_).count();

    if (elapsed_s < display_interval_s) {
      return;
    }
  }

  last_display_time_ = now;

  try {
    const auto cv_image =
      cv_bridge::toCvShare(
      message,
      "rgb8");

    const cv::Mat & matrix =
      cv_image->image;

    QImage image(
      matrix.data,
      matrix.cols,
      matrix.rows,
      static_cast<int>(matrix.step),
      QImage::Format_RGB888);

    const QImage copy =
      image.copy();

    QMetaObject::invokeMethod(
      this,
      "updateImage",
      Qt::QueuedConnection,
      Q_ARG(QImage, copy));
  }
  catch (const std::exception & error) {
    const QString detail =
      QString(
      "Image conversion failed: %1")
      .arg(error.what());

    Q_EMIT streamStateChanged(
      false,
      detail);
  }
}

void CameraViewWidget::updateImage(
  const QImage & image)
{
  const QPixmap pixmap =
    QPixmap::fromImage(image);

  image_label_->setPixmap(
    pixmap.scaled(
      image_label_->size(),
      Qt::KeepAspectRatio,
      Qt::FastTransformation));
}

void CameraViewWidget::checkStreamTimeout()
{
  if (!image_received_) {
    if (stream_ready_) {
      stream_ready_ = false;

      Q_EMIT streamStateChanged(
        false,
        "No camera image received");
    }

    return;
  }

  const double age_s =
    std::chrono::duration<double>(
    std::chrono::steady_clock::now() -
    last_image_time_).count();

  const bool ready =
    age_s <= maximum_image_age_s_;

  if (ready == stream_ready_) {
    return;
  }

  stream_ready_ = ready;

  Q_EMIT streamStateChanged(
    ready,
    ready
    ? "RealSense color image is updating"
    : QString(
      "Camera image stale: %1 s")
      .arg(
        age_s,
        0,
        'f',
        2));
}

}  // namespace anobot_ui