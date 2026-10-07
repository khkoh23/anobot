#ifndef ANOBOT_UI__RVIZ_WIDGET_HPP_
#define ANOBOT_UI__RVIZ_WIDGET_HPP_

#include <memory>
#include <QShowEvent>
#include <QWidget>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/visualization_frame.hpp>

namespace anobot_ui{

class RvizWidget : public QWidget{
  Q_OBJECT

public:
  explicit RvizWidget(QWidget * parent = nullptr);
  ~RvizWidget() override = default;

protected:
  void showEvent(QShowEvent * event) override;

private Q_SLOTS:
  void initialize();

private:
  bool initialized_{false};
  rviz_common::VisualizationFrame * frame_{nullptr};
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> rviz_ros_node_;
};

}  // namespace anobot_ui

#endif  // ANOBOT_UI__RVIZ_WIDGET_HPP_