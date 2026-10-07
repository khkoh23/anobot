#ifndef ANOBOT_UI__MAIN_WINDOW_HPP_
#define ANOBOT_UI__MAIN_WINDOW_HPP_

#include <memory>
#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QMainWindow>
#include <QMap>
#include <QPushButton>
#include <QTextEdit>
#include <QTimer>
#include <QTreeWidget>
#include <rclcpp/rclcpp.hpp>
#include "anobot_ui/ros_bridge.hpp"
#include "anobot_ui/rviz_widget.hpp"

namespace anobot_ui{

class MainWindow : public QMainWindow{
  Q_OBJECT

public:
  explicit MainWindow(const rclcpp::Node::SharedPtr & node, QWidget * parent = nullptr);
  ~MainWindow() override = default;

private:
  void buildUi();
  void connectSignals();
  void populateOperations();
  void populateBtTree();
  void setTreeItemStatus(const QString & node_name, const QString & status);
  rclcpp::Node::SharedPtr node_;
  RosBridge * ros_bridge_{nullptr};
  RvizWidget * rviz_widget_{nullptr};
  QTimer * ros_spin_timer_{nullptr};
  QComboBox * operation_selector_{nullptr};
  QPushButton * run_button_{nullptr};
  QPushButton * cancel_button_{nullptr};
  QPushButton * refresh_button_{nullptr};
  QCheckBox * allow_execution_checkbox_{nullptr};
  QLabel * overall_state_label_{nullptr};
  QLabel * active_node_label_{nullptr};
  QLabel * message_label_{nullptr};
  QLabel * execution_state_label_{nullptr};
  QTreeWidget * bt_tree_widget_{nullptr};
  QTextEdit * log_text_{nullptr};
  QMap<QString, QTreeWidgetItem *> tree_items_;
  bool updating_execution_checkbox_{false};
};

}  // namespace anobot_ui

#endif  // ANOBOT_UI__MAIN_WINDOW_HPP_