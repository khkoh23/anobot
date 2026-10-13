#ifndef ANOBOT_UI__MAIN_WINDOW_HPP_
#define ANOBOT_UI__MAIN_WINDOW_HPP_

#include <memory>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDomElement>
#include <QGridLayout>
#include <QLabel>
#include <QMainWindow>
#include <QMap>
#include <QPushButton>
#include <QStringList>
#include <QTextEdit>
#include <QTimer>
#include <QTreeWidget>
#include <rclcpp/rclcpp.hpp>
#include "anobot_ui/camera_view_widget.hpp"
#include "anobot_ui/ros_bridge.hpp"
#include "anobot_ui/rviz_widget.hpp"

namespace anobot_ui{

struct TaskDefinition{
  QString display_name;
  QString task_name;
  QString xml_path;
  QStringList required_components;
};

class MainWindow : public QMainWindow{
  Q_OBJECT

public:
  explicit MainWindow(const rclcpp::Node::SharedPtr & node, QWidget * parent = nullptr);
  ~MainWindow() override;

protected:
  void closeEvent(QCloseEvent * event) override;

private:
  enum class WorkstationWorkflow{NONE, ACQUIRING, APPLYING_SCENE, REMOVING_SCENE, CLEARING_ACQUISITION};
  void buildUi();
  void connectSignals();
  void populateOperations();
  void populateTaskDefinitions();
  void loadSelectedTree();
  void loadTreeIntoWidget(const QString & xml_path);
  QTreeWidgetItem * buildTreeItemFromDom(const QDomElement & element);
  QString selectedTaskName() const;
  QString selectedTreePath() const;
  QStringList selectedTaskRequirements() const;
  QStringList missingTaskRequirements() const;
  void updateRunAvailability();
  void refreshSystemState();
  void setTreeItemStatus(const QString & node_name, const QString & status);
  void setTaskActive(bool active);
  void resetBtDisplay();
  void finalizeBtDisplay(const QString & final_state);
  void updateHealthIndicator(const QString & component, bool ready, const QString & detail);
  void setWorkstationWorkflow(WorkstationWorkflow workflow);
  void finishWorkstationWorkflow(bool success, const QString & message);
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
  QLabel * mode_banner_label_{nullptr};
  QLabel * rod_status_label_{nullptr};
  bool task_active_{false};
  bool task_terminal_{false};
  QTimer * health_timer_{nullptr};
  QLabel * system_readiness_label_{nullptr};
  QMap<QString, QLabel *> health_value_labels_;
  QMap<QString, QString> health_details_;
  QMap<QString, bool> health_states_;
  QMap<QString, TaskDefinition> task_definitions_;
  QString bt_root_name_;
  QLabel * task_requirements_label_{nullptr};
  QPushButton * refresh_system_button_{nullptr};
  CameraViewWidget * camera_view_widget_{nullptr};
  QString camera_topic_{"/camera/camera/color/image_raw"};
  bool real_perception_enabled_{false};
  WorkstationWorkflow workstation_workflow_{WorkstationWorkflow::NONE};
};

}  // namespace anobot_ui

#endif  // ANOBOT_UI__MAIN_WINDOW_HPP_