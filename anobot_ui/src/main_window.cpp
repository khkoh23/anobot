#include "anobot_ui/main_window.hpp"

#include <QBrush>
#include <QColor>
#include <QDateTime>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QSplitter>
#include <QVBoxLayout>
#include <QWidget>

namespace anobot_ui{

MainWindow::MainWindow(const rclcpp::Node::SharedPtr & node, QWidget * parent) : QMainWindow(parent), node_(node){
  ros_bridge_ = new RosBridge(node_, this);
  buildUi();
  populateOperations();
  populateBtTree();
  connectSignals();
  ros_spin_timer_ = new QTimer(this);
  connect(ros_spin_timer_, &QTimer::timeout, this, [this](){
    rclcpp::spin_some(node_);
  });
  ros_spin_timer_->start(20);
  QTimer::singleShot(1000, this, [this](){
    ros_bridge_->requestExecutionAllowed();
  });
  QTimer::singleShot(5000, this, [this](){
    ros_bridge_->callOperation("apply_workstation_scene");
  });
}

void MainWindow::buildUi(){
  setWindowTitle("Anobot Operator Console");
  resize(1500, 900);
  auto * central = new QWidget(this);
  auto * root_layout = new QVBoxLayout(central);
  auto * top_splitter = new QSplitter(Qt::Horizontal, central);
  auto * camera_group = new QGroupBox("Camera / Perception", top_splitter);
  auto * camera_layout = new QVBoxLayout(camera_group);
  auto * camera_placeholder = new QLabel("RealSense and ArUco view\n" "will be connected in the real-perception milestone.", camera_group);
  camera_placeholder->setAlignment(Qt::AlignCenter);
  camera_placeholder->setStyleSheet("background-color: #252525;" "color: #dddddd;" "border: 1px solid #555555;" "font-size: 16px;");
  camera_placeholder->setMinimumSize(480, 360);
  camera_layout->addWidget(camera_placeholder);
  auto * rviz_group = new QGroupBox("Embedded RViz", top_splitter);
  auto * rviz_layout = new QVBoxLayout(rviz_group);
  rviz_widget_ = new RvizWidget(rviz_group);
  rviz_widget_->setMinimumSize(640, 480);
  rviz_layout->addWidget(rviz_widget_);
  top_splitter->addWidget(camera_group);
  top_splitter->addWidget(rviz_group);
  top_splitter->setStretchFactor(0, 2);
  top_splitter->setStretchFactor(1, 3);
  root_layout->addWidget(top_splitter, 3);
  auto * bottom_splitter = new QSplitter(Qt::Horizontal, central);
  auto * controls_group = new QGroupBox("Task and Developer Controls", bottom_splitter);
  auto * controls_layout = new QVBoxLayout(controls_group);
  allow_execution_checkbox_ = new QCheckBox("Allow trajectory execution " "(mock hardware only)", controls_group);
  allow_execution_checkbox_-> setStyleSheet("QCheckBox {" "font-weight: bold;" "color: #aa0000;" "}");
  controls_layout->addWidget(allow_execution_checkbox_);
  execution_state_label_ = new QLabel("Execution permission: UNKNOWN", controls_group);
  controls_layout->addWidget(execution_state_label_);
  operation_selector_ = new QComboBox(controls_group);
  controls_layout->addWidget(operation_selector_);
  auto * button_layout = new QHBoxLayout();
  run_button_ = new QPushButton("Run", controls_group);
  run_button_->setStyleSheet("background-color: #8fe8a0;" "font-weight: bold;" "min-height: 38px;");
  cancel_button_ = new QPushButton("Cancel Task", controls_group);
  cancel_button_->setStyleSheet("background-color: #f66151;" "font-weight: bold;" "min-height: 38px;");
  refresh_button_ = new QPushButton("Refresh Permission", controls_group);
  button_layout->addWidget(run_button_);
  button_layout->addWidget(cancel_button_);
  button_layout->addWidget(refresh_button_);
  controls_layout->addLayout(button_layout);
  overall_state_label_ = new QLabel("State: IDLE", controls_group);
  active_node_label_ = new QLabel("Active node: -", controls_group);
  message_label_ = new QLabel("Message: -", controls_group);
  message_label_->setWordWrap(true);
  controls_layout->addWidget(overall_state_label_);
  controls_layout->addWidget(active_node_label_);
  controls_layout->addWidget(message_label_);
  controls_layout->addStretch();
  auto * monitoring_splitter = new QSplitter(Qt::Vertical, bottom_splitter);
  auto * bt_group = new QGroupBox("Behavior Tree", monitoring_splitter);
  auto * bt_layout = new QVBoxLayout(bt_group);
  bt_tree_widget_ = new QTreeWidget(bt_group);
  bt_tree_widget_->setColumnCount(2);
  bt_tree_widget_->setHeaderLabels({"BT Node", "Status"});
  bt_tree_widget_->header()->setSectionResizeMode(QHeaderView::Stretch);
  bt_layout->addWidget(bt_tree_widget_);
  auto * log_group = new QGroupBox("Log", monitoring_splitter);
  auto * log_layout = new QVBoxLayout(log_group);
  log_text_ = new QTextEdit(log_group);
  log_text_->setReadOnly(true); 
  log_layout->addWidget(log_text_);
  monitoring_splitter->addWidget(bt_group);
  monitoring_splitter->addWidget(log_group);
  controls_group->setMinimumWidth(430);
  bottom_splitter->addWidget(controls_group);
  bottom_splitter->addWidget(monitoring_splitter);
  bottom_splitter->setStretchFactor(0, 1);
  bottom_splitter->setStretchFactor(1, 2);
  root_layout->addWidget(bottom_splitter, 2);
  setCentralWidget(central);
}

void MainWindow::populateOperations(){
  operation_selector_->clear();
  operation_selector_->addItem("Run complete mock rod pickup", "task:mock_rod_pickup");
  operation_selector_->insertSeparator(operation_selector_->count());
  operation_selector_->addItem("Apply workstation collision scene", "service:apply_workstation_scene");
  operation_selector_->addItem("Reset rod on workstation", "service:reset_rod");
  operation_selector_->addItem("Show rod status", "service:rod_status");
  operation_selector_->insertSeparator(operation_selector_->count());
  operation_selector_->addItem("Plan pre-grasp", "service:plan_pregrasp");
  operation_selector_->addItem("Execute pre-grasp", "service:execute_pregrasp");
  operation_selector_->addItem("Plan linear grasp", "service:plan_grasp");
  operation_selector_->addItem("Execute linear grasp", "service:execute_grasp");
  operation_selector_->addItem("Attach rod", "service:attach_rod");
  operation_selector_->addItem("Plan linear retreat", "service:plan_retreat");
  operation_selector_->addItem("Execute linear retreat", "service:execute_retreat");
  operation_selector_->addItem("Detach rod", "service:detach_rod");
}

void MainWindow::populateBtTree(){
  bt_tree_widget_->clear();
  tree_items_.clear();
  auto * root = new QTreeWidgetItem({"MockRodPickupSequence", "IDLE"});
  bt_tree_widget_->addTopLevelItem(root);
  const QStringList node_names = {
    "ResetRod",
    "VerifyRodAtWorkstation",
    "PlanPregrasp",
    "ExecutePregrasp",
    "PlanLinearGrasp",
    "ExecuteLinearGrasp",
    "AttachRod",
    "VerifyRodAttached",
    "PlanLinearRetreat",
    "ExecuteLinearRetreat",
    "VerifyRodAfterRetreat"
  };
  tree_items_["MockRodPickupSequence"] = root;
  for (const auto & name : node_names) {
    auto * item = new QTreeWidgetItem({name, "IDLE"});
    root->addChild(item);
    tree_items_[name] = item;
  }
  bt_tree_widget_->expandAll();
}

void MainWindow::connectSignals() {
  connect(run_button_, &QPushButton::clicked, this, [this](){
    const QString command = operation_selector_->currentData().toString();
    if (command.startsWith("task:")) {
      ros_bridge_->startTask(command.mid(5));
      return;
    }
    if (command.startsWith("service:")) {
      ros_bridge_->callOperation(command.mid(8));
    }
  });
  connect(cancel_button_, &QPushButton::clicked, ros_bridge_, &RosBridge::cancelTask);
  connect(refresh_button_, &QPushButton::clicked, ros_bridge_, &RosBridge::requestExecutionAllowed);
  connect(allow_execution_checkbox_, &QCheckBox::toggled, this, [this](bool checked){
    if (updating_execution_checkbox_){
      return;
    }
    ros_bridge_->setExecutionAllowed(checked);
  });
  connect(ros_bridge_, &RosBridge::executionAllowedUpdated, this, [this](bool allowed, bool success){
    updating_execution_checkbox_ = true;
    allow_execution_checkbox_->setChecked(success && allowed);
    updating_execution_checkbox_ = false;
    execution_state_label_->setText(success ? QString("Execution permission: %1").arg(allowed ? "ENABLED" : "LOCKED") : "Execution permission: " "UNAVAILABLE");
    execution_state_label_->setStyleSheet(success && allowed ? "color: #aa0000;" "font-weight: bold;" : "color: #006000;" "font-weight: bold;");
  });
  connect(ros_bridge_, &RosBridge::taskStateUpdated, this, [this](const QString &, const QString & state, const QString & active_node, const QString & message){
    overall_state_label_->setText("State: " + state);
    active_node_label_->setText("Active node: " + (active_node.isEmpty() ? "-" : active_node));
    message_label_->setText("Message: " + message);
  });
  connect(ros_bridge_, &RosBridge::btNodeStatusUpdated, this, [this](const QString & node_name, const QString &, const QString & status){
    setTreeItemStatus(node_name, status);
  });
  connect(ros_bridge_, &RosBridge::logMessage, this, [this](const QString & message){
    const QString stamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    log_text_->append(QString("[%1] %2").arg(stamp, message));
  });
  connect(ros_bridge_, &RosBridge::operationFinished, this, [this](const QString & operation, bool success, const QString & message){
    const QString stamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    log_text_->append(QString("[%1] %2: %3: %4").arg(stamp, operation, success ? "SUCCESS" : "FAILURE", message));
  });
}

void MainWindow::setTreeItemStatus(const QString & node_name, const QString & status){
  if (!tree_items_.contains(node_name)){
    return;
  }
  auto * item = tree_items_[node_name];
  item->setText(1, status);
  QColor color = Qt::white;
  if (status == "RUNNING") {
    color = QColor(255, 235, 130);
  } 
  else if (status == "SUCCESS") {
    color = QColor(170, 255, 170);
  } 
  else if (status == "FAILURE") {
    color = QColor(255, 170, 170);
  } 
  else if (status == "IDLE") {
    color = QColor(245, 245, 245);
  }
  item->setBackground(0, QBrush(color));
  item->setBackground(1, QBrush(color));
}

}  // namespace anobot_ui