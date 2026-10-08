#include "anobot_ui/main_window.hpp"

#include <chrono>
#include <thread>
#include <QBrush>
#include <QColor>
#include <QDateTime>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QSplitter>
#include <QVBoxLayout>
#include <QWidget>
#include <QMessageBox>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QDomDocument>
#include <QFile>
#include <QStringList>

namespace anobot_ui{

MainWindow::MainWindow(const rclcpp::Node::SharedPtr & node, QWidget * parent) : QMainWindow(parent), node_(node){
  ros_bridge_ = new RosBridge(node_, this);
  buildUi();
  populateTaskDefinitions();
  populateOperations();
  loadSelectedTree();
  connectSignals();
  updateRunAvailability();
  ros_spin_timer_ = new QTimer(this);
  connect(ros_spin_timer_, &QTimer::timeout, this, [this](){
    rclcpp::spin_some(node_);
  });
  ros_spin_timer_->start(20);
  health_timer_ = new QTimer(this);
  connect(health_timer_, &QTimer::timeout, this, [this](){
    ros_bridge_->pollSystemHealth();
  });
  health_timer_->start(1000);
  QTimer::singleShot(250, this, [this](){
    ros_bridge_->pollSystemHealth();
  });
  QTimer::singleShot(5000, this, [this](){
    refreshSystemState();
  });
}

MainWindow::~MainWindow(){
  if (ros_bridge_ != nullptr) {
    ros_bridge_->lockExecutionOnShutdown();
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    rclcpp::spin_some(node_);
  }
}

void MainWindow::buildUi(){
  setWindowTitle("Anobot Operator Console");
  resize(1500, 900);
  auto * central = new QWidget(this);
  auto * root_layout = new QVBoxLayout(central);
  mode_banner_label_ = new QLabel("DEVELOPMENT MODE: MOCK HARDWARE", central);
  mode_banner_label_->setAlignment(Qt::AlignCenter);
  mode_banner_label_->setStyleSheet("background-color: #e6a23c;" "color: #202020;" "font-size: 16px;" "font-weight: bold;" "padding: 7px;" "border: 1px solid #a86b00;");
  root_layout->addWidget(mode_banner_label_);
  auto * top_splitter = new QSplitter(Qt::Horizontal, central);
  auto * camera_group = new QGroupBox("Camera / Perception", top_splitter);
  auto * camera_layout = new QVBoxLayout(camera_group);
  auto * camera_placeholder = new QLabel("RealSense and ArUco view\n" "will be connected in the real-perception milestone.", camera_group);
  camera_placeholder->setAlignment(Qt::AlignCenter);
  camera_placeholder->setStyleSheet("background-color: #252525;" "color: #dddddd;" "border: 1px solid #555555;" "font-size: 16px;");
  camera_placeholder->setMinimumSize(420, 400);
  camera_layout->addWidget(camera_placeholder);
  auto * rviz_group = new QGroupBox("Embedded RViz", top_splitter);
  auto * rviz_layout = new QVBoxLayout(rviz_group);
  rviz_widget_ = new RvizWidget(rviz_group);
  rviz_widget_->setMinimumSize(600, 400);
  rviz_layout->addWidget(rviz_widget_);
  top_splitter->addWidget(camera_group);
  top_splitter->addWidget(rviz_group);
  top_splitter->setStretchFactor(0, 2);
  top_splitter->setStretchFactor(1, 3);
  root_layout->addWidget(top_splitter, 6);
  auto * bottom_splitter = new QSplitter(Qt::Horizontal, central);
  auto * readiness_group = new QGroupBox("System Readiness", bottom_splitter);
  auto * readiness_layout = new QVBoxLayout(readiness_group);
  auto * controls_group = new QGroupBox("Task and Developer Controls", bottom_splitter);
  auto * controls_layout = new QVBoxLayout(controls_group);
  allow_execution_checkbox_ = new QCheckBox("Allow trajectory execution", controls_group);
  allow_execution_checkbox_->setToolTip("Allow trajectory execution on mock hardware only");
  allow_execution_checkbox_-> setStyleSheet("QCheckBox {" "font-weight: bold;" "color: #aa0000;" "}");
  controls_layout->addWidget(allow_execution_checkbox_);
  execution_state_label_ = new QLabel("Execution permission: UNKNOWN", controls_group);
  rod_status_label_ = new QLabel("Rod state: UNKNOWN", controls_group);
  rod_status_label_->setWordWrap(true);
  rod_status_label_->setStyleSheet("font-weight: bold;" "color: #404060;");
  auto * health_layout = new QGridLayout();
  const QList<QPair<QString, QString>> health_components = {
    {"controller", "Trajectory controller"},
    {"moveit", "MoveIt"},
    {"workstation_marker", "Workstation marker"},
    {"workstation_scene", "Workstation scene"},
    {"manipulation", "Manipulation planner"},
    {"rod_lifecycle", "Rod lifecycle"},
    {"bt_executor", "BT executor"}
  };
  int health_row = 0;
  for (const auto & component : health_components){
    auto * name_label = new QLabel(component.second, readiness_group);
    auto * value_label = new QLabel("UNKNOWN", readiness_group);
    value_label->setAlignment(Qt::AlignCenter);
    value_label->setMinimumWidth(100);
    value_label->setStyleSheet("background-color: #888888;" "color: white;" "font-weight: bold;" "padding: 3px;" "border-radius: 3px;"); 
    health_layout->addWidget(name_label, health_row, 0);
    health_layout->addWidget(value_label, health_row, 1);
    health_value_labels_[component.first] = value_label;
    health_states_[component.first] = false;
    ++health_row;
  }
  system_readiness_label_ = new QLabel("Overall readiness: CHECKING", readiness_group);
  system_readiness_label_->setAlignment(Qt::AlignCenter);
  system_readiness_label_->setStyleSheet("background-color: #888888;" "color: white;" "font-weight: bold;" "padding: 5px;");
  health_layout->addWidget(system_readiness_label_, health_row, 0, 1, 2);
  readiness_layout->addLayout(health_layout);
  refresh_system_button_ = new QPushButton("Refresh System State", readiness_group);
  refresh_system_button_->setToolTip("Refresh component readiness, " "execution permission, rod state, " "and workstation collision scene.");
  readiness_layout->addWidget(refresh_system_button_);
  readiness_layout->addStretch();
  controls_layout->addWidget(execution_state_label_);
  controls_layout->addWidget(rod_status_label_);
  controls_layout->addSpacing(8);
  operation_selector_ = new QComboBox(controls_group);
  controls_layout->addWidget(operation_selector_);
  task_requirements_label_ = new QLabel("Task requirements: checking...", controls_group);
  task_requirements_label_->setWordWrap(true);
  task_requirements_label_->setStyleSheet("font-size: 12px;" "color: #606060;");
  controls_layout->addWidget(task_requirements_label_);
  auto * button_layout = new QHBoxLayout();
  run_button_ = new QPushButton("Run", controls_group);
  run_button_->setStyleSheet("background-color: #8fe8a0;" "font-weight: bold;" "min-height: 38px;");
  cancel_button_ = new QPushButton("Cancel Task", controls_group);
  cancel_button_->setStyleSheet("background-color: #f66151;" "font-weight: bold;" "min-height: 38px;");
  cancel_button_->setEnabled(false);
  refresh_button_ = new QPushButton("Refresh Permission", controls_group);
  button_layout->addWidget(run_button_);
  button_layout->addWidget(cancel_button_);
  button_layout->addWidget(refresh_button_);
  controls_layout->addLayout(button_layout);
  controls_layout->addSpacing(8);
  overall_state_label_ = new QLabel("State: IDLE", controls_group);
  active_node_label_ = new QLabel("Active node: -", controls_group);
  message_label_ = new QLabel("Message: -", controls_group);
  message_label_->setWordWrap(true);
  message_label_->setMaximumHeight(70);
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
  bottom_splitter->addWidget(readiness_group);
  bottom_splitter->addWidget(controls_group);
  bottom_splitter->addWidget(monitoring_splitter);
  bottom_splitter->setStretchFactor(0, 1);
  bottom_splitter->setStretchFactor(1, 2);
  readiness_group->setMinimumWidth(300);
  controls_group->setMinimumWidth(360);
  monitoring_splitter->setMinimumWidth(620);
  bottom_splitter->setStretchFactor(0, 3);
  bottom_splitter->setStretchFactor(1, 4);
  bottom_splitter->setStretchFactor(2, 7);
  root_layout->addWidget(bottom_splitter, 4);
  setCentralWidget(central);
}

void MainWindow::connectSignals() {
  connect(run_button_, &QPushButton::clicked, this, [this](){
    const QString command = operation_selector_->currentData().toString();
    if (command.startsWith("task:")) {
      const QStringList missing = missingTaskRequirements();
      if (!missing.isEmpty()) {
        QMessageBox::warning(this, "Task blocked", "Required components are not ready:\n\n" + missing.join("\n"));
        updateRunAvailability();
        return;
      }
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
    if (checked) {
      const auto answer = QMessageBox::warning(this,
        "Enable trajectory execution",
        "Enable trajectory execution on mock hardware?\n\n"
        "The robot model will move when execution "
        "services or behavior-tree tasks are called.",
        QMessageBox::Yes |
        QMessageBox::No,
        QMessageBox::No);
      if (answer != QMessageBox::Yes) {
        updating_execution_checkbox_ = true;
        allow_execution_checkbox_->setChecked(false);
        updating_execution_checkbox_ = false;
        return;
      }
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
    if (state == "SUCCESS" || state == "FAILURE" || state == "FAULT" || state == "CANCELED"){
      finalizeBtDisplay(state);
      ros_bridge_->callOperation("rod_status");
    }
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
  connect(ros_bridge_, &RosBridge::taskActiveChanged, this, [this](bool active){
    setTaskActive(active);
    if (active) {
      resetBtDisplay();
    }
  });
  connect(ros_bridge_, &RosBridge::rodStatusUpdated, this, [this](const QString & status, bool success){
    rod_status_label_->setText(success ? status : "Rod state: UNAVAILABLE");
    rod_status_label_->setStyleSheet(success ? "font-weight: bold;" "color: #204080;" : "font-weight: bold;" "color: #aa0000;");
  });
  connect(ros_bridge_, &RosBridge::componentHealthUpdated, this, [this](const QString & component, bool ready, const QString & detail){
    updateHealthIndicator(component, ready, detail);
  });
  connect(refresh_system_button_, &QPushButton::clicked, this, [this](){
    refreshSystemState();
  });
}

void MainWindow::populateOperations(){
  operation_selector_->clear();
  for (auto iterator = task_definitions_.constBegin(); iterator != task_definitions_.constEnd(); ++iterator){
    const auto & definition = iterator.value();
    operation_selector_->addItem(definition.display_name, "task:" + definition.task_name);
  }
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

void MainWindow::populateTaskDefinitions(){
  task_definitions_.clear();
  const QString package_share = QString::fromStdString(ament_index_cpp::get_package_share_directory("anobot_bt"));
  TaskDefinition mock_pickup;
  mock_pickup.display_name = "Run complete mock rod pickup";
  mock_pickup.task_name = "mock_rod_pickup";
  mock_pickup.xml_path = package_share + "/behavior_trees/mock_rod_pickup.xml";
  mock_pickup.required_components = QStringList{
    "controller",
    "moveit",
    "workstation_marker",
    "workstation_scene",
    "manipulation",
    "rod_lifecycle",
    "bt_executor"
  };
  task_definitions_.insert(mock_pickup.task_name, mock_pickup);
}

void MainWindow::loadSelectedTree(){
  const QString path = selectedTreePath();
  if (path.isEmpty()) {
    return;
  }
  loadTreeIntoWidget(path);
}

void MainWindow::loadTreeIntoWidget(const QString & xml_path){
  bt_tree_widget_->clear();
  tree_items_.clear();
  bt_root_name_.clear();
  QFile file(xml_path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)){
    log_text_->append("Unable to open behavior tree: " + xml_path);
    return;
  }
  QDomDocument document;
  QString parse_error;
  int error_line = 0;
  int error_column = 0;
  if (!document.setContent(&file, &parse_error, &error_line, &error_column)){
    log_text_->append(QString("BT XML error at line %1, " "column %2: %3").arg(error_line).arg(error_column).arg(parse_error));
    return;
  }
  const QDomElement root = document.documentElement();
  QDomElement behavior_tree;
  QDomNode child = root.firstChild();
  while (!child.isNull()) {
    if (child.isElement() && child.toElement().tagName() == "BehaviorTree"){
      behavior_tree = child.toElement();
      break;
    }
    child = child.nextSibling();
  }
  if (behavior_tree.isNull()) {
    log_text_->append("No BehaviorTree element found in " + xml_path);
    return;
  }
  QDomNode tree_child = behavior_tree.firstChild();
  while (!tree_child.isNull()) {
    if (tree_child.isElement()) {
      QTreeWidgetItem * item = buildTreeItemFromDom(tree_child.toElement());
      if (item != nullptr) {
        bt_tree_widget_->addTopLevelItem(item);
        if (bt_root_name_.isEmpty()) {
          bt_root_name_ = item->text(0);
        }
      }
    }
    tree_child = tree_child.nextSibling();
  }
  bt_tree_widget_->expandAll();
  resetBtDisplay();
}

QTreeWidgetItem * MainWindow::buildTreeItemFromDom(const QDomElement & element){
  if (element.isNull()) {
    return nullptr;
  }
  QString node_name;
  if (element.hasAttribute("name")) {
    node_name = element.attribute("name");
  } 
  else {
    node_name = element.tagName();
  }
  auto * item = new QTreeWidgetItem({node_name, "IDLE"});
  tree_items_[node_name] = item;
  QDomNode child = element.firstChild();
  while (!child.isNull()) {
    if (child.isElement()) {
      QTreeWidgetItem * child_item = buildTreeItemFromDom(child.toElement());
      if (child_item != nullptr) {
        item->addChild(child_item);
      }
    }
    child = child.nextSibling();
  }
  return item;
}

QString MainWindow::selectedTaskName() const{
  const QString command = operation_selector_->currentData().toString();
  if (!command.startsWith("task:")) {
    return "";
  }
  return command.mid(5);
}

QString MainWindow::selectedTreePath() const{
  const QString task_name = selectedTaskName();
  if (task_name.isEmpty() || !task_definitions_.contains(task_name)){
    return "";
  }
  return task_definitions_[task_name].xml_path;
}

QStringList MainWindow::selectedTaskRequirements() const{
  const QString task_name = selectedTaskName();
  if (task_name.isEmpty() || !task_definitions_.contains(task_name)){
    return {};
  }
  return task_definitions_[task_name].required_components;
}

QStringList MainWindow::missingTaskRequirements() const{
  QStringList missing;
  const QStringList requirements = selectedTaskRequirements();
  for (const QString & component : requirements){
    if (!health_states_.contains(component) || !health_states_.value(component)){
      missing.append(component);
    }
  }
  return missing;
}

static QString healthDisplayName(const QString & component){
  static const QMap<QString, QString> names = {
    {"controller", "Trajectory controller"},
    {"moveit", "MoveIt"},
    {"workstation_marker", "Workstation marker"},
    {"workstation_scene", "Workstation scene"},
    {"manipulation", "Manipulation planner"},
    {"rod_lifecycle", "Rod lifecycle"},
    {"bt_executor", "BT executor"}
  };
  return names.value(component, component);
}

void MainWindow::updateRunAvailability(){
  if (task_active_) {
    run_button_->setEnabled(false);
    task_requirements_label_->setText("Task is currently running.");
    task_requirements_label_->setStyleSheet("font-size: 12px;" "font-weight: bold;" "color: #805000;");
    return;
  }
  const QString task_name = selectedTaskName();
  if (task_name.isEmpty()) {
    run_button_->setEnabled(true);
    task_requirements_label_->setText("Developer operation. " "Use only when the robot state " "and selected operation are understood.");
    task_requirements_label_->setStyleSheet("font-size: 12px;" "color: #805000;");
    run_button_->setToolTip("Run the selected developer operation");
    return;
  }
  const QStringList missing = missingTaskRequirements();
  if (missing.isEmpty()) {
    run_button_->setEnabled(true);
    task_requirements_label_->setText("Task requirements: READY");
    task_requirements_label_->setStyleSheet("font-size: 12px;" "font-weight: bold;" "color: #207030;");
    run_button_->setToolTip("All required components are ready");
    return;
  }
  run_button_->setEnabled(false);
  QStringList display_missing;
  for (const QString & component : missing){
    display_missing.append(healthDisplayName(component));
  }
  const QString message = "Task blocked. Missing: " + display_missing.join(", ");
  task_requirements_label_->setText(message);
  task_requirements_label_->setStyleSheet("font-size: 12px;" "font-weight: bold;" "color: #aa0000;");
  run_button_->setToolTip(message);
}

void MainWindow::refreshSystemState(){
  ros_bridge_->pollSystemHealth();
  ros_bridge_->requestExecutionAllowed();
  ros_bridge_->callOperation("rod_status");
  ros_bridge_->callOperation("apply_workstation_scene");
}

void MainWindow::setTreeItemStatus(const QString & node_name, const QString & status){
  if (task_terminal_ && status == "IDLE"){
    return;
  }
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

void MainWindow::setTaskActive(bool active){
  task_active_ = active;
  operation_selector_->setEnabled(!active);
  cancel_button_->setEnabled(active);
  if (active) {
    run_button_->setEnabled(false);
    run_button_->setText("Task Running...");
  } 
  else {
    run_button_->setText("Run");
    updateRunAvailability();
  }
}

void MainWindow::resetBtDisplay(){
  task_terminal_ = false;
  for (auto iterator = tree_items_.begin(); iterator != tree_items_.end(); ++iterator){
    setTreeItemStatus(iterator.key(), "IDLE");
  }
}

void MainWindow::finalizeBtDisplay(const QString & final_state){
  task_terminal_ = true;
  if (final_state == "SUCCESS") {
    for (auto iterator = tree_items_.begin(); iterator != tree_items_.end(); ++iterator){
      setTreeItemStatus(iterator.key(), "SUCCESS");
    }
    return;
  }
  if (final_state == "CANCELED") {
    if (!bt_root_name_.isEmpty() && tree_items_.contains(bt_root_name_)){
      setTreeItemStatus(bt_root_name_, "IDLE");
    }
    return;
  }
  if (final_state == "FAILURE" || final_state == "FAULT"){
    if (!bt_root_name_.isEmpty() && tree_items_.contains(bt_root_name_)){
      setTreeItemStatus(bt_root_name_, "FAILURE");
    }
  }
}

void MainWindow::updateHealthIndicator(const QString & component, bool ready, const QString & detail){
  if (!health_value_labels_.contains(component)){
    return;
  }
  health_states_[component] = ready;
  health_details_[component] = detail;
  auto * label = health_value_labels_[component];
  label->setText(ready ? "READY" : "NOT READY");
  label->setToolTip(detail);
  label->setStyleSheet(ready
    ? "background-color: #2e9d4d;" "color: white;" "font-weight: bold;" "padding: 3px;" "border-radius: 3px;"
    : "background-color: #c63d3d;" "color: white;" "font-weight: bold;" "padding: 3px;" "border-radius: 3px;"
  );
  bool all_ready = true;
  for (auto iterator = health_states_.constBegin(); iterator != health_states_.constEnd(); ++iterator){
    if (!iterator.value()) {
      all_ready = false;
      break;
    }
  }
  system_readiness_label_->setText(all_ready ? "Overall readiness: READY" : "Overall readiness: NOT READY");
  system_readiness_label_->setStyleSheet(all_ready
    ? "background-color: #2e9d4d;" "color: white;" "font-weight: bold;" "padding: 5px;"
    : "background-color: #c68a2e;" "color: #202020;" "font-weight: bold;" "padding: 5px;"
  );
  updateRunAvailability();
}

void MainWindow::closeEvent(QCloseEvent * event){
  if (task_active_) {
    const auto answer = QMessageBox::warning(
      this,
      "Task still active",
      "A behavior-tree task is still active.\n\n"
      "Cancel the task and close the UI?",
      QMessageBox::Yes |
      QMessageBox::No,
      QMessageBox::No);
    if (answer != QMessageBox::Yes) {
      event->ignore();
      return;
    }
    ros_bridge_->cancelTask();
  }
  ros_bridge_->setExecutionAllowed(false);
  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  rclcpp::spin_some(node_);
  event->accept();
}

}  // namespace anobot_ui