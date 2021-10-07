![image alt](http://moveit2_tutorials.picknik.ai/_images/move_group_interface_tutorial_start_screen.png)

在 MoveIt 中，最简单的用户界面是通过 MoveGroupInterface 类。 

它为用户可能想要执行的大多数操作提供了易于使用的功能，特别是设置关节或姿势目标、创建运动计划、移动机器人、将对象添加到环境中以及从机器人附加/分离对象。

该接口通过 ROS 主题、服务和操作与 MoveGroup 节点进行通信。

观看此快速 YouTube 视频演示，了解移动组界面的强大功能！

# 入门

如果你还没有这样做，请确保你已完成入门中的步骤。

> 注意：由于 MoveitVisualTools 尚未移植到 ROS2，因此本教程使用了 xterm 和一个简单的提示器来帮助用户完成每个演示步骤。

要安装 xterm，请运行以下命令：

```bash
sudo apt-get install -y xterm
```

# 运行代码

打开两个shell。

在第一个 shell 中启动 RViz 并等待一切完成加载：

```bash
ros2 launch moveit2_tutorials move_group.launch.py
```

在第二个 shell 中，运行启动文件：

```bash
ros2 launch moveit2_tutorials move_group_interface_tutorial.launch.py
```

> 注意：RvizVisualToolsGui 面板尚未移植到 ROS2 本教程使用 RvizVisualToolsGui 面板逐步完成演示。
> 要将此面板添加到 RViz，请按照可视化教程中的说明进行操作。

片刻之后，RViz 窗口应该会出现，看起来与此页面顶部的类似。

要完成每个演示步骤，请按屏幕底部 RvizVisualToolsGui 面板中的下一步按钮或选择屏幕顶部工具面板中的关键工具，然后在 RViz 聚焦时按键盘上的 N。

# 预期产出

有关预期输出，请参阅本教程顶部的 YouTube 视频。

在RViz中，我们应该能够看到以下内容：

1. 机器人将其手臂移动到其前方的姿势目标。
2. 机器人将其手臂移动到其侧面的关节目标。
3. 机器人将其手臂移回新的姿势目标，同时保持末端执行器水平。
4. 机器人沿着所需的笛卡尔路径（一个三角形向下、向右、向上+向左）移动其手臂。
5. 一个盒子对象被添加到手臂右侧的环境中。
6. 机器人将其手臂移动到姿势目标，避免与盒子发生碰撞。
7. 物体附着在手腕上（它的颜色会变成紫色/橙色/绿色）。
8. 物体与手腕分离（其颜色将变回绿色）。
9. 对象从环境中移除。

![image alt](http://moveit2_tutorials.picknik.ai/_images/move_group_interface_tutorial_robot_with_box.png)

# 整个代码

整个代码可以在 MoveIt GitHub 项目中看到。

接下来，我们逐条逐句逐句地解释其功能。

# 设置

MoveIt 对称为“规划组”的关节集进行操作，并将它们存储在称为 JointModelGroup 的对象中。

在整个 MoveIt 中，术语“计划组”和“关节模型组”可互换使用。

```c++
static const std::string PLANNING_GROUP = "panda_arm";
```

只需使用你想要控制和计划的计划组的名称，即可轻松设置 MoveGroupInterface 类。

```c++
moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
```

我们将使用 PlanningSceneInterface 类在我们的“虚拟世界”场景中添加和删除碰撞对象

```c++
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
```

原始指针经常用于引用计划组以提高性能。

```c++
const moveit::core::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
```

# 可视化

MoveitVisualTools 尚未移植到 ROS2，因此我们使用 RvizVisualTools 进行可视化。

MoveItVisualTools 包提供了许多功能，用于在 RViz 中可视化对象、机器人和轨迹以及调试工具，例如脚本的逐步内省。

```c++
namespace rvt = rviz_visual_tools;
rviz_visual_tools::RvizVisualTools visual_tools("panda_link0", "move_group_tutorial", move_group_node);
/* moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0"); */
visual_tools.deleteAllMarkers();

/* Remote control is an introspection tool that allows users to step through a high level script */
/* via buttons and keyboard shortcuts in RViz */
/* visual_tools.loadRemoteControl(); */
```

RViz 提供了多种类型的标记，在这个演示中我们将使用文本、圆柱体和球体

```c++
Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
text_pose.translation().z() = 1.0;
visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);
```

批量发布用于减少发送到 RViz 进行大型可视化的消息数量

```c++
visual_tools.trigger();
```

# 获取基本信息

我们可以打印这个机器人的坐标系名称。

```c++
RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
```

我们还可以打印该组的末端执行器链接的名称。

```c++
RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
```

我们可以得到机器人中所有组的列表：

```c++
RCLCPP_INFO(LOGGER, "Available Planning Groups:");
std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
          std::ostream_iterator<std::string>(std::cout, ", "));
```

## 开始演示

```c++
prompt("Press 'Enter' to start the demo");
/* visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo"); */
```

## 计划到一个姿势目标

我们可以将该组的运动规划为末端执行器所需的姿势。

```c++
geometry_msgs::msg::Pose target_pose1;
target_pose1.orientation.w = 1.0;
target_pose1.position.x = 0.28;
target_pose1.position.y = -0.2;
target_pose1.position.z = 0.5;
move_group.setPoseTarget(target_pose1);
```

现在，我们调用计划器来计算计划并将其可视化。

请注意，我们只是在计划，而不是要求 move_group 实际移动机器人。

```c++
moveit::planning_interface::MoveGroupInterface::Plan my_plan;

bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
```

## 可视化计划

我们还可以将计划可视化为 RViz 中带有标记的线。

```c++
RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
visual_tools.publishAxisLabeled(target_pose1, "pose1");
visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
/* visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group); */
visual_tools.trigger();
prompt("Press 'Enter' to continue the demo");
/* visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo"); */
```

## 移动到姿势目标

移动到姿势目标与上面的步骤类似，除了我们现在使用 move() 函数。

请注意，我们之前设置的姿势目标仍然处于活动状态，因此机器人将尝试移动到该目标。

我们不会在本教程中使用该函数，因为它是一个阻塞函数，需要一个控制器处于活动状态并报告轨迹执行成功。

```c++
/* Uncomment below line when working with a real robot */
/* move_group.move(); */
```

## 规划联合空间目标

让我们设定一个共同的空间目标并朝着它前进。

这将替换我们上面设置的姿势目标。

首先，我们将创建一个引用当前机器人状态的指针。

RobotState 是包含所有当前位置/速度/加速度数据的对象。

```c++
moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
```

接下来获取组的当前关节值集。

```c++
std::vector<double> joint_group_positions;
current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
```

现在，让我们修改其中一个关节，规划新的关节空间目标并可视化该计划。

```c++
joint_group_positions[0] = -1.0;  // radians
move_group.setJointValueTarget(joint_group_positions);
```

我们将允许的最大速度和加速度降低到其最大值的 5%。

默认值为 10% (0.1)。

在机器人的 moveit_config 的 joint_limits.yaml 文件中设置你的首选默认值，或者如果您需要机器人移动得更快，则在你的代码中设置显式因素。

```c++
  move_group.setMaxVelocityScalingFactor(0.05);
  move_group.setMaxAccelerationScalingFactor(0.05);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");


Visualize the plan in RViz
```

```c++
visual_tools.deleteAllMarkers();
visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
/* visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group); */
visual_tools.trigger();
prompt("Press 'Enter' to continue the demo");
/* visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo"); */
```

# 使用路径约束进行规划

可以轻松地为机器人上的链接指定路径约束。

让我们为我们的组指定一个路径约束和一个姿势目标。

首先定义路径约束。

```c++
moveit_msgs::msg::OrientationConstraint ocm;
ocm.link_name = "panda_link7";
ocm.header.frame_id = "panda_link0";
ocm.orientation.w = 1.0;
ocm.absolute_x_axis_tolerance = 0.1;
ocm.absolute_y_axis_tolerance = 0.1;
ocm.absolute_z_axis_tolerance = 0.1;
ocm.weight = 1.0;
```

现在，将其设置为组的路径约束。

```c++
moveit_msgs::msg::Constraints test_constraints;
test_constraints.orientation_constraints.push_back(ocm);
move_group.setPathConstraints(test_constraints);
```

## 在关节空间执行规划

根据规划问题，MoveIt 在关节空间和笛卡尔空间之间选择问题表示。

在ompl_planning.yaml 文件中设置组参数enforce_joint_model_state_space:true 强制所有计划使用关节空间。

默认情况下，在笛卡尔空间中对具有方向路径约束的规划请求进行采样，以便调用 IK 作为生成采样器。

通过强制关节空间，规划过程将使用拒绝抽样来查找有效请求。

请注意，这可能会大大增加计划时间。

我们将重用我们已有的旧目标并计划实现它。

请注意，这仅在当前状态已经满足路径约束时才有效。

所以我们需要将开始状态设置为一个新的姿势。

```c++
moveit::core::RobotState start_state(*move_group.getCurrentState());
geometry_msgs::msg::Pose start_pose2;
start_pose2.orientation.w = 1.0;
start_pose2.position.x = 0.55;
start_pose2.position.y = -0.05;
start_pose2.position.z = 0.8;
start_state.setFromIK(joint_model_group, start_pose2);
move_group.setStartState(start_state);
```

现在我们将从我们刚刚创建的新开始状态计划到较早的姿势目标。

```c++
move_group.setPoseTarget(target_pose1);
```

使用约束进行规划可能会很慢，因为每个样本都必须调用逆运动学求解器。

让我们将计划时间从默认的 5 秒增加，以确保计划者有足够的时间成功。

```c++
move_group.setPlanningTime(10.0);

success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
RCLCPP_INFO(LOGGER, "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");
```

在 RViz 中可视化计划

```c++
visual_tools.deleteAllMarkers();
visual_tools.publishAxisLabeled(start_pose2, "start");
visual_tools.publishAxisLabeled(target_pose1, "goal");
visual_tools.publishText(text_pose, "Constrained_Goal", rvt::WHITE, rvt::XLARGE);
/* visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group); */
visual_tools.trigger();
prompt("Press 'Enter' to continue the demo");
/* visual_tools.prompt("next step"); */
```

完成路径约束后，请务必清除它。

```c++
move_group.clearPathConstraints();
```

## 笛卡尔路径

你可以通过指定末端执行器要经过的路标列表来直接规划笛卡尔路径。

请注意，我们是从上面的新开始状态开始的。

初始姿势（开始状态）不需要添加到航点列表，但添加它可以帮助可视化

```c++
std::vector<geometry_msgs::msg::Pose> waypoints;
waypoints.push_back(start_pose2);

geometry_msgs::msg::Pose target_pose3 = start_pose2;

target_pose3.position.z -= 0.2;
waypoints.push_back(target_pose3);  // down

target_pose3.position.y -= 0.2;
waypoints.push_back(target_pose3);  // right

target_pose3.position.z += 0.2;
target_pose3.position.y += 0.2;
target_pose3.position.x -= 0.2;
waypoints.push_back(target_pose3);  // up and left
```

我们希望笛卡尔路径以 1 厘米的分辨率进行插值，这就是为什么我们将指定 0.01 作为笛卡尔平移的最大步长。 
我们将跳跃阈值指定为 0.0，从而有效地禁用它。
警告 - 在操作真实硬件时禁用跳跃阈值会导致冗余关节发生大量不可预测的运动，并且可能是一个安全问题

```c++
moveit_msgs::msg::RobotTrajectory trajectory;
const double jump_threshold = 0.0;
const double eef_step = 0.01;
double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
```

在 RViz 中可视化计划

```c++
visual_tools.deleteAllMarkers();
visual_tools.publishText(text_pose, "Cartesian_Path", rvt::WHITE, rvt::XLARGE);
visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
for (std::size_t i = 0; i < waypoints.size(); ++i)
  visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
visual_tools.trigger();
prompt("Press 'Enter' to continue the demo");
/* visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo"); */
```

笛卡尔运动通常应该很慢，例如 接近物体时。

笛卡尔计划的速度目前无法通过 maxVelocityScalingFactor 设置，但需要你手动为轨迹计时，如此处所述。

欢迎拉取请求。

你可以执行这样的轨迹。

```c++
/* move_group.execute(trajectory); */
```

## 将对象添加到环境中

首先让我们计划另一个没有任何障碍物的简单目标。

```c++
move_group.setStartState(*move_group.getCurrentState());
geometry_msgs::msg::Pose another_pose;
another_pose.orientation.w = 0;
another_pose.orientation.x = -1.0;
another_pose.position.x = 0.7;
another_pose.position.y = 0.0;
another_pose.position.z = 0.59;
move_group.setPoseTarget(another_pose);

success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
RCLCPP_INFO(LOGGER, "Visualizing plan 5 (with no obstacles) %s", success ? "" : "FAILED");

visual_tools.deleteAllMarkers();
visual_tools.publishText(text_pose, "Clear_Goal", rvt::WHITE, rvt::XLARGE);
visual_tools.publishAxisLabeled(another_pose, "goal");
/* visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group); */
visual_tools.trigger();
prompt("Press 'Enter' to continue the demo");
/* visual_tools.prompt("next step"); */
```

结果可能如下所示：

![image alt](http://moveit2_tutorials.picknik.ai/_images/move_group_interface_tutorial_clear_path.gif)

现在让我们为机器人定义一个碰撞对象 ROS 消息来避免。

```c++
moveit_msgs::msg::CollisionObject collision_object;
collision_object.header.frame_id = move_group.getPlanningFrame();
```

对象的 id 用于标识它。

```c++
collision_object.id = "box1";
```

定义要添加到世界的框。

```c++
shape_msgs::msg::SolidPrimitive primitive;
primitive.type = primitive.BOX;
primitive.dimensions.resize(3);
primitive.dimensions[primitive.BOX_X] = 0.1;
primitive.dimensions[primitive.BOX_Y] = 1.5;
primitive.dimensions[primitive.BOX_Z] = 0.5;
```

定义框的姿势（相对于 frame_id 指定）

```c++
geometry_msgs::msg::Pose box_pose;
box_pose.orientation.w = 1.0;
box_pose.position.x = 0.48;
box_pose.position.y = 0.0;
box_pose.position.z = 0.25;

collision_object.primitives.push_back(primitive);
collision_object.primitive_poses.push_back(box_pose);
collision_object.operation = collision_object.ADD;

std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
collision_objects.push_back(collision_object);
```

现在，让我们将碰撞对象添加到世界中（使用可以包含其他对象的向量）

```c++
RCLCPP_INFO(LOGGER, "Add an object into the world");
planning_scene_interface.addCollisionObjects(collision_objects);
```

在状态的RViz中显示文本并等待MoveGroup接收并处理碰撞对象消息

```c++
visual_tools.publishText(text_pose, "Add_object", rvt::WHITE, rvt::XLARGE);
visual_tools.trigger();
prompt("Press 'Enter' to continue once the collision object appears in RViz");
/* visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz"); */
```

现在，当我们计划一条轨迹时，它将避开障碍物

```c++
success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
RCLCPP_INFO(LOGGER, "Visualizing plan 6 (pose goal move around cuboid) %s", success ? "" : "FAILED");
visual_tools.publishText(text_pose, "Obstacle_Goal", rvt::WHITE, rvt::XLARGE);
/* visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group); */
visual_tools.trigger();
prompt("Press 'Enter' to continue once the plan is complete");
/* visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete"); */
```

结果可能如下所示：

![image alt](http://moveit2_tutorials.picknik.ai/_images/move_group_interface_tutorial_avoid_path.gif)


## 将物体连接到机器人

你可以将对象附加到机器人，使其随机器人几何体移动。

这模拟了为了操纵物体而拾取物体。

运动规划也应避免两个对象之间的碰撞。

```c++
moveit_msgs::msg::CollisionObject object_to_attach;
object_to_attach.id = "cylinder1";

shape_msgs::msg::SolidPrimitive cylinder_primitive;
cylinder_primitive.type = primitive.CYLINDER;
cylinder_primitive.dimensions.resize(2);
cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;
```

我们为这个圆柱体定义了框架/姿势，以便它出现在夹具中

```c++
object_to_attach.header.frame_id = move_group.getEndEffectorLink();
geometry_msgs::msg::Pose grab_pose;
grab_pose.orientation.w = 1.0;
grab_pose.position.z = 0.2;
```

首先，我们将对象添加到世界中（不使用向量）

```c++
object_to_attach.primitives.push_back(cylinder_primitive);
object_to_attach.primitive_poses.push_back(grab_pose);
object_to_attach.operation = object_to_attach.ADD;
planning_scene_interface.applyCollisionObject(object_to_attach);
```

然后，我们将物体“附加”到机器人上。

它使用 frame_id 来确定它连接到哪个机器人链接，我们还需要告诉 MoveIt 允许对象与抓手的手指链接发生碰撞。

你还可以使用 applyAttachedCollisionObject 将对象直接附加到机器人。

```c++
RCLCPP_INFO(LOGGER, "Attach the object to the robot");
std::vector<std::string> touch_links;
touch_links.push_back("panda_rightfinger");
touch_links.push_back("panda_leftfinger");
move_group.attachObject(object_to_attach.id, "panda_hand", touch_links);

visual_tools.publishText(text_pose, "Object_attached_to_robot", rvt::WHITE, rvt::XLARGE);
visual_tools.trigger();

/* Wait for MoveGroup to receive and process the attached collision object message */
prompt("Press 'Enter' once the collision object attaches to the robot");
/* visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot"); */
```

重新计划，但现在手头有对象。

```c++
move_group.setStartStateToCurrentState();
success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
RCLCPP_INFO(LOGGER, "Visualizing plan 7 (move around cuboid with cylinder) %s", success ? "" : "FAILED");
/* visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group); */
visual_tools.trigger();
prompt("Press 'Enter' once the plan is complete");
/* visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete"); */
```

结果可能如下所示：

![image alt](http://moveit2_tutorials.picknik.ai/_images/move_group_interface_tutorial_attached_object.gif)

## 分离和移除对象

现在，让我们从机器人的抓手上拆下气缸。

```c++
RCLCPP_INFO(LOGGER, "Detach the object from the robot");
move_group.detachObject(object_to_attach.id);
```

在状态的 RViz 中显示文本

```c++
visual_tools.deleteAllMarkers();
visual_tools.publishText(text_pose, "Object_detached_from_robot", rvt::WHITE, rvt::XLARGE);
visual_tools.trigger();

/* Wait for MoveGroup to receive and process the attached collision object message */
prompt("Press 'Enter' once the collision object detaches from the robot");
/* visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot"); */
```

现在，让我们从世界中移除对象。

```c++
RCLCPP_INFO(LOGGER, "Remove the objects from the world");
std::vector<std::string> object_ids;
object_ids.push_back(collision_object.id);
object_ids.push_back(object_to_attach.id);
planning_scene_interface.removeCollisionObjects(object_ids);
```

在状态的 RViz 中显示文本

```c++
visual_tools.publishText(text_pose, "Objects_removed", rvt::WHITE, rvt::XLARGE);
visual_tools.trigger();

/* Wait for MoveGroup to receive and process the attached collision object message */
prompt("Press 'Enter' once the collision object disappears");
/* visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears"); */
```

## 启动文件

整个启动文件在 GitHub 上。

本教程中的所有代码都可以从作为 MoveIt 设置一部分的 moveit_tutorials 包运行。

## 关于设置公差的注意事项

请注意，MoveGroupInterface 的 setGoalTolerance() 和相关方法设置了计划的容差，而不是执行的容差。

如果要配置执行容差，如果使用 FollowJointTrajectory 控制器，则必须编辑 controller.yaml 文件，或者手动将其添加到计划器生成的轨迹消息中。
