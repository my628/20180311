# 创建 MoveIt 插件

本页面详细介绍了如何在 ROS 中添加插件。
两个必要的元素是基类和插件类。
插件类继承自基类并覆盖其虚函数。
用于此目的的主要库是 pluginlib。
本教程包含三种不同类型的插件，
即运动规划器、控制器管理器和约束采样器。



## 运动规划器插件

在本节中，我们将展示如何将新的运动规划器作为插件添加到 MoveIt。
MoveIt 中的基类是planning_interface，任何新的规划器插件都应该从它继承。
出于演示目的，创建了一个线性插值规划器 (lerp)，它规划关节空间中两个状态之间的运动。
此规划器可用作添加任何新规划器的起点，因为它包含必要的基础知识。
本教程中设计的最终源文件可在此处获得。
下图显示了在 MoveIt 中添加新计划器的类之间关系的简要总体视图。

![alt image](http://moveit2_tutorials.picknik.ai/_images/lerp_planner.png)

首先我们在 moveit_tutorials 包中创建插件类。
要为 lerp 制作插件类，请在 src 文件夹中创建一个名为 lerp_planner_manager.cpp 的文件。
在这个文件中，LERPPlanPlannerManager 覆盖了来自planning_interface 的PlannerManager 类的功能。
在这个文件的最后，我们需要注册 LERPPlanPlannerManager 类作为插件，这是通过 class_loader 中的 CLASS_LOADER_REGISTER_CLASS 宏完成的：

```c++
CLASS_LOADER_REGISTER_CLASS(emptyplan_interface::EmptyPlanPlannerManager, planning_interface::PlannerManager);
```

接下来，我们创建覆盖 PlanningContext 功能的 LERPPlanningContext 类。该类将覆盖规划器解决问题并返回解决方案的解决函数。由于解决函数的实现可能需要规划器代码库中的许多类，因此创建另一个名为 LERPInterface 的类更具可读性，规划器解决方法的实际实现将在其中发生。基本上，这个类是新的运动规划器算法的入口点。此求解函数中的响应以 moveit_msgs::MotionPlanDetailedResponse 类型准备，该类型转换为 LERPPlanningContext 类中的 Planning_interface::MotionPlanDetailedResponse。

此外，PlannerConfigurationSettings 可用于传递特定于规划器的参数。传递这些参数的另一种方法是使用从 yaml 文件读取的 ROS 参数服务器。在本教程中，对于我们的 lerp 规划器，我们使用 panda_moveit_config 包中的 lerp_planning.yaml，该包仅包含一个参数 num_steps，每当调用它的求解函数时，都会在 lerp_interface 中加载该参数。



### 导出插件

首先，我们需要使插件可用于 ROS 工具链。
为此，应创建一个插件描述 xml 文件 (emptyplan_interface_plugin_description.xml)，其中包含具有以下选项的库标记：

```xml
<library  path="libmoveit_emptyplan_planner_plugin">
  <class name="emptyplan_interface/EmptyPlanPlanner" type="emptyplan_interface::EmptyPlanPlannerManager" base_class_type="planning_interface::PlannerManager">
   <description>
   </description>
  </class>
</library>
```

然后，为了导出插件，我们使用上述xml文件的地址和package.xml文件中的export标签：

```xml
<export>
   <moveit_core plugin="${prefix}/emptyplan_interface_plugin_description.xml"/>
</export>
```

请注意，标记的名称 moveit_core 与基类 Planning_interface 所在的包的名称相同。



### 检查插件

使用以下命令，可以验证新插件是否正确创建和导出：

```bash
rospack plugins --attrib=plugin moveit_core
```

结果应该包含带有插件描述xml文件地址的moveit_planners_lerp：

```
moveit_tutorials <ros_workspace>/src/moveit_tutorials/creating_moveit_plugins/lerp_motion_planner/lerp_interface_plugin_description.xml
```

### 插件使用

在本小节中，我们将解释如何加载和使用我们之前创建的 lerp 规划器。 
为此，创建了一个名为 lerp_example.cpp 的 ros 节点。 
第一步是通过以下几行代码获取与请求的规划组以及规划场景相关的机器人的状态和关节组：

```c++
moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
```

下一步是使用 pluginlib 加载规划器并将参数 planner_plugin_name 设置为我们创建的参数：

```c++
boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
planning_interface::PlannerManagerPtr planner_instance;
std::string planner_plugin_name =  "lerp_interface/LERPPlanner";
node_handle.setParam("planning_plugin", planner_plugin_name);
```

加载规划器后，是时候为运动规划问题设置开始和目标状态了。 
启动状态是机器人的当前状态，设置为 req.start_state。 
另一方面，目标约束是通过使用目标状态和联合模型组创建一个 moveit_msgs::Constraints 来设置的。 
这个约束被提供给 req.goal_constraint。 以下代码显示了如何执行这些步骤：

```c++
// Get the joint values of the start state and set them in request.start_state
std::vector<double> start_joint_values;
robot_state->copyJointGroupPositions(joint_model_group, start_joint_values);
req.start_state.joint_state.position = start_joint_values;

// Goal constraint
moveit::core::RobotState goal_state(robot_model);
std::vector<double> joint_values = { 0.8, 0.7, 1, 1.3, 1.9, 2.2, 3 };
goal_state.setJointGroupPositions(joint_model_group, joint_values);
moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
req.goal_constraints.clear();
req.goal_constraints.push_back(joint_goal);
```

到目前为止，我们已经加载了规划器并为运动规划问题创建了开始和目标状态，但我们还没有解决问题。
通过关于开始和目标状态的给定信息解决关节状态下的运动规划问题是通过创建 PlanningContext 实例并调用其解决函数来完成的。
请记住，传递给这个求解函数的响应应该是planning_interface::MotionPlanResponse 类型：

```c++
planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
```

最后，为了运行这个节点，我们需要在启动文件夹中 roslaunch lerp_example.launch。
这个启动文件通过传递 lerp 作为规划器的名称来启动包 panda_moveit_config 的 demo.launch。
然后，启动 lerp_example 并加载 lerp_planning.yaml 以将特定于 lerp 的参数设置为 ROS 参数服务器。




## 示例控制器管理器插件

MoveIt 控制器管理器，有点用词不当，是你自定义低级控制器的接口。
考虑它们的更好方法是控制器接口。
对于大多数用例，如果你的机器人控制器已经为 FollowJointTrajectory 提供 ROS 操作，则包含的 MoveItSimpleControllerManager 就足够了。
如果你使用 ros_control，则包含的 MoveItRosControlInterface 也是理想的选择。

但是，对于某些应用程序，你可能需要更自定义的控制器管理器。此处提供了用于启动自定义控制器管理器的示例模板。



## 示例约束采样器插件

- 创建一个 ROBOT_moveit_plugins 包，并在其中为你的 ROBOT_constraint_sampler 插件创建一个子文件夹。
  修改ROBOT_moveit_plugins/ROBOT_moveit_constraint_sampler_plugin提供的模板

- 在 ROBOT_moveit_config/launch/move_group.launch 文件中的 <node name="move_group"> 中，添加参数：
  
  ```xml
  <param name="constraint_samplers" value="ROBOT_moveit_constraint_sampler/ROBOTConstraintSamplerAllocator"/>
  ```
  
- 现在，当你启动 move_group 时，它应该默认为您的新约束采样器。
