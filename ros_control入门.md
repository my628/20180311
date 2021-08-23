## 概述

ros2_control 是一个为ROS2机器人提供（实时）控制的框架。

它的包是对 ROS（机器人操作系统）中使用的 ros_control 包的重写。

ros2_control 的目标是简化新硬件的集成并克服一些缺点。

如果你不熟悉控制理论，请了解它（例如，在维基百科）以熟悉本手册中使用的术语。



## 入门

ros2_control 框架是为 ROS2 Foxy 发布的。

要使用它，你必须安装 ros-foxy-ros2-control 和 ros-foxy-ros2-controllers 包。

其他依赖项会自动安装。


## 编译

如果要从源代码安装框架，请在 worspace 主文件夹中使用以下命令：

```bash
wget https://raw.githubusercontent.com/ros-controls/ros2_control/master/ros2_control/ros2_control.repos
vcs import src < ros2_control.repos
```



## 架构

ros2_control 框架的源代码可以在 ros2_control 和 ros2_controllers GitHub-repositories 中找到。

下图展示了 ros2_control 框架的架构。

![alt image](https://ros-controls.github.io/control.ros.org/_images/components_architecture.png)



## 控制器管理器

控制器管理器 (CM) 连接 ros2_control 框架的控制器和硬件抽象端。

它还作为用户通过 ROS 服务的入口点。 

CM 实现了一个没有执行器的节点，因此它可以集成到自定义设置中。

尽管如此，对于标准用户，建议使用在 controller_manager 包中的 ros2_control_node 文件中实现的默认节点设置。

本手册假定你使用此默认节点设置。

一方面，CM 管理（例如，加载、激活、停用、卸载）控制器并从中管理所需的接口。

另一方面，它可以访问硬件组件（通过资源管理器），即它们的接口。

控制器管理器匹配所需和提供的接口，在激活时为控制器提供对硬件的访问权限，或者如果存在访问冲突则报告错误。

控制循环的执行由 CM 的 update() 方法管理。

该方法从硬件组件读取数据，更新所有活动控制器的输出，并将结果写入组件。




## 资源管理器

资源管理器 (RM) 为 ros2_control 框架抽象物理硬件及其驱动程序（称为硬件组件）。 

RM 使用 pluginlib-library 加载组件，管理它们的生命周期以及组件的状态和命令接口。

RM 提供的这种抽象使实现的硬件组件（例如机器人和抓手）的可重用性成为可能，而无需任何实现和灵活的状态和命令接口硬件应用程序，例如用于电机控制和编码器读取的单独硬件/通信库。

在控制循环执行中，RM 的 read() 和 write() 方法处理与硬件组件的通信。



## 控制器

ros2_control 框架中的控制器具有与控制理论中定义的相同的功能。

他们将参考值与测量输出进行比较，并根据此误差计算系统的输入（有关更多详细信息，请访问 Wikipedia）。

控件是从 ControllerInterface（ros2_control 中的 controller_interface 包）派生的对象，并使用 pluginlib-library 作为插件导出。

例如控制器检查 ros2_controllers 存储库中的 ForwardCommandController 实现。

控制器的生命周期基于实现状态机的 LifecycleNode-Class，如 Node Lifecycle Design 文档中所述。

执行控制循环时调用 update() 方法。该方法可以访问最新的硬件状态，并使控制器能够编写硬件的命令接口。



## 用户界面

用户使用 Controller Manager 的服务与 ros2_control 框架进行交互。

有关服务及其定义的列表，请检查 controller_manager_msgs 包中的 srv 文件夹。

虽然可以直接从命令行或通过节点使用服务调用，但存在与 ros2 cli 集成的用户友好的命令行界面 (CLI)。这支持自动完成并提供一系列常用命令。

基本命令是 ros2 控制。

有关我们的 CLI 功能的描述，请参阅 ros2controlcli 包的 README.md 文件。



## 硬件组件

硬件组件实现与物理硬件的通信，并在 ros2_control 框架中表示其抽象。

必须使用 pluginlib-library 将组件导出为插件。

资源管理器动态加载这些插件并管理它们的生命周期。

组件分为三种基本类型：

- ### 系统

    复杂（多自由度）机器人硬件，如工业机器人。
    
    Actuator 组件之间的主要区别是可以使用复杂的传输，如类人机器人的手所需。
    
    该组件具有读写功能。
    
    当与硬件（例如 KUKA-RSI）只有一个逻辑通信通道时使用。

- ### 传感器

    机器人硬件用于感知其环境。
    
    传感器组件与关节（例如编码器）或连杆（例如力-扭矩传感器）相关。
    
    此组件类型仅具有读取功能。

- ### 执行器

    简单 (1 DOF) 机器人硬件，如电机、阀门等。
    
    一个执行器实现只与一个关节相关。
    
    此组件类型具有读取和写入功能。
    
    如果不可能，则不强制读取（例如，使用 Arduino 板控制直流电机）。
    
    如果其硬件支持模块化设计，例如与每个电机独立进行 CAN 通信，则执行器类型也可以与多自由度机器人一起使用。


通过控制器设计文档的硬件访问中给出了硬件组件的详细说明。




## URDF 中的硬件描述

ros2_control 框架使用机器人 URDF 文件中的 <ros2_control>-tag 来描述其组件，即硬件设置。

所选择的结构可以将多个 xacro 宏一起跟踪为一个，而无需任何更改。

下面的例子

```
<ros2_control name="RRBotSystemPositionOnly" type="system">
 <hardware>
   <plugin>ros2_control_demo_hardware/RRBotSystemPositionOnlyHardware</plugin>
   <param name="example_param_write_for_sec">2</param>
   <param name="example_param_read_for_sec">2</param>
 </hardware>
 <joint name="joint1">
   <command_interface name="position">
     <param name="min">-1</param>
     <param name="max">1</param>
   </command_interface>
   <state_interface name="position"/>
 </joint>
 <joint name="joint2">
   <command_interface name="position">
     <param name="min">-1</param>
     <param name="max">1</param>
   </command_interface>
   <state_interface name="position"/>
 </joint>
</ros2_control>
<ros2_control name="RRBotForceTorqueSensor1D" type="sensor">
 <hardware>
   <plugin>ros2_control_demo_hardware/ForceTorqueSensor1DHardware</plugin>
   <param name="example_param_read_for_sec">0.43</param>
 </hardware>
 <sensor name="tcp_fts_sensor">
   <state_interface name="force"/>
   <param name="frame_id">rrbot_tcp</param>
   <param name="min_force">-100</param>
   <param name="max_force">100</param>
 </sensor>
</ros2_control>
<ros2_control name="RRBotGripper" type="actuator">
 <hardware>
   <plugin>ros2_control_demo_hardware/PositionActuatorHardware</plugin>
   <param name="example_param_write_for_sec">1.23</param>
   <param name="example_param_read_for_sec">3</param>
 </hardware>
 <joint name="gripper_joint ">
   <command_interface name="position">
     <param name="min">0</param>
     <param name="max">50</param>
   </command_interface>
   <state_interface name="position"/>
   <state_interface name="velocity"/>
 </joint>
</ros2_control>
```



## 为你的机器人运行框架

要运行 ros2_control 框架，请执行以下操作。

示例文件可以在 ros2_control_demos 存储库中找到。

使用控制器管理器和两个控制器的配置创建一个 YAML 文件。（RRBot 的示例配置）

使用所需的 <ros2_control> 标签扩展机器人的 URDF 描述。

建议使用宏文件 (xacro) 而不是纯 URDF。（RRBot 的 URDF 示例）

创建启动文件以使用 Controller Manager 启动节点。

你可以使用默认的 ros2_control 节点（推荐）或将控制器管理器集成到你的软件堆栈中。（RRBot 的示例启动文件）

> 注意：你也可以使用我们的维护人员之一提供的脚本，使用脚本来创建“hardware_interface”包的框架。
