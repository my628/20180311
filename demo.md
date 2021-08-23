

## 目标

存储库有三个目标：

1. 实现 ros-controls/roadmap 存储库文件 components_architecture_and_urdf_examples 中描述的示例配置。
2. 它提供模板以更快地实现自定义硬件和控制器；
3. 存储库是 ros2_control 概念的验证环境，只能在运行时进行测试（例如，控制器管理器执行控制器，机器人硬件和控制器之间的通信）。



## 说明

该存储库的灵感来自 Dave Coleman 的 ros_control_boilerplate 存储库。

根据机器人ROS包的通常结构，示例分为三个部分/包：

1. 启动包 ros2_control_demo_bringup 包含演示机器人的启动文件和运行时配置。
2. 描述包 rrbot_description 和 diffbot_description（在 ros2_control_demo_description 内），存储演示机器人的 URDF 描述文件、rviz 配置和网格。
3. 硬件接口包ros2_control_demo_hardware，实现了路线图中描述的硬件接口。

RRBot 和 DiffBot 的示例是简单的模拟，用于演示和测试 ros2_control 概念。

除了 ros2 核心包之外，此包没有任何依赖项，因此可以在 SoC 硬件或无头系统上使用。

该存储库演示了以下 ros2_control 概念：

- 为系统、传感器和执行器创建 HardwareInterface。
- 以 URDF 文件的形式创建机器人描述。
- 使用启动文件加载配置并启动机器人。
- 控制差分移动基地 DiffBot。
- 控制 RRBot 的两个关节。
- 使用模拟机器人并使用 Gazebo 模拟器启动 ros2_control。
- 为机器人实现控制器切换策略。
- 在 ros2_control 中使用关节限制和传输概念。



## 快速提示

这些是一些快速提示，特别是对于那些来自 ROS1 控制背景的人：

1. 现在有三类硬件组件：传感器、执行器和系统。

    Sensor 用于单个传感器； Actuator 用于单个执行器；系统适用于多个传感器/执行器的任意组合。

    你可以将 Sensor 视为只读。

    所有组件都用作插件，因此使用 ```PLUGINLIB_EXPORT_CLASS``` 宏导出。

2. ros_control 只允许三种硬件接口类型：位置、速度和作用力。

    ros2_control 允许你通过定义自定义字符串来创建任何接口类型。

    例如，你可以定义 position_in_degrees 或温度接口。

    最常见的（位置、速度、加速度、作用力）已经在 hardware_interface/types/hardware_interface_type_values.hpp 中定义为常量。

URDF 中 <ros2_control> 标签中的关节名称必须与控制器的配置兼容。

在 ros2_control 中，驱动程序的所有参数都在 URDF 中指定。

ros2_control 框架使用 URDF 中的 <ros2_control> 标签。

URDF 中 <ros2_control> 标签中的关节名称必须与控制器的配置兼容。

从源代码构建
