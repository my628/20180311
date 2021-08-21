# 扩展 ROS 2 接口



**目标：** 了解更多在 ROS 2 中实现自定义接口的方法

**教程级别：** 初学者

**时间：** 15分钟



## 背景

在之前的教程中，你学习了如何创建自定义 msg 和 srv 接口。

虽然最佳实践是在专用接口包中声明接口，但有时在一个包中声明、创建和使用接口会很方便。

回想一下，接口目前只能在 CMake 包中定义。
但是，可以在 CMake 包中包含 Python 库和节点（使用 ament_cmake_python），因此你可以在一个包中一起定义接口和 Python 节点。
为简单起见，我们将在此处使用 CMake 包和 C++ 节点。

本教程将重点介绍 msg 接口类型，但此处的步骤适用于所有接口类型。



## 先决条件

我们假设你在完成本教程之前已经查看了创建自定义 ROS 2 msg 和 srv 文件教程中的基础知识。

你应该已经安装了 ROS 2、一个工作区并了解创建包。

与往常一样，不要忘记在你打开的每个新终端中使用 ROS 2。




## 任务

1. ### 创建一个包

    在你的工作区 src 目录中，创建一个包 more_interfaces 并在其中为 msg 文件创建一个文件夹：
    
    ```bash
    ros2 pkg create --build-type ament_cmake more_interfaces
    mkdir more_interfaces/msg
    ```
