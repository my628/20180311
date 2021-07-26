# 开始使用ros2doctor



**目标：** 使用ros2doctor工具识别ROS2设置中的问题。

**教程级别：** 初学者

**时间：** 10分钟

**内容**



- ## 背景

    当你的ROS2设置未按预期运行时，你可以使用ros2doctor工具检查其设置。

    ros2doctor检查ROS2的各个方面，包括平台、版本、网络、环境、运行系统等，并警告您可能的错误和问题的原因。

- ## 先决条件

    ros2doctor是ros2cli包的一部分。 
    只要你安装了ros2cli（任何正常安装都应该有），你就可以使用ros2doctor。

    本教程使用海龟模拟器来说明一些示例。

- ## 任务

    1. ### 检查你的设置
    
        让我们使用ros2doctor来检查你的一般ROS2设置。
        首先，在新终端中输入ROS2，然后输入命令：
        
        ```
        ros2 doctor
        ```
        
        这将对你的所有设置模块进行检查并返回警告和错误。

        如果你的ROS2设置处于完美状态，你将看到类似于以下内容的消息：
        
        ```
        All <n> checks passed
        ```
        
        但是，返回一些警告并不罕见。 
        UserWarning并不意味着你的设置不可用； 
        这更有可能只是表明某些东西的配置方式并不理想。

        如果你确实收到警告，它将如下所示：
        
        ```
        <path>: <line>: UserWarning: <message>
        ```
        
        例如，如果你使用不稳定的ROS2发行版，ros2doctor会发现此警告：
        
        ```
        UserWarning: Distribution <distro> is not fully supported or tested. To get more consistent features, download a stable version at 
        https://index.ros.org/doc/ros2/Installation/
        ```
        
        如果```ros2doctor```只在你的系统中发现警告，
        你仍然会收到所有 <n> 检查已通过的消息。

        大多数检查被归类为警告而不是错误。
        ```ros2doctor```返回的反馈的重要性主要取决于你，用户。
        如果它确实在你的设置中发现了一个罕见的错误，由```UserWarning: ERROR:```指示，则检查被视为失败。
  
        你将在问题反馈列表后看到一条与此类似的消息：
  
        ```
        1/3 checks failed

        Failed modules:  network
        ```
  
        错误表示系统缺少对ROS2至关重要的重要设置或功能。
        应解决错误以确保系统正常运行。

  
  
  2. ### 检查系统

    你还可以检查正在运行的ROS2系统以确定问题的可能原因。
    要查看```ros2doctor```在正在运行的系统上工作，让我们运行海龟模拟器，它的节点之间积极地相互通信。

    通过打开一个新终端，采购ROS2并输入命令来启动系统：
  
    ```bash
    ros2 run turtlesim turtlesim_node
    ```
  
    打开另一个终端并获取ROS2以运行Teleop控件：
  
    ```bash
    ros2 run turtlesim turtle_teleop_key
    ```
  
    现在在它自己的终端中再次运行```ros2doctor```。
    你将看到上次在设置中运行```ros2doctor```时出现的警告和错误（如果有的话）。
    接下来是一些与系统本身相关的新警告：
  
    ```
    UserWarning: Publisher without subscriber detected on /turtle1/color_sensor.
    UserWarning: Publisher without subscriber detected on /turtle1/pose.
    ```
