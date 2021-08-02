# 配置你的ROS2环境

**目标** ：这个教程将向你演示如何去准备ROS2环境。

**教程等级** ：初学者

**用时** ：5分钟

**内容** ：

- ## 背景
    ROS2依赖于使用 shell 环境来组合工作区的概念。“工作区”是一个ROS术语，用来表示ROS2开发在你系统上的位置。
    核心的ROS2工作区称为底层。 后续的本地工作区称为叠加层。
    当进行ROS2开发时，你通常会有多个工作区同时处于激活状态。
    
    组合工作区可以更轻松地针对不同版本的ROS2或针对不同的软件包集进行开发。
    它还允许在同一台计算机上安装多个ROS2发行版(例如Dashing和Eloquent)并在它们之间进行切换。
    
    这是通过在每次打开新shell时获取安装文件，或通过将source命令添加到shell启动脚本一次来实现的。
    如果不获取安装文件，你将无法访问ROS2的命令，也无法找到或使用ROS2的包。 换句话说，你将无法使用ROS2。
    
- ## 先决条件
    在开始这些教程之前，请按照ROS2安装页面上的说明安装ROS2。
    
    本教程中使用的命令假设您遵循了操作系统的二进制包安装指南（Linux的Debian包）。
    如果你是从源代码构建的，你仍然可以继续，但你的安装文件的路径可能会有所不同。
    如果你是从源代码安装的，你也将无法使用```sudo apt install ros-<distro>-<package>```命令（在初学者级别的教程中经常使用）。
    
    如果你使用的是Linux或macOS，但还不熟悉shell，本教程将有所帮助。
    
- ## 任务
    1. 读取并执行setup文件中的shell命令
        你需要在新打开的每个shell上运行这个命令才能访问ROS2命令，如下所示：
        ```bash
        source /opt/ros/galactic/setup.bash
        ```
        > 确切的命令取决于您安装ROS2的位置。如果您遇到问题，请确保文件路径指向您的安装。

    2. 添加sourcing到你的shell启动脚本
        如果你不想在每次打开新shell时都获取安装文件的源代码（跳过任务1），那么你可以将此命令添加到你的shell启动脚本中：
        ```bash
        echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
        ```
     
     3. 添加colcon_cd到你的shell启动脚本
         命令colcon_cd允许你快速将shell当前的工作目录更改为包目录。 
         例如colcon_cd some_ros_package会很快带你到目录~/ros2_install/src/some_ros_package。
         ```bash
         echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
         echo "export _colcon_cd_root=~/ros2_install" >> ~/.bashrc
         ```
         根据你安装colcon_cd的方式和你的工作区所在的位置，上述说明可能会有所不同，请参阅文档了解更多详细信息。 
         要在Linux和macOS中撤消此操作，请找到系统shell的启动脚本并删除附加的source和export命令。
  
     4. 检查环境变量
         读取和执行ROS2安装文件中的shell命令将设置操作ROS2所需的几个必要的环境变量。 
         如果你在查找或使用ROS2包时遇到问题，请确保使用以下命令正确设置你的环境：
         ```bash
         printenv | grep -i ROS
         ```
         检查是否设置了ROS_DISTRO和ROS_VERSION等变量。
         ```bash
         ROS_VERSION=2
         ROS_PYTHON_VERSION=3
         ROS_DISTRO=galactic
         ```
         如果环境变量设置不正确，请返回你遵循的安装指南的ROS2包安装部分。 
         如果你需要更具体的帮助（因为环境设置文件可能来自不同的地方），你可以从社区获得答案。
         
         
 - ## 总结
    使用前需要正确配置ROS2开发环境。 
    这可以通过两种方式完成：在你新打开的每个shell中读并执行安装文件中的shell命令，或者将source命令添加到你的启动脚本中。

    如果你在ROS2中定位或使用包时遇到任何问题，你应该做的第一件事是检查你的环境变量并确保它们设置为你想要的版本和发行版。
    
- ## 下一步
    现在你已经安装了ROS2并且知道如何读取并执行其安装文件中的shell命令，你可以开始使用turtlesim工具学习ROS2的来龙去脉。




