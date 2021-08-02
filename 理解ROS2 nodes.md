# 理解ROS2 nodes

**目标** ：学习ROS2中关于nodes的函数和与它们互动的工具。

**教程等级** ：初学者

**用时** ：10分钟

**内容** ：

- ## 背景
    1. ### ROS2图
    在接下来的的教程，你将学习关于构成所谓ROS2图的一系列核心概念。
    ROS2图是一个ROS2元素们同时处理数据的网络。
    它包含
    如果您要将   所有可执行文件和它们之间的联系绘制出来并进行可视化。
    
    2. ### Nodes in ROS2
    在ROS里每个node的用途,应该负责单一模块(列如一个node为控制轮电机，一个node为激光测距仪等等)。
    每个node能通过主题，服务，动作或参数向其它的node发送和接收数据。
    ![Alt Text](http://docs.ros.org/en/galactic/_images/Nodes-TopicandService.gif) 
    
    一个完整的机器人系统由很多协同工作的node所组成。
    在ROS2中，一个单一可执行文件(C++程序，python程序等等)能容纳一个或多个node。
    
- ## 先决条件
    前面的教程向你展示了如何安装和使用海龟模拟器这个包。

    与往常一样，不要忘记在你打开的每个新终端中读取和执行ROS2设置文件中的shell命令。
    
- ## 任务
    
    1. ROS2运行命令
        ros2 run命令从包中启动可执行文件。
        
        ```bash
        ros2 run <package_name> <executable_name>
        ```
        
        要运行海龟模拟器，请打开一个新终端，然后输入以下命令：
        
        ```bash
        ros2 run turtlesim turtlesim_node
        ```
        
        正如你在上一教程中看到的那样，海龟模拟器窗口将打开。

        这里，包名是turtlesim，可执行文件名是turtlesim_node。

        然而，我们仍然不知道节点名称。 
        你可以使用ros2节点列表查找节点名称。
        
    2. ROS2节点列表命令
    
        ```ros2 node list```将显示所有正在运行的节点的名称。 
        当你想与节点交互时，或者当你的系统运行许多节点并需要跟踪它们时，这尤其有用。

        当海龟模拟器仍在另一个终端中运行时，打开一个新终端然后输入以下命令：
        
        ```bash
        ros2 node list
        ```
        
        终端将返回节点名称：
        
        ```
        /turtlesim
        ```
        
        打开另一个新终端并使用以下命令启动Teleop节点：
        
        ```bash
        ros2 run turtlesim turtle_teleop_key
        ```
        
        在这里，我们再次搜索turtlesim包，这次是搜索名为```turtle_teleop_key```的可执行文件。

        返回你运行```ros2 node list```的终端并再次运行它。 
        你现在将看到两个活动节点的名称：
        
        ```
        /turtlesim
        /teleop_turtle
        ```
        
        1. #### 重映射

            重映射允许你将默认节点属性（如节点名称、主题名称、服务名称等）重新分配给自定义值。 
            在上一个教程中，你使用了```turtle_teleop_key```上的重映射来更改被控制的默认海龟。

            现在，让我们重新分配/turtlesim节点的名称。 
            在新终端中，运行以下命令：

            ```bash
            ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
            ```

            由于你再次在海龟模拟器上调用ros2 run，这将打开另一个海龟模拟器窗口。 
            但是，现在如果你返回运行```ros2 node list```的终端，再次运行它，你将看到三个节点名称：

            ```
            /my_turtle
            /turtlesim
            /teleop_turtle
            ```
            
          
    3. ### ROS2节点信息

        现在你知道节点的名称，你可以通过以下方式访问有关它们的更多信息：
        
        ```bash
        ros2 node info <node_name>
        ```
        
        要检查你的最新节点my_turtle，请运行以下命令：
        
        ```bash
        ros2 node info /my_turtle
        ```
        
        ```ros2 node info```返回与该节点交互的订阅者、发布者、服务和操作（ROS图连接）的列表。 
        输出应如下所示：
        
        ```
        /my_turtle
          Subscribers:
            /parameter_events: rcl_interfaces/msg/ParameterEvent
            /turtle1/cmd_vel: geometry_msgs/msg/Twist
          Publishers:
            /parameter_events: rcl_interfaces/msg/ParameterEvent
            /rosout: rcl_interfaces/msg/Log
            /turtle1/color_sensor: turtlesim/msg/Color
            /turtle1/pose: turtlesim/msg/Pose
          Service Servers:
            /clear: std_srvs/srv/Empty
            /kill: turtlesim/srv/Kill
            /my_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
            /my_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
            /my_turtle/get_parameters: rcl_interfaces/srv/GetParameters
            /my_turtle/list_parameters: rcl_interfaces/srv/ListParameters
            /my_turtle/set_parameters: rcl_interfaces/srv/SetParameters
            /my_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
            /reset: std_srvs/srv/Empty
            /spawn: turtlesim/srv/Spawn
            /turtle1/set_pen: turtlesim/srv/SetPen
            /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
            /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
          Service Clients:

          Action Servers:
            /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
          Action Clients:
        ```

        现在尝试在/teleop_turtle节点上运行相同的命令，看看它的连接与my_turtle有何不同。

        在即将到来的教程中你将了解更多有关ROS图连接概念的信息，包括消息类型。

- ## 总结

    节点是一个基本的ROS2元素，在机器人系统中服务于单一的、模块化的目的。

    在本教程中，你通过运行可执行文件turtlesim_node和turtle_teleop_key来利用从turtlesim包创建的节点。

    你学习了如何使用```ros2 node list```来发现活动节点名称和```ros2 node info```以在单个节点上进行自检。 
    
    这些工具对于理解复杂的、真实的机器人系统中的数据流至关重要。
    
    
- ## 下一步

    现在你了解了ROS2中的节点，你可以继续学习主题教程。 
    
    主题是连接节点的通信类型之一。
    
    
- ## 相关内容 

    [概念]页为节点的概念加入了更多细节。
    
    
    
    
    
