# 理解ROS2动作



**目标** ：内检ROS2中的动作。

**教程级别** ：初学者

**时间** ： 15 分钟

**内容**



- ## 背景

    动作是ROS2中的一种通信类型，用于长时间运行的任务。
    它们由三部分组成：目标、反馈和结果。

    操作建立在主题和服务上。
    它们的功能类似于服务，但可以取消操作。
    它们还提供稳定的反馈，而不是返回单一响应的服务。

    动作使用客户端-服务器模型，类似于发布者-订阅者模型（在主题教程```<ROS2Topics>```中描述）。
    “动作客户端”节点向“动作服务器”节点发送目标，该节点确认目标并返回反馈流和结果。

    ![图片alt](http://docs.ros.org/en/galactic/_images/Action-SingleActionClient.gif)


- ## 先决条件

    本教程基于之前的教程中介绍的概念，例如ROS2 Nodes和ROS2 Topics。

    本教程使用海龟模拟器包。

    与往常一样，不要忘记在你打开的每个新终端读取并执行ROS2设置文件中的shell命令。

    
    
- ## 任务

    1. ### 设置

        启动两个turtlesim节点，```/turtlesim```和```/teleop_turtle```。

        打开一个新终端并运行：

        ```bash
        ros2 run turtlesim turtlesim_node
        ```

        打开另一个终端并运行：

        ```bash
        ros2 run turtlesim turtle_teleop_key
        ```

    2. ### 使用actions

        当你启动```/teleop_turtle```节点时，你将在终端中看到以下消息：

        ```
        Use arrow keys to move the turtle.
        Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.
        ```
    
        让我们关注第二行，它对应于一个动作。（第一条指令对应于“cmd_vel”主题，之前在教程[ROS2 Topics]中讨论过。）

        请注意，字母键“G|B|V|C|D|E|R|T”在美国QWERTY键盘上的“F”键周围形成一个“框”（如果你不使用QWERTY键盘，请参阅[此链接](https://en.wikipedia.org/wiki/QWERTY#/media/File:KB_United_States.svg)以继续操作）。
        F周围的每个键的位置对应于turtlesim中的方向。
        例如，“E”会将海龟的方向旋转到左上角。

        注意```/turtlesim```节点运行的终端。
        每次按下这些键之一时，都会向作为```/turtlesim```节点一部分的动作服务器发送一个目标。
        目标是旋转海龟以面向特定方向。
        乌龟完成旋转后，应显示一条中继目标结果的消息：

        ```
        [INFO] [turtlesim]: Rotation goal completed successfully
        ```
    
        ```F```键将在执行过程中取消目标。

        尝试按“C”键，然后在海龟完成旋转之前按“F”键。
        在运行```/turtlesim```节点的终端中，您将看到以下消息：

        ```
        [INFO] [turtlesim]: Rotation goal canceled
        ```
    
        不仅客户端（你在Teleop中的输入）可以阻止目标，服务器端（```/turtlesim```节点）也可以。
        当服务器端选择停止处理一个目标时，称为“中止”该目标。

        尝试按“D”键，然后在第一次旋转完成之前按“G”键。
        在运行```/turtlesim```节点的终端中，你将看到以下消息：
        
        ```
        [WARN] [turtlesim]: Rotation goal received before a previous goal finished. Aborting previous goal
        ```
    
        这个动作服务器选择中止第一个目标，因为它有一个新的目标。
        它可以选择其他东西，例如拒绝新目标或在第一个目标完成后执行第二个目标。
        不要假设每个动作服务器在获得新目标时都会选择中止当前目标。

    3. ### ros2 节点信息
    
        要查看```/turtlesim```节点的操作，请打开一个新终端并运行以下命令：

        ```
        ros2 node info /turtlesim
        ```

        这将返回/turtlesim的订阅者、发布者、服务、动作服务器和动作客户端的列表：

        ```
        /turtlesim
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
            /reset: std_srvs/srv/Empty
            /spawn: turtlesim/srv/Spawn
            /turtle1/set_pen: turtlesim/srv/SetPen
            /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
            /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
            /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
            /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
            /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
            /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
            /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
            /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
          Service Clients:

          Action Servers:
            /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
          Action Clients:
        ```
    
        请注意，```/turtlesim```的```/turtle1/rotate_absolute```操作位于Action Servers下。
        这意味着```/turtlesim```响应```/turtle1/rotate_absolute```动作并提供反馈。

        ```/teleop_turtle```节点在Action Clients下的名称为```/turtle1/rotate_absolute```，这意味着它会为该名称发送目标。

        ```bash
        ros2 node info /teleop_turtle
        ```
        
        将返回：
        
        ```
        /teleop_turtle
          Subscribers:
            /parameter_events: rcl_interfaces/msg/ParameterEvent
          Publishers:
            /parameter_events: rcl_interfaces/msg/ParameterEvent
            /rosout: rcl_interfaces/msg/Log
            /turtle1/cmd_vel: geometry_msgs/msg/Twist
          Service Servers:
            /teleop_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
            /teleop_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
            /teleop_turtle/get_parameters: rcl_interfaces/srv/GetParameters
            /teleop_turtle/list_parameters: rcl_interfaces/srv/ListParameters
            /teleop_turtle/set_parameters: rcl_interfaces/srv/SetParameters
            /teleop_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
          Service Clients:

          Action Servers:

          Action Clients:
            /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
        ```
    
    
    
    4. ### ros2动作列表

        要识别ROS图中的所有操作，请运行以下命令：

        ```bash
        ros2 action list
        ```

        将返回：

        ```
        /turtle1/rotate_absolute
        ```

        这是目前ROS图中的唯一Action。
        正如你之前看到的，它控制着海龟的旋转。
        通过使用```ros2 node info <node_name>```命令，你也已经知道有一个动作客户端（/teleop_turtle的一部分）和一个动作服务器（/turtlesim 的一部分）用于此动作。
    
    
    
            1. #### ros2 动作列表 -t
                动作有类型，类似于主题和服务。 要查找 /turtle1/rotate_absolute 的类型，请运行以下命令：
                
                ```bash
                ros2 action list -t
                ```
    
                将返回：
                
                ```
                /turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
                ```
    
                在每个动作名称（在本例中仅为 /turtle1/rotate_absolute）右侧的括号中是动作类型，turtlesim/action/RotateAbsolute。
                当你想从命令行或代码执行操作时，你将需要它。
    
    
    5. ### ros2动作信息
    
        你可以使用以下命令进一步内检/turtle1/rotate_absolute操作：
        
        ```bash
        ros2 action info /turtle1/rotate_absolute
        ```
    
        将返回:
    
        ```
        Action: /turtle1/rotate_absolute
        Action clients: 1
            /teleop_turtle
        Action servers: 1
            /turtlesim
        ```
    
        这告诉我们之前在每个节点上运行ros2节点信息时学到的：```/teleop_turtle```节点有一个动作客户端，/turtlesim节点有一个用于/turtle1/rotate_absolute动作的动作服务器。
    
    
    
    6. ### ros2接口显示
    
        在自己发送或执行操作目标之前，你还需要的另一条信息是操作类型的结构。

        回想一下，你在运行命令```ros2 action list -t```时确定了/turtle1/rotate_absolute的类型。 
        在终端中输入以下带有操作类型的命令：
    
        ```bash
        ros2 interface show turtlesim/action/RotateAbsolute
        ```
    
        将返回：
    
        ```
        #The desired heading in radians
        float32 theta
        ---
        #The angular displacement in radians to the starting position
        float32 delta
        ---
        #The remaining rotation in radians
        float32 remaining
        ```
    
        此消息的第一部分，在 --- 上方，是目标请求的结构（数据类型和名称）。 
        下一部分是结果的结构。 
        最后一部分是反馈的结构。
    
    
    
    7. ### ros2动作发送目标

        现在让我们使用以下语法从命令行发送一个动作目标：
    
        ```bash
        ros2 action send_goal <action_name> <action_type> <values>
        ```
    
        ```values```需要采用YAML格式。

        注意海龟模拟器窗口，然后在终端中输入以下命令：
    
        ```bash
        ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
        ```
    
        你应该会看到海龟在旋转，并在终端中显示以下消息：
    
        ```
        Waiting for an action server to become available...
        Sending goal:
           theta: 1.57

        Goal accepted with ID: f8db8f44410849eaa93d3feb747dd444

        Result:
          delta: -1.568000316619873

        Goal finished with status: SUCCEEDED
        ```
    
        所有目标都有一个唯一的ID，显示在返回消息中。 
        你还可以看到结果，一个名为 delta的字段，它是到起始位置的位移。

        要查看此目标的反馈，请在```ros2 action send_goal```命令中添加```--feedback```：
    
        ```bash
        ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback
        ```
    
        你的终端将返回消息：
    
        ```
        Sending goal:
           theta: -1.57

        Goal accepted with ID: e6092c831f994afda92f0086f220da27

        Feedback:
          remaining: -3.1268222332000732

        Feedback:
          remaining: -3.1108222007751465

        …

        Result:
          delta: 3.1200008392333984

        Goal finished with status: SUCCEEDED
        ```
    
        你将继续收到反馈，剩余的弧度，直到目标完成。
    
    
    
    
    
- ## 总结
    
    操作就像服务，允许你执行长时间运行的任务、提供定期反馈并且可以取消。

    机器人系统可能会使用Action进行导航。 
    动作目标可以告诉机器人行进到一个位置。
    当机器人导航到该位置时，它可以沿途发送更新（即反馈），然后在到达目的地后发送最终结果消息。

    海龟模拟器有一个动作服务器，动作客户端可以将目标发送到旋转海龟。 
    在本教程中，你内检了该动作/turtle1/rotate_absolute，以更好地了解什么是动作以及它们如何工作。

    
    
- ## 下一步

    现在你已经涵盖了ROS2的所有核心概念。
    “用户”集中的最后几篇教程将从使用```rqt_console```开始，
    向你介绍一些工具和技术，这些工具和技术将使你更轻松地使用ROS2。
    

- ## 相关内容

    你可以在此处阅读有关ROS2中操作背后的设计决策的更多信息。
    
    
    
    
