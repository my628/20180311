# 理解ROS2服务



**目标** ：使用命令行工具了解ROS2中的服务。

**教程等级** ：初学者

**用时** ：10分钟

**内容** ：



- ## 背景

    服务是ROS图中节点的另一种通信方法。
    服务基于调用和响应模型，而不是主题的发布者-订阅者模型。
    虽然主题允许节点订阅数据流并获得持续更新，但服务仅在客户端专门调用时才提供数据。
    
    ![图片alt](http://docs.ros.org/en/galactic/_images/Service-SingleServiceClient.gif)
    
    ![图片alt](http://docs.ros.org/en/galactic/_images/Service-MultipleServiceClient.gif)
    
- ## 先决条件

    本教程中提到的一些概念（如节点和主题）已在本系列的先前教程中介绍过。

    你将需要海龟模拟器包。

    与往常一样，不要忘记在你打开的每个新终端中读取和执行ROS2设置文件中的shell命令。
    
    
    
- ## 任务

    1. ### 设置
        启动两个海龟模拟器节点，/turtlesim和/teleop_turtle。

        打开一个新终端并运行：

        ```bash
        ros2 run turtlesim turtlesim_node
        ```
        
        打开另一个终端并运行：
        
        ```bash
        ros2 run turtlesim turtle_teleop_key
        ```
        
        
    2. ### ROS2服务列表

        在新终端中运行```ros2 service list```命令将返回系统中当前激活的所有服务的列表：
        
        ```
        /clear
        /kill
        /reset
        /spawn
        /teleop_turtle/describe_parameters
        /teleop_turtle/get_parameter_types
        /teleop_turtle/get_parameters
        /teleop_turtle/list_parameters
        /teleop_turtle/set_parameters
        /teleop_turtle/set_parameters_atomically
        /turtle1/set_pen
        /turtle1/teleport_absolute
        /turtle1/teleport_relative
        /turtlesim/describe_parameters
        /turtlesim/get_parameter_types
        /turtlesim/get_parameters
        /turtlesim/list_parameters
        /turtlesim/set_parameters
        /turtlesim/set_parameters_atomically
        ```
        
        你将看到两个节点都具有相同的六个服务，其名称中带有参数。 
        ROS2中的几乎每个节点都有这些基础设施服务，参数是基于这些服务构建的。
        在下一个教程中将有更多关于参数的内容。
        在本教程中，将省略参数服务的讨论。

        现在，让我们专注于海龟模拟器特定的服务，/clear、/kill、/reset、/spawn、/turtle1/set_pen、/turtle1/teleport_absolute和/turtle1/teleport_relative。
        你可能还记得在[介绍海龟模拟器和rqt]教程中使用rqt与其中一些服务进行交互。
        
        
        
    3. ### ROS2服务类型

        服务的类型描述了服务请求和响应的数据结构。 
        服务类型的定义与主题类型类似，但服务类型有两个部分：一个用于请求的消息，另一个用于响应的消息。

        要找出服务的类型，请使用以下命令：
        
        ```bash
        ros2 service type <service_name>
        ```
        
        我们来看看海龟模拟器的/clear服务。 
        在新终端中，输入命令：
        
        ```bash
        ros2 service type /clear
        ```
        
        应该返回：
        
        ```
        std_srvs/srv/Empty
        ```
        
        Empty类型表示服务调用在发出请求时不发送数据，在接收响应时不接收数据。
        
        
        1. #### ROS2服务列表 -t

            要同时查看所有激活的服务类型，可以将--show-types选项（缩写为 -t）附加到list命令：
            
            ```bash
            ros2 service list -t
            ```
            
            将返回：
            
            ```
            /clear [std_srvs/srv/Empty]
            /kill [turtlesim/srv/Kill]
            /reset [std_srvs/srv/Empty]
            /spawn [turtlesim/srv/Spawn]
            ...
            /turtle1/set_pen [turtlesim/srv/SetPen]
            /turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
            /turtle1/teleport_relative [turtlesim/srv/TeleportRelative]
            ...
            ```
            
    4. ### ROS2服务查找

        如果要查找特定类型的所有服务，可以使用以下命令：
        
        ```bash
        ros2 service find <type_name>
        ```
        
        例如，你可以像这样找到所有Empty类型的服务：
        
        ```bash
        ros2 service find std_srvs/srv/Empty
        ```
        
        将返回：
        
        ```
        /clear
        /reset
        ```
        
        
    5. ### ROS2服务显示

        你可以从命令行调用服务，但首先你需要知道输入参数的结构。
        
        ```bash
        ros2 interface show <type_name>
        ```
        
        要在/clear服务的类型Empty上运行此命令：
        
        ```bash
        ros2 interface show std_srvs/srv/Empty
        ```
        
        将返回：
        
        ```
        ---
        ```
        
        --- 将请求结构（上）与响应结构（下）分开。 
        但是，正如你之前了解到的，Empty类型不发送或接收任何数据。
        所以，自然而然，它的结构是空白的。

        让我们内检一个具有发送和接收数据类型的服务，例如/spawn。
        从```ros2 service list -t```的结果，我们知道/spawn的类型是turtlesim/srv/Spawn。

        要查看/spawn调用和请求中的参数，请运行以下命令：
        
        ```bash
        ros2 interface show turtlesim/srv/Spawn
        ```
        
        将返回：
        
        ```
        float32 x
        float32 y
        float32 theta
        string name # Optional.  A unique name will be created and returned if this is empty
        ---
        string name
        ```
        
        ---行上方的信息告诉我们调用/spawn所需的参数。
        x, y和theta确定生成的海龟的位置，名称显然是可选的。

        在这种情况下，你不需要了解该行下方的信息，但它可以帮助你理解从调用中获得响应的数据类型。



    6. ### ROS2服务调用

        现在你知道什么是服务类型、如何查找服务类型以及如何查找该类型参数的结构，你可以使用以下方法调用服务：
        
        ```bash
        ros2 service call <service_name> <service_type> <arguments>
        ```
        
        ```arguments```部分是可选的。 
        例如，你知道Empty类型服务没有任何参数：
        
        ```bash
        ros2 service call /clear std_srvs/srv/Empty
        ```
        
        此命令将清除乌龟绘制的任何线条在海龟模拟器的窗口。
        
        ![图片alt](http://docs.ros.org/en/galactic/_images/clear.png)
        
        现在让我们通过调用/spawn并输入参数来生成一个新的海龟。
        从命令行在服务调用中输入arguments需要采用YAML语法。

        输入命令：
        
        ```bash
        ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
        ```
        
        你将获得有关正在发生的事情的这种方法风格的视图，然后是服务响应：
        
        ```
        requester: making request: turtlesim.srv.Spawn_Request(x=2.0, y=2.0, theta=0.2, name='')

        response:
        turtlesim.srv.Spawn_Response(name='turtle2')
        ```
        
        你的海龟模拟器窗口将立即更新使用新生成的海龟：
        
        ![图片alt](http://docs.ros.org/en/galactic/_images/spawn.png)
    
    
    
- ## 总结

    节点可以使用ROS2中的服务进行通信。
    与主题不同 - 一种单向通信模式，其中节点发布可由一个或多个订阅者使用的信息 - 服务是一种请求/响应模式，其中客户端向节点发出请求提供服务，服务处理请求并生成响应。

    你通常不想使用连续调用的服务；topic甚至action会更适合。

    在本教程中，你使用命令行工具来识别、详细说明和调用服务。

- ## 下一步

    在下一个教程[理解ROS2参数中]，你将了解如何配置节点设置。

- ## 相关内容
    
    查看[本教程]； 这是使用Robotis机械臂的“理解ROS2服务”的出色现实应用。
    
    
    
    
