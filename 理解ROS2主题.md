# 理解ROS2主题



**目标** ：使用rqt_graph和命令行工具来内检ROS2的主题。

**教程等级** ：初学者

**用时** ：20分钟

**内容** ：



- ## 背景

    ROS2将复杂系统分解为许多模块化节点。 
    主题是ROS图的重要元素，充当节点交换消息的总线。
    
    ![图片alt](http://docs.ros.org/en/galactic/_images/Topic-SinglePublisherandSingleSubscriber.gif)
    
    一个节点可以向任意数量的主题发布数据并同时订阅任意数量的主题。
    
    ![图片alt](http://docs.ros.org/en/galactic/_images/Topic-MultiplePublisherandMultipleSubscriber.gif)

    主题是数据在节点之间移动的主要方式之一，因此在系统的不同部分之间移动。
    
- ## 先决条件

    上一个教程提供了一些关于在此处构建的节点的有用背景信息。
    
    与往常一样，不要忘记在你打开的每个新终端中读取并执行ROS2设置文件中的shell命令。
    

- ## 任务

    1. ### 设置

        到现在为止，你应该可以轻松启动海龟模拟器了。

        打开一个新终端并运行：
        
        ```bash
        ros2 run turtlesim turtlesim_node
        ```

        打开另一个终端并运行：
        
        ```bash
        ros2 run turtlesim turtle_teleop_key
        ```
        
        回想一下之前的教程，这些节点的名称默认为/turtlesim和/teleop_turtle。
        
      
    2. ### rqt_graph

        在本教程中，我们将使用rqt_graph来可视化不断变化的节点和主题，以及它们之间的联系。

        [海龟模拟器教程]告诉你如何安装rqt及其所有插件，包括rqt_graph。

        要运行rqt_graph，请打开一个新终端并输入命令：
        
        ```bash
        rqt_graph
        ```
        
        你还可以通过打开rqt并选择Plugins>Introspection>Node Graph来打开rqt_graph。
        
        ![图片alt](http://docs.ros.org/en/galactic/_images/rqt_graph.png)
        
        你应该会看到上面的节点和主题，以及围绕图外围的两个动作（让我们暂时忽略它们）。 
        
        如果你将鼠标悬停在中心的主题上，你将看到如上图所示的颜色突出显示。

        该图描绘了/turtlesim节点和/teleop_turtle节点如何通过主题相互通信。 
        /teleop_turtle节点正在向/turtle1/cmd_vel主题发布数据（你输入的用于移动海龟的按键），/turtlesim节点订阅该主题以接收数据。

        rqt_graph的突出显示功能在检查具有以多种不同方式连接的许多节点和主题的更复杂系统时非常有用。
        
        rqt_graph是一个图形化自检工具。 
        现在我们将看看一些用于自检主题的命令行工具。
        
        
        
    3. ### ros2主题列表

        在新终端中运行```ros2 topic list```命令将返回系统中当前活动的所有主题的列表：
        
        ```
        /parameter_events
        /rosout
        /turtle1/cmd_vel
        /turtle1/color_sensor
        /turtle1/pose
        ```
        
        ```ros2 topic list -t```将返回相同的主题列表，这次将主题类型附加在括号中：
        
        ```
        /parameter_events [rcl_interfaces/msg/ParameterEvent]
        /rosout [rcl_interfaces/msg/Log]
        /turtle1/cmd_vel [geometry_msgs/msg/Twist]
        /turtle1/color_sensor [turtlesim/msg/Color]
        /turtle1/pose [turtlesim/msg/Pose]
        ```
        
        这些属性，尤其是类型，是节点如何知道他们在讨论相同的信息，因为它在主题上移动。

        如果你想知道rqt_graph中所有这些主题的位置，你可以取消选中隐藏下的所有框：
        
        ![image alt](http://docs.ros.org/en/galactic/_images/unhide.png)
        
        不过，就目前而言，请选中这些选项以避免混淆。
        
        
        
    4. ### ROS2主题应答

        要查看关于某个主题发布的数据，请使用：
        
        ```bash
        ros2 topic echo <topic_name>
        ```
        
        由于我们知道/teleop_turtle通过/turtle1/cmd_vel主题将数据发布到/turtlesim，让我们使用echo来自省该主题：
        
        ```bash
        ros2 topic echo /turtle1/cmd_vel
        ```
        
        起初，这个命令不会返回任何数据。 
        那是因为它在等待/teleop_turtle发布一些东西。

        返回到运行turtle_teleop_key的终端并使用箭头移动海龟。 
        观察你运行echo的终端，你将看到为你所做的每一个动作发布的位置数据：

        ```
        linear:
          x: 2.0
          y: 0.0
          z: 0.0
        angular:
          x: 0.0
          y: 0.0
          z: 0.0
          ---
        ```
        
        现在返回到rqt_graph并取消选中调试框。
        
        ![图片alt](http://docs.ros.org/en/galactic/_images/debug.png)
        
        /_ros2cli_26646是我们刚刚运行的 echo 创建的节点（数字可能不同）。 
        现在可以看到发布者正在通过cmd_vel主题发布数据，并且有两个订阅者订阅了。
        
        

    5. ### ROS2主题信息

        话题不只是点对点的通讯；它可以是一对多、多对一或多对多。

        另一种看待它的方法是运行：
        
        ```bash
        ros2 topic info /turtle1/cmd_vel
        ```
        
        将返回：
        
        ```
        Type: geometry_msgs/msg/Twist
        Publisher count: 1
        Subscription count: 2
        ```
        
        
        
    6. ### ROS2接口展示

        节点使用消息在主题上发送数据。 
        发布者和订阅者必须发送和接收相同类型的消息才能进行通信。

        我们之前在运行```ros2 topic list -t```后看到的主题类型让我们知道每个主题使用的消息类型。 
        回想一下cmd_vel主题的类型：
        
        ```
        geometry_msgs/msg/Twist
        ```
        
        这意味着在包geometry_msgs中有一个名为Twist的msg。

        现在我们可以在这个类型上运行```ros2 interface show <msg type>```来了解它的细节，特别是消息预设的数据结构。
        
        ```bash
        ros2 interface show geometry_msgs/msg/Twist
        ```
        
        对于上面的消息类型，它产生：
        
        ```
        #This expresses velocity in free space broken into its linear and angular parts.

        Vector3  linear
                float64 x
                float64 y
                float64 z
        Vector3  angular
                float64 x
                float64 y
                float64 z
        ```
        
        这告诉你/turtlesim节点需要一条消息，其中包含两个向量（线性和角度），每个向量包含三个元素。 
        如果你回忆一下我们看到/teleop_turtle使用echo命令传递给/turtlesim 的数据，它的结构相同：
        
        ```
        linear:
          x: 2.0
          y: 0.0
          z: 0.0
        angular:
          x: 0.0
          y: 0.0
          z: 0.0
          ---
        ```
        
        
    7. ### ROS2主题发布

        现在你有了消息结构，你可以使用以下命令直接从命令行将数据发布到主题上：
        
        ```bash
        ros2 topic pub <topic_name> <msg_type> '<args>'
        ```
        
        ```<args>```参数是你将传递给主题的实际数据，在你刚刚在上一节中发现的结构中。

        需要注意的是，此参数需要以YAML语法输入。 
        像这样输入完整的命令：
        
        ```bash
        ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
        ```
        
        ```--once```是一个可选参数，意思是“发布一条消息然后退出”。

        你将在终端中收到以下消息：
        
        ```
        publisher: beginning loop
        publishing #1: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=2.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=1.8))
        ```
        
        你会看到你的乌龟像这样移动：
        
        ![图像alt](http://docs.ros.org/en/galactic/_images/pub_once.png)
        
        海龟（通常是它要模拟的真实机器人）需要稳定的命令流才能连续运行。 
        所以，为了让海龟继续移动，你可以运行：
        
        ```bash
        ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
        ```
        
        你可以刷新rqt_graph以图形方式查看发生的情况。 
        你将看到ros2主题发布... 节点 (/_ros2cli_30358)正在/turtle1/cmd_vel主题上发布，并且现在正在被ros2主题应答...节点 (/_ros2cli_26646)和/turtlesim节点接收 。
        
        ![图片alt](http://docs.ros.org/en/galactic/_images/rqt_graph2.png)
        
        最后，你可以在```pose```主题上运行```echo```并重新检查rqt_graph：
        
        ```bash
        ros2 topic echo /turtle1/pose
        ```
        
        ![图片alt](http://docs.ros.org/en/galactic/_images/rqt_graph3.png)
        
        你可以看到/turtlesim节点也在发布新的```echo```节点订阅的```pose```主题。
        
        
        
    8. ### ROS2主题速度

        对于此过程的最后一次反思，你可以使用以下方法查看数据发布的速度：
        
        ```bash
        ros2 topic hz /turtle1/pose
        ```
        
        它将返回有关/turtlesim节点向```pose```主题发布数据的速率的数据。
        
        ```
        average rate: 59.354
        min: 0.005s max: 0.027s std dev: 0.00284s window: 58
        ```
        
        回想一下，你使用```ros2 topic pub --rate 1```将```turtle1/cmd_vel```的发布速率设置为稳定的1Hz。
        如果使用```turtle1/cmd_vel```而不是```turtle1/pose```运行上述命令，您将看到反映该速率的平均值。
        
        
         
    9. ### 清理
        此时，您将有很多节点在运行。 不要忘记通过在每个终端中输入 Ctrl+C 来停止它们。



- ## 总结
    节点通过主题发布信息，这允许任意数量的其他节点订阅和访问该信息。 
    在本教程中，你使用rqt_graph和命令行工具检查了多个节点之间关于主题的联系。
    你现在应该很好地了解数据如何在ROS2系统中移动。

- ## 下一步
    接下来，你将通过教程[理解ROS2服务]了解ROS图中的另一种通信类型。
    
    
    
    
