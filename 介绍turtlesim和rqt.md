# 介绍turtlesim和rqt

**目标** ：安装并使用turtlesim包和rqt工具为即将到来的教程做准备。

**教程等级** ：初学者

**用时** ：15分钟

**内容** ：


- ## 背景
Turtlesim是一个用于学习ROS2的轻量级模拟器。它在最基本的层面上阐明了ROS2的作用，让你了解稍后将使用真实机器人或机器人模拟器来做什么。

rqt是ROS2的GUI工具。
在rqt中所做的一切都可以在命令行上完成，但它提供了一种更简单、更用户友好的方式来操纵ROS2的元件。

本教程涉及核心的ROS2概念，例如节点、主题和服务的分离。 
所有这些概念将在后面的教程中详细说明； 现在，您只需设置工具并感受一下它们。



- ## 先决条件
上一个教程，配置你的ROS2环境，将向你展示如何设置你的环境。


- ## 任务

    1. ### 安装海龟模拟器
        与往常一样，首先在新终端中读取并执行设置文件中shell命令，如上一教程中所述。
        为你的ROS2发行版安装turtlesim包：
        ```bash
        sudo apt update
        sudo apt install ros-galactic-turtlesim
        ```
        检查是否安装了软件包：
        ```bash
        ros2 pkg executables turtlesim
        ```
        上面的命令应该返回一个turtlesim的可执行文件列表：
        ```
        turtlesim draw_square
        turtlesim mimic
        turtlesim turtle_teleop_key
        turtlesim turtlesim_node
        ```
    
    2. ### 启动海龟模拟器
        要启动海龟模拟器，请在终端中输入以下命令：
        ```
        ros2 run turtlesim turtlesim_node
        ```
        模拟器窗口应该出现，中间有一个随机的海龟。
        
        ![图片alt](http://docs.ros.org/en/galactic/_images/turtlesim.png)
        
        在命令下的终端中，你将看到来自节点的消息：
        ```
        [INFO] [turtlesim]: Starting turtlesim with node name /turtlesim
        [INFO] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
        ```
        
    3. ### 使用海龟模拟器
        
        打开一个新终端并再次获取ROS2。

        现在你将运行一个新节点来控制第一个节点中的海龟：
        
        ```bash
        ros2 run turtlesim turtle_teleop_key
        ```
        
        此时你应该打开三个窗口：一个运行turtlesim_node的终端，一个运行turtle_teleop_key的终端和turtlesim窗口。 
        排列这些窗口，以便您可以看到turtlesim窗口，同时还要使运行turtle_teleop_key 的终端处于活动状态，以便您可以在turtlesim 中控制海龟。
        
        使用键盘上的箭头键来控制乌龟。 它将在屏幕上移动，使用其附带的“笔”绘制到目前为止所遵循的路径。
        
        > 按箭头键只会使海龟移动一小段距离然后停止。 这是因为，实际上，如果操作员失去与机器人的连接，您不希望机器人继续执行指令。
        
        您可以使用 list 命令查看节点及其关联的服务、主题和操作：
        
        ```bash
        ros2 node list
        ros2 topic list
        ros2 service list
        ros2 action list
        ```

    4. ### 安装rqt
        
        打开一个新终端来安装rqt及其插件：
        
        ```bash
        sudo apt update
        sudo apt install ~nros-galactic-rqt*
        ```
        
        运行rqt
        
        ```bash
        rqt
        ```

    5. ### 使用rqt
    
        第一次运行rqt后，窗口将是空白的。
        不用担心; 只需从顶部的菜单栏中选择插件 > 服务 > 服务调用者。
        
        > rqt可能需要一些时间来定位所有插件本身。 
        > 如果你单击插件，但没有看到服务或任何其他选项，则应关闭rqt，在终端中输入命令 ```rqt --force-discover``` 。
        
        ![图片alt](http://docs.ros.org/en/galactic/_images/rqt.png)
        
        使用服务下拉列表左侧的刷新按钮确保您的海龟模拟器节点的所有服务都可用。

        单击服务下拉列表查看海龟模拟器的服务，然后选择/spawn服务。
        
        1. #### 尝试spawn服务
        
            让我们使用rqt来调用/spawn服务。 
            你可以从它的名字中猜出/spawn会在turtlesim窗口中创建另一只海龟。
            
            通过在表达式列中的空单引号之间双击，为新海龟指定一个唯一名称，例如turtle2。 
            可以看到，这个表达式对应的是名称值，并且是字符串类型。

            输入海龟生成的新坐标，例如 x = 1.0 和 y = 1.0。
            
            ![图片alt](http://docs.ros.org/en/galactic/_images/spawn1.png)
            
            > 如果你尝试生成与现有海龟同名的新海龟，例如默认的turtle1，那么你将在运行turtlesim_node的终端中收到一条错误消息：
            
            ```
            [ERROR] [turtlesim]: A turtle named [turtle1] already exists
            ```
            
            要生成turtle2，您必须通过单击rqt 窗口右上角的Call按钮来调用该服务。
            你将看到在你输入的x和y坐标处生成了一只新海龟（再次采用随机设计）。

            如果刷新rqt中的服务列表，你还会看到现在除了/turtle1/...之外，还有与新乌龟相关的服务，/turtle2/...。
            
        2. #### 尝试set_pen服务
            
            现在让我们使用/set_pen服务给turtle1一个独有的笔：
            
            ![图片alt](http://docs.ros.org/en/galactic/_images/set_pen.png)
            
            r、g和b的值在0到255之间，将设置海龟1笔绘制的颜色，width设置线条的粗细。

            要让海龟1绘制一条明显的红线，请将r的值更改为255，将width的值更改为5。不要忘记在更新值后调用服务。

            如果你回到运行turtle_teleop_node的终端，按方向键，你会看到海龟1的笔已经改变了。
            
            ![图片alt](http://docs.ros.org/en/galactic/_images/new_pen.png)
            
            你可能已经注意到无法移动海龟2。 
            你可以通过将海龟1的cmd_vel主题重新映射到海龟2来实现此目的。
            
  
    6. ### 重映射
    
        在一个新终端中，输入然后运行：
        
        ```bash
        ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
        ```
        
        现在你可以在此终端处于活动状态时移动海龟2，并在运行turtle_teleop_key的另一个终端处于激活状态时移动海龟1。
        
        
    7. ### 关闭海龟模拟器

        要停止模拟，你可以在turtlesim_node终端中输入Ctrl + C，在teleop终端中输入q。
        

- ## 总结
    使用turtlesim和rqt是学习ROS 2核心概念的好方法。
    
- ## 下一步
    现在你已经启动并运行了海龟模拟器和rqt，并且了解了它们的工作原理，让我们在下一个教程理解ROS2节点中深入探讨ROS2的第一个核心概念。
    
- ## 相关内容
    可以在ros_tutorials存储库中找到海龟模拟器的包。 
    确保调整分支以查看与你安装的ROS2发行版相对应的海龟模拟器版本。

    这个社区贡献的视频演示了本教程中涵盖的许多项目。
    
    
    

