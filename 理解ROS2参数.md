# 理解ROS2参数



**目标** ：学习如何在ROS2中获取、设置、保存和重新加载参数。

**教程等级** ：初学者

**用时** ：5分钟

**内容** ：



- ## 背景

    参数是节点的配置值。 
    你可以将参数视为节点设置。 
    节点可以将参数存储为整数、浮点数、布尔值、字符串和列表。 
    在ROS2中，每个节点维护自己的参数。
    所有参数都是动态可重新配置的，并基于ROS2服务构建。

- ## 先决条件

    本教程使用了海龟模拟器包。

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
        
        
        
    2. ### ros2参数列表

        要查看属于你的节点的参数，请打开一个新终端并输入命令：
        
        ```bash
        ros2 param list
        ```
        
        你将看到节点的命名空间/teleop_turtle和/turtlesim，然后是每个节点的参数：
        
        ```
        /teleop_turtle:
          qos_overrides./parameter_events.publisher.depth
          qos_overrides./parameter_events.publisher.durability
          qos_overrides./parameter_events.publisher.history
          qos_overrides./parameter_events.publisher.reliability
          scale_angular
          scale_linear
          use_sim_time
        /turtlesim:
          background_b
          background_g
          background_r
          qos_overrides./parameter_events.publisher.depth
          qos_overrides./parameter_events.publisher.durability
          qos_overrides./parameter_events.publisher.history
          qos_overrides./parameter_events.publisher.reliability
          use_sim_time
        ```
        
        每个节点都有参数```use_sim_time```；这不是海龟模拟器独有的。

        根据它们的名称，看起来/turtlesim的参数使用RGB颜色值来确定turtlesim窗口的背景颜色。

        要确定参数的类型，你可以使用```ros2 param get```。
        
        
        
    3. ### ros2参数获取

        要显示参数的类型和当前值，请使用以下命令：
        
        ```bash
        ros2 param get <node_name> <parameter_name>
        ```
        
        让我们找出/turtlesim的参数background_g的当前值：
        
        ```bash
        ros2 param get /turtlesim background_g
        ```
        
        将返回值：
        
        ```
        Integer value is: 86
        ```
        
        现在你知道background_g持有一个整数值。

        如果你在background_r和background_b上运行相同的命令，你将分别获得值69和255。
        
        
        
    4. ### ros2参数设置

        要在运行时更改参数的值，请使用以下命令：
        
        ```bash
        ros2 param set <node_name> <parameter_name> <value>
        ```
        
        让我们改变/turtlesim的背景颜色：
        
        ```bash
        ros2 param set /turtlesim background_r 150
        ```
        
        你的终端应返回以下消息：
        
        ```
        Set parameter successful
        ```
        
        并且你的海龟模拟器窗口的背景应该改变颜色：
        
        ![图像alt](http://docs.ros.org/en/galactic/_images/set.png)
        
        使用set命令设置参数只会在你当前的会话中更改它们，而不是永久更改。 
        但是，你可以保存设置并在下次启动节点时重新加载它们。
        
        
        
    5. ### ros2参数转储

        你可以使用以下命令将节点的所有当前参数值转储到文件中以供以后使用：
        
        ```bash
        ros2 param dump <node_name>
        ```
        
        要保存/turtlesim参数的当前配置，请输入以下命令：
        
        ```bash
        ros2 param dump /turtlesim
        ```
        
        你的终端将返回消息：
        
        ```
        Saving to:  ./turtlesim.yaml
        ```
        
        你将在工作区运行的目录中找到一个新文件。 
        如果打开此文件，你将看到以下内容：
        
        ```
        /turtlesim:
          ros__parameters:
            background_b: 255
            background_g: 86
            background_r: 150
            qos_overrides:
              /parameter_events:
                publisher:
                  depth: 1000
                  durability: volatile
                  history: keep_last
                  reliability: reliable
            use_sim_time: false
        ```
        
        如果你想在将来使用相同的参数重新加载节点，则转储参数会派上用场。
        
        
        
    6. ### ros2参数加载

        你可以使用以下命令将文件中的参数加载到当前运行的节点：
        
        ```bash
        ros2 param load <node_name> <parameter_file>
        ```
        
        要将使用```ros2 param dump```生成的./turtlesim.yaml文件加载到/turtlesim 节点的参数中，请输入命令：
        
        ```bash
        ros2 param load /turtlesim ./turtlesim.yaml
        ```
        
        你的终端将返回消息：
        
        ```
        Set parameter background_b successful
        Set parameter background_g successful
        Set parameter background_r successful
        Set parameter qos_overrides./parameter_events.publisher.depth failed: parameter 'qos_overrides./parameter_events.publisher.depth' cannot be set because it is read-only
        Set parameter qos_overrides./parameter_events.publisher.durability failed: parameter 'qos_overrides./parameter_events.publisher.durability' cannot be set because it is read-only
        Set parameter qos_overrides./parameter_events.publisher.history failed: parameter 'qos_overrides./parameter_events.publisher.history' cannot be set because it is read-only
        Set parameter qos_overrides./parameter_events.publisher.reliability failed: parameter 'qos_overrides./parameter_events.publisher.reliability' cannot be set because it is read-only
        Set parameter use_sim_time successful
        ```
        
        > 只读参数只能在启动时修改，之后不能修改，这就是“qos_overrides”参数有一些警告的原因。
        
        
        
    7. ### 节点启动时加载参数文件

        要使用你保存的参数值启动同一个节点，请使用：
        
        ```bash
        ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
        ```
        
        这与你总是用来启动turtlesim的命令相同，添加了标志--ros-args和--params-file，后跟要加载的文件。

        停止正在运行的turtlesim节点，以便你可以尝试使用保存的参数重新加载它，使用：
        
        ```bash
        ros2 run turtlesim turtlesim_node --ros-args --params-file ./turtlesim.yaml
        ```
        
        海龟模拟器窗口应该像往常一样出现，但背景是你之前设置的紫色。
        
        > 在这种情况下，参数在启动时被修改，因此指定的只读参数也会生效。



- ## 总结

    节点具有用于定义其默认配置值的参数。
    你可以从命令行获取和设置参数值。
    你还可以将参数设置保存到文件中，以便在以后的会话中重新加载它们。
    

- ## 下一步

    回到ROS2通信方法，在下一个教程中，你将学习actions。
    
    
