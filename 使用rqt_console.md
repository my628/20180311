# 使用rqt_console



**目标：** 了解rqt_console，一个用于检查日志消息的工具。

**教程级别：** 初学者

**时间：** 5分钟

**内容**    


- ## 背景
    
    ```rqt_console```是一个GUI工具，用于内检ROS2中的日志消息。
    通常，日志消息会显示在你的终端中。
    使用```rqt_console```，你可以随着时间的推移收集这些消息，以更有条理的方式仔细查看它们，过滤它们，保存它们，甚至重新加载保存的文件以在不同的时间进行自检。

    节点使用日志以多种方式输出有关事件和状态的消息。
    为了用户，它们的内容通常是信息性的。
    
    
    
- ## 先决条件
    
    你将需要安装rqt_console和海龟模拟器。

    与往常一样，不要忘记在你打开的每个新终端中读取和执行ROS2设置文件中的shell命令。
    
    
    
- ## 任务

    1. ### 设置
    
        使用以下命令在新终端中启动rqt_console：
    
        ```bash
        ```
    
    2. ### rqt_console上的消息
    
        为了生成rqt_console显示的日志消息，让乌龟撞到墙上。 
        在新终端中，输入下面的```ros2 topic pub```命令（在主题教程中详细讨论）：
    
    
    3. ### 记录器级别

        ROS2的记录器级别按严重性排序：
    
    
        1. #### 设置默认记录器级别
        
            你可以在第一次使用重新映射运行/turtlesim节点时设置默认记录器级别。 
            在终端中输入以下命令：
    
    
    
    
    
- ## 总结

    如果你需要仔细检查来自系统的日志消息，rqt_console会非常有用。 
    你可能出于各种原因想要检查日志消息，通常是为了找出哪里出了问题以及导致该问题的一系列事件。

    
    
- ## 下一步

    下一个教程将教您如何创建启动文件。
