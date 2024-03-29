# Markers: Sending Basic Shapes (C++)



**说明：**  展示如何使用可视化信息/标记消息将基本形状（立方体、球体、圆柱体、箭头）发送到rviz。

**教程级别：** 初学者

**下一个教程：** 标记：点和线



1. ## 介绍

    与其他显示不同，标记显示使你可以在rviz中可视化数据，而无需rviz了解有关解释该数据的任何信息。
    相反，原始对象通过visualization_msgs/Marker消息发送到显示器，这让你可以显示箭头、框、球体和线条等内容。
    本教程将向你展示如何发送四种基本形状（盒子、球体、圆柱体和箭头）。
    我们将创建一个程序，每秒发送一个新标记，用不同的形状替换最后一个。
    注意：rviz_visual_tools包为C++用户提供了便利的功能。
    
    
    
2. ## 创建包

    在开始之前，让我们在包路径中的某处创建一个名为using_markers的包：
    
    
    
    
3. ## 发送Markers

    1. ### 代码
    
        将以下内容粘贴到 src/basic_shapes.cpp中：
        
        
        
        现在编辑using_markers包中的 CMakeLists.txt 文件，并添加：
        
        ```
        add_executable(basic_shapes src/basic_shapes.cpp)
        target_link_libraries(basic_shapes ${catkin_LIBRARIES})
        ```
        
        至底部。
        


    2. ### 代码解释
    
        好的，让我们一块一块地分解代码：
        
       
        
        你现在应该已经看到了ROS。
        我们还包括了visualization_msgs/Marker消息定义。
        
       
        
        这应该看起来很熟悉。 我们初始化ROS，并在visualization_marker 主题上创建一个ros::Publisher。
        
        

        在这里，我们创建一个整数来跟踪我们将要发布的形状。
        我们将在这里使用的四种类型都以相同的方式使用了visualization_msgs/Marker消息，因此我们可以简单地切换形状类型来演示四种不同的形状。
        
       
        
        这开始了程序的主要内容。
        首先我们创建一个visualization_msgs/Marker，并开始填写它。
        这里的头文件是一个roslib/Header，如果你看过tf教程应该很熟悉。
        我们将frame_id成员设置为/my_frame作为示例。
        在正在运行的系统中，这应该是相对于你希望解释标记姿势的帧。
        
       
        
        命名空间(ns)和id用于为此标记创建唯一名称。
        如果收到具有相同ns和id的标记消息，则新标记将替换旧标记。
        
        
        
        这个类型字段指定了我们发送的标记类型。
        在visualization_msgs/Marker消息中列举了可用的类型。
        在这里，我们将类型设置为我们的形状变量，每次循环都会改变。
        
        
        
        action字段指定如何处理标记。
        选项是visualization_msgs::Marker::ADD和visualization_msgs::Marker::DELETE。
        ADD有点用词不当，它的真正意思是“创建或修改”。

        Indigo中的新功能添加了一个新操作，用于删除特定 Rviz 显示中的所有标记，无论 ID 或命名空间如何。 该值为 3，在未来的 ROS 版本中，该消息将更改为具有值        visualization_msgs::Marker::DELETEALL。
        
        
        
        在这里我们设置标记的姿势。
        geometry_msgs/Pose 消息包含一个 geometry_msgs/Vector3 来指定位置和一个 geometry_msgs/Quaternion 来指定方向。
        在这里，我们将位置设置为原点，将方向设置为身份方向（注意 w 为 1.0）。
        
        
        
        现在我们指定标记的比例。 对于基本形状，所有方向的刻度为 1 表示一侧为 1 米。
        
        
        
        标记的颜色被指定为 std_msgs/ColorRGBA。
        每个成员应介于 0 和 1 之间。alpha (a) 值为 0 表示完全透明（不可见），而 1 表示完全不透明。
        
       
        
        生命周期字段指定此标记在被自动删除之前应保留多长时间。 ros::Duration() 的值意味着永远不会自动删除。

        如果在达到生命周期之前接收到新标记消息，则生命周期将重置为新标记消息中的值。
        
        
        
        我们等待标记有订阅者，然后我们发布标记。 
        请注意，您还可以使用锁定发布者作为此代码的替代方法。
        
       
        
        这段代码让我们显示所有四个形状，同时只发布一个标记消息。
        根据当前形状，我们设置下一个要发布的形状。
        
        
        
        睡觉并循环回到顶部。



    3. ### 构建代码

        您应该能够使用以下代码构建代码：
        
       
        
    4. ### 运行代码

        您应该能够使用以下代码运行代码：
        
        
        
        
4. ## 查看标记

    现在您正在发布标记，您需要设置 rviz 以查看它们。 首先，确保构建了rviz：
    
   
    现在，运行 rviz：
    
    
    
    如果您以前从未使用过 rviz，请参阅用户指南以开始使用。

    要做的第一件事，因为我们没有任何 tf 转换设置，是将固定帧设置为我们将标记设置为上面的帧，/my_frame。 为此，请将固定帧字段设置为“/my_frame”。

    接下来添加一个标记显示。 请注意，指定的默认主题visualization_marker 与发布的主题相同。

    您现在应该在原点看到一个每秒改变形状的标记：
    
    
    
    
5. ## 更多信息

    有关此处显示的标记之外的不同类型标记的更多信息，请参阅标记显示页面

