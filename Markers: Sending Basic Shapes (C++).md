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
    
    ```bash
    ```
    
    
3. ## 发送Markers

    1. ### 代码
    
        将以下内容粘贴到 src/basic_shapes.cpp中：
        
        ```c++
        ```
        
        现在编辑using_markers包中的 CMakeLists.txt 文件，并添加：
        
        ```
        add_executable(basic_shapes src/basic_shapes.cpp)
        target_link_libraries(basic_shapes ${catkin_LIBRARIES})
        ```
        
        至底部。
        


    2. ### 代码解释
    
        好的，让我们一块一块地分解代码：
        
