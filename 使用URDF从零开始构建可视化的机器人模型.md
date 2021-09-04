# 使用URDF从零开始构建可视化的机器人模型



**描述：** 了解如何构建可以在Rviz中查看可视化的机器人模型

**教程级别：** 初学者


在本教程中，我们将构建一个可视化的机器人模型，该模型看起来很像R2D2。

在后面的教程中，你将学习如何清晰地表达模型，添加一些物理属性，使用xacro生成更整洁的代码，并使其在Gazebo中移动。

但现在，我们将重点关注如何获得正确的视觉几何。

继续之前，请确保已安装joint_state_publisher软件包。

如果你使用apt get安装了urdf教程，那么应该已经是这样了。

如果没有，请更新你的安装以包含该软件包（使用rosdep进行检查）。

本教程中提到的所有机器人模型（以及源文件）都可以在urdf教程包中找到。



1. ## 一个形状

    首先，我们将探索一个简单的形状。
    这里有一个尽可能简单的urdf。

    ```xml
    <?xml version="1.0"?>
    <robot name="myfirst">
      <link name="base_link">
        <visual>
          <geometry>
            <cylinder length="0.6" radius="0.2"/>
          </geometry>
        </visual>
      </link>
    </robot>
    ```


    这是一个名为myfirst的机器人，它只包含一个链接（也称为part），其可视组件只是一个0.6米长、半径为0.2米的圆柱体。

    对于一个简单的“hello world”类型的示例，这看起来像是很多封闭的标记，但相信我，它会变得更复杂。

    要检查模型，请启动display.launch文件：
    
    ```bash
    roslaunch urdf_tutorial display.launch model:=urdf/01-myfirst.urdf
    ```
    
    这有三件事。
    
    - 将指定的模型加载到参数服务器中
    - 运行节点以发布传感器sensor_msgs/JointState和变换（稍后将详细介绍这些）
    - 使用配置文件启动Rviz
    
    注意，上面的roslaunch行假设你是从urdf教程包目录执行它的（即：urdf目录是当前工作目录的直接子目录）。
    
    如果不是这样，01-myfirst.urdf的相对路径将无效，并且一旦roslaunch尝试将urdf加载到参数服务器，
    
    你将收到一个错误。
    
    稍加修改的参数允许此操作在不考虑当前工作目录的情况下运行：
