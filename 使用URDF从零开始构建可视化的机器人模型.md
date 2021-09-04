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
    
    ```bash
    roslaunch urdf_tutorial display.launch model:='$(find urdf_tutorial)/urdf/01-myfirst.urdf'
    ```
    
    请注意参数值周围的单引号。
    
    如果你不是从urdf教程包位置运行这些教程中给出的所有示例行，则必须更改它们。
    
    启动display.launch后，你应该会看到RViz向您显示以下内容：
    
    我的第一张照片
    
    注意事项：
    
    - 固定帧是栅格中心所在的变换帧。
        这里，它是一个由我们的一个链接，base_link定义的框架。
    - 默认情况下，可视元素（圆柱体）的原点位于其几何图形的中心。
        因此，一半圆柱体位于栅格下方。
    
2. ## 多种形状
    
    现在让我们看看如何添加多个形状/链接。
    
    如果我们只是向urdf添加更多链接元素，解析器将不知道将它们放在哪里。
    
    因此，我们必须添加接头。
    
    关节图元可以指柔性关节和非柔性关节。
    
    我们将从刚性或固定接头开始。
    
    ```xml
    <?xml version="1.0"?>
    <robot name="multipleshapes">
      <link name="base_link">
        <visual>
          <geometry>
            <cylinder length="0.6" radius="0.2"/>
          </geometry>
        </visual>
      </link>

      <link name="right_leg">
        <visual>
          <geometry>
            <box size="0.6 0.1 0.2"/>
          </geometry>
        </visual>
      </link>

      <joint name="base_to_right_leg" type="fixed">
        <parent link="base_link"/>
        <child link="right_leg"/>
      </joint>

    </robot>
    ```
    
    - 请注意我们是如何定义一个0.6m x 0.1m x 0.2m的长方体的关节是根据父项和子项定义的。
    
    - URDF最终是一个具有一个根链接的树结构。
      这意味着腿的位置取决于基础链接的位置。

roslaunch urdf教程display.launch model:=urdf/02-multipleshapes.urdf多个形状
这两个形状彼此重叠，因为它们具有相同的原点。

如果我们不想让它们重叠，我们就必须定义更多的起源。



3. ## 原点

    所以R2D2的腿连接到他躯干的上半部分，在侧面。
    
    这就是我们指定关节原点的地方。
    
    另外，它不会附着到腿的中间，而是附着到腿的上部，因此我们也必须偏移腿的原点。
    
    我们还旋转腿，使其直立。
    
    ```xml
    <?xml version="1.0"?>
    <robot name="origins">
      <link name="base_link">
        <visual>
          <geometry>
            <cylinder length="0.6" radius="0.2"/>
          </geometry>
        </visual>
      </link>

      <link name="right_leg">
        <visual>
          <geometry>
            <box size="0.6 0.1 0.2"/>
          </geometry>
          <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
        </visual>
      </link>

      <joint name="base_to_right_leg" type="fixed">
        <parent link="base_link"/>
        <child link="right_leg"/>
        <origin xyz="0 -0.22 0.25"/>
      </joint>

    </robot>
    ```
