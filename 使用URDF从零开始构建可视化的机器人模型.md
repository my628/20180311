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



1. ### 一个形状

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
    
2. ### 多种形状
    
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

      如果我们不想让它们重叠，我们就必须定义更多的原点。



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
    
    - 让我们从检查关节的原点开始。
        它是根据父对象的参照系定义的。
        所以我们在y方向上是-0.22米（在我们的左边，但相对于轴的右边），在z方向上是0.25米（向上）。
        这意味着无论子链接的可视原点标记如何，子链接的原点都将向上并向右。
        因为我们没有指定rpy（横摇-俯仰-偏航）属性，所以默认情况下，子帧的方向与父帧的方向相同。
    
    - 现在，查看腿部的视觉原点，它同时具有xyz和rpy偏移。
        这定义了视觉元素的中心相对于其原点的位置。
        因为我们希望腿附着在顶部，所以通过将z偏移设置为-0.3米来向下偏移原点。
        因为我们希望腿的长部分平行于z轴，所以我们绕Y轴旋转视觉部分PI/2。
        
    roslaunch urdf_教程display.launch model:=urdf/03-origins.urdf origins屏幕截图
    
    - 启动文件运行的包将基于URDF为模型中的每个链接创建TF帧。
        Rviz使用此信息确定在何处显示每个形状。
    - 如果给定的URDF链路不存在TF帧，则它将以白色放置在原点（参考相关问题）。


4. ### 材质

    “好吧，”我听见你说这很可爱，但不是每个人都有B21。
    
    我的机器人和R2D2不是红色的！”这是一个很好的观点。让我们看一下材质标签。
    
    ```xml
    <?xml version="1.0"?>
    <robot name="materials">

      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>

      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>


      <link name="base_link">
        <visual>
          <geometry>
            <cylinder length="0.6" radius="0.2"/>
          </geometry>
          <material name="blue"/>
        </visual>
      </link>

      <link name="right_leg">
        <visual>
          <geometry>
            <box size="0.6 0.1 0.2"/>
          </geometry>
          <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
          <material name="white"/>
        </visual>
      </link>

      <joint name="base_to_right_leg" type="fixed">
        <parent link="base_link"/>
        <child link="right_leg"/>
        <origin xyz="0 -0.22 0.25"/>
      </joint>

      <link name="left_leg">
        <visual>
          <geometry>
            <box size="0.6 0.1 0.2"/>
          </geometry>
          <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
          <material name="white"/>
        </visual>
      </link>

      <joint name="base_to_left_leg" type="fixed">
        <parent link="base_link"/>
        <child link="left_leg"/>
        <origin xyz="0 0.22 0.25"/>
      </joint>

    </robot>
    ```
    
    - 身体现在是蓝色的。
        我们定义了一种称为“蓝色”的新材料，红色、绿色、蓝色和阿尔法通道分别定义为0、0、0.8和1。    
        所有值都可以在[0,1]范围内。然后，此材质将被基本链接的视觉元素引用。
        白色材质的定义类似
    - 你还可以从可视元素中定义材质标记，甚至在其他链接中引用它。
        如果你重新定义它，没有人会抱怨。
    - 还可以使用纹理指定用于为对象着色的图像文件

    roslaunch urdf_教程display.launch model:=urdf/04-materials.urdf materials屏幕截图



5. ### 完成模型

    现在，我们用更多的形状来完成模型：脚、轮子和头。
    
    最值得注意的是，我们添加了一个球体和一些网格。
    
    我们还将添加一些稍后使用的其他部件。
    
    ```xml
    <?xml version="1.0"?>
    <robot name="visual">

      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>

      <link name="base_link">
        <visual>
          <geometry>
            <cylinder length="0.6" radius="0.2"/>
          </geometry>
          <material name="blue"/>
        </visual>
      </link>

      <link name="right_leg">
        <visual>
          <geometry>
            <box size="0.6 0.1 0.2"/>
          </geometry>
          <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
          <material name="white"/>
        </visual>
      </link>

      <joint name="base_to_right_leg" type="fixed">
        <parent link="base_link"/>
        <child link="right_leg"/>
        <origin xyz="0 -0.22 0.25"/>
      </joint>

      <link name="right_base">
        <visual>
          <geometry>
            <box size="0.4 0.1 0.1"/>
          </geometry>
          <material name="white"/>
        </visual>
      </link>

      <joint name="right_base_joint" type="fixed">
        <parent link="right_leg"/>
        <child link="right_base"/>
        <origin xyz="0 0 -0.6"/>
      </joint>

      <link name="right_front_wheel">
        <visual>
          <origin rpy="1.57075 0 0" xyz="0 0 0"/>
          <geometry>
            <cylinder length="0.1" radius="0.035"/>
          </geometry>
          <material name="black"/>
        </visual>
      </link>
      <joint name="right_front_wheel_joint" type="fixed">
        <parent link="right_base"/>
        <child link="right_front_wheel"/>
        <origin rpy="0 0 0" xyz="0.133333333333 0 -0.085"/>
      </joint>

      <link name="right_back_wheel">
        <visual>
          <origin rpy="1.57075 0 0" xyz="0 0 0"/>
          <geometry>
            <cylinder length="0.1" radius="0.035"/>
          </geometry>
          <material name="black"/>
        </visual>
      </link>
      <joint name="right_back_wheel_joint" type="fixed">
        <parent link="right_base"/>
        <child link="right_back_wheel"/>
        <origin rpy="0 0 0" xyz="-0.133333333333 0 -0.085"/>
      </joint>

      <link name="left_leg">
        <visual>
          <geometry>
            <box size="0.6 0.1 0.2"/>
          </geometry>
          <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
          <material name="white"/>
        </visual>
      </link>

      <joint name="base_to_left_leg" type="fixed">
        <parent link="base_link"/>
        <child link="left_leg"/>
        <origin xyz="0 0.22 0.25"/>
      </joint>

      <link name="left_base">
        <visual>
          <geometry>
            <box size="0.4 0.1 0.1"/>
          </geometry>
          <material name="white"/>
        </visual>
      </link>

      <joint name="left_base_joint" type="fixed">
        <parent link="left_leg"/>
        <child link="left_base"/>
        <origin xyz="0 0 -0.6"/>
      </joint>

      <link name="left_front_wheel">
        <visual>
          <origin rpy="1.57075 0 0" xyz="0 0 0"/>
          <geometry>
            <cylinder length="0.1" radius="0.035"/>
          </geometry>
          <material name="black"/>
        </visual>
      </link>
      <joint name="left_front_wheel_joint" type="fixed">
        <parent link="left_base"/>
        <child link="left_front_wheel"/>
        <origin rpy="0 0 0" xyz="0.133333333333 0 -0.085"/>
      </joint>

      <link name="left_back_wheel">
        <visual>
          <origin rpy="1.57075 0 0" xyz="0 0 0"/>
          <geometry>
            <cylinder length="0.1" radius="0.035"/>
          </geometry>
          <material name="black"/>
        </visual>
      </link>
      <joint name="left_back_wheel_joint" type="fixed">
        <parent link="left_base"/>
        <child link="left_back_wheel"/>
        <origin rpy="0 0 0" xyz="-0.133333333333 0 -0.085"/>
      </joint>

      <joint name="gripper_extension" type="fixed">
        <parent link="base_link"/>
        <child link="gripper_pole"/>
        <origin rpy="0 0 0" xyz="0.19 0 0.2"/>
      </joint>

      <link name="gripper_pole">
        <visual>
          <geometry>
            <cylinder length="0.2" radius="0.01"/>
          </geometry>
          <origin rpy="0 1.57075 0 " xyz="0.1 0 0"/>
        </visual>
      </link>

      <joint name="left_gripper_joint" type="fixed">
        <origin rpy="0 0 0" xyz="0.2 0.01 0"/>
        <parent link="gripper_pole"/>
        <child link="left_gripper"/>
      </joint>

      <link name="left_gripper">
        <visual>
          <origin rpy="0.0 0 0" xyz="0 0 0"/>
          <geometry>
            <mesh filename="package://urdf_tutorial/meshes/l_finger.dae"/>
          </geometry>
        </visual>
      </link>

      <joint name="left_tip_joint" type="fixed">
        <parent link="left_gripper"/>
        <child link="left_tip"/>
      </joint>

      <link name="left_tip">
        <visual>
          <origin rpy="0.0 0 0" xyz="0.09137 0.00495 0"/>
          <geometry>
            <mesh filename="package://urdf_tutorial/meshes/l_finger_tip.dae"/>
          </geometry>
        </visual>
      </link>
      <joint name="right_gripper_joint" type="fixed">
        <origin rpy="0 0 0" xyz="0.2 -0.01 0"/>
        <parent link="gripper_pole"/>
        <child link="right_gripper"/>
      </joint>

      <link name="right_gripper">
        <visual>
          <origin rpy="-3.1415 0 0" xyz="0 0 0"/>
          <geometry>
            <mesh filename="package://urdf_tutorial/meshes/l_finger.dae"/>
          </geometry>
        </visual>
      </link>

      <joint name="right_tip_joint" type="fixed">
        <parent link="right_gripper"/>
        <child link="right_tip"/>
      </joint>

      <link name="right_tip">
        <visual>
          <origin rpy="-3.1415 0 0" xyz="0.09137 0.00495 0"/>
          <geometry>
            <mesh filename="package://urdf_tutorial/meshes/l_finger_tip.dae"/>
          </geometry>
        </visual>
      </link>

      <link name="head">
        <visual>
          <geometry>
            <sphere radius="0.2"/>
          </geometry>
          <material name="white"/>
        </visual>
      </link>
      <joint name="head_swivel" type="fixed">
        <parent link="base_link"/>
        <child link="head"/>
        <origin xyz="0 0 0.3"/>
      </joint>

      <link name="box">
        <visual>
          <geometry>
            <box size="0.08 0.08 0.08"/>
          </geometry>
          <material name="blue"/>
        </visual>
      </link>

      <joint name="tobox" type="fixed">
        <parent link="head"/>
        <child link="box"/>
        <origin xyz="0.1814 0 0.1414"/>
      </joint>
    </robot>
    ```
    
    如何添加球体应该是不言自明的
    
    ```
    <link name="head">
      <visual>
        <geometry>
          <sphere radius="0.2"/>
        </geometry>
        <material name="white"/>
      </visual>
    </link>
    ```
    
    这里的网格是从 PR2 借来的。
    
    它们是单独的文件，你必须为其指定路径。 
    
    你应该使用 package://NAME_OF_PACKAGE/path 表示法。
    
    本教程的网格位于 urdf_tutorial 包中名为 meshes 的文件夹中。
    
    ```xml
      <link name="left_gripper">
        <visual>
          <origin rpy="0.0 0 0" xyz="0 0 0"/>
          <geometry>
            <mesh filename="package://urdf_tutorial/meshes/l_finger.dae"/>
          </geometry>
        </visual>
      </link>
    ```
    
    - 网格可以以多种不同的格式导入。
        STL 相当普遍，但引擎也支持 DAE，它可以拥有自己的颜色数据，这意味着你不必指定颜色/材质。
        这些通常位于单独的文件中。
        这些网格也引用了网格文件夹中的 .tif 文件。
    - 也可以使用相对缩放参数或边界框大小来调整网格的大小。
    - 我们也可以在一个完全不同的包中引用网格，即 package://pr2_description/meshes/gripper_v0/l_finger.dae 如果安装了 pr2_description 包，它将起作用。

    你有它。 
    
    类似 R2D2 的 URDF 模型。
    
    现在你可以继续下一步，让它移动。
