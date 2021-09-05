# 使用Xacro清理URDF文件

**描述：** 学习一些使用 Xacro 减少 URDF 文件中代码量的技巧

**教程级别：** 初学者

到现在为止，如果你在家中使用自己的机器人设计遵循所有这些步骤，你可能已经厌倦了进行各种数学运算来获得非常简单的机器人描述来正确解析。

幸运的是，你可以使用 xacro 包使你的生活更简单。

它做了三件非常有用的事情。

     常数
     简单数学
     宏

在本教程中，我们将了解所有这些快捷方式，以帮助减小 URDF 文件的整体大小并使其更易于阅读和维护。



1. ### 使用 Xacro

    顾名思义，xacro 是一种宏语言。

    xacro 程序运行所有宏并输出结果。

    典型用法如下所示：

    > xacro --inorder model.xacro > model.urdf

    在 ROS 发行版 melodic 及更高版本上，你应该省略 {--inorder} 参数。

    你还可以在启动文件中自动生成 urdf。

    这很方便，因为它可以保持最新状态并且不会占用硬盘空间。

    但是，生成确实需要时间，因此请注意你的启动文件可能需要更长的时间才能启动。

    ```xml
    <param name="robot_description"
      command="xacro --inorder '$(find pr2_description)/robots/pr2.urdf.xacro'" />
    ```
    
    在 URDF 文件的顶部，你必须指定一个命名空间，以便正确解析文件。 
    
    例如，这些是有效 xacro 文件的前两行：
    
    ```xml
    <?xml version="1.0"?>
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="firefighter">
    ```
    
2. ### 常数

    让我们快速浏览一下 R2D2 中的 base_link。
    
    ```xml
      <link name="base_link">
        <visual>
          <geometry>
            <cylinder length="0.6" radius="0.2"/>
          </geometry>
          <material name="blue"/>
        </visual>
        <collision>
          <geometry>
            <cylinder length="0.6" radius="0.2"/>
          </geometry>
        </collision>
      </link>
    ```
    
    这里的信息有点多余。 我们两次指定圆柱体的长度和半径。 
    
    更糟糕的是，如果我们想改变它，我们需要在两个不同的地方这样做。

    幸运的是，xacro 允许你指定充当常量的属性。
    
    代替上面的代码，我们可以写这个。
    
    ```xml
    <xacro:property name="width" value="0.2" />
    <xacro:property name="bodylen" value="0.6" />
    <link name="base_link">
        <visual>
            <geometry>
                <cylinder radius="${width}" length="${bodylen}"/>
            </geometry>
            <material name="blue"/>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="${width}" length="${bodylen}"/>
            </geometry>
        </collision>
    </link>
    ```
    
    - 这两个值在前两行中指定。 
        它们几乎可以在任何地方（假设有效的 XML）、在任何级别、在使用之前或之后定义。 通常他们走在顶端。
    - 我们没有在几何元素中指定实际半径，而是使用美元符号和大括号来表示值。
    - 此代码将生成与上面显示的相同的代码。

    然后使用 ${} 构造的内容值替换 ${}。 这意味着您可以将其与属性中的其他文本组合。
    
    ```xml
    <xacro:property name=”robotname” value=”marvin” />
    <link name=”${robotname}s_leg” />
    ```
    
    这将产生
    
    ```xml
    <link name=”marvins_leg” />
    ```
    
    然而，${} 中的内容不必只是一个属性，这将我们带到下一点......



3. ### 数学

    你可以使用四个基本运算（+、-、*、/）、一元减号和括号在 ${} 构造中构建任意复杂的表达式。 
    
    例子：
    
    ```xml
    <cylinder radius="${wheeldiam/2}" length="0.1"/>
    <origin xyz="${reflect*(width+.02)} 0 0.25" />
    ```
    
    所有的数学运算都是使用浮点数完成的，因此
    
    ```xml
    <link name="${5/6}"/>
    ```
    
    评估为
    
    ```xml
    <link name="0.833333333333"/>
    ```
    
    在 Jade 和更高版本的发行版中，你可以使用的不仅仅是上面列出的基本操作，尤其是 sin 和 cos。



4. ### 宏

    这是 xacro 包中最大和最有用的组件。

    1. 简单宏

        我们来看一个简单的无用宏。

        ```xml
        <xacro:macro name="default_origin">
            <origin xyz="0 0 0" rpy="0 0 0"/>
        </xacro:macro>
        <xacro:default_origin />
        ```

        （这是无用的，因为如果未指定原点，它的值与此相同。）

        此代码将生成以下内容。

        ```xml
        <origin rpy="0 0 0" xyz="0 0 0"/>
        ```

        - 该名称在技术上不是必需的元素，但你需要指定它才能使用它。

        - <xacro:$NAME /> 的每个实例都替换为 xacro:macro 标记的内容。
        - 请注意，即使它不完全相同（两个属性已切换顺序），生成的 XML 也是等效的。
        - 如果没有找到具有指定名称的 xacro，它不会被扩展并且不会产生错误。

    2. 参数化宏

        你还可以参数化宏，以便它们不会每次都生成完全相同的文本。
        
        当与数学功能相结合时，这会更加强大。

        首先，让我们举一个在 R2D2 中使用的简单宏的例子。
        
        ```xml
            <xacro:macro name="default_inertial" params="mass">
                <inertial>
                        <mass value="${mass}" />
                        <inertia ixx="1.0" ixy="0.0" ixz="0.0"
                             iyy="1.0" iyz="0.0"
                             izz="1.0" />
                </inertial>
            </xacro:macro>
        ```
        
        这可以与代码一起使用
        
        ```xml
        <xacro:default_inertial mass="10"/>
        ```
        
        参数就像属性一样，你可以在表达式中使用它们

        你也可以使用整个块作为参数。
        
        ```xml
        <xacro:macro name="blue_shape" params="name *shape">
            <link name="${name}">
                <visual>
                    <geometry>
                        <xacro:insert_block name="shape" />
                    </geometry>
                    <material name="blue"/>
                </visual>
                <collision>
                    <geometry>
                        <xacro:insert_block name="shape" />
                    </geometry>
                </collision>
            </link>
        </xacro:macro>

        <xacro:blue_shape name="base_link">
            <cylinder radius=".42" length=".01" />
        </xacro:blue_shape>
        ```
        
        - 要指定模块参数，请在其参数名称前包含一个星号。
        - 可以使用 insert_block 命令插入块
        - 根据需要多次插入块。



5. ### 实际使用

    xacro语言在允许你执行的操作方面相当灵活。
    
    除了上面显示的默认惯性宏之外，这里还有一些在 R2D2 模型中使用 xacro 的有用方法。

    要查看由 xacro 文件生成的模型，请运行与之前教程相同的命令：roslaunch urdf_tutorial display.launch model:=urdf/08-macroed.urdf.xacro

    （启动文件一直在运行 xacro 命令，但由于没有宏可以扩展，所以没有关系）


    1. 腿宏

        通常，你希望在不同位置创建多个外观相似的对象。
        
        通常，这些位置会有一些对称性。
        
        你可以使用宏和一些简单的数学运算来减少必须编写的代码量，就像我们对 R2 的两条腿所做的那样。
        
        ```xml
        <xacro:macro name="leg" params="prefix reflect">
            <link name="${prefix}_leg">
                <visual>
                    <geometry>
                        <box size="${leglen} 0.1 0.2"/>
                    </geometry>
                    <origin xyz="0 0 -${leglen/2}" rpy="0 ${pi/2} 0"/>
                    <material name="white"/>
                </visual>
                <collision>
                    <geometry>
                        <box size="${leglen} 0.1 0.2"/>
                    </geometry>
                    <origin xyz="0 0 -${leglen/2}" rpy="0 ${pi/2} 0"/>
                </collision>
                <xacro:default_inertial mass="10"/>
            </link>

            <joint name="base_to_${prefix}_leg" type="fixed">
                <parent link="base_link"/>
                <child link="${prefix}_leg"/>
                <origin xyz="0 ${reflect*(width+.02)} 0.25" />
            </joint>
            <!-- A bunch of stuff cut -->
        </xacro:macro>
        <xacro:leg prefix="right" reflect="1" />
        <xacro:leg prefix="left" reflect="-1" />
        ```
        
        
        - 常见技巧 1：使用名称前缀获得两个名称相似的对象
        - 常见技巧 2：使用数学计算关节原点。 
            在你改变机器人尺寸的情况下，用一些数学方法改变一个属性来计算关节偏移量会省去很多麻烦。
        - 常见技巧 3：使用反射参数，并将其设置为 1 或 -1。
            看看我们如何使用反射参数将腿放在身体两侧的 base_to_${prefix}_leg 原点。



    2. 其他技巧

        随意在这里附加你自己的技巧。



    3. 下一步

        本节到此结束，但是如果你已完成所有这些步骤，则你已做好进入仿真的准备。 
        
        继续在 Gazebo 中使用 URDF
