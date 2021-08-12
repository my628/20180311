**描述：** 在本教程中，您将开始创建自己的URDF机器人描述文件。

**教程级别：** 初学者


1. ### 创建树结构

    在本教程中，我们将创建下图所示“机器人”的URDF描述。
    
    ![alt image](http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file?action=AttachFile&do=get&target=link.png)
    
    图中的机器人是树状结构。
    让我们从非常简单的开始，创建该树结构的描述，无需担心维度等。
    启动你最喜欢的文本编辑器，并创建一个名为my_robot.urdf的文件：
    
    ```
    <robot name="test_robot">
      <link name="link1" />
      <link name="link2" />
      <link name="link3" />
      <link name="link4" />

      <joint name="joint1" type="continuous">
        <parent link="link1"/>
        <child link="link2"/>
      </joint>

      <joint name="joint2" type="continuous">
        <parent link="link1"/>
        <child link="link3"/>
      </joint>

      <joint name="joint3" type="continuous">
        <parent link="link3"/>
        <child link="link4"/>
      </joint>
    </robot>
    ```
    
    所以，只是创建结构非常简单！
    现在让我们看看我们是否可以解析这个URDF文件。
    有一个简单的命令行工具可以为你解析URDF文件，并告诉你语法是否正确。

    你可能需要安装urdfdom作为上游的ROS独立包：
    
    ```bash
    sudo apt install liburdfdom-tools
    ```
    
    现在运行检查命令：
    
    ```bash
    rosmake urdfdom_model              # only needed if installed from source
    check_urdf my_robot.urdf                    # hydro and later
    # for older ROS distros, use the following commands (see footnote at bottom of page for why above commands are different)
    rosrun urdfdom check_urdf my_robot.urdf     # groovy
    rosrun urdf_parser check_urdf my_robot.urdf # electric and fuerte
    rosrun urdf check_urdf my_robot.urdf        # diamondback and earlier
    ```
    
    如果一切正常，输出应如下所示：
    
    ```
    robot name is: test_robot
    ---------- Successfully Parsed XML ---------------
    root Link: link1 has 2 child(ren)
        child(1):  link2
        child(2):  link3
            child(1):  link4
    ```
    
    
    
2. ### 添加维度

    所以现在我们有了基本的树结构，让我们添加适当的维度。
    正如你在机器人图像中所注意到的，每个链接（绿色）的参考系位于链接的底部，并且与关节的参考系相同。
    因此，要为我们的树添加维度，我们只需要指定从链接到其子节点的关节的偏移量。
    为此，我们将字段 <origin> 添加到每个关节。

    让我们看看第二个关节。
    “Joint2”在Y方向上与link1有偏移，在X轴负方向上与link1稍有偏移，并绕Z轴旋转90度。
    因此，我们需要添加以下<origin>元素：
    
    ```xml
    <origin xyz="-2 5 0" rpy="0 0 1.57" />
    ```
    
    如果你对所有元素重复此操作，我们的URDF将如下所示：
    
    ```xml
    <robot name="test_robot">
      <link name="link1" />
      <link name="link2" />
      <link name="link3" />
      <link name="link4" />


      <joint name="joint1" type="continuous">
        <parent link="link1"/>
        <child link="link2"/>
        <origin xyz="5 3 0" rpy="0 0 0" />
      </joint>

      <joint name="joint2" type="continuous">
        <parent link="link1"/>
        <child link="link3"/>
        <origin xyz="-2 5 0" rpy="0 0 1.57" />
      </joint>

      <joint name="joint3" type="continuous">
        <parent link="link3"/>
        <child link="link4"/>
        <origin xyz="5 0 0" rpy="0 0 -1.57" />
      </joint>
    </robot>
    ```
    
    更新你的文件my_robot.urdf并通过解析器运行它：
    
    ```bash
    check_urdf my_robot.urdf
    ```
    
    如果一切看起来不错，你可以进行下一步。
    
    
    
3. ### 完成运动学

    我们还没有指定关节围绕哪个轴旋转。
    一旦我们添加了它，我们实际上就有了这个机器人的完整运动学模型！
    我们需要做的就是将<axis>元素添加到每个关节。
    轴指定局部坐标系中的旋转轴。

    因此，如果你查看“Joint2”，你会看到它围绕正Y轴旋转。
    因此，只需将以下xml添加到联合元素：
    
    ```xml
    <axis xyz="0 1 0" />
    ```
    同样，"joint1"围绕以下轴旋转：
    
    ```xml
    <axis xyz="-0.707 0.707 0" />
    ```
    
    请注意，这是一个好主意，标准化轴。

    如果我们将其添加到机器人的所有关节，我们URDF如下所示：
    
    ```xml
    <robot name="test_robot">
      <link name="link1" />
      <link name="link2" />
      <link name="link3" />
      <link name="link4" />

      <joint name="joint1" type="continuous">
        <parent link="link1"/>
        <child link="link2"/>
        <origin xyz="5 3 0" rpy="0 0 0" />
        <axis xyz="-0.9 0.15 0" />
      </joint>

      <joint name="joint2" type="continuous">
        <parent link="link1"/>
        <child link="link3"/>
        <origin xyz="-2 5 0" rpy="0 0 1.57" />
        <axis xyz="-0.707 0.707 0" />
      </joint>

      <joint name="joint3" type="continuous">
        <parent link="link3"/>
        <child link="link4"/>
        <origin xyz="5 0 0" rpy="0 0 -1.57" />
        <axis xyz="0.707 -0.707 0" />
      </joint>
    </robot>
    ```
    
    更新你的文件my_robot.urdf并通过解析器运行它：
    
    ```bash
    check_urdf my_robot.urdf
    ```
    
    就是这样，你创建了你的第一个URDF机器人描述！
    现在你可以尝试使用graphiz可视化URDF：
    
    ```bash
    urdf_to_graphiz my_robot.urdf
    ```
    
    并使用你最喜欢的pdf查看器打开生成的文件：
    
    ```
    evince test_robot.pdf
    ```
    
    ![alt image](http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file?action=AttachFile&do=get&target=graphiz.png)
    
